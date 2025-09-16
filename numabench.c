// CLI:
//   [-s size]      total size (e.g., 32G). Rounded down to 4 KiB.
//   [-t threads]   number of worker threads (default 8)
//   [-R pct]       percent of loads that are remote-by-owner (0..100, default 0)
//   [-M]           manual page placement: map each thread's pages to one NUMA node (RR across nodes)
//   [-B]           bind each thread to CPUs of its designated node (same RR mapping as -M)
//   [-I secs]      reporting interval seconds (float, default 1.0)
//   [-D secs]      duration seconds (float, default: infinite until Ctrl-C)

#define _GNU_SOURCE
#include <pthread.h>
#include <numa.h>
#include <numaif.h>
#include <sys/mman.h>
#include <sys/sysinfo.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>
#include <errno.h>
#include <signal.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <sched.h>   // sched_setaffinity, CPU_ALLOC, etc.

#ifndef MADV_NOHUGEPAGE
#define MADV_NOHUGEPAGE 15
#endif

#if defined(__x86_64__) || defined(__i386__)
static inline uint64_t rdtscp(void){ unsigned int aux,lo,hi; __asm__ volatile("rdtscp":"=a"(lo),"=d"(hi),"=c"(aux)); return ((uint64_t)hi<<32)|lo; }
static inline void cpuid_barrier(void){ unsigned int a,b,c,d; a=0; __asm__ volatile("cpuid": "=a"(a),"=b"(b),"=c"(c),"=d"(d):"a"(a):"memory"); }
#define HAS_TSC 1
#else
#define HAS_TSC 0
#endif

static inline uint64_t nsec_now(void){ struct timespec ts; clock_gettime(CLOCK_MONOTONIC_RAW,&ts); return (uint64_t)ts.tv_sec*1000000000ull + ts.tv_nsec; }

enum { PAGE_SZ = 4096, CL_SIZE = 64 };

typedef struct {
    // immutable
    int tid, tcount;
    unsigned char *base;
    size_t total_pages;
    size_t chunk_first_page;
    size_t chunk_pages;

    // policy
    int remote_pct;    // 0..100
    int bind_cpu;
    const cpu_set_t *cpu_mask;
    size_t cpu_mask_sz;

    // rng state
    uint64_t rng;

    // live stats (atomically updated)
    volatile uint64_t sum_lat;    // cycles or ns
    volatile uint64_t count;      // samples

    // control
    volatile int *running;        // shared flag
} worker_t;

typedef struct {
    // CLI
    size_t total_size;
    int tcount;
    int remote_pct;
    int manual_place;
    int bind_to_node;
    double interval_sec;
    double duration_sec;
} params_t;

// ---------- util ----------
static size_t parse_size(const char *s){
    char *end=NULL; double v=strtod(s,&end); if(end==s||v<=0) return 0;
    while(*end==' ') end++;
    double mul=1.0;
    if(*end){
        if(*end=='K'||*end=='k') mul=1024.0;
        else if(*end=='M'||*end=='m') mul=1024.0*1024.0;
        else if(*end=='G'||*end=='g') mul=1024.0*1024.0*1024.0;
        else if(*end=='T'||*end=='t') mul=1024.0*1024.0*1024.0*1024.0;
    }
    return (size_t)(v*mul);
}

static void usage(const char *p){
    fprintf(stderr,
        "Usage: %s [-s size] [-t threads] [-R pct] [-M] [-B] [-I secs] [-D secs]\n"
        "Defaults: size=32G, threads=8, R=0, -M off, -B off, I=1.0, D=inf\n", p);
}

static int build_cpu_mask_for_node(int node, cpu_set_t **mask_out, size_t *sz_out){
    long ncpus = sysconf(_SC_NPROCESSORS_CONF);
    if (ncpus <= 0) ncpus = 1;
    size_t sz = CPU_ALLOC_SIZE((int)ncpus);
    cpu_set_t *mask = CPU_ALLOC((int)ncpus);
    if (!mask) return -1;
    CPU_ZERO_S(sz, mask);
    struct bitmask *cpus_on_node = numa_allocate_cpumask();
    if (!cpus_on_node){ CPU_FREE(mask); return -1; }
    if (numa_node_to_cpus(node, cpus_on_node) != 0){
        numa_free_cpumask(cpus_on_node); CPU_FREE(mask); return -1;
    }
    for (int c=0; c<ncpus; c++){
        if (numa_bitmask_isbitset(cpus_on_node, c)) CPU_SET_S(c, sz, mask);
    }
    numa_free_cpumask(cpus_on_node);
    *mask_out = mask; *sz_out = sz;
    return 0;
}

static void fault_all_pages(unsigned char *base, size_t pages){
    for (size_t i=0;i<pages;i++){
        base[i*(size_t)PAGE_SZ] = (unsigned char)i;
    }
}

// Returns: total_ok (>=0 status), total_on_target (status==nodes[k]), total_fail (<0)
static void move_pages_to_node_with_stats(unsigned char *base, size_t first_page, size_t n_pages, int node,
                                          long long *total_ok, long long *total_on_target, long long *total_fail){
    const size_t B = 16384;
    void **addrs = (void**)malloc(B*sizeof(void*));
    int *nodes = (int*)malloc(B*sizeof(int));
    int *status = (int*)malloc(B*sizeof(int));
    if (!addrs || !nodes || !status){ perror("malloc move_pages"); goto out; }
    size_t p = 0;
    while (p < n_pages){
        size_t take = (n_pages - p) < B ? (n_pages - p) : B;
        for (size_t k=0;k<take;k++){
            addrs[k] = (void*)(base + ((first_page + p + k) * (size_t)PAGE_SZ));
            nodes[k] = node;
            status[k] = 0;
        }
        long rc = move_pages(0, (int)take, addrs, nodes, status, MPOL_MF_MOVE);
        if (rc < 0 && errno != EPERM && errno != EACCES) perror("move_pages");
        for (size_t k=0;k<take;k++){
            if (status[k] < 0) { (*total_fail)++; }
            else { (*total_ok)++; if (status[k] == nodes[k]) (*total_on_target)++; }
        }
        p += take;
    }
out:
    free(addrs); free(nodes); free(status);
}

// xorshift64*
static inline uint64_t rng_next(uint64_t *s){
    uint64_t x = *s;
    x ^= x >> 12;
    x ^= x << 25;
    x ^= x >> 27;
    *s = x;
    return x * 2685821657736338717ULL;
}

// ---------- worker ----------
static void *worker_main(void *arg){
    worker_t *w = (worker_t*)arg;

    if (w->bind_cpu && w->cpu_mask){
        if (sched_setaffinity(0, w->cpu_mask_sz, w->cpu_mask) != 0) perror("sched_setaffinity");
    }

    const size_t my_first = w->chunk_first_page;
    const size_t my_pages = w->chunk_pages;

    while (__atomic_load_n(w->running, __ATOMIC_RELAXED)){
        // Decide owner: local or remote-by-owner
        uint64_t r = rng_next(&w->rng);
        int go_remote = (w->remote_pct > 0) && ((int)(r % 100) < w->remote_pct) && (w->tcount > 1);
        int owner_tid = w->tid;

        if (go_remote){
            // pick victim != self
            int delta = (int)( (rng_next(&w->rng) % (uint64_t)(w->tcount - 1)) ) + 1;
            owner_tid = (w->tid + delta) % w->tcount;
        }
        // owner's chunk
        size_t owner_first = my_first;
        size_t owner_pages = my_pages;
        if (owner_tid != w->tid){
            // map tid -> chunk (even split)
            size_t per = w->total_pages / (size_t)w->tcount;
            owner_first = per * (size_t)owner_tid;
            owner_pages = (owner_tid == w->tcount-1) ? (w->total_pages - owner_first) : per;
        }

        // choose random page & 64B-aligned offset within the page
        size_t page_idx = (size_t)(rng_next(&w->rng) % owner_pages);
        size_t cloffs = ((size_t)(rng_next(&w->rng) % (PAGE_SZ/CL_SIZE))) * CL_SIZE;

        unsigned char *addr = w->base + ((owner_first + page_idx) * (size_t)PAGE_SZ) + cloffs;

#if HAS_TSC
        cpuid_barrier();
        uint64_t t0 = rdtscp();
        (void)*(volatile uint64_t*)addr;
        uint64_t t1 = rdtscp();
        cpuid_barrier();
        uint64_t dt = (t1 - t0);
#else
        uint64_t t0 = nsec_now();
        (void)*(volatile uint64_t*)addr;
        uint64_t t1 = nsec_now();
        uint64_t dt = (t1 - t0);
#endif
        __atomic_fetch_add(&w->sum_lat, dt, __ATOMIC_RELAXED);
        __atomic_fetch_add(&w->count, 1, __ATOMIC_RELAXED);
    }
    return NULL;
}

static void randomize_pages_across_nodes(unsigned char *base, size_t total_pages,
                                         const int *allowed, int an, uint64_t *rng,
                                         long long *total_ok, long long *total_on_target, long long *total_fail){
    const size_t B = 16384;
    void **addrs = malloc(B*sizeof(void*));
    int *nodes = malloc(B*sizeof(int));
    int *status = malloc(B*sizeof(int));
    if(!addrs||!nodes||!status){ perror("malloc randomize"); goto out; }
    size_t p = 0;
    while (p < total_pages){
        size_t take = (total_pages - p) < B ? (total_pages - p) : B;
        for (size_t k=0;k<take;k++){
            addrs[k] = (void*)(base + ((p + k) * (size_t)PAGE_SZ));
            uint64_t r = rng_next(rng);
            nodes[k] = allowed[r % (uint64_t)an];
            status[k] = 0;
        }
        long rc = move_pages(0, (int)take, addrs, nodes, status, MPOL_MF_MOVE);
        if (rc < 0 && errno != EPERM && errno != EACCES) perror("move_pages randomize");
        for (size_t k=0;k<take;k++){
            if (status[k] < 0) { (*total_fail)++; }
            else { (*total_ok)++; if (status[k] == nodes[k]) (*total_on_target)++; }
        }
        p += take;
    }
out:
    free(addrs); free(nodes); free(status);
}

// ---------- main / reporter ----------
static volatile int g_running = 1;
static void on_sigint(int sig){ (void)sig; g_running = 0; }

int main(int argc, char **argv){
    setvbuf(stdout, NULL, _IOLBF, 0);

    if (numa_available() < 0){ fprintf(stderr,"NUMA not available.\n"); return 1; }

    // defaults
    params_t P = {
        .total_size   = 0, // set below
        .tcount       = 8,
        .remote_pct   = 0,
        .manual_place = 0,
        .bind_to_node = 0,
        .interval_sec = 1.0,
        .duration_sec = -1.0
    };
    P.total_size = parse_size("32G");

    int opt;
    while((opt=getopt(argc,argv,"s:t:R:MBI:D:h"))!=-1){
        switch(opt){
            case 's': { size_t v=parse_size(optarg); if(v) P.total_size=v; break; }
            case 't': P.tcount=atoi(optarg); if(P.tcount<=0) P.tcount=1; break;
            case 'R': P.remote_pct=atoi(optarg); if(P.remote_pct<0) P.remote_pct=0; if(P.remote_pct>100) P.remote_pct=100; break;
            case 'M': P.manual_place=1; break;
            case 'B': P.bind_to_node=1; break;
            case 'I': P.interval_sec=strtod(optarg,NULL); if(P.interval_sec<=0) P.interval_sec=1.0; break;
            case 'D': P.duration_sec=strtod(optarg,NULL); break;
            case 'h': default: usage(argv[0]); return 2;
        }
    }

    // page-align size
    P.total_size = (P.total_size / PAGE_SZ) * PAGE_SZ;
    size_t total_pages = (P.total_size / PAGE_SZ);
    if (total_pages < (size_t)P.tcount){
        fprintf(stderr,"Total size too small for thread count.\n");
        return 2;
    }

    // mmap
    unsigned char *base = mmap(NULL, P.total_size, PROT_READ|PROT_WRITE,
                               MAP_PRIVATE|MAP_ANONYMOUS|MAP_NORESERVE, -1, 0);
    if (base == MAP_FAILED){ perror("mmap"); return 1; }
    // Anti-THP / random access hints
    //madvise(base, P.total_size, MADV_NOHUGEPAGE);
    //madvise(base, P.total_size, MADV_RANDOM);

    // Fault pages once
    fault_all_pages(base, total_pages);

    // Build allowed[] once so we can reuse for randomize/-M/-B
    int maxnode = numa_max_node();
    int *allowed = (int*)malloc((maxnode+1)*sizeof(int));
    int an = 0;
    for (int n=0;n<=maxnode;n++) allowed[an++] = n;
    if (an == 0){ fprintf(stderr,"No NUMA nodes found.\n"); return 1; }

    // If not doing -M, start from a randomized (worst-ish) placement
    long long rnd_ok=0, rnd_on_target=0, rnd_fail=0;
    if (!P.manual_place) {
        uint64_t seed = 0x1234cafef00dULL;
        randomize_pages_across_nodes(base, total_pages, allowed, an, &seed,
                                     &rnd_ok, &rnd_on_target, &rnd_fail);
        fprintf(stderr, "Randomize: ok=%lld on_target=%lld fail=%lld\n",
                rnd_ok, rnd_on_target, rnd_fail);
    }

    // Per-thread chunking (even split except last gets remainder)
    size_t per_pages = total_pages / (size_t)P.tcount;

    // Manual placement: RR across nodes
    long long man_ok=0, man_on_target=0, man_fail=0;
    if (P.manual_place){
        for (int t=0;t<P.tcount; t++){
            size_t first = per_pages * (size_t)t;
            size_t count = (t == P.tcount-1) ? (total_pages - first) : per_pages;
            int node = allowed[t % an];
            move_pages_to_node_with_stats(base, first, count, node, &man_ok, &man_on_target, &man_fail);
        }
        fprintf(stderr, "Manual place: ok=%lld on_target=%lld fail=%lld\n",
                man_ok, man_on_target, man_fail);
    }

    // Optional per-thread CPU binding to designated node (RR mapping same as above)
    cpu_set_t **node_masks = NULL; size_t *node_mask_sz = NULL; int *node_for_tid = NULL;
    if (P.bind_to_node){
        node_masks   = (cpu_set_t**)calloc((size_t)P.tcount, sizeof(cpu_set_t*));
        node_mask_sz = (size_t*)calloc((size_t)P.tcount, sizeof(size_t));
        node_for_tid = (int*)calloc((size_t)P.tcount, sizeof(int));
        if(!node_masks||!node_mask_sz||!node_for_tid){ perror("calloc"); return 1; }
        for (int t=0;t<P.tcount;t++){
            int node = allowed[t % an];
            node_for_tid[t] = node;
            if (build_cpu_mask_for_node(node, &node_masks[t], &node_mask_sz[t]) != 0){
                node_masks[t] = NULL; node_mask_sz[t] = 0;
                fprintf(stderr, "Warning: node %d has empty CPU mask; T%d not pinned\n", node, t);
            }
        }
    }

    fprintf(stderr,
        "Config: size=%.2f GiB, pages=%zu, threads=%d, R=%d%%, M=%d, B=%d, I=%.3fs, D=%s\n",
        (double)P.total_size/(1024.0*1024.0*1024.0), total_pages, P.tcount, P.remote_pct,
        P.manual_place, P.bind_to_node, P.interval_sec, (P.duration_sec>0? "set" : "infinite"));

    // Install SIGINT handler to stop
    struct sigaction sa; memset(&sa,0,sizeof(sa)); sa.sa_handler = on_sigint; sigaction(SIGINT,&sa,NULL);

    // Launch workers
    pthread_t *ths = (pthread_t*)malloc((size_t)P.tcount*sizeof(pthread_t));
    worker_t  *ws  = (worker_t*)calloc((size_t)P.tcount, sizeof(worker_t));
    if (!ths || !ws){ perror("alloc threads"); return 1; }

    for (int t=0;t<P.tcount;t++){
        size_t first = per_pages * (size_t)t;
        size_t count = (t == P.tcount-1) ? (total_pages - first) : per_pages;

        ws[t].tid = t;
        ws[t].tcount = P.tcount;
        ws[t].base = base;
        ws[t].total_pages = total_pages;
        ws[t].chunk_first_page = first;
        ws[t].chunk_pages = count;
        ws[t].remote_pct = P.remote_pct;
        ws[t].bind_cpu = P.bind_to_node ? 1 : 0;
        ws[t].cpu_mask = (P.bind_to_node && node_masks) ? node_masks[t] : NULL;
        ws[t].cpu_mask_sz = (P.bind_to_node && node_masks) ? node_mask_sz[t] : 0;
        ws[t].rng = 0x9e3779b97f4a7c15ull ^ (uint64_t)(t*0x85ebca6bull);
        ws[t].sum_lat = 0;
        ws[t].count = 0;
        ws[t].running = &g_running;

        if (pthread_create(&ths[t], NULL, worker_main, &ws[t]) != 0){
            perror("pthread_create"); return 1;
        }

        int node_print = (node_for_tid ? node_for_tid[t] : -1);
        fprintf(stderr, "T%02d -> node %d  pages [%zu..%zu)\n", t, node_print, first, first+count);
    }

    // Reporter loop
    uint64_t *prev_sum = (uint64_t*)calloc((size_t)P.tcount, sizeof(uint64_t));
    uint64_t *prev_cnt = (uint64_t*)calloc((size_t)P.tcount, sizeof(uint64_t));
    if (!prev_sum || !prev_cnt){ perror("alloc prev"); return 1; }

    double elapsed = 0.0;
    while (g_running){
        struct timespec ts;
        ts.tv_sec = (time_t)P.interval_sec;
        ts.tv_nsec = (long)((P.interval_sec - (double)ts.tv_sec) * 1e9);
        if (ts.tv_nsec < 0) ts.tv_nsec = 0;
        nanosleep(&ts, NULL);

        for (int t=0;t<P.tcount;t++){
            uint64_t sum = __atomic_load_n(&ws[t].sum_lat, __ATOMIC_RELAXED);
            uint64_t cnt = __atomic_load_n(&ws[t].count,   __ATOMIC_RELAXED);
            uint64_t dsum = (sum - prev_sum[t]);
            uint64_t dcnt = (cnt - prev_cnt[t]);
            prev_sum[t] = sum; prev_cnt[t] = cnt;

            unsigned long long out = 0ULL;
            if (dcnt) {
                double avg = (double)dsum / (double)dcnt;
                out = (unsigned long long)(avg + 0.5);
            }
            if (t) fputc('\t', stdout);
            printf("%llu", out);
        }
        fputc('\n', stdout);
        fflush(stdout);

        if (P.duration_sec > 0){
            elapsed += P.interval_sec;
            if (elapsed + 1e-9 >= P.duration_sec){
                g_running = 0;
            }
        }
    }

    // Join and cleanup
    for (int t=0;t<P.tcount;t++) pthread_join(ths[t], NULL);

    if (node_masks){
        for (int t=0;t<P.tcount;t++) if (node_masks[t]) CPU_FREE(node_masks[t]);
        free(node_masks); free(node_mask_sz); free(node_for_tid);
    }
    free(prev_sum); free(prev_cnt);
    munmap(base, P.total_size);
    free(ths); free(ws);
    free(allowed);
    return 0;
}
