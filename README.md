numabench
===================
This is a microbenchmark to study the performance of Linux's NUMA balancing.
- Memory is split into per-thread contiguous chunks (assumes 4 KiB pages).
- Each thread repeatedly computes a random within the chunk, loads it, and measures the latency.
- With -R <pct>, a fraction of accesses target another thread's chunk (remote-by-owner).
- Threads run continuously; a reporter prints average latency per thread every interval.

### Build:
```
gcc -O2 -pthread -o numabench numabench.c -lnuma
```
  
### CLI:
```
  [-s size]      total size (e.g., 32G). Rounded down to 4 KiB.
  [-t threads]   number of worker threads (default 8)
  [-R pct]       percent of loads that are remote-by-owner (0..100, default 0)
  [-M]           manual page placement: map each thread's pages to one NUMA node (RR across nodes)
  [-B]           bind each thread to CPUs of its designated node (same RR mapping as -M)
  [-I secs]      reporting interval seconds (float, default 1.0)
  [-D secs]      duration seconds (float, default: infinite until Ctrl-C)
```

### Example:
```
./numabench -s 16G -B
watch -n 1 numastat -p $(pgrep numabench)
perf stat -e ocr.reads_to_core.local_dram,ocr.reads_to_core.remote_dram
-- There's also ocr.reads_to_core.snc_dram for different SNC, same socket, but it seems to induce multiplexing and makes results weird.
```

### Output: 
One line per interval; tab-separated averages per thread (cycles on x86, ns elsewhere).

### Notes:
- "Local" and "Remote" are by ownership (thread chunk), not by NUMA locality.
- With AutoNUMA enabled, local-by-owner should trend faster if pages migrate near where threads run.
- To estimate an upper bound, use -M and -B. They bind each thread and it's chunk to the same NUMA domain.