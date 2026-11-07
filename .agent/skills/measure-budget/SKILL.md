---
name: measure-budget
description: Measure peak RSS, CPU time per second of IMU data, thread count. Use for any budget claim.
---
1. `/usr/bin/time -v taskset -c 0 src/cpp/build/main <in> /tmp/o.txt 2> /tmp/time.txt`
2. Peak RSS: `grep "Maximum resident" /tmp/time.txt`
3. CPU: user+system seconds from the same file, divided by data duration
   (last timestamp minus first, in seconds); report in ms per 1 s.
4. Report raw lines plus the computed numbers. Never estimate.
