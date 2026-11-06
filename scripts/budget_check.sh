#!/usr/bin/env bash
set -euo pipefail
IN="$1"; BIN=src/cpp/build/main; OUT=$(mktemp)
/usr/bin/time -v taskset -c 0 "$BIN" "$IN" "$OUT" 2> /tmp/time.txt
RSS_KB=$(grep "Maximum resident" /tmp/time.txt | awk '{print $NF}')
USER=$(grep "User time" /tmp/time.txt | awk '{print $NF}')
SYS=$(grep "System time" /tmp/time.txt | awk '{print $NF}')
DUR=$(python3 tools/data_duration.py "$IN")
MS=$(python3 -c "print(($USER+$SYS)/$DUR*1000)")
echo "peak_rss_kb=$RSS_KB cpu_ms_per_s=$MS duration_s=$DUR"
[ "$RSS_KB" -le 4096 ] || { echo "FAIL: RSS gate"; exit 1; }
python3 -c "import sys; sys.exit(0 if $MS <= 2.0 else 1)" || { echo "FAIL: CPU gate"; exit 1; }
"$BIN" "$IN" "$OUT"; sha1sum "$OUT" > /tmp/a.sha
"$BIN" "$IN" "$OUT"; sha1sum "$OUT" > /tmp/b.sha
cmp /tmp/a.sha /tmp/b.sha || { echo "FAIL: nondeterministic"; exit 1; }
THREADS=$(grep -c . /dev/null; true)
python3 tools/validate_trajectory.py "$IN" "$OUT"
echo "BUDGET OK"
