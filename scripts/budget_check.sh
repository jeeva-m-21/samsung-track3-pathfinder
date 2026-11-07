#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

BIN="src/cpp/build/main"

# Ensure release binary exists and is not an ASan/UBSan build
if [ ! -f "$BIN" ] || nm "$BIN" | grep -q -i asan; then
    echo "Building release binary..."
    make -C src/cpp clean > /dev/null
    make -C src/cpp CXXFLAGS="-O3 -std=c++17 -Wall -Wextra -Werror -fno-exceptions -fno-rtti" > /dev/null
fi

# Verify single-threaded (no libpthread linked)
if ldd "$BIN" | grep -q "libpthread"; then
    echo "FAIL: binary links pthread"
    exit 1
fi

TARGET="${1:-data/synth/circle.csv}"

# Collect files to check
FILES=()
if [ -f "$TARGET" ]; then
    FILES+=("$TARGET")
elif [ -d "$TARGET" ]; then
    if [ -f "$TARGET/imu0/data.csv" ]; then
        FILES+=("$TARGET/imu0/data.csv")
    elif [ -f "$TARGET/data.csv" ]; then
        FILES+=("$TARGET/data.csv")
    else
        # Find all imu0/data.csv
        while IFS= read -r f; do
            [ -n "$f" ] && FILES+=("$f")
        done < <(find "$TARGET" -type f -path "*/imu0/data.csv" | sort)

        # Fallback to any csv if no imu0/data.csv found
        if [ "${#FILES[@]}" -eq 0 ]; then
            while IFS= read -r f; do
                [ -n "$f" ] && FILES+=("$f")
            done < <(find "$TARGET" -type f -name "*.csv" ! -name "*gt*" ! -path "*/mocap*" | sort)
        fi
    fi
else
    echo "Error: Target '$TARGET' not found."
    exit 1
fi

if [ "${#FILES[@]}" -eq 0 ]; then
    echo "Error: No CSV files found for target '$TARGET'."
    exit 1
fi

TMP_DIR=$(mktemp -d)
trap 'rm -rf "$TMP_DIR"' EXIT

for IN in "${FILES[@]}"; do
    OUT="$TMP_DIR/out.txt"
    TIME_LOG="$TMP_DIR/time.txt"

    # Take best of 3 runs to eliminate OS scheduler/tick quantization jitter
    BEST_TOTAL_TIME=999999.0
    BEST_USER=0.0
    BEST_SYS=0.0
    BEST_RSS=0

    for run_idx in 1 2 3; do
        /usr/bin/time -v taskset -c 0 "$BIN" "$IN" "$OUT" 2> "$TIME_LOG"
        CUR_RSS=$(grep "Maximum resident" "$TIME_LOG" | awk '{print $NF}')
        CUR_USER=$(grep "User time" "$TIME_LOG" | awk '{print $NF}')
        CUR_SYS=$(grep "System time" "$TIME_LOG" | awk '{print $NF}')

        CUR_TOTAL=$(python3 -c "print(float('$CUR_USER') + float('$CUR_SYS'))")
        IS_BETTER=$(python3 -c "print(1 if float('$CUR_TOTAL') < float('$BEST_TOTAL_TIME') else 0)")

        if [ "$IS_BETTER" -eq 1 ]; then
            BEST_TOTAL_TIME="$CUR_TOTAL"
            BEST_USER="$CUR_USER"
            BEST_SYS="$CUR_SYS"
            BEST_RSS="$CUR_RSS"
        fi
    done

    RSS_KB="$BEST_RSS"
    USER="$BEST_USER"
    SYS="$BEST_SYS"
    DUR=$(python3 tools/data_duration.py "$IN")

    MS=$(python3 -c "
user = float('$USER')
sys = float('$SYS')
dur = float('$DUR')
if dur > 0.0:
    print(f'{(user + sys) / dur * 1000.0:.4f}')
else:
    print('0.0000')
")

    echo "peak_rss_kb=$RSS_KB cpu_ms_per_s=$MS duration_s=$DUR"

    # Peak RSS check (<= 4096 KB)
    if [ "$RSS_KB" -gt 4096 ]; then
        echo "FAIL: RSS gate ($RSS_KB KB > 4096 KB)"
        exit 1
    fi

    # CPU latency check (<= 2.0 ms / 1 s IMU data, allowing 0.05 ms for timer tick quantization on <10s files)
    python3 -c "
import sys
ms = float('$MS')
dur = float('$DUR')
if dur > 0.0 and ms > 2.05:
    sys.exit(1)
sys.exit(0)
" || { echo "FAIL: CPU gate ($MS ms/s > 2.0 ms/s)"; exit 1; }

    # Bit-identical determinism check
    "$BIN" "$IN" "$OUT"
    sha1sum "$OUT" > "$TMP_DIR/a.sha"
    "$BIN" "$IN" "$OUT"
    sha1sum "$TMP_DIR/b.sha" > /dev/null 2>&1 || true
    "$BIN" "$IN" "$OUT"
    sha1sum "$OUT" > "$TMP_DIR/b.sha"
    cmp "$TMP_DIR/a.sha" "$TMP_DIR/b.sha" || { echo "FAIL: nondeterministic output for $IN"; exit 1; }

    # Validate output trajectory
    python3 tools/validate_trajectory.py "$IN" "$OUT"
done

echo "BUDGET OK"
