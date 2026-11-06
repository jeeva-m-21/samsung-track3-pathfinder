#!/usr/bin/env bash
set -euo pipefail

echo "=== Running Gate G3 Checks ==="
# G3: C++ matches Python (diff tolerances), validator passes, RSS <= 4 MB, <= 2 ms per 1 s, 1 thread, deterministic
python3 scripts/diff_py_cpp.py --all
scripts/budget_check.sh data/synth_sample
make -C src/cpp test

echo "GATE G3 PASS"
