#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

echo "=== Running Gate G4 Checks ==="

# Step 1: Sanitizer check
echo "Step 1: Building sanitized binary and running adversarial fuzz suite under ASan/UBSan..."
make -C src/cpp sanitize
pytest tests/fuzz/

# Step 2: Build clean release binary
echo "Step 2: Building clean release binary..."
make -C src/cpp clean
make -C src/cpp CXXFLAGS="-O3 -std=c++17 -Wall -Wextra -Werror -fno-exceptions -fno-rtti"

# Step 3: Run budget checks on synthetic sequences
echo "Step 3: Running budget checks on synthetic sequences..."
if [ ! -f "data/synth/figure_eight.csv" ]; then
    python3 tools/synth.py --type figure_eight --duration 10.0 --rate 200.0 --output-imu data/synth/figure_eight.csv --output-gt data/synth/figure_eight_gt.txt
fi

scripts/budget_check.sh data/synth/circle.csv
scripts/budget_check.sh data/synth/figure_eight.csv

# If degraded directory exists, run budget checks on it as well
if [ -d "data/degraded" ]; then
    echo "Running budget checks on data/degraded..."
    scripts/budget_check.sh data/degraded/
fi

echo "GATE G4 PASS"
