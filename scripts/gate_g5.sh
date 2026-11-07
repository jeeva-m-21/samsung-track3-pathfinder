#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

echo "=== Running Gate G5 Checks ==="

# Step 1: Check that results/final_ablation.md exists and is non-empty
if [ ! -s "results/final_ablation.md" ]; then
    echo "FAIL: results/final_ablation.md missing or empty"
    exit 1
fi
echo "Check: results/final_ablation.md verified."

# Step 2: Check that visualization plots exist in results/plots/
for plot in results/plots/trajectory_circle_degraded.png \
            results/plots/error_circle_degraded.png \
            results/plots/trajectory_figure_eight_degraded.png \
            results/plots/error_figure_eight_degraded.png; do
    if [ ! -s "$plot" ]; then
        echo "FAIL: $plot missing or empty"
        exit 1
    fi
done
echo "Check: visualization plots verified."

# Step 3: Verify held-out score improvement
python3 tools/tune.py --verify-held-out

# Step 4: Run Gate G3 (Parity, budget, C++ unit tests)
scripts/gate_g3.sh

# Step 5: Run Gate G4 (ASan/UBSan adversarial fuzz suite, clean release build, budget regression)
scripts/gate_g4.sh

echo "GATE G5 PASS"
