#!/usr/bin/env bash
set -euo pipefail

echo "=== Running Gate G5 Checks ==="
# G5: Tuned params improve held-out score over untuned; ablation table in results/; G3 and G4 re-pass
python3 tools/tune.py --verify-held-out
scripts/gate_g3.sh
scripts/gate_g4.sh

echo "GATE G5 PASS"
