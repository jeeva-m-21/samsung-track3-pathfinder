#!/usr/bin/env bash
set -euo pipefail

echo "=== Running Gate G4 Checks ==="
# G4: Fuzz suite green under ASan/UBSan; budget regression green
make -C src/cpp sanitize
pytest tests/fuzz/
scripts/budget_check.sh data/synth_sample

echo "GATE G4 PASS"
