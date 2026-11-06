#!/usr/bin/env bash
set -euo pipefail

echo "=== Running Gate G1 Checks ==="
# G1: pytest for degrade/validate/metrics green; naive baseline shows quadratic drift; run_eval.py prints a table
pytest tests/test_degrade.py tests/test_validate.py tests/test_metrics.py tests/test_baseline.py
python3 tools/run_eval.py --smoke-test

echo "GATE G1 PASS"
