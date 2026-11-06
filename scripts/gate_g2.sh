#!/usr/bin/env bash
set -euo pipefail

echo "=== Running Gate G2 Checks ==="
# G2: Python ESKF beats naive baseline on ATE and blind-5 s on all held-out sequences; unit tests green
pytest tests/
python3 tools/run_eval.py --eval-py-ref

echo "GATE G2 PASS"
