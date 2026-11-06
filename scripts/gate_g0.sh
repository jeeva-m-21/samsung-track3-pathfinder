#!/usr/bin/env bash
set -euo pipefail

echo "=== Running Gate G0 Checks ==="

# Check repository structure
REQUIRED_DIRS=(
    ".agent/rules"
    ".agent/workflows"
    ".agent/skills"
    "docs/packets"
    "src/ref"
    "src/cpp"
    "tools"
    "tests"
    "scripts"
    "configs"
    "results"
    "data"
)

for d in "${REQUIRED_DIRS[@]}"; do
    if [ ! -d "$d" ]; then
        echo "FAIL: Missing directory $d"
        exit 1
    fi
done

REQUIRED_FILES=(
    "PLAN.md"
    "GEMINI.md"
    ".agent/rules/00-global.md"
    ".agent/rules/10-numerics.md"
    ".agent/workflows/next.md"
    ".agent/workflows/review.md"
    ".agent/workflows/gate.md"
    ".agent/workflows/blocked.md"
    "docs/STATE.md"
    "docs/SPEC.md"
    "docs/decisions.md"
    "docs/gate_log.md"
    "docs/blockers.md"
    "configs/params.json"
    ".gitignore"
)

for f in "${REQUIRED_FILES[@]}"; do
    if [ ! -f "$f" ]; then
        echo "FAIL: Missing file $f"
        exit 1
    fi
done

# Check basic build tools
which g++ >/dev/null 2>&1 || { echo "FAIL: g++ not installed"; exit 1; }
which make >/dev/null 2>&1 || { echo "FAIL: make not installed"; exit 1; }
which python3 >/dev/null 2>&1 || { echo "FAIL: python3 not installed"; exit 1; }

echo "Repository layout and basic tools verified."
echo "GATE G0 PASS"
