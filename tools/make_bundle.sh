#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

TEAM_NAME="${1:-VITC_SingleThread}"
BUNDLE_ZIP="${TEAM_NAME}.zip"

echo "=== Building Submission Bundle for ${TEAM_NAME} (PRD Section 8.1) ==="

# Step 1: Ensure binaries and estimated_trajectory.txt are fresh
echo "[1/5] Building release binary..."
make clean > /dev/null
make > /dev/null

if [ ! -f "estimated_trajectory.txt" ]; then
    echo "[2/5] Generating estimated_trajectory.txt from official evaluation sequence (EuRoC MH_01_easy)..."
    ./main data/euroc/MH_01_easy estimated_trajectory.txt
else
    echo "[2/5] Using existing estimated_trajectory.txt..."
fi

# Validate trajectory format
echo "      Validating estimated_trajectory.txt..."
python3 tools/validate_trajectory.py data/euroc/MH_01_easy/mav0/imu0/data.csv estimated_trajectory.txt

# Step 2: Validate metadata.json
echo "[3/5] Validating metadata.json..."
if [ ! -f "metadata.json" ]; then
    echo "Error: metadata.json not found!"
    exit 1
fi
python3 -c "import json; d = json.load(open('metadata.json')); assert d['team'] == '${TEAM_NAME}', f'Team name mismatch: {d[\"team\"]}'"

# Clean build artifacts inside src/ before packaging so clean-room tests build from scratch
make -C src clean > /dev/null

# Step 3: Package bundle into zip
echo "[4/5] Packaging ${BUNDLE_ZIP}..."
rm -f "${BUNDLE_ZIP}"

# Per PRD Section 8.1, the bundle must contain:
# team_name.zip
# ├── estimated_trajectory.txt
# ├── metadata.json
# └── src/
zip -r "${BUNDLE_ZIP}" estimated_trajectory.txt metadata.json src/ \
    -x "src/cpp/build/*" -x "src/main" -x "*.pyc" -x "*/__pycache__/*" > /dev/null

echo "      Bundle created successfully: ${BUNDLE_ZIP} ($(du -h "${BUNDLE_ZIP}" | cut -f1))"

# Step 4: Verify Clean-Room Build
echo "[5/5] Performing Clean-Room Build Verification..."
CLEAN_ROOM=$(mktemp -d /tmp/clean_room_check_XXXXXX)
trap 'rm -rf "$CLEAN_ROOM"' EXIT

unzip -q "${BUNDLE_ZIP}" -d "${CLEAN_ROOM}"

# Verify extracted layout
[ -f "${CLEAN_ROOM}/estimated_trajectory.txt" ] || { echo "FAIL: estimated_trajectory.txt missing in bundle"; exit 1; }
[ -f "${CLEAN_ROOM}/metadata.json" ] || { echo "FAIL: metadata.json missing in bundle"; exit 1; }
[ -d "${CLEAN_ROOM}/src" ] || { echo "FAIL: src/ directory missing in bundle"; exit 1; }
[ -f "${CLEAN_ROOM}/src/README.md" ] || { echo "FAIL: src/README.md missing in bundle"; exit 1; }

# Compile in clean room from src/
echo "      Compiling binary in isolated clean room..."
make -C "${CLEAN_ROOM}/src" > /dev/null

# Verify compiled binary
[ -x "${CLEAN_ROOM}/src/main" ] || { echo "FAIL: Clean room binary src/main was not built"; exit 1; }
[ -x "${CLEAN_ROOM}/main" ] || { echo "FAIL: Clean room binary main was not copied to root"; exit 1; }

# Run binary in clean room on test trajectory
echo "      Executing clean-room binary..."
"${CLEAN_ROOM}/main" data/synth/circle.csv "${CLEAN_ROOM}/test_out.txt"
python3 tools/validate_trajectory.py data/synth/circle.csv "${CLEAN_ROOM}/test_out.txt"

# Measure clean-room memory footprint
CLEAN_RSS=$(/usr/bin/time -v taskset -c 0 "${CLEAN_ROOM}/main" data/synth/circle.csv "${CLEAN_ROOM}/test_out.txt" 2>&1 | grep "Maximum resident" | awk '{print $NF}')
echo "      Clean-room peak RSS: ${CLEAN_RSS} KB (Budget: <= 4096 KB)"
[ "$CLEAN_RSS" -le 4096 ] || { echo "FAIL: Clean-room RSS exceeded budget ($CLEAN_RSS KB)"; exit 1; }

echo "=== CLEAN-ROOM VERIFICATION PASSED ==="
echo "Submission zip is ready: $(pwd)/${BUNDLE_ZIP}"
unzip -l "${BUNDLE_ZIP}" | head -n 25 || true
