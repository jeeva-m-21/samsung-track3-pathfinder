#!/usr/bin/env bash
set -euo pipefail

if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <WP-id>"
    exit 1
fi

WP="$1"
PACKET_FILE="docs/packets/${WP}.md"

if [ ! -f "$PACKET_FILE" ]; then
    echo "Error: Packet file $PACKET_FILE does not exist"
    exit 1
fi

# Extract allowlist line from packet
# Example format: "Allowlist: src/ref/mathlib.py, tests/test_mathlib.py"
ALLOWLIST_LINE=$(grep -i "^Allowlist:" "$PACKET_FILE" | head -n 1 | sed 's/^[Aa]llowlist:[[:space:]]*//')

if [ -z "$ALLOWLIST_LINE" ]; then
    echo "Error: No Allowlist found in $PACKET_FILE"
    exit 1
fi

# Convert comma-separated allowlist into an array / list of patterns
IFS=',' read -r -a ALLOWED_ITEMS <<< "$ALLOWLIST_LINE"

# Trim whitespace from items
CLEAN_ALLOWED=()
for item in "${ALLOWED_ITEMS[@]}"; do
    trimmed=$(echo "$item" | xargs)
    if [ -n "$trimmed" ]; then
        CLEAN_ALLOWED+=("$trimmed")
    fi
done

# Get modified files (tracked modifications and untracked files)
# Check git diff against HEAD if HEAD exists, or against index
if git rev-parse --verify HEAD >/dev/null 2>&1; then
    CHANGED_FILES=$(git diff --name-only HEAD; git ls-files --others --exclude-standard)
else
    CHANGED_FILES=$(git diff --name-only; git ls-files --others --exclude-standard)
fi

FAIL=0
for file in $CHANGED_FILES; do
    [ -z "$file" ] && continue
    MATCHED=0
    for allowed in "${CLEAN_ALLOWED[@]}"; do
        # Exact match or prefix directory match or wildcard match
        if [[ "$file" == "$allowed" ]] || [[ "$file" == $allowed ]] || [[ "$file" == "$allowed"* ]]; then
            MATCHED=1
            break
        fi
    done
    if [ "$MATCHED" -eq 0 ]; then
        echo "SCOPE VIOLATION: $file is not in Allowlist (${CLEAN_ALLOWED[*]})"
        FAIL=1
    fi
done

if [ "$FAIL" -ne 0 ]; then
    echo "Check scope failed for $WP."
    exit 1
fi

echo "Scope OK for $WP"
exit 0
