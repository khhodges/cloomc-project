#!/usr/bin/env bash
# CI guard: fail if any stale CR7 signal names survive into synthesised Verilog.
#
# The registers were renamed CR7→CR14 in hardware/core.py.  If someone
# regenerates the netlists from an unpatched source the old cr7_wr_* names
# will silently reappear.  This script catches that before it reaches tapeout.
#
# Exit codes:
#   0 — no stale names found (clean)
#   1 — one or more matches found (stale output detected), or a required file is missing

set -euo pipefail

# If file paths are supplied as arguments, check those; otherwise fall back to
# the canonical synthesised-output locations.
if [ "$#" -gt 0 ]; then
    VERILOG_FILES=("$@")
else
    VERILOG_FILES=(
        "verilog/church_core.v"
        "verilog/church_tang_nano_20k.v"
    )
fi

PATTERN="cr7_wr_"

FAILED=0

for f in "${VERILOG_FILES[@]}"; do
    if [ ! -f "$f" ]; then
        echo "FAIL: $f not found — expected synthesised output is missing" >&2
        FAILED=1
        continue
    fi

    matches=$(grep -c "$PATTERN" "$f" || true)
    if [ "$matches" -gt 0 ]; then
        echo "FAIL: $f contains $matches occurrence(s) of '$PATTERN'" >&2
        grep -n "$PATTERN" "$f" >&2
        FAILED=1
    else
        echo "OK:   $f — no stale '$PATTERN' signal names"
    fi
done

if [ "$FAILED" -eq 1 ]; then
    echo ""
    echo "Stale CR7 signal names detected in synthesised output." >&2
    echo "Re-run 'python hardware/gen_verilog.py' (or equivalent) after" >&2
    echo "verifying hardware/core.py uses the CR14 names throughout." >&2
    exit 1
fi

echo ""
echo "check-stale-cr7: all clear"
