#!/usr/bin/env bash
# run-all-tests.sh — runs every CI test suite in sequence.
# Exits non-zero as soon as any suite fails, printing which suite failed.

set -euo pipefail

PASS=0
FAIL=0
FAILED_SUITES=()

run_suite() {
    local name="$1"
    shift
    echo ""
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "  SUITE: $name"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    if eval "$*"; then
        echo "  ✔  $name PASSED"
        PASS=$((PASS + 1))
    else
        echo "  ✘  $name FAILED"
        FAIL=$((FAIL + 1))
        FAILED_SUITES+=("$name")
        echo ""
        echo "STOPPING: suite '$name' failed."
        echo "Passed so far: $PASS   Failed: $FAIL"
        exit 1
    fi
}

cd "$(dirname "$0")/.."

run_suite "check-stale-cr7" \
    'bash scripts/check_stale_cr7.sh'

run_suite "check-selftest-lump-stale" \
    'node scripts/check_selftest_lump_stale.js && node scripts/test_check_selftest_lump_stale.js'

run_suite "check-capabilities-blocks" \
    'node scripts/check-capabilities-blocks.js'

run_suite "check-api-reference-stale" \
    'node scripts/gen-api-reference.js --check'

run_suite "lump-consistency" \
    'python -m pytest tests/lump/test_lump_consistency.py -v'

run_suite "assembler-tests" \
    'npm test'

run_suite "fault-recovery-tests" \
    'node simulator/test_fault_recovery.js'

run_suite "lump-binary-tests" \
    'node simulator/test_load_lump_binary.js'

run_suite "lump-roundtrip" \
    'node simulator/test_lump_roundtrip.js'

run_suite "catalog-compile-tests" \
    'node simulator/test_catalog_compile.js'

run_suite "boot-entry-sync-tests" \
    'node simulator/test_boot_entry_sync.js'

run_suite "warning-panel-tests" \
    'node simulator/test_asm_warning_panel.js'

run_suite "rci-threading-tests" \
    'node simulator/test_rci_threading.js'

run_suite "pending-gt-tests" \
    'node simulator/test_lazy_resolve_pending.js'

run_suite "selftest-lump-runs" \
    'python -m pytest tests/simulator/test_selftest_lump_runs.py -v'

run_suite "boot-image-matches-sim" \
    'python3 -m pytest tests/boot/test_boot_image_matches_simulator.py -v'

run_suite "boot-image-loads-and-boots" \
    'python -m pytest tests/boot/test_boot_image_loads_and_boots.py -v'

run_suite "boot-image-upload-endpoint" \
    'python -m pytest tests/boot/test_boot_image_upload_endpoint.py -v'

run_suite "boot-image-serve-endpoints" \
    'python -m pytest tests/boot/test_boot_image_serve_endpoints.py -v'

run_suite "boot-layout-regression" \
    'python -m pytest tests/boot/test_boot_layout_no_null_slot2.py -v'

run_suite "version-telemetry-tests" \
    'python3 -m pytest tests/server/test_version_telemetry.py -v'

run_suite "hardware-sim" \
    'python -m ctmm_cap_amaranth.testbench && python -m hardware.test_mwin_seal && python -m hardware.test_outform_mode2 && python -m hardware.test_shift_ops'

run_suite "e2e-tests" \
    'CHROMIUM=$(which chromium) && mkdir -p .cache/ms-playwright/chromium-1217/chrome-linux64 && ln -sf "$CHROMIUM" .cache/ms-playwright/chromium-1217/chrome-linux64/chrome && PLAYWRIGHT_SKIP_BROWSER_DOWNLOAD=1 npx --yes playwright test'

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  ALL SUITES PASSED ($PASS suites)"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
exit 0
