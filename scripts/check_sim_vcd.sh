#!/usr/bin/env bash
# CI guard: verify regenerated simulation VCDs meet quality requirements.
#
# Checks performed:
#   1. Neither VCD contains stale cr5_stack signal references (removed in #451).
#   2. ctmm_amaranth/sim_output.vcd contains church_op=CHANGE (binary 5) and
#      church_op=CALL (binary 3) value-change records, proving the
#      install-via-CHANGE path is exercised.
#   3. ctmm_amaranth/sim_output.vcd contains boot_cr5_wr_en=1 and
#      boot_cr5_wr_gt.perms=b11000 (L|S = 0x18) transitions — direct
#      DUT waveform proof that CR5 is installed by the boot/CHANGE
#      mechanism with correct capabilities and no dedicated hardware stack.
#   4. church_machine/test_change_sto.py: runs a hardware-level address-
#      generation simulation and asserts that CHANGE writes exactly 17 words
#      (16 DRs + 1 packed-PC) before STO_INITIAL=17, so the first CALL frame
#      lands at thread_base + 17*4.  The produced VCD is also parsed for the
#      packed-PC write at thread_base + 16*4 (= 0x1040 with thread_base=0x1000).
#
# Regeneration commands:
#   python3 -m ctmm_amaranth.testbench           → ctmm_amaranth/sim_output.vcd
#   python3 -m church_machine.test_pico_ice      → build/church_pico_ice_test.vcd
#   python3 -m church_machine.test_change_sto    → build/church_change_sto_test.vcd
#
# Exit codes:
#   0 — all checks pass
#   1 — one or more checks failed

set -euo pipefail

SIM_VCD="ctmm_amaranth/sim_output.vcd"
PICO_VCD="build/church_pico_ice_test.vcd"
FAILED=0

# ── 1. Stale cr5_stack check ─────────────────────────────────────────────────
for vcd in "$SIM_VCD" "$PICO_VCD"; do
    if [ ! -f "$vcd" ]; then
        echo "FAIL: $vcd not found" >&2
        FAILED=1
        continue
    fi
    count=$(grep -c "cr5_stack" "$vcd" || true)
    if [ "$count" -gt 0 ]; then
        echo "FAIL: $vcd contains $count stale cr5_stack reference(s)" >&2
        FAILED=1
    else
        echo "OK:   $vcd — no cr5_stack references"
    fi
done

# ── 2. CHANGE and CALL opcodes in sim_output.vcd ─────────────────────────────
if [ -f "$SIM_VCD" ]; then
    church_op_id=$(grep '\$var.*church_op ' "$SIM_VCD" | grep -o ' [0-9A-Za-z!-~]\+ church_op' | awk '{print $1}' | head -1 || true)
    if [ -z "$church_op_id" ]; then
        echo "FAIL: church_op signal not found in $SIM_VCD" >&2
        FAILED=1
    else
        # CHANGE opcode = 5 = b00101; VCD omits leading zeros → b101
        if grep -qE "^b0*101[[:space:]]+${church_op_id}$" "$SIM_VCD"; then
            echo "OK:   $SIM_VCD — church_op=CHANGE (b101=5) waveform activity confirmed"
        else
            echo "FAIL: $SIM_VCD — church_op=CHANGE (b101) not found (id=${church_op_id})" >&2
            FAILED=1
        fi
        # CALL opcode = 3 = b00011; VCD omits leading zeros → b11
        if grep -qE "^b0*11[[:space:]]+${church_op_id}$" "$SIM_VCD"; then
            echo "OK:   $SIM_VCD — church_op=CALL (b11=3) waveform activity confirmed"
        else
            echo "FAIL: $SIM_VCD — church_op=CALL (b11) not found (id=${church_op_id})" >&2
            FAILED=1
        fi
    fi
fi

# ── 3. Direct DUT signal: boot_cr5_wr_en high + correct perms ────────────────
# This confirms CR5 is installed by the boot/CHANGE mechanism (no hardware
# cr5_stack) with L|S permissions (b11000 = 0x18 = PERM_MASK_L | PERM_MASK_S).
if [ -f "$SIM_VCD" ]; then
    # Find VCD signal IDs for boot_cr5_wr_en and boot_cr5_wr_gt.perms
    wr_en_id=$(grep '\$var.*boot_cr5_wr_en' "$SIM_VCD" \
                 | grep -o '\$var wire [0-9]* [^ ]* ' | awk '{print $4}' | head -1 || true)
    perms_id=$(grep 'boot_cr5_wr_gt.perms' "$SIM_VCD" \
                 | grep -o '\$var wire [0-9]* [^ ]* ' | awk '{print $4}' | head -1 || true)

    if [ -z "$wr_en_id" ]; then
        echo "FAIL: boot_cr5_wr_en signal not found in $SIM_VCD" >&2
        FAILED=1
    elif grep -q "1${wr_en_id}" "$SIM_VCD"; then
        echo "OK:   $SIM_VCD — boot_cr5_wr_en=1 fired (CR5 install write confirmed)"
    else
        echo "FAIL: $SIM_VCD — boot_cr5_wr_en never went high" >&2
        FAILED=1
    fi

    if [ -z "$perms_id" ]; then
        echo "FAIL: boot_cr5_wr_gt.perms signal not found in $SIM_VCD" >&2
        FAILED=1
    elif grep -qE "^b0*11000[[:space:]]+${perms_id}$" "$SIM_VCD"; then
        echo "OK:   $SIM_VCD — boot_cr5_wr_gt.perms=b11000 (L|S=0x18) confirmed in waveform"
    else
        echo "FAIL: $SIM_VCD — boot_cr5_wr_gt.perms=b11000 not found (id=${perms_id})" >&2
        FAILED=1
    fi
fi

# ── 4. Hardware-level STO=17 assertion via test_change_sto ───────────────────
# Runs the Amaranth address-generation sim; the test asserts:
#   - 17 writes from CHANGE (16 DRs + 1 packed-PC)
#   - last write at thread_base + 16*4
#   - STO_INITIAL = 17 → first CALL frame at thread_base + 17*4
# Then parses the produced VCD to confirm packed-PC write address (0x1040)
# is visible in the waveform as a concrete simulation signal transition.
STO_VCD="build/church_change_sto_test.vcd"
sto_out=$(python3 -m church_machine.test_change_sto 2>&1) || { echo "FAIL: church_machine.test_change_sto exited non-zero"; FAILED=1; }
if [ "${FAILED:-0}" -eq 0 ]; then
    echo "$sto_out"
    # Assert "first CALL frame at 0x00001044 (thread_base + 17*4)" appears in test output
    if echo "$sto_out" | grep -q "first CALL frame at 0x00001044"; then
        echo "OK:   church_machine.test_change_sto — STO_INITIAL=17 asserted, first CALL frame at thread_base+17*4 (0x00001044)"
    else
        echo "FAIL: test_change_sto did not confirm first CALL frame at 0x00001044 (thread_base+17*4)" >&2
        FAILED=1
    fi
fi
# VCD check: packed-PC write at thread_base(=0x1000) + 16*4 = 0x1040
# Binary: 0x1040 = 0b1000001000000 (13 bits, leading zeros stripped by VCD)
if [ "${FAILED:-0}" -eq 0 ] && [ -f "$STO_VCD" ]; then
    wr_addr_id=$(grep 'mem_wr_addr' "$STO_VCD" \
                   | grep -o '\$var wire [0-9]* [^ ]* ' | awk '{print $4}' | head -1 || true)
    if [ -z "$wr_addr_id" ]; then
        echo "FAIL: mem_wr_addr signal not found in $STO_VCD" >&2
        FAILED=1
    elif grep -qF "b1000001000000 ${wr_addr_id}" "$STO_VCD"; then
        echo "OK:   $STO_VCD — packed-PC write at 0x1040 (thread_base+16*4) confirmed in waveform"
    else
        echo "FAIL: $STO_VCD — packed-PC write 0x1040 not found in waveform (id=${wr_addr_id})" >&2
        FAILED=1
    fi
fi

if [ "$FAILED" -eq 1 ]; then
    echo ""
    echo "check-sim-vcd: FAILED — regenerate with:" >&2
    echo "  python3 -m ctmm_amaranth.testbench" >&2
    echo "  python3 -m church_machine.test_pico_ice" >&2
    echo "  python3 -m church_machine.test_change_sto" >&2
    exit 1
fi

echo ""
echo "check-sim-vcd: all checks passed"
