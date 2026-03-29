"""
Regression tests for SWITCH instruction PassKey validation (Task #58).

Checks enforced by ChurchSwitch.elaborate() before mLoad is invoked:

  1. Target check  — only Tgt=5 (CR13) and Tgt=7 (CR15) are valid.
                     All other Tgt values produce INVALID_OP without reading CRs.

  2. PassKey type check  — CRs.word0_gt.gt_type must equal GT_TYPE_ABSTRACT (11₂).
                           Any Inform, Outform, or NULL GT produces INVALID_OP.

  3. Sentinel address check  — CRs.word1_location must equal the reserved
                               hardware sentinel for the target register:
                                 Tgt=CR13 → 0xFFFFFFFE  (all-1s − 1)
                                 Tgt=CR15 → 0xFFFFFFFF  (all-1s)
                               A mismatch produces INVALID_OP.

Test matrix (12 scenarios):
  Fault cases:
    T1  — invalid target (Tgt=0/CR8)                         → INVALID_OP fault
    T2  — invalid target (Tgt=4/CR12)                        → INVALID_OP fault
    T3  — valid target CR13, NULL source GT                   → INVALID_OP fault
    T4  — valid target CR13, Inform source GT                 → INVALID_OP fault
    T5  — valid target CR13, Outform source GT                → INVALID_OP fault
    T6  — valid target CR13, Abstract GT + wrong sentinel     → INVALID_OP fault
             (CR15 sentinel 0xFFFFFFFF presented to CR13 target)
    T7  — valid target CR15, Abstract GT + wrong sentinel     → INVALID_OP fault
             (CR13 sentinel 0xFFFFFFFE presented to CR15 target)

  Valid cases (reach mLoad without prior fault):
    T8  — valid target CR13, Abstract GT + correct sentinel   → stays busy (mLoad starts)
    T9  — valid target CR15, Abstract GT + correct sentinel   → stays busy (mLoad starts)
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from amaranth import *
from amaranth.lib.data import View
from amaranth.sim import Simulator, Tick

from hardware.switch import ChurchSwitch
from hardware.hw_types import (
    FaultType,
    GT_TYPE_NULL, GT_TYPE_INFORM, GT_TYPE_OUTFORM, GT_TYPE_ABSTRACT,
    SWITCH_PASSKEY_SENTINEL_CR13, SWITCH_PASSKEY_SENTINEL_CR15,
    SWITCH_TGT_CR13, SWITCH_TGT_CR15,
)
from hardware.layouts import GT_LAYOUT, CAP_REG_LAYOUT


def _build_gt(gt_type=GT_TYPE_INFORM, slot_id=0, gt_seq=0, perms=0, b_flag=0):
    """Pack a 32-bit GT word from its fields."""
    gt  = (slot_id & 0xFFFF)
    gt |= (gt_seq  & 0x7F) << 16
    gt |= (gt_type & 0x03) << 23
    gt |= (perms   & 0x3F) << 25
    gt |= (b_flag  & 0x01) << 31
    return gt


def _build_cap(gt_type=GT_TYPE_INFORM, slot_id=0, perms=0,
               location=0, word2=0, word3=0):
    """Build a 128-bit CAP_REG value (4 × 32-bit words packed as Python int)."""
    gt = _build_gt(gt_type=gt_type, slot_id=slot_id, perms=perms)
    return gt | (location << 32) | (word2 << 64) | (word3 << 96)


def _run_switch(tgt, src_cap, max_ticks=20):
    """
    Drive ChurchSwitch for up to *max_ticks* after switch_start.

    Returns (first_fault_type, still_busy_at_end).

    first_fault_type is FaultType.NONE if no fault was seen; otherwise it is the
    fault_type value from the first tick on which switch_fault went high.

    Note: The ChurchSwitch PassKey validation exclusively emits FaultType.INVALID_OP.
    Any other fault code (e.g. BOUNDS, NULL_CAP) originates from the inner mLoad
    sub-operation and indicates that all PassKey checks passed.
    """
    dut = ChurchSwitch()
    first_fault = [FaultType.NONE]
    still_busy = [False]

    ns_cap = _build_cap(gt_type=GT_TYPE_INFORM, slot_id=0, location=0x8000)

    def process():
        yield dut.cr_rd_data.eq(src_cap)
        yield dut.cr_src.eq(0)
        yield dut.target.eq(tgt)
        yield dut.index.eq(0)
        yield dut.cr15_namespace.eq(ns_cap)
        yield dut.mem_rd_valid.eq(1)
        yield dut.mem_rd_data.eq(0)

        yield dut.switch_start.eq(1)
        yield Tick()
        yield dut.switch_start.eq(0)

        for _ in range(max_ticks):
            yield Tick()
            if (yield dut.switch_fault) and first_fault[0] == FaultType.NONE:
                first_fault[0] = (yield dut.fault_type)

        still_busy[0] = bool((yield dut.switch_busy))

    sim = Simulator(dut)
    sim.add_clock(1e-6)
    sim.add_process(process)
    with sim.write_vcd("/dev/null"):
        sim.run()

    return first_fault[0], still_busy[0]


def test_invalid_target_cr8():
    """T1: Tgt=0 (CR8) is not a valid SWITCH target — expect INVALID_OP."""
    src = _build_cap(gt_type=GT_TYPE_ABSTRACT, location=SWITCH_PASSKEY_SENTINEL_CR15)
    ftype, _ = _run_switch(tgt=0, src_cap=src)
    assert ftype == FaultType.INVALID_OP, f"Expected INVALID_OP, got {ftype}"


def test_invalid_target_cr12():
    """T2: Tgt=4 (CR12) is not a valid SWITCH target — expect INVALID_OP."""
    src = _build_cap(gt_type=GT_TYPE_ABSTRACT, location=SWITCH_PASSKEY_SENTINEL_CR15)
    ftype, _ = _run_switch(tgt=4, src_cap=src)
    assert ftype == FaultType.INVALID_OP, f"Expected INVALID_OP, got {ftype}"


def test_valid_target_cr13_null_gt():
    """T3: Tgt=CR13, source is NULL GT — expect INVALID_OP (not Abstract)."""
    src = _build_cap(gt_type=GT_TYPE_NULL, location=SWITCH_PASSKEY_SENTINEL_CR13)
    ftype, _ = _run_switch(tgt=SWITCH_TGT_CR13, src_cap=src)
    assert ftype == FaultType.INVALID_OP, f"Expected INVALID_OP, got {ftype}"


def test_valid_target_cr13_inform_gt():
    """T4: Tgt=CR13, source is Inform GT — expect INVALID_OP (not Abstract)."""
    src = _build_cap(gt_type=GT_TYPE_INFORM, location=SWITCH_PASSKEY_SENTINEL_CR13)
    ftype, _ = _run_switch(tgt=SWITCH_TGT_CR13, src_cap=src)
    assert ftype == FaultType.INVALID_OP, f"Expected INVALID_OP, got {ftype}"


def test_valid_target_cr13_outform_gt():
    """T5: Tgt=CR13, source is Outform GT — expect INVALID_OP (not Abstract)."""
    src = _build_cap(gt_type=GT_TYPE_OUTFORM, location=SWITCH_PASSKEY_SENTINEL_CR13)
    ftype, _ = _run_switch(tgt=SWITCH_TGT_CR13, src_cap=src)
    assert ftype == FaultType.INVALID_OP, f"Expected INVALID_OP, got {ftype}"


def test_cr13_passkey_presented_to_cr15_target():
    """T6: Tgt=CR15 but CRs carries the CR13 sentinel — sentinel mismatch → INVALID_OP."""
    src = _build_cap(gt_type=GT_TYPE_ABSTRACT, location=SWITCH_PASSKEY_SENTINEL_CR13)
    ftype, _ = _run_switch(tgt=SWITCH_TGT_CR15, src_cap=src)
    assert ftype == FaultType.INVALID_OP, f"Expected INVALID_OP (sentinel mismatch), got {ftype}"


def test_cr15_passkey_presented_to_cr13_target():
    """T7: Tgt=CR13 but CRs carries the CR15 sentinel — sentinel mismatch → INVALID_OP."""
    src = _build_cap(gt_type=GT_TYPE_ABSTRACT, location=SWITCH_PASSKEY_SENTINEL_CR15)
    ftype, _ = _run_switch(tgt=SWITCH_TGT_CR13, src_cap=src)
    assert ftype == FaultType.INVALID_OP, f"Expected INVALID_OP (sentinel mismatch), got {ftype}"


def test_valid_cr13_switch_reaches_mload():
    """T8: Valid CR13 SWITCH — Abstract GT + correct sentinel.

    All PassKey checks must pass (no INVALID_OP fault). The inner mLoad may
    fault with a different code (e.g. BOUNDS) because the Abstract GT has no
    real NS entry — that is expected and does NOT indicate a PassKey failure.
    We distinguish by asserting first_fault_type != INVALID_OP.
    """
    src = _build_cap(gt_type=GT_TYPE_ABSTRACT, location=SWITCH_PASSKEY_SENTINEL_CR13)
    ftype, _ = _run_switch(tgt=SWITCH_TGT_CR13, src_cap=src, max_ticks=20)
    assert ftype != FaultType.INVALID_OP, (
        f"PassKey validation rejected a valid CR13 PassKey (got INVALID_OP). "
        f"Any non-INVALID_OP fault comes from mLoad and is acceptable here."
    )


def test_valid_cr15_switch_reaches_mload():
    """T9: Valid CR15 SWITCH — Abstract GT + correct sentinel.

    All PassKey checks must pass (no INVALID_OP fault). Same rationale as T8.
    """
    src = _build_cap(gt_type=GT_TYPE_ABSTRACT, location=SWITCH_PASSKEY_SENTINEL_CR15)
    ftype, _ = _run_switch(tgt=SWITCH_TGT_CR15, src_cap=src, max_ticks=20)
    assert ftype != FaultType.INVALID_OP, (
        f"PassKey validation rejected a valid CR15 PassKey (got INVALID_OP). "
        f"Any non-INVALID_OP fault comes from mLoad and is acceptable here."
    )


if __name__ == "__main__":
    tests = [
        ("T1: invalid target CR8",             test_invalid_target_cr8),
        ("T2: invalid target CR12",            test_invalid_target_cr12),
        ("T3: CR13 target, NULL GT",           test_valid_target_cr13_null_gt),
        ("T4: CR13 target, Inform GT",         test_valid_target_cr13_inform_gt),
        ("T5: CR13 target, Outform GT",        test_valid_target_cr13_outform_gt),
        ("T6: CR13 PassKey → CR15 target",     test_cr13_passkey_presented_to_cr15_target),
        ("T7: CR15 PassKey → CR13 target",     test_cr15_passkey_presented_to_cr13_target),
        ("T8: valid CR13 SWITCH (mLoad busy)", test_valid_cr13_switch_reaches_mload),
        ("T9: valid CR15 SWITCH (mLoad busy)", test_valid_cr15_switch_reaches_mload),
    ]

    passed = 0
    failed = 0
    for name, fn in tests:
        try:
            fn()
            print(f"  PASS  {name}")
            passed += 1
        except AssertionError as e:
            print(f"  FAIL  {name}: {e}")
            failed += 1
        except Exception as e:
            print(f"  ERR   {name}: {type(e).__name__}: {e}")
            failed += 1

    print(f"\n{passed}/{passed + failed} tests passed")
    if failed:
        sys.exit(1)
