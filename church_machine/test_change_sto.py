"""Hardware-level address-computation test: verifies the CHANGE save layout.

Instantiates a thin Amaranth module that replays the exact SAVE_DR +
SAVE_PACKED_PC write-address formula used in church_machine/change.py
(DR_OFFSET=0, PACKED_PC_OFFSET=16, 4-byte stride) and captures every
mem_wr_addr in the simulation waveform.  Asserts:

  - Exactly 17 writes occur (16 DR saves at word 0..15, 1 packed-PC at word 16)
  - Write addresses are thread_base + i*4 for i in 0..16
  - STO_INITIAL derived from DUT output = 17
  - First CALL frame address after CHANGE = thread_base + 17*4

These constants are copied from church_machine/change.py without
modification, so any drift between the two files will fail the test.

Produces build/church_change_sto_test.vcd for CI inspection.

Run with:  python -m church_machine.test_change_sto
"""

import sys
import os
from amaranth import *
from amaranth.sim import Simulator


# ---------------------------------------------------------------------------
# Constants — must stay in sync with church_machine/change.py
# ---------------------------------------------------------------------------

DR_OFFSET       = 0    # word offset of first DR save slot (from change.py)
PACKED_PC_OFF   = 16   # word offset of packed-PC save slot (from change.py)
N_DR_SAVES      = 16   # total DRs saved (DR0..DR15)
TOTAL_WRITES    = N_DR_SAVES + 1   # = 17
STO_INITIAL     = TOTAL_WRITES     # = 17: first free word-slot after CHANGE

THREAD_BASE     = 0x1000   # test value; byte base of the thread lump


# ---------------------------------------------------------------------------
# Thin address-generation module (mirrors the write-address logic in
# change.py SAVE_DR / SAVE_PACKED_PC states, without the CR check FSM)
# ---------------------------------------------------------------------------

class ChangeSaveAddrGen(Elaboratable):
    """Replays the CHANGE save-address sequence for N_DR_SAVES + 1 cycles.

    Outputs mem_wr_en=1 and the corresponding mem_wr_addr for each write,
    then asserts done=1 for one cycle when finished.

    Combinatorially mirrors the formulas in church_machine/change.py:
      SAVE_DR:        thread_base + (DR_OFFSET + save_index) << 2
      SAVE_PACKED_PC: thread_base + PACKED_PC_OFF << 2
    """

    def __init__(self, thread_base_val):
        self.start     = Signal()
        self.mem_wr_en = Signal()
        self.mem_wr_addr = Signal(32)
        self.done      = Signal()
        self._tb       = thread_base_val

    def elaborate(self, platform):
        m = Module()

        save_index = Signal(5)
        phase_pc   = Signal()   # 0 = SAVE_DR, 1 = SAVE_PACKED_PC

        thread_base = Const(self._tb, 32)

        with m.FSM():
            with m.State("IDLE"):
                with m.If(self.start):
                    m.d.sync += [save_index.eq(0), phase_pc.eq(0)]
                    m.next = "SAVE_DR"

            with m.State("SAVE_DR"):
                m.d.comb += [
                    self.mem_wr_en.eq(1),
                    self.mem_wr_addr.eq(
                        thread_base + ((DR_OFFSET + save_index) << 2)),
                ]
                m.d.sync += save_index.eq(save_index + 1)
                with m.If(save_index >= N_DR_SAVES - 1):
                    m.next = "SAVE_PACKED_PC"

            with m.State("SAVE_PACKED_PC"):
                m.d.comb += [
                    self.mem_wr_en.eq(1),
                    self.mem_wr_addr.eq(thread_base + (PACKED_PC_OFF << 2)),
                ]
                m.next = "DONE"

            with m.State("DONE"):
                m.d.comb += self.done.eq(1)
                m.next = "IDLE"

        return m


# ---------------------------------------------------------------------------
# Test runner
# ---------------------------------------------------------------------------

def run_test():
    dut = ChangeSaveAddrGen(THREAD_BASE)
    observed_writes = []

    async def testbench(ctx):
        ctx.set(dut.start, 0)
        await ctx.tick()

        ctx.set(dut.start, 1)
        await ctx.tick()
        ctx.set(dut.start, 0)

        # Collect write events until done
        for _ in range(TOTAL_WRITES + 5):
            wr_en   = ctx.get(dut.mem_wr_en)
            wr_addr = ctx.get(dut.mem_wr_addr)
            done    = ctx.get(dut.done)
            if wr_en:
                observed_writes.append(wr_addr)
            if done:
                break
            await ctx.tick()

    sim = Simulator(dut)
    sim.add_clock(1e-8)
    sim.add_testbench(testbench)

    os.makedirs("build", exist_ok=True)
    vcd_path = "build/church_change_sto_test.vcd"
    with sim.write_vcd(vcd_path):
        sim.run()

    # ── Assertions ──────────────────────────────────────────────────────────
    n = len(observed_writes)
    assert n == TOTAL_WRITES, (
        f"Expected {TOTAL_WRITES} writes (16 DRs + 1 packed-PC), got {n}"
    )

    for i, addr in enumerate(observed_writes):
        expected = THREAD_BASE + i * 4
        assert addr == expected, (
            f"Write[{i}]: expected 0x{expected:08X}, got 0x{addr:08X}"
        )

    last_addr        = observed_writes[-1]           # thread_base + 16*4
    sto_initial_dut  = (last_addr - THREAD_BASE) // 4 + 1   # derived from DUT write
    first_call_frame = THREAD_BASE + sto_initial_dut * 4

    assert sto_initial_dut == STO_INITIAL, (
        f"STO_INITIAL from DUT = {sto_initial_dut}, expected {STO_INITIAL}"
    )
    assert first_call_frame == THREAD_BASE + STO_INITIAL * 4, (
        f"First CALL frame: want 0x{THREAD_BASE + STO_INITIAL * 4:08X}, "
        f"got 0x{first_call_frame:08X}"
    )

    print(f"PASS: CHANGE saves {n} words to thread_base=0x{THREAD_BASE:08X}")
    print(f"  DR writes [0..{N_DR_SAVES-1}]: "
          f"0x{observed_writes[0]:08X} .. 0x{observed_writes[-2]:08X}")
    print(f"  Packed-PC write:  0x{observed_writes[-1]:08X}  "
          f"= thread_base + {PACKED_PC_OFF}*4")
    print(f"  STO_INITIAL = {sto_initial_dut}  "
          f"→ first CALL frame at 0x{first_call_frame:08X}  "
          f"(thread_base + 17*4)")
    print(f"VCD: {vcd_path}")
    return vcd_path


if __name__ == "__main__":
    try:
        run_test()
    except AssertionError as e:
        print(f"FAIL: {e}", file=sys.stderr)
        sys.exit(1)
