from amaranth import *
from amaranth.sim import Simulator

from .types import *
from .layouts import GT_LAYOUT, CAP_REG_LAYOUT, COND_FLAGS_LAYOUT
from .core import CTMMCore


def build_gt(offset, perms, spare=0):
    return offset | (spare << 32) | (perms << 48)


def encode_turing(opcode, cond, dr_dst, dr_src1, dr_src2, use_imm, imm_val):
    instr = (opcode & 0x1F) << 27
    instr |= (cond & 0xF) << 23
    instr |= (use_imm & 1) << 22
    instr |= (dr_dst & 0xF) << 18
    instr |= (dr_src1 & 0xF) << 14
    instr |= (dr_src2 & 0xF) << 10
    if use_imm:
        instr |= imm_val & 0x3FFF
    return instr


def run_testbench():
    dut = CTMMCore()
    sim = Simulator(dut)
    sim.add_clock(1e-6)

    imem = [0] * 256

    imem[0] = encode_turing(TuringOpcode.LDI, CondCode.AL, 0, 0, 0, 0, 42)
    imem[1] = encode_turing(TuringOpcode.ADD, CondCode.AL, 1, 0, 0, 1, 10)
    imem[2] = encode_turing(TuringOpcode.CMP, CondCode.AL, 0, 1, 0, 1, 52)

    async def testbench(ctx):
        print("=" * 50)
        print("CTMM Amaranth Testbench - Starting")
        print("=" * 50)

        print("\n[TEST 1] Boot Sequence")
        print("-" * 40)

        ctx.set(dut.boot_start, 1)
        await ctx.tick()
        ctx.set(dut.boot_start, 0)

        for _ in range(10):
            await ctx.tick()
            bs = ctx.get(dut.boot_state)
            if bs == BootState.COMPLETE:
                break

        boot_done = ctx.get(dut.boot_complete)
        boot_st = ctx.get(dut.boot_state)
        print(f"  Boot state: {boot_st} (COMPLETE={BootState.COMPLETE})")
        print(f"  Boot complete: {boot_done}")
        assert boot_done == 1, "Boot did not complete!"

        print("\n[TEST 2] Turing Instructions (LDI, ADD, CMP)")
        print("-" * 40)

        for cycle in range(10):
            nia_val = ctx.get(dut.nia)
            instr_idx = (nia_val >> 2) & 0xFF
            if instr_idx < len(imem):
                ctx.set(dut.imem_data, imem[instr_idx])
                ctx.set(dut.imem_valid, 1)
            else:
                ctx.set(dut.imem_data, 0)
                ctx.set(dut.imem_valid, 0)
            await ctx.tick()

        final_nia = ctx.get(dut.nia)
        print(f"  NIA after execution: {final_nia}")
        fault_v = ctx.get(dut.fault_valid)
        fault_t = ctx.get(dut.fault)
        print(f"  Fault valid: {fault_v}, type: {fault_t}")

        print("\n[TEST 3] Fault Aggregation")
        print("-" * 40)
        ctx.set(dut.imem_data, 0x00000000)
        ctx.set(dut.imem_valid, 1)
        await ctx.tick()
        fault_v = ctx.get(dut.fault_valid)
        print(f"  Invalid opcode fault: valid={fault_v}")

        print("\n" + "=" * 50)
        print("CTMM Amaranth Testbench - Complete")
        print("=" * 50)

    sim.add_testbench(testbench)

    with sim.write_vcd("ctmm_amaranth/sim_output.vcd"):
        sim.run()


if __name__ == "__main__":
    run_testbench()
