from amaranth import *
from amaranth.lib.data import View

from .types import *
from .layouts import COND_FLAGS_LAYOUT


class CTMMDecoder(Elaboratable):
    def __init__(self):
        self.instruction = Signal(32)
        self.instr_valid = Signal()
        self.flags = Signal(COND_FLAGS_LAYOUT)

        self.exec_enable = Signal()
        self.is_church_op = Signal()
        self.is_turing_op = Signal()

        self.church_op = Signal(5)
        self.cr_src = Signal(3)
        self.cr_dst = Signal(3)
        self.clist_index = Signal(10)
        self.tperm_preset = Signal(4)
        self.call_mask = Signal(10)
        self.imm_mode = Signal()
        self.switch_target = Signal(3)

        self.turing_op = Signal(5)
        self.dr_src1 = Signal(4)
        self.dr_src2 = Signal(4)
        self.dr_dst = Signal(4)
        self.immediate = Signal(14)
        self.ldi_immediate = Signal(22)
        self.branch_offset = Signal(18)

        self.excl_result_dr = Signal(4)
        self.reg_list = Signal(16)

        self.fault = Signal(4)
        self.fault_valid = Signal()

    def elaborate(self, platform):
        m = Module()

        opcode_field = self.instruction[27:32]
        cond_field = self.instruction[23:27]
        i_bit = self.instruction[22]
        operand_field = self.instruction[0:22]

        m.d.comb += self.imm_mode.eq(i_bit)

        flags_view = View(COND_FLAGS_LAYOUT, self.flags)
        cond_pass = Signal()

        with m.Switch(cond_field):
            with m.Case(CondCode.EQ):
                m.d.comb += cond_pass.eq(flags_view.Z)
            with m.Case(CondCode.NE):
                m.d.comb += cond_pass.eq(~flags_view.Z)
            with m.Case(CondCode.CS):
                m.d.comb += cond_pass.eq(flags_view.C)
            with m.Case(CondCode.CC):
                m.d.comb += cond_pass.eq(~flags_view.C)
            with m.Case(CondCode.MI):
                m.d.comb += cond_pass.eq(flags_view.N)
            with m.Case(CondCode.PL):
                m.d.comb += cond_pass.eq(~flags_view.N)
            with m.Case(CondCode.VS):
                m.d.comb += cond_pass.eq(flags_view.V)
            with m.Case(CondCode.VC):
                m.d.comb += cond_pass.eq(~flags_view.V)
            with m.Case(CondCode.HI):
                m.d.comb += cond_pass.eq(flags_view.C & ~flags_view.Z)
            with m.Case(CondCode.LS):
                m.d.comb += cond_pass.eq(~flags_view.C | flags_view.Z)
            with m.Case(CondCode.GE):
                m.d.comb += cond_pass.eq(flags_view.N == flags_view.V)
            with m.Case(CondCode.LT):
                m.d.comb += cond_pass.eq(flags_view.N != flags_view.V)
            with m.Case(CondCode.GT):
                m.d.comb += cond_pass.eq(~flags_view.Z & (flags_view.N == flags_view.V))
            with m.Case(CondCode.LE):
                m.d.comb += cond_pass.eq(flags_view.Z | (flags_view.N != flags_view.V))
            with m.Case(CondCode.AL):
                m.d.comb += cond_pass.eq(1)
            with m.Case(CondCode.NV):
                m.d.comb += cond_pass.eq(0)

        m.d.comb += self.exec_enable.eq(self.instr_valid & cond_pass)

        m.d.comb += [
            self.is_church_op.eq((opcode_field >= ChurchOpcode.LOAD) & (opcode_field <= ChurchOpcode.STM)),
            self.is_turing_op.eq(opcode_field[4]),
        ]

        m.d.comb += [
            self.church_op.eq(opcode_field),
            self.cr_dst.eq(operand_field[19:22]),
            self.cr_src.eq(operand_field[16:19]),
            self.clist_index.eq(operand_field[6:16]),
            self.switch_target.eq(operand_field[16:19]),
            self.tperm_preset.eq(operand_field[0:4]),
            self.call_mask.eq(operand_field[6:16]),
            self.excl_result_dr.eq(operand_field[0:4]),
            self.reg_list.eq(Cat(operand_field[0:8], Const(0, 8))),
        ]

        m.d.comb += [
            self.turing_op.eq(opcode_field),
            self.dr_dst.eq(operand_field[18:22]),
            self.dr_src1.eq(operand_field[14:18]),
            self.dr_src2.eq(operand_field[10:14]),
            self.immediate.eq(operand_field[0:14]),
            self.ldi_immediate.eq(operand_field),
            self.branch_offset.eq(operand_field[0:18]),
        ]

        m.d.comb += [
            self.fault_valid.eq(0),
            self.fault.eq(FaultType.NONE),
        ]

        with m.If(self.instr_valid):
            with m.If(~self.is_church_op & ~self.is_turing_op):
                m.d.comb += [
                    self.fault_valid.eq(1),
                    self.fault.eq(FaultType.INVALID_OP),
                ]
            with m.Elif((opcode_field == ChurchOpcode.TPERM) &
                         ((operand_field[0:4] == TpermPreset.RSV1) |
                          (operand_field[0:4] == TpermPreset.RSV2))):
                m.d.comb += [
                    self.fault_valid.eq(1),
                    self.fault.eq(FaultType.TPERM_RSV),
                ]

        return m
