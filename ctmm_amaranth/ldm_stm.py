from amaranth import *
from amaranth.lib.data import View

from .types import *
from .layouts import GT_LAYOUT, CAP_REG_LAYOUT


class CTMMLdmStm(Elaboratable):
    def __init__(self):
        self.ldm_start = Signal()
        self.stm_start = Signal()
        self.cr_base = Signal(3)
        self.reg_list = Signal(16)
        self.busy = Signal()
        self.complete = Signal()
        self.fault_valid = Signal()
        self.fault_type = Signal(4)

        self.cr_rd_addr = Signal(4)
        self.cr_rd_data = Signal(CAP_REG_LAYOUT)
        self.cr_wr_addr = Signal(4)
        self.cr_wr_data = Signal(CAP_REG_LAYOUT)
        self.cr_wr_en = Signal()
        self.cr15_namespace = Signal(CAP_REG_LAYOUT)

        self.mload_start = Signal()
        self.mload_src = Signal(4)
        self.mload_dst = Signal(4)
        self.mload_index = Signal(10)
        self.mload_busy = Signal()
        self.mload_done = Signal()
        self.mload_fault = Signal()
        self.mload_fault_type = Signal(4)

        self.msave_start = Signal()
        self.msave_dst = Signal(4)
        self.msave_gt = Signal(64)
        self.msave_index = Signal(10)
        self.msave_busy = Signal()
        self.msave_done = Signal()
        self.msave_fault = Signal()
        self.msave_fault_type = Signal(4)

    def elaborate(self, platform):
        m = Module()

        is_load = Signal()
        reg_list_remaining = Signal(16)
        current_reg = Signal(4)
        current_index = Signal(10)
        base_reg_latched = Signal(CAP_REG_LAYOUT)
        store_gt = Signal(64)
        fault_flag = Signal()
        fault_latched_type = Signal(4)

        next_reg = Signal(4)
        has_more_regs = Signal()

        m.d.comb += has_more_regs.eq(reg_list_remaining.any())
        for i in range(16):
            with m.If(reg_list_remaining[i] & ~has_more_regs):
                m.d.comb += next_reg.eq(i)
                m.d.comb += has_more_regs.eq(1)

        prio_found = Signal()
        m.d.comb += [prio_found.eq(0), next_reg.eq(0)]
        for i in range(16):
            with m.If(reg_list_remaining[i] & ~prio_found):
                m.d.comb += [next_reg.eq(i), prio_found.eq(1)]
        m.d.comb += has_more_regs.eq(prio_found)

        rd_view = View(CAP_REG_LAYOUT, self.cr_rd_data)

        with m.FSM(name="ldm_stm") as fsm:
            with m.State("IDLE"):
                m.d.sync += [fault_flag.eq(0), fault_latched_type.eq(FaultType.NONE)]
                with m.If(self.ldm_start | self.stm_start):
                    m.d.sync += [
                        is_load.eq(self.ldm_start),
                        reg_list_remaining.eq(self.reg_list),
                        current_index.eq(0),
                    ]
                    m.d.comb += self.cr_rd_addr.eq(Cat(self.cr_base, Const(0, 1)))
                    m.next = "READ_BASE"

            with m.State("READ_BASE"):
                m.d.comb += self.cr_rd_addr.eq(Cat(self.cr_base, Const(0, 1)))
                m.d.sync += base_reg_latched.eq(self.cr_rd_data)
                with m.If(has_more_regs):
                    m.next = "PROCESS_REG"
                with m.Else():
                    m.next = "COMPLETE"

            with m.State("PROCESS_REG"):
                m.d.sync += [
                    current_reg.eq(next_reg),
                    reg_list_remaining.bit_select(next_reg, 1).eq(0),
                ]
                with m.If(is_load):
                    m.next = "START_MLOAD"
                with m.Else():
                    m.d.comb += self.cr_rd_addr.eq(next_reg)
                    m.next = "START_MSAVE"

            with m.State("START_MLOAD"):
                m.d.comb += [
                    self.mload_start.eq(1),
                    self.mload_src.eq(Cat(self.cr_base, Const(0, 1))),
                    self.mload_dst.eq(current_reg),
                    self.mload_index.eq(current_index),
                ]
                m.next = "WAIT_MLOAD"

            with m.State("WAIT_MLOAD"):
                m.d.comb += [
                    self.mload_src.eq(Cat(self.cr_base, Const(0, 1))),
                    self.mload_dst.eq(current_reg),
                    self.mload_index.eq(current_index),
                ]
                with m.If(self.mload_fault):
                    m.d.sync += [fault_flag.eq(1), fault_latched_type.eq(self.mload_fault_type)]
                    m.next = "FAULT"
                with m.Elif(self.mload_done):
                    m.next = "NEXT_REG"

            with m.State("START_MSAVE"):
                m.d.comb += self.cr_rd_addr.eq(current_reg)
                m.d.sync += store_gt.eq(rd_view.word0_gt)
                m.d.comb += [
                    self.msave_start.eq(1),
                    self.msave_dst.eq(Cat(self.cr_base, Const(0, 1))),
                    self.msave_gt.eq(rd_view.word0_gt),
                    self.msave_index.eq(current_index),
                ]
                m.next = "WAIT_MSAVE"

            with m.State("WAIT_MSAVE"):
                m.d.comb += [
                    self.msave_dst.eq(Cat(self.cr_base, Const(0, 1))),
                    self.msave_gt.eq(store_gt),
                    self.msave_index.eq(current_index),
                ]
                with m.If(self.msave_fault):
                    m.d.sync += [fault_flag.eq(1), fault_latched_type.eq(self.msave_fault_type)]
                    m.next = "FAULT"
                with m.Elif(self.msave_done):
                    m.next = "NEXT_REG"

            with m.State("NEXT_REG"):
                m.d.sync += current_index.eq(current_index + 1)
                with m.If(has_more_regs):
                    m.next = "PROCESS_REG"
                with m.Else():
                    m.next = "COMPLETE"

            with m.State("COMPLETE"):
                m.next = "IDLE"

            with m.State("FAULT"):
                m.next = "IDLE"

        m.d.comb += [
            self.busy.eq(~fsm.ongoing("IDLE")),
            self.complete.eq(fsm.ongoing("COMPLETE")),
            self.fault_valid.eq(fault_flag),
            self.fault_type.eq(fault_latched_type),
        ]

        return m
