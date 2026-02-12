from amaranth import *
from amaranth.lib.data import View

from .types import *
from .layouts import GT_LAYOUT, CAP_REG_LAYOUT
from .mload import CTMMMLoad
from .msave import CTMMMSave


class CTMMChange(Elaboratable):
    def __init__(self):
        self.change_start = Signal()
        self.cr_src = Signal(4)
        self.index = Signal(8)
        self.change_mask = Signal(16)
        self.change_busy = Signal()
        self.change_complete = Signal()
        self.change_fault = Signal()
        self.fault_type = Signal(4)

        self.cr_rd_addr = Signal(4)
        self.cr_rd_data = Signal(CAP_REG_LAYOUT)
        self.cr_wr_addr = Signal(4)
        self.cr_wr_data = Signal(CAP_REG_LAYOUT)
        self.cr_wr_en = Signal()

        self.cr8_thread = Signal(CAP_REG_LAYOUT)
        self.cr15_namespace = Signal(CAP_REG_LAYOUT)

        self.mem_rd_addr = Signal(64)
        self.mem_rd_en = Signal()
        self.mem_rd_data = Signal(64)
        self.mem_rd_valid = Signal()
        self.mem_wr_addr = Signal(64)
        self.mem_wr_data = Signal(64)
        self.mem_wr_en = Signal()
        self.mem_wr_done = Signal()

        self.thread_wr_en = Signal()
        self.thread_wr_idx = Signal(4)
        self.thread_wr_data = Signal(64)
        self.g_bit_reset = Signal()
        self.g_bit_addr = Signal(64)

    def elaborate(self, platform):
        m = Module()

        RESERVED_MASK = 0b1000_0001_1000_0000

        u_msave = CTMMMSave()
        u_mload = CTMMMLoad()
        m.submodules.u_msave = u_msave
        m.submodules.u_mload = u_mload

        cr_index = Signal(4)
        crn_reg_latched = Signal(CAP_REG_LAYOUT)
        current_cr_latched = Signal(CAP_REG_LAYOUT)
        index_latched = Signal(8)
        mask_latched = Signal(16)
        fault_latched = Signal()
        fault_type_latched = Signal(4)

        msave_start_reg = Signal()
        msave_done_latched = Signal()
        msave_fault_latched = Signal()
        mload_start_reg = Signal()
        mload_done_latched = Signal()
        mload_fault_latched = Signal()

        effective_mask = Signal(16)
        m.d.comb += effective_mask.eq(mask_latched & ~RESERVED_MASK)

        skip_current_cr = Signal()
        m.d.comb += skip_current_cr.eq((cr_index > 14) | ~effective_mask.bit_select(cr_index, 1))

        crn_view = View(CAP_REG_LAYOUT, crn_reg_latched)
        crn_gt = View(GT_LAYOUT, crn_view.word0_gt)
        crn_has_l_perm = crn_gt.perms[PERM_L]

        cur_view = View(CAP_REG_LAYOUT, current_cr_latched)

        m.d.comb += [
            u_msave.sub_start.eq(msave_start_reg),
            u_msave.sub_dst_cap.eq(self.cr8_thread),
            u_msave.sub_src_gt.eq(cur_view.word0_gt),
            u_msave.sub_index.eq(cr_index),
            u_msave.mem_wr_done.eq(self.mem_wr_done),
        ]

        mload_src = Signal(4)
        mload_dst = Signal(4)
        mload_index = Signal(10)

        m.d.comb += [
            u_mload.sub_start.eq(mload_start_reg),
            u_mload.sub_cr_src.eq(mload_src),
            u_mload.sub_cr_dst.eq(mload_dst),
            u_mload.sub_index.eq(mload_index),
            u_mload.sub_direct.eq(0),             # CHANGE uses C-List fetch mode
            u_mload.sub_direct_gt.eq(0),
            u_mload.cr_rd_data.eq(self.cr_rd_data),
            u_mload.cr15_namespace.eq(self.cr15_namespace),
            u_mload.mem_rd_data.eq(self.mem_rd_data),
            u_mload.mem_rd_valid.eq(self.mem_rd_valid),
        ]

        m.d.comb += [
            self.mem_wr_addr.eq(u_msave.mem_wr_addr),
            self.mem_wr_data.eq(u_msave.mem_wr_data),
            self.mem_wr_en.eq(u_msave.mem_wr_en),
            self.mem_rd_addr.eq(u_mload.mem_addr),
            self.mem_rd_en.eq(u_mload.mem_rd_en),
            self.cr_wr_addr.eq(u_mload.cr_wr_addr),
            self.cr_wr_data.eq(u_mload.cr_wr_data),
            self.cr_wr_en.eq(u_mload.cr_wr_en),
            self.thread_wr_en.eq(u_mload.thread_wr_en),
            self.thread_wr_idx.eq(u_mload.thread_wr_idx),
            self.thread_wr_data.eq(u_mload.thread_wr_data),
            self.g_bit_reset.eq(u_mload.g_bit_reset),
            self.g_bit_addr.eq(u_mload.g_bit_addr),
        ]

        with m.FSM(name="change") as fsm:
            with m.State("IDLE"):
                m.d.sync += [fault_latched.eq(0), fault_type_latched.eq(FaultType.NONE)]
                m.d.sync += [msave_done_latched.eq(0), msave_fault_latched.eq(0)]
                m.d.sync += [mload_done_latched.eq(0), mload_fault_latched.eq(0)]
                with m.If(self.change_start):
                    m.d.sync += [
                        index_latched.eq(self.index),
                        mask_latched.eq(self.change_mask),
                        cr_index.eq(0),
                    ]
                    m.d.comb += self.cr_rd_addr.eq(self.cr_src)
                    m.next = "READ_CRN"

            with m.State("READ_CRN"):
                m.d.comb += self.cr_rd_addr.eq(self.cr_src)
                m.next = "LATCH_CRN"

            with m.State("LATCH_CRN"):
                m.d.sync += crn_reg_latched.eq(self.cr_rd_data)
                m.d.comb += self.cr_rd_addr.eq(self.cr_src)
                with m.If(~crn_has_l_perm):
                    m.d.sync += [fault_latched.eq(1), fault_type_latched.eq(FaultType.PERM_L)]
                    m.next = "FAULT"
                with m.Else():
                    m.next = "SAVE_READ_CR"

            with m.State("SAVE_READ_CR"):
                m.d.comb += self.cr_rd_addr.eq(cr_index)
                with m.If(skip_current_cr):
                    m.d.sync += cr_index.eq(cr_index + 1)
                    with m.If(cr_index >= 14):
                        m.next = "LOAD_THREAD"
                with m.Else():
                    m.next = "SAVE_LATCH_CR"

            with m.State("SAVE_LATCH_CR"):
                m.d.sync += current_cr_latched.eq(self.cr_rd_data)
                m.d.comb += self.cr_rd_addr.eq(cr_index)
                m.d.sync += msave_start_reg.eq(1)
                m.next = "SAVE_CALL"

            with m.State("SAVE_CALL"):
                m.d.sync += msave_start_reg.eq(0)
                m.d.sync += [msave_done_latched.eq(0), msave_fault_latched.eq(0)]
                with m.If(u_msave.sub_done):
                    m.d.sync += msave_done_latched.eq(1)
                with m.If(u_msave.sub_fault):
                    m.d.sync += msave_fault_latched.eq(1)
                    m.d.sync += [fault_latched.eq(1), fault_type_latched.eq(u_msave.sub_fault_type)]
                with m.If(msave_fault_latched):
                    m.next = "FAULT"
                with m.Elif(msave_done_latched):
                    m.next = "SAVE_NEXT"

            with m.State("SAVE_NEXT"):
                m.d.sync += cr_index.eq(cr_index + 1)
                with m.If(cr_index >= 14):
                    m.next = "LOAD_THREAD"
                with m.Else():
                    m.next = "SAVE_READ_CR"

            with m.State("LOAD_THREAD"):
                m.d.comb += [
                    mload_src.eq(self.cr_src),
                    mload_dst.eq(8),
                    mload_index.eq(index_latched),
                ]
                m.d.sync += mload_start_reg.eq(1)
                m.d.sync += [mload_done_latched.eq(0), mload_fault_latched.eq(0)]
                with m.If(u_mload.sub_done):
                    m.d.sync += mload_done_latched.eq(1)
                with m.If(u_mload.sub_fault):
                    m.d.sync += mload_fault_latched.eq(1)
                    m.d.sync += [fault_latched.eq(1), fault_type_latched.eq(u_mload.sub_fault_type)]
                with m.If(mload_fault_latched):
                    m.next = "FAULT"
                with m.Elif(mload_done_latched):
                    m.d.sync += cr_index.eq(0)
                    m.next = "RESTORE_CALL"

            with m.State("RESTORE_CALL"):
                m.d.comb += [
                    mload_src.eq(8),
                    mload_dst.eq(cr_index),
                    mload_index.eq(cr_index),
                ]
                with m.If(skip_current_cr):
                    m.d.sync += cr_index.eq(cr_index + 1)
                    with m.If(cr_index >= 14):
                        m.next = "COMPLETE"
                with m.Else():
                    m.d.sync += [mload_done_latched.eq(0), mload_fault_latched.eq(0)]
                    m.d.sync += mload_start_reg.eq(1)
                    with m.If(u_mload.sub_done):
                        m.d.sync += mload_done_latched.eq(1)
                    with m.If(u_mload.sub_fault):
                        m.d.sync += mload_fault_latched.eq(1)
                        m.d.sync += [fault_latched.eq(1), fault_type_latched.eq(u_mload.sub_fault_type)]
                    with m.If(mload_fault_latched):
                        m.next = "FAULT"
                    with m.Elif(mload_done_latched):
                        m.next = "RESTORE_NEXT"

            with m.State("RESTORE_NEXT"):
                m.d.sync += cr_index.eq(cr_index + 1)
                with m.If(cr_index >= 14):
                    m.next = "COMPLETE"
                with m.Else():
                    m.next = "RESTORE_CALL"

            with m.State("COMPLETE"):
                m.next = "IDLE"

            with m.State("FAULT"):
                m.next = "IDLE"

        m.d.comb += [
            self.change_busy.eq(~fsm.ongoing("IDLE")),
            self.change_complete.eq(fsm.ongoing("COMPLETE")),
            self.change_fault.eq(fault_latched),
            self.fault_type.eq(fault_type_latched),
        ]

        return m
