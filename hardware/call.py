from amaranth import *
from amaranth.lib.data import View

from .hw_types import *
from .layouts import GT_LAYOUT, CAP_REG_LAYOUT, WORD2_LAYOUT, LUMP_HEADER_LAYOUT


class ChurchCall(Elaboratable):
    def __init__(self):
        self.call_start = Signal()
        self.cr_src = Signal(4)
        self.index = Signal(16)
        self.mask = Signal(16)
        self.call_busy = Signal()
        self.call_complete = Signal()
        self.call_fault = Signal()
        self.fault_type = Signal(4)

        self.cr_rd_addr = Signal(4)
        self.cr_rd_data = Signal(CAP_REG_LAYOUT)
        self.cr_wr_addr = Signal(4)
        self.cr_wr_data = Signal(CAP_REG_LAYOUT)
        self.cr_wr_en = Signal()

        self.mload_start = Signal()
        self.mload_cr_src = Signal(4)
        self.mload_cr_dst = Signal(4)
        self.mload_index = Signal(16)
        self.mload_direct = Signal()
        self.mload_direct_gt = Signal(32)
        self.mload_m_elevated = Signal()

        self.mload_done = Signal()
        self.mload_fault = Signal()
        self.mload_fault_type = Signal(4)

        self.saved_cr5_gt = Signal(32)

        self.nia_set = Signal()
        self.nia_value = Signal(32)
        self.dr_clear_mask = Signal(16)
        self.cr_clear_mask = Signal(16)

        # Direct memory read for NS lump header fetch (post Phase 2)
        self.mem_rd_addr = Signal(32)
        self.mem_rd_en = Signal()
        self.mem_rd_data = Signal(32)
        self.mem_rd_valid = Signal()

        # CR15 namespace for computing NS entry address of callee
        self.cr15_namespace = Signal(CAP_REG_LAYOUT)

        # CR14 code cap for NIA base (populated after Phase 2 mload)
        self.cr14_code = Signal(CAP_REG_LAYOUT)

    def elaborate(self, platform):
        m = Module()

        CR6_CLIST = 6
        CR14_CODE = 14
        MAX_SRC_REG = 5
        B_BIT_POS = 31
        LIMIT_WIDTH = 32

        phase = Signal()
        src_reg_latched = Signal(CAP_REG_LAYOUT)
        mask_latched = Signal(16)
        fault_latched = Signal()
        fault_type_latched = Signal(4)
        sub_start_reg = Signal()
        sub_done_latched = Signal()
        sub_fault_latched = Signal()

        local_cr_rd_addr = Signal(4)

        b_idx = Signal(3)
        b_cr_data = Signal(CAP_REG_LAYOUT)
        b_clear_wr_en = Signal()
        b_clear_wr_addr = Signal(4)
        b_clear_wr_data = Signal(CAP_REG_LAYOUT)

        src_in_range = Signal()
        m.d.comb += src_in_range.eq(self.cr_src <= MAX_SRC_REG)

        src_view = View(CAP_REG_LAYOUT, src_reg_latched)
        src_gt = View(GT_LAYOUT, src_view.word0_gt)
        src_has_e_perm = src_gt.perms[PERM_E]

        mload_src = Signal(4)
        mload_dst = Signal(4)
        mload_index = Signal(16)
        m.d.comb += [
            mload_src.eq(Mux(phase, CR6_CLIST, self.cr_src)),
            mload_dst.eq(Mux(phase, CR14_CODE, CR6_CLIST)),
            mload_index.eq(Mux(phase, 0, self.index)),
        ]

        m.d.comb += [
            self.mload_start.eq(sub_start_reg),
            self.mload_cr_src.eq(mload_src),
            self.mload_cr_dst.eq(mload_dst),
            self.mload_index.eq(mload_index),
            self.mload_direct.eq(0),
            self.mload_m_elevated.eq(1),
            self.mload_direct_gt.eq(0),
        ]

        m.d.comb += [
            self.cr_wr_addr.eq(b_clear_wr_addr),
            self.cr_wr_data.eq(b_clear_wr_data),
            self.cr_wr_en.eq(b_clear_wr_en),
        ]

        m.d.comb += self.cr_rd_addr.eq(local_cr_rd_addr)

        cr_preserve = mask_latched[5:11]
        dr1_5_preserve = mask_latched[0:5]

        dr_clear_computed = Signal(16)
        cr_clear_computed = Signal(16)
        m.d.comb += [
            dr_clear_computed.eq(Cat(Const(0, 1), ~dr1_5_preserve, Const(0x3FF, 10))),
            cr_clear_computed.eq(Cat(~cr_preserve, Const(0, 10))),
        ]

        # NS lump header fetch state
        cr14_view = View(CAP_REG_LAYOUT, self.cr14_code)
        cr14_gt = View(GT_LAYOUT, cr14_view.word0_gt)
        cr15_view = View(CAP_REG_LAYOUT, self.cr15_namespace)

        callee_ns_entry_addr = Signal(32)
        m.d.comb += callee_ns_entry_addr.eq(
            cr15_view.word1_location + (cr14_gt.slot_id << 4)
        )

        lump_reg = Signal(32)
        lump_view = View(LUMP_HEADER_LAYOUT, lump_reg)

        # Latched fields from LUMP_HEADER (set after FETCH_LUMP completes)
        mw_reg        = Signal(6)   # max-word (arg count)
        cc_reg        = Signal(8)   # calling-convention flags
        n_minus_6_reg = Signal(4)   # total frame words minus 6

        nia_computed = Signal(32)
        m.d.comb += nia_computed.eq(
            cr14_view.word1_location + ((1 + lump_view.mw) << 2)
        )

        with m.FSM(name="call") as fsm:
            with m.State("IDLE"):
                m.d.sync += [phase.eq(0), fault_latched.eq(0), fault_type_latched.eq(FaultType.NONE)]
                m.d.sync += [sub_done_latched.eq(0), sub_fault_latched.eq(0)]
                with m.If(self.call_start):
                    m.d.sync += mask_latched.eq(self.mask)
                    m.next = "CHECK_SRC"

            with m.State("CHECK_SRC"):
                m.d.comb += local_cr_rd_addr.eq(self.cr_src)
                with m.If(~src_in_range):
                    m.d.sync += [fault_latched.eq(1), fault_type_latched.eq(FaultType.PERM_E)]
                    m.next = "FAULT"
                with m.Else():
                    m.next = "READ_SRC"

            with m.State("READ_SRC"):
                m.d.comb += local_cr_rd_addr.eq(self.cr_src)
                m.d.sync += src_reg_latched.eq(self.cr_rd_data)
                m.next = "CHECK_PERM"

            with m.State("CHECK_PERM"):
                with m.If(~src_has_e_perm):
                    m.d.sync += [fault_latched.eq(1), fault_type_latched.eq(FaultType.PERM_E)]
                    m.next = "FAULT"
                with m.Else():
                    m.d.comb += local_cr_rd_addr.eq(5)
                    m.next = "READ_CR5"

            with m.State("READ_CR5"):
                m.d.comb += local_cr_rd_addr.eq(5)
                cr5_view = View(CAP_REG_LAYOUT, self.cr_rd_data)
                m.d.sync += self.saved_cr5_gt.eq(cr5_view.word0_gt)
                m.d.sync += sub_start_reg.eq(1)
                m.next = "PHASE1"

            with m.State("PHASE1"):
                m.d.sync += sub_start_reg.eq(0)
                with m.If(self.mload_done):
                    m.d.sync += sub_done_latched.eq(1)
                with m.If(self.mload_fault):
                    m.d.sync += sub_fault_latched.eq(1)
                    m.d.sync += [fault_latched.eq(1), fault_type_latched.eq(self.mload_fault_type)]
                with m.If(sub_fault_latched):
                    m.next = "FAULT"
                with m.Elif(sub_done_latched):
                    m.next = "PHASE1_DONE"

            with m.State("PHASE1_DONE"):
                m.d.sync += [phase.eq(1), sub_done_latched.eq(0), sub_fault_latched.eq(0)]
                m.d.sync += sub_start_reg.eq(1)
                m.next = "PHASE2"

            with m.State("PHASE2"):
                m.d.sync += sub_start_reg.eq(0)
                with m.If(self.mload_done):
                    m.d.sync += sub_done_latched.eq(1)
                with m.If(self.mload_fault):
                    m.d.sync += sub_fault_latched.eq(1)
                    m.d.sync += [fault_latched.eq(1), fault_type_latched.eq(self.mload_fault_type)]
                with m.If(sub_fault_latched):
                    m.next = "FAULT"
                with m.Elif(sub_done_latched):
                    m.next = "FETCH_LUMP"

            with m.State("FETCH_LUMP"):
                # Fetch NS word3_lump (+12) for callee; extract mw/cc/n_minus_6 for NIA
                m.d.comb += [
                    self.mem_rd_addr.eq(callee_ns_entry_addr + 12),
                    self.mem_rd_en.eq(1),
                ]
                with m.If(self.mem_rd_valid):
                    m.d.sync += lump_reg.eq(self.mem_rd_data)
                    m.d.sync += [
                        mw_reg.eq(View(LUMP_HEADER_LAYOUT, self.mem_rd_data).mw),
                        cc_reg.eq(View(LUMP_HEADER_LAYOUT, self.mem_rd_data).cc),
                        n_minus_6_reg.eq(View(LUMP_HEADER_LAYOUT, self.mem_rd_data).n_minus_6),
                    ]
                    m.next = "CLEAR_B_INIT"

            with m.State("CLEAR_B_INIT"):
                m.d.sync += b_idx.eq(0)
                m.next = "CLEAR_B_CHECK"

            with m.State("CLEAR_B_CHECK"):
                with m.If(b_idx > 5):
                    m.next = "COMPLETE"
                with m.Elif(cr_preserve.bit_select(b_idx, 1)):
                    m.d.comb += local_cr_rd_addr.eq(b_idx)
                    m.next = "CLEAR_B_READ"
                with m.Else():
                    m.d.sync += b_idx.eq(b_idx + 1)

            with m.State("CLEAR_B_READ"):
                m.d.comb += local_cr_rd_addr.eq(b_idx)
                m.d.sync += b_cr_data.eq(self.cr_rd_data)
                m.next = "CLEAR_B_WRITE"

            with m.State("CLEAR_B_WRITE"):
                b_src = View(CAP_REG_LAYOUT, b_cr_data)
                b_dst = View(CAP_REG_LAYOUT, b_clear_wr_data)
                m.d.comb += [
                    b_dst.word0_gt.slot_id.eq(b_src.word0_gt.slot_id),
                    b_dst.word0_gt.gt_seq.eq(b_src.word0_gt.gt_seq),
                    b_dst.word0_gt.gt_type.eq(b_src.word0_gt.gt_type),
                    b_dst.word0_gt.perms.eq(b_src.word0_gt.perms),
                    b_dst.word0_gt.b_flag.eq(0),
                    b_dst.word1_location.eq(b_src.word1_location),
                    b_dst.word2_w2.eq(b_src.word2_w2),
                    b_dst.word3_w3.eq(b_src.word3_w3),
                    b_clear_wr_en.eq(1),
                    b_clear_wr_addr.eq(b_idx),
                ]
                m.d.sync += b_idx.eq(b_idx + 1)
                m.next = "CLEAR_B_CHECK"

            with m.State("COMPLETE"):
                m.next = "IDLE"

            with m.State("FAULT"):
                m.next = "IDLE"

        m.d.comb += [
            self.call_busy.eq(~fsm.ongoing("IDLE")),
            self.call_complete.eq(fsm.ongoing("COMPLETE")),
            self.call_fault.eq(fault_latched),
            self.fault_type.eq(fault_type_latched),
            self.nia_set.eq(fsm.ongoing("COMPLETE")),
            self.nia_value.eq(nia_computed),
            self.dr_clear_mask.eq(Mux(fsm.ongoing("COMPLETE"), dr_clear_computed, 0)),
            self.cr_clear_mask.eq(Mux(fsm.ongoing("COMPLETE"), cr_clear_computed, 0)),
        ]

        return m
