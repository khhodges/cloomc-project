from amaranth import *
from amaranth.lib.data import View

from .types import *
from .layouts import GT_LAYOUT, CAP_REG_LAYOUT


class CTMMMLoad(Elaboratable):
    def __init__(self):
        self.sub_start = Signal()
        self.sub_cr_src = Signal(4)
        self.sub_cr_dst = Signal(4)
        self.sub_index = Signal(10)
        self.sub_direct = Signal()        # Direct GT mode: skip C-List fetch
        self.sub_direct_gt = Signal(64)   # GT value for direct validation (RETURN)
        self.sub_busy = Signal()
        self.sub_done = Signal()
        self.sub_fault = Signal()
        self.sub_fault_type = Signal(4)

        self.cr_rd_addr = Signal(4)
        self.cr_rd_data = Signal(CAP_REG_LAYOUT)

        self.cr_wr_addr = Signal(4)
        self.cr_wr_data = Signal(CAP_REG_LAYOUT)
        self.cr_wr_en = Signal()

        self.cr15_namespace = Signal(CAP_REG_LAYOUT)

        self.mem_addr = Signal(64)
        self.mem_rd_en = Signal()
        self.mem_rd_data = Signal(64)
        self.mem_rd_valid = Signal()

        self.thread_wr_en = Signal()
        self.thread_wr_idx = Signal(3)
        self.thread_wr_data = Signal(64)

        self.g_bit_reset = Signal()
        self.g_bit_addr = Signal(64)

    def elaborate(self, platform):
        m = Module()

        cr_src_reg = Signal(4)
        cr_dst_reg = Signal(4)
        index_reg = Signal(10)
        direct_mode = Signal()
        direct_gt_reg = Signal(64)
        src_cap = Signal(CAP_REG_LAYOUT)
        result_cap = Signal(CAP_REG_LAYOUT)
        fault_type_reg = Signal(4)

        src_view = View(CAP_REG_LAYOUT, src_cap)
        result_view = View(CAP_REG_LAYOUT, result_cap)
        ns_view = View(CAP_REG_LAYOUT, self.cr15_namespace)

        src_gt = View(GT_LAYOUT, src_view.word0_gt)
        result_gt = View(GT_LAYOUT, result_view.word0_gt)
        ns_gt = View(GT_LAYOUT, ns_view.word0_gt)

        has_l_perm = src_gt.perms[PERM_L]
        has_load_perm = has_l_perm
        src_is_null = Signal()
        m.d.comb += src_is_null.eq(src_view.word0_gt.as_value() == 0)
        bounds_ok = Signal()
        m.d.comb += bounds_ok.eq(Cat(index_reg, Const(0, 54)) < src_view.word2_limit)

        cr15_has_l = ns_gt.perms[PERM_L]
        gt_offset_in_bounds = Signal()
        m.d.comb += gt_offset_in_bounds.eq(Cat(result_gt.offset, Const(0, 32)) < ns_view.word2_limit)
        step4_ok = cr15_has_l & gt_offset_in_bounds

        gt_has_g_bit = result_gt.g_bit

        clist_gt_addr = Signal(64)
        m.d.comb += clist_gt_addr.eq(src_view.word1_location + (Cat(index_reg, Const(0, 54)) << 3))

        ns_entry_addr = Signal(64)
        m.d.comb += ns_entry_addr.eq(ns_view.word1_location + Cat(result_gt.offset, Const(0, 32)))

        with m.FSM(name="mload") as fsm:
            with m.State("IDLE"):
                with m.If(self.sub_start):
                    m.d.sync += [
                        cr_src_reg.eq(self.sub_cr_src),
                        cr_dst_reg.eq(self.sub_cr_dst),
                        index_reg.eq(self.sub_index),
                        direct_mode.eq(self.sub_direct),
                        direct_gt_reg.eq(self.sub_direct_gt),
                        result_cap.eq(0),
                        fault_type_reg.eq(FaultType.NONE),
                    ]
                    m.next = "FETCH_SRC"

            with m.State("FETCH_SRC"):
                with m.If(direct_mode):
                    m.d.sync += result_view.word0_gt.eq(direct_gt_reg)
                    m.next = "CHECK_NS"
                with m.Else():
                    m.d.comb += self.cr_rd_addr.eq(cr_src_reg)
                    m.d.sync += src_cap.eq(self.cr_rd_data)
                    m.next = "CHECK_L"

            with m.State("CHECK_L"):
                with m.If(src_is_null):
                    m.d.sync += fault_type_reg.eq(FaultType.NULL_CAP)
                    m.next = "FAULT"
                with m.Elif(~has_load_perm):
                    m.d.sync += fault_type_reg.eq(FaultType.PERM_L)
                    m.next = "FAULT"
                with m.Else():
                    m.next = "CHECK_BOUNDS"

            with m.State("CHECK_BOUNDS"):
                with m.If(~bounds_ok):
                    m.d.sync += fault_type_reg.eq(FaultType.BOUNDS)
                    m.next = "FAULT"
                with m.Else():
                    m.next = "FETCH_W0"

            with m.State("FETCH_W0"):
                m.d.comb += [
                    self.mem_addr.eq(clist_gt_addr),
                    self.mem_rd_en.eq(1),
                ]
                with m.If(self.mem_rd_valid):
                    m.d.sync += result_view.word0_gt.eq(self.mem_rd_data)
                    m.next = "CHECK_NS"

            with m.State("CHECK_NS"):
                with m.If(~step4_ok):
                    m.d.sync += fault_type_reg.eq(FaultType.BOUNDS)
                    m.next = "FAULT"
                with m.Else():
                    m.next = "FETCH_W1"

            with m.State("FETCH_W1"):
                m.d.comb += [
                    self.mem_addr.eq(ns_entry_addr),
                    self.mem_rd_en.eq(1),
                ]
                with m.If(self.mem_rd_valid):
                    m.d.sync += result_view.word1_location.eq(self.mem_rd_data)
                    m.next = "FETCH_W2"

            with m.State("FETCH_W2"):
                m.d.comb += [
                    self.mem_addr.eq(ns_entry_addr + 8),
                    self.mem_rd_en.eq(1),
                ]
                with m.If(self.mem_rd_valid):
                    m.d.sync += result_view.word2_limit.eq(self.mem_rd_data)
                    m.next = "FETCH_W3"

            with m.State("FETCH_W3"):
                m.d.comb += [
                    self.mem_addr.eq(ns_entry_addr + 16),
                    self.mem_rd_en.eq(1),
                ]
                with m.If(self.mem_rd_valid):
                    m.d.sync += result_view.word3_seals.eq(self.mem_rd_data)
                    m.next = "CHECK_MAC"

            with m.State("CHECK_MAC"):
                with m.If(gt_has_g_bit):
                    m.next = "RESET_G"
                with m.Else():
                    m.next = "UPDATE_THREAD"

            with m.State("RESET_G"):
                m.d.comb += [
                    self.g_bit_reset.eq(1),
                    self.g_bit_addr.eq(ns_entry_addr + 16),
                ]
                m.next = "UPDATE_THREAD"

            with m.State("UPDATE_THREAD"):
                gt_g_cleared = Signal(64)
                m.d.comb += [
                    gt_g_cleared.eq(result_view.word0_gt),
                ]
                gt_g_view = View(GT_LAYOUT, gt_g_cleared)
                m.d.comb += gt_g_view.g_bit.eq(0)
                with m.If(cr_dst_reg <= 7):
                    m.d.comb += [
                        self.thread_wr_en.eq(1),
                        self.thread_wr_idx.eq(cr_dst_reg),
                        self.thread_wr_data.eq(gt_g_cleared),
                    ]
                m.next = "COMPLETE"

            with m.State("COMPLETE"):
                wr_data = Signal(CAP_REG_LAYOUT)
                m.d.comb += wr_data.eq(result_cap)
                wr_view = View(CAP_REG_LAYOUT, wr_data)
                wr_gt = View(GT_LAYOUT, wr_view.word0_gt)
                m.d.comb += wr_gt.g_bit.eq(0)

                m.d.comb += [
                    self.cr_wr_addr.eq(cr_dst_reg),
                    self.cr_wr_data.eq(wr_data),
                    self.cr_wr_en.eq(1),
                ]
                m.next = "IDLE"

            with m.State("FAULT"):
                m.next = "IDLE"

        m.d.comb += [
            self.sub_busy.eq(~fsm.ongoing("IDLE")),
            self.sub_done.eq(fsm.ongoing("COMPLETE")),
            self.sub_fault.eq(fsm.ongoing("FAULT")),
            self.sub_fault_type.eq(fault_type_reg),
        ]

        return m
