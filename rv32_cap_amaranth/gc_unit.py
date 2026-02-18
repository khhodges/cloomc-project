from amaranth import *
from amaranth.lib.data import View

from .types import *
from .layouts import GT_LAYOUT, SEALS_LAYOUT


class RV32CapGCUnit(Elaboratable):
    def __init__(self):
        self.gc_start = Signal()
        self.gc_sweep_en = Signal()
        self.gc_busy = Signal()
        self.gc_done = Signal()

        self.clist_start_index = Signal(17)
        self.clist_end_index = Signal(17)

        self.clist_base_addr = Signal(32)
        self.ns_base_addr = Signal(32)

        self.marked_count = Signal(32)
        self.garbage_count = Signal(32)

        self.mem_addr = Signal(32)
        self.mem_rd_en = Signal()
        self.mem_rd_data = Signal(32)
        self.mem_rd_valid = Signal()
        self.mem_wr_data = Signal(32)
        self.mem_wr_en = Signal()

    def elaborate(self, platform):
        m = Module()

        current_index = Signal(17)
        mark_counter = Signal(32)
        garbage_counter = Signal(32)

        latched_gt = Signal(GT_LAYOUT)
        latched_seals = Signal(32)

        gt_view = View(GT_LAYOUT, latched_gt)
        seals_view = View(SEALS_LAYOUT, latched_seals)

        is_null = Signal()
        m.d.comb += is_null.eq(gt_view.gt_type == GT_TYPE_NULL)

        version_mismatch = Signal()
        m.d.comb += version_mismatch.eq(gt_view.version != seals_view.version)

        clist_gt_addr = Signal(32)
        m.d.comb += clist_gt_addr.eq(self.clist_base_addr + (current_index << 2))

        ns_seals_addr = Signal(32)
        m.d.comb += ns_seals_addr.eq(self.ns_base_addr + (gt_view.index * 12) + 8)

        next_version = Signal(7)
        m.d.comb += next_version.eq(gt_view.version + 1)

        with m.FSM(name="gc") as fsm:
            with m.State("IDLE"):
                with m.If(self.gc_start & self.gc_sweep_en):
                    m.d.sync += [
                        current_index.eq(self.clist_start_index),
                        mark_counter.eq(0),
                        garbage_counter.eq(0),
                    ]
                    m.next = "SWEEP_FETCH_GT"

            with m.State("SWEEP_FETCH_GT"):
                m.d.comb += [
                    self.mem_addr.eq(clist_gt_addr),
                    self.mem_rd_en.eq(1),
                ]
                with m.If(self.mem_rd_valid):
                    m.d.sync += latched_gt.eq(self.mem_rd_data)
                    m.next = "SWEEP_CHECK_NULL"

            with m.State("SWEEP_CHECK_NULL"):
                with m.If(is_null):
                    m.d.sync += current_index.eq(current_index + 1)
                    with m.If(current_index >= self.clist_end_index):
                        m.next = "COMPLETE"
                    with m.Else():
                        m.next = "SWEEP_FETCH_GT"
                with m.Else():
                    m.next = "SWEEP_FETCH_SEALS"

            with m.State("SWEEP_FETCH_SEALS"):
                m.d.comb += [
                    self.mem_addr.eq(ns_seals_addr),
                    self.mem_rd_en.eq(1),
                ]
                with m.If(self.mem_rd_valid):
                    m.d.sync += latched_seals.eq(self.mem_rd_data)
                    m.next = "SWEEP_CHECK"

            with m.State("SWEEP_CHECK"):
                m.d.sync += mark_counter.eq(mark_counter + 1)
                with m.If(version_mismatch):
                    m.d.sync += garbage_counter.eq(garbage_counter + 1)
                    m.next = "SWEEP_NULLIFY_GT"
                with m.Else():
                    m.d.sync += current_index.eq(current_index + 1)
                    with m.If(current_index >= self.clist_end_index):
                        m.next = "COMPLETE"
                    with m.Else():
                        m.next = "SWEEP_FETCH_GT"

            with m.State("SWEEP_NULLIFY_GT"):
                new_gt = Signal(GT_LAYOUT)
                new_gt_view = View(GT_LAYOUT, new_gt)
                m.d.comb += [
                    new_gt_view.gt_type.eq(GT_TYPE_NULL),
                    new_gt_view.perms.eq(0),
                    new_gt_view.index.eq(gt_view.index),
                    new_gt_view.version.eq(next_version),
                ]
                m.d.comb += [
                    self.mem_addr.eq(clist_gt_addr),
                    self.mem_wr_data.eq(new_gt),
                    self.mem_wr_en.eq(1),
                ]
                m.next = "SWEEP_UPDATE_SEALS"

            with m.State("SWEEP_UPDATE_SEALS"):
                new_seals = Signal(32)
                new_seals_view = View(SEALS_LAYOUT, new_seals)
                m.d.comb += [
                    new_seals_view.version.eq(next_version),
                    new_seals_view.seal.eq(0),
                ]
                m.d.comb += [
                    self.mem_addr.eq(ns_seals_addr),
                    self.mem_wr_data.eq(new_seals),
                    self.mem_wr_en.eq(1),
                ]
                m.d.sync += current_index.eq(current_index + 1)
                with m.If(current_index >= self.clist_end_index):
                    m.next = "COMPLETE"
                with m.Else():
                    m.next = "SWEEP_FETCH_GT"

            with m.State("COMPLETE"):
                m.next = "IDLE"

        m.d.comb += [
            self.gc_busy.eq(~fsm.ongoing("IDLE") & ~fsm.ongoing("COMPLETE")),
            self.gc_done.eq(fsm.ongoing("COMPLETE")),
            self.marked_count.eq(mark_counter),
            self.garbage_count.eq(garbage_counter),
        ]

        return m
