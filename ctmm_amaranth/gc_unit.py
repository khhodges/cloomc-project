from amaranth import *
from amaranth.lib.data import View

from .types import *
from .layouts import GT_LAYOUT


# PP250 Design: Mark-Scan-Sweep.
#   Mark:  Sets G=1 on all namespace entries.
#   Scan:  Relies on mLoad (in the LOAD/CALL paths) resetting G=0 on every
#          valid access. No explicit scan state needed in hardware — the
#          normal execution path between Mark and Sweep IS the scan phase.
#   Sweep: Identifies entries still with G=1 as garbage.
# TODO: Sweep currently only counts garbage entries. To fully reclaim,
#       add a SWEEP_WRITE state to bump version and clear G on garbage.
class CTMMGCUnit(Elaboratable):
    def __init__(self):
        self.gc_start = Signal()
        self.gc_mark_en = Signal()
        self.gc_sweep_en = Signal()
        self.gc_busy = Signal()
        self.gc_done = Signal()

        self.ns_addr = Signal(32)
        self.ns_rd_en = Signal()
        self.ns_rd_data = Signal(GT_LAYOUT)
        self.ns_wr_data = Signal(GT_LAYOUT)
        self.ns_wr_en = Signal()

        self.ns_start_addr = Signal(32)
        self.ns_end_addr = Signal(32)

        self.marked_count = Signal(32)
        self.garbage_count = Signal(32)

        self.access_addr = Signal(32)
        self.valid_key_access = Signal()
        self.is_namespace_access = Signal()
        self.g_bit_reset = Signal()

    def elaborate(self, platform):
        m = Module()

        current_addr = Signal(32)
        mark_counter = Signal(32)
        garbage_counter = Signal(32)

        rd_view = View(GT_LAYOUT, self.ns_rd_data)

        with m.FSM(name="gc") as fsm:
            with m.State("IDLE"):
                with m.If(self.gc_start & self.gc_mark_en):
                    m.d.sync += [
                        current_addr.eq(self.ns_start_addr),
                        mark_counter.eq(0),
                        garbage_counter.eq(0),
                    ]
                    m.next = "MARK_READ"
                with m.Elif(self.gc_start & self.gc_sweep_en):
                    m.d.sync += [
                        current_addr.eq(self.ns_start_addr),
                        mark_counter.eq(0),
                        garbage_counter.eq(0),
                    ]
                    m.next = "SWEEP_READ"

            with m.State("MARK_READ"):
                m.d.comb += [
                    self.ns_addr.eq(current_addr),
                    self.ns_rd_en.eq(1),
                ]
                m.next = "MARK_WRITE"

            with m.State("MARK_WRITE"):
                wr_view = View(GT_LAYOUT, self.ns_wr_data)
                m.d.comb += [
                    self.ns_wr_data.eq(self.ns_rd_data),
                    self.ns_wr_en.eq(1),
                ]
                m.d.comb += wr_view.g_bit.eq(1)

                with m.If(~rd_view.g_bit):
                    m.d.sync += mark_counter.eq(mark_counter + 1)

                m.d.sync += current_addr.eq(current_addr + 1)

                with m.If(current_addr >= self.ns_end_addr):
                    with m.If(self.gc_sweep_en):
                        m.d.sync += current_addr.eq(self.ns_start_addr)
                        m.next = "SWEEP_READ"
                    with m.Else():
                        m.next = "COMPLETE"
                with m.Else():
                    m.next = "MARK_READ"

            with m.State("SWEEP_READ"):
                m.d.comb += [
                    self.ns_addr.eq(current_addr),
                    self.ns_rd_en.eq(1),
                ]
                m.next = "SWEEP_CHECK"

            with m.State("SWEEP_CHECK"):
                with m.If(rd_view.g_bit):
                    m.d.sync += garbage_counter.eq(garbage_counter + 1)

                m.d.sync += current_addr.eq(current_addr + 1)

                with m.If(current_addr >= self.ns_end_addr):
                    m.next = "COMPLETE"
                with m.Else():
                    m.next = "SWEEP_READ"

            with m.State("COMPLETE"):
                m.next = "IDLE"

        m.d.comb += [
            self.gc_busy.eq(~fsm.ongoing("IDLE") & ~fsm.ongoing("COMPLETE")),
            self.gc_done.eq(fsm.ongoing("COMPLETE")),
            self.marked_count.eq(mark_counter),
            self.garbage_count.eq(garbage_counter),
        ]

        m.d.comb += self.g_bit_reset.eq(self.valid_key_access & self.is_namespace_access)

        return m
