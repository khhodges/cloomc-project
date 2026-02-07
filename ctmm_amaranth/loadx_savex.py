from amaranth import *
from amaranth.lib.data import View

from .types import *
from .layouts import GT_LAYOUT, CAP_REG_LAYOUT, EXCL_MONITOR_LAYOUT


class CTMMLoadxSavex(Elaboratable):
    def __init__(self):
        self.loadx_start = Signal()
        self.savex_start = Signal()
        self.cr_src = Signal(3)
        self.cr_base = Signal(3)
        self.cr_dst = Signal(3)
        self.offset = Signal(10)
        self.result_dr = Signal(4)
        self.thread_id = Signal(4)
        self.busy = Signal()
        self.complete = Signal()
        self.fault_valid = Signal()
        self.fault_type = Signal(4)

        self.cr_rd_addr = Signal(4)
        self.cr_rd_data = Signal(CAP_REG_LAYOUT)
        self.cr_wr_addr = Signal(4)
        self.cr_wr_data = Signal(CAP_REG_LAYOUT)
        self.cr_wr_en = Signal()

        self.dr_wr_addr = Signal(4)
        self.dr_wr_data = Signal(64)
        self.dr_wr_en = Signal()

        self.mem_addr = Signal(64)
        self.mem_rd_en = Signal()
        self.mem_rd_data = Signal(64)
        self.mem_rd_valid = Signal()
        self.mem_wr_data = Signal(64)
        self.mem_wr_en = Signal()

        self.ext_addr_match = Signal()
        self.ext_access_addr = Signal(32)

    def elaborate(self, platform):
        m = Module()

        NUM_THREADS = 16

        monitor_states = [Signal(2, name=f"mon_state_{i}") for i in range(NUM_THREADS)]
        monitor_addrs = [Signal(32, name=f"mon_addr_{i}") for i in range(NUM_THREADS)]

        base_reg_latched = Signal(CAP_REG_LAYOUT)
        fetched_cap = Signal(CAP_REG_LAYOUT)
        addr_latched = Signal(32)
        fault_flag = Signal()
        fault_latched = Signal(4)

        base_view = View(CAP_REG_LAYOUT, base_reg_latched)
        base_gt = View(GT_LAYOUT, base_view.word0_gt)
        fetched_view = View(CAP_REG_LAYOUT, fetched_cap)

        has_l_perm = base_gt.perms[PERM_L]
        has_s_perm = base_gt.perms[PERM_S]

        rd_data_view = View(CAP_REG_LAYOUT, self.cr_rd_data)
        target_addr = Signal(32)
        loc_val = Signal(64)
        m.d.comb += loc_val.eq(rd_data_view.word1_location)
        m.d.comb += target_addr.eq(loc_val[:32] + Cat(Const(0, 5), self.offset, Const(0, 17)))

        bounds_ok = Signal()
        limit_val = Signal(64)
        m.d.comb += limit_val.eq(base_view.word2_limit)
        m.d.comb += bounds_ok.eq(Cat(self.offset, Const(0, 20)) < limit_val[:32])

        current_mon_state = Array(monitor_states)[self.thread_id]
        current_mon_addr = Array(monitor_addrs)[self.thread_id]

        monitor_valid = Signal()
        m.d.comb += monitor_valid.eq(
            (current_mon_state == ExclMonitorState.ACTIVE) &
            (current_mon_addr == addr_latched)
        )

        for i in range(NUM_THREADS):
            with m.If(self.ext_addr_match):
                with m.If((monitor_states[i] == ExclMonitorState.ACTIVE) &
                          (monitor_addrs[i] == self.ext_access_addr)):
                    m.d.sync += monitor_states[i].eq(ExclMonitorState.CLEARED)

        with m.FSM(name="loadx_savex") as fsm:
            with m.State("IDLE"):
                m.d.sync += [fault_flag.eq(0), fault_latched.eq(FaultType.NONE)]
                with m.If(self.loadx_start):
                    m.d.comb += self.cr_rd_addr.eq(Cat(self.cr_base, Const(0, 1)))
                    m.next = "LOADX_READ_BASE"
                with m.Elif(self.savex_start):
                    m.d.comb += self.cr_rd_addr.eq(Cat(self.cr_base, Const(0, 1)))
                    m.next = "SAVEX_READ_BASE"

            with m.State("LOADX_READ_BASE"):
                m.d.comb += self.cr_rd_addr.eq(Cat(self.cr_base, Const(0, 1)))
                m.d.sync += [base_reg_latched.eq(self.cr_rd_data), addr_latched.eq(target_addr)]
                m.next = "LOADX_CHECK_PERM"

            with m.State("LOADX_CHECK_PERM"):
                with m.If(~has_l_perm):
                    m.d.sync += [fault_flag.eq(1), fault_latched.eq(FaultType.PERM_L)]
                    m.next = "FAULT"
                with m.Else():
                    m.next = "LOADX_CALC_ADDR"

            with m.State("LOADX_CALC_ADDR"):
                with m.If(~bounds_ok):
                    m.d.sync += [fault_flag.eq(1), fault_latched.eq(FaultType.BOUNDS)]
                    m.next = "FAULT"
                with m.Else():
                    m.next = "LOADX_FETCH_W0"

            with m.State("LOADX_FETCH_W0"):
                m.d.comb += [self.mem_addr.eq(Cat(addr_latched, Const(0, 32))), self.mem_rd_en.eq(1)]
                with m.If(self.mem_rd_valid):
                    m.d.sync += fetched_view.word0_gt.eq(self.mem_rd_data)
                    m.next = "LOADX_FETCH_W1"

            with m.State("LOADX_FETCH_W1"):
                m.d.comb += [self.mem_addr.eq(Cat(addr_latched, Const(0, 32)) + 8), self.mem_rd_en.eq(1)]
                with m.If(self.mem_rd_valid):
                    m.d.sync += fetched_view.word1_location.eq(self.mem_rd_data)
                    m.next = "LOADX_FETCH_W2"

            with m.State("LOADX_FETCH_W2"):
                m.d.comb += [self.mem_addr.eq(Cat(addr_latched, Const(0, 32)) + 16), self.mem_rd_en.eq(1)]
                with m.If(self.mem_rd_valid):
                    m.d.sync += fetched_view.word2_limit.eq(self.mem_rd_data)
                    m.next = "LOADX_FETCH_W3"

            with m.State("LOADX_FETCH_W3"):
                m.d.comb += [self.mem_addr.eq(Cat(addr_latched, Const(0, 32)) + 24), self.mem_rd_en.eq(1)]
                with m.If(self.mem_rd_valid):
                    m.d.sync += fetched_view.word3_seals.eq(self.mem_rd_data)
                    m.next = "LOADX_SET_MONITOR"

            with m.State("LOADX_SET_MONITOR"):
                m.d.sync += [
                    current_mon_state.eq(ExclMonitorState.ACTIVE),
                    current_mon_addr.eq(addr_latched),
                ]
                m.next = "LOADX_WRITE_DST"

            with m.State("LOADX_WRITE_DST"):
                m.d.comb += [
                    self.cr_wr_addr.eq(Cat(self.cr_dst, Const(0, 1))),
                    self.cr_wr_data.eq(fetched_cap),
                    self.cr_wr_en.eq(1),
                ]
                m.next = "COMPLETE"

            with m.State("SAVEX_READ_BASE"):
                m.d.comb += self.cr_rd_addr.eq(Cat(self.cr_base, Const(0, 1)))
                m.d.sync += [base_reg_latched.eq(self.cr_rd_data), addr_latched.eq(target_addr)]
                m.next = "SAVEX_CHECK_MONITOR"

            with m.State("SAVEX_CHECK_MONITOR"):
                m.d.comb += self.cr_rd_addr.eq(Cat(self.cr_src, Const(0, 1)))
                m.next = "SAVEX_CHECK_PERM"

            with m.State("SAVEX_CHECK_PERM"):
                with m.If(~has_s_perm):
                    m.d.sync += [fault_flag.eq(1), fault_latched.eq(FaultType.PERM_S)]
                    m.next = "FAULT"
                with m.Elif(monitor_valid):
                    m.next = "SAVEX_WRITE_MEM"
                with m.Else():
                    m.next = "SAVEX_WRITE_RESULT"

            with m.State("SAVEX_WRITE_MEM"):
                src_view = View(CAP_REG_LAYOUT, self.cr_rd_data)
                m.d.comb += [
                    self.mem_addr.eq(Cat(addr_latched, Const(0, 32))),
                    self.mem_wr_data.eq(src_view.word0_gt),
                    self.mem_wr_en.eq(1),
                ]
                m.next = "SAVEX_WRITE_RESULT"

            with m.State("SAVEX_WRITE_RESULT"):
                m.d.comb += [
                    self.dr_wr_addr.eq(self.result_dr),
                    self.dr_wr_data.eq(Mux(monitor_valid, 0, 1)),
                    self.dr_wr_en.eq(1),
                ]
                m.d.sync += current_mon_state.eq(ExclMonitorState.IDLE)
                m.next = "COMPLETE"

            with m.State("COMPLETE"):
                m.next = "IDLE"

            with m.State("FAULT"):
                m.next = "IDLE"

        m.d.comb += [
            self.busy.eq(~fsm.ongoing("IDLE")),
            self.complete.eq(fsm.ongoing("COMPLETE")),
            self.fault_valid.eq(fault_flag),
            self.fault_type.eq(fault_latched),
        ]

        return m
