"""Diagnostic UART test for pico-ice — SPRAM isolation test.
Uses DebugPrinter + SPRAM init (writes data to SPRAM, reads it back)
but NO ChurchCore. Tests whether SPRAM usage crashes the RP2040.
"""

from amaranth import *
from .uart_tx import UartTx, DebugPrinter
from .boot_rom import DEMO_NAMESPACE, DEMO_CLIST
from .pico_ice import ICE40SPRAM, ICE40RGBLED


class UartTestTop(Elaboratable):
    def __init__(self, clk_freq=12_000_000, baud=115200):
        self.clk_freq = clk_freq
        self.baud = baud
        self.uart_tx = Signal(init=1)
        self.led_r = Signal()
        self.led_g = Signal()
        self.led_b = Signal()
        self.push_button = Signal()
        self.uart_rx = Signal()

    def elaborate(self, platform):
        m = Module()

        debug = DebugPrinter(self.clk_freq, self.baud)
        m.submodules.debug = debug
        m.d.comb += self.uart_tx.eq(debug.tx)

        rgb = ICE40RGBLED()
        m.submodules.rgb = rgb

        spram = ICE40SPRAM()
        m.submodules.spram = spram

        ns_flat = []
        for i in range(0, len(DEMO_NAMESPACE), 3):
            if i + 2 < len(DEMO_NAMESPACE):
                ns_flat.extend([DEMO_NAMESPACE[i], DEMO_NAMESPACE[i+1], DEMO_NAMESPACE[i+2]])

        clist_flat = list(DEMO_CLIST[:64])

        init_data = ns_flat + [0] * (192 - len(ns_flat)) + clist_flat + [0] * (64 - len(clist_flat))
        init_total = len(init_data)

        init_idx = Signal(range(init_total + 1))
        init_done = Signal()
        init_word = Signal(32)

        with m.Switch(init_idx):
            for i, word in enumerate(init_data):
                if word != 0:
                    with m.Case(i):
                        m.d.comb += init_word.eq(word)
            with m.Default():
                m.d.comb += init_word.eq(0)

        with m.If(~init_done):
            m.d.comb += [
                spram.addr.eq(init_idx),
                spram.wr_data.eq(init_word),
                spram.wr_en.eq(1),
            ]
            with m.If(init_idx < init_total):
                m.d.sync += init_idx.eq(init_idx + 1)
            with m.Else():
                m.d.sync += init_done.eq(1)

        rd_addr = Signal(14)
        rd_data_latched = Signal(32)
        rd_phase = Signal()

        counter = Signal(8, init=0)
        delay_ctr = Signal(24)
        heartbeat = Signal()

        with m.FSM(name="diag_fsm"):
            with m.State("WAIT_INIT"):
                with m.If(init_done):
                    m.d.sync += rd_addr.eq(0)
                    m.next = "READ_SETUP"

            with m.State("READ_SETUP"):
                m.d.comb += [
                    spram.addr.eq(rd_addr),
                    spram.wr_en.eq(0),
                ]
                m.next = "READ_LATCH"

            with m.State("READ_LATCH"):
                m.d.comb += [
                    spram.addr.eq(rd_addr),
                    spram.wr_en.eq(0),
                ]
                m.d.sync += rd_data_latched.eq(spram.rd_data)
                m.next = "SEND_HEX"

            with m.State("SEND_HEX"):
                with m.If(~debug.busy):
                    m.d.comb += [
                        debug.data.eq(rd_data_latched),
                        debug.send.eq(1),
                    ]
                    m.d.sync += rd_addr.eq(rd_addr + 1)
                    m.next = "WAIT_DONE"

            with m.State("WAIT_DONE"):
                with m.If(~debug.busy):
                    with m.If(rd_addr < 10):
                        m.next = "READ_SETUP"
                    with m.Else():
                        m.next = "COUNTER_LOOP"

            with m.State("COUNTER_LOOP"):
                m.d.sync += delay_ctr.eq(delay_ctr + 1)
                with m.If(delay_ctr == (self.clk_freq // 4) - 1):
                    m.d.sync += [delay_ctr.eq(0), heartbeat.eq(~heartbeat)]
                    m.next = "SEND_COUNTER"

            with m.State("SEND_COUNTER"):
                with m.If(~debug.busy):
                    m.d.comb += [
                        debug.data.eq(counter),
                        debug.send.eq(1),
                    ]
                    m.d.sync += counter.eq(counter + 1)
                    m.next = "WAIT_COUNTER"

            with m.State("WAIT_COUNTER"):
                with m.If(~debug.busy):
                    m.next = "COUNTER_LOOP"

        m.d.comb += [
            rgb.r.eq(0),
            rgb.g.eq(heartbeat),
            rgb.b.eq(~init_done),
        ]

        m.d.comb += [
            self.led_r.eq(0),
            self.led_g.eq(heartbeat),
            self.led_b.eq(~init_done),
        ]

        return m
