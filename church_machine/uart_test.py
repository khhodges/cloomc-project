"""Pico-ice design with UART full-image loader.

All memory (instructions, namespace, c-list) is uploaded over serial.
No Boot ROM — instructions live in a second SPRAM pair.

Protocol:
  1. Host sends 4-byte header: total word count (little-endian u32)
  2. Host sends N x 4-byte words (little-endian u32):
     - Words 0..255:   Instruction memory (256 words = 1KB boot program)
     - Words 256..447:  Namespace (192 words = 16 entries x 3 words)
     - Words 448..511:  C-list (64 words)
  3. FPGA boots and sends "CHURCH v1.0" banner

LED blue while waiting for upload.
"""

from amaranth import *
from .uart_tx import DebugPrinter
from .uart_rx import UartRx
from .core import ChurchCore
from .pico_ice import ICE40SPRAM, ICE40RGBLED

IMEM_WORDS = 256
NS_WORDS = 192
CLIST_WORDS = 64
DMEM_WORDS = NS_WORDS + CLIST_WORDS
TOTAL_WORDS = IMEM_WORDS + DMEM_WORDS


class UartTestTop(Elaboratable):
    def __init__(self, clk_freq=12_000_000, baud=115200):
        self.clk_freq = clk_freq
        self.baud = baud
        self.uart_tx = Signal(init=1)
        self.led_r = Signal()
        self.led_g = Signal()
        self.led_b = Signal()
        self.push_button = Signal()
        self.uart_rx = Signal(init=1)

    def elaborate(self, platform):
        m = Module()

        debug = DebugPrinter(self.clk_freq, self.baud)
        m.submodules.debug = debug
        m.d.comb += self.uart_tx.eq(debug.tx)

        uart_rx = UartRx(self.clk_freq, self.baud)
        m.submodules.uart_rx = uart_rx
        m.d.comb += uart_rx.rx.eq(self.uart_rx)

        rgb = ICE40RGBLED()
        m.submodules.rgb = rgb

        core = ChurchCore()
        m.submodules.core = core

        imem_spram = ICE40SPRAM()
        m.submodules.imem_spram = imem_spram

        dmem_spram = ICE40SPRAM()
        m.submodules.dmem_spram = dmem_spram

        load_done = Signal()

        imem_core_addr = Signal(14)
        m.d.comb += imem_core_addr.eq(core.imem_addr[2:16])

        dmem_core_addr = Signal(14)
        dmem_core_wr_data = Signal(32)
        dmem_core_wr_en = Signal()

        any_ns_access = Signal()
        any_clist_access = Signal()
        m.d.comb += [
            any_ns_access.eq(core.ns_rd_en | core.ns_wr_en),
            any_clist_access.eq(core.clist_rd_en | core.clist_wr_en),
        ]
        with m.If(any_ns_access):
            m.d.comb += dmem_core_addr.eq(core.ns_addr[2:16])
        with m.Elif(any_clist_access):
            m.d.comb += dmem_core_addr.eq(core.clist_addr[2:16])
        with m.Else():
            m.d.comb += dmem_core_addr.eq(core.dmem_addr[2:16])

        with m.If(core.ns_wr_en):
            m.d.comb += [dmem_core_wr_data.eq(core.ns_wr_data[:32]), dmem_core_wr_en.eq(1)]
        with m.Elif(core.clist_wr_en):
            m.d.comb += [dmem_core_wr_data.eq(core.clist_wr_data), dmem_core_wr_en.eq(1)]
        with m.Else():
            m.d.comb += [dmem_core_wr_data.eq(core.dmem_wr_data), dmem_core_wr_en.eq(core.dmem_wr_en)]

        m.d.comb += core.dmem_rd_data.eq(dmem_spram.rd_data)
        m.d.comb += [
            core.ns_rd_data.eq(Cat(dmem_spram.rd_data, C(0, 64))),
            core.clist_rd_data.eq(dmem_spram.rd_data),
        ]
        m.d.comb += core.imem_data.eq(imem_spram.rd_data)

        load_word = Signal(32)
        load_byte_idx = Signal(2)
        load_word_count = Signal(range(TOTAL_WORDS + 1))
        load_words_received = Signal(range(TOTAL_WORDS + 1))

        loader_imem_addr = Signal(14)
        loader_imem_data = Signal(32)
        loader_imem_wr = Signal()

        loader_dmem_addr = Signal(14)
        loader_dmem_data = Signal(32)
        loader_dmem_wr = Signal()

        with m.If(load_done):
            m.d.comb += [
                imem_spram.addr.eq(imem_core_addr),
                imem_spram.wr_data.eq(0),
                imem_spram.wr_en.eq(0),
                dmem_spram.addr.eq(dmem_core_addr),
                dmem_spram.wr_data.eq(dmem_core_wr_data),
                dmem_spram.wr_en.eq(dmem_core_wr_en),
            ]
        with m.Else():
            m.d.comb += [
                imem_spram.addr.eq(loader_imem_addr),
                imem_spram.wr_data.eq(loader_imem_data),
                imem_spram.wr_en.eq(loader_imem_wr),
                dmem_spram.addr.eq(loader_dmem_addr),
                dmem_spram.wr_data.eq(loader_dmem_data),
                dmem_spram.wr_en.eq(loader_dmem_wr),
            ]

        is_imem_word = Signal()
        dmem_offset = Signal(range(DMEM_WORDS + 1))
        m.d.comb += [
            is_imem_word.eq(load_words_received < IMEM_WORDS),
            dmem_offset.eq(load_words_received - IMEM_WORDS),
        ]

        with m.FSM(name="loader_fsm"):
            with m.State("RECV_HEADER"):
                with m.If(uart_rx.valid):
                    with m.If(load_byte_idx == 0):
                        m.d.sync += load_word[:8].eq(uart_rx.data)
                    with m.Elif(load_byte_idx == 1):
                        m.d.sync += load_word[8:16].eq(uart_rx.data)
                    with m.Elif(load_byte_idx == 2):
                        m.d.sync += load_word[16:24].eq(uart_rx.data)
                    with m.Else():
                        m.d.sync += load_word[24:32].eq(uart_rx.data)

                    with m.If(load_byte_idx == 3):
                        m.d.sync += [
                            load_words_received.eq(0),
                            load_byte_idx.eq(0),
                        ]
                        m.next = "HEADER_LATCH"
                    with m.Else():
                        m.d.sync += load_byte_idx.eq(load_byte_idx + 1)

            with m.State("HEADER_LATCH"):
                with m.If(load_word[:10] <= TOTAL_WORDS):
                    m.d.sync += load_word_count.eq(load_word[:10])
                with m.Else():
                    m.d.sync += load_word_count.eq(TOTAL_WORDS)
                m.d.sync += load_word.eq(0)
                m.next = "RECV_DATA"

            with m.State("RECV_DATA"):
                with m.If(uart_rx.valid):
                    with m.If(load_byte_idx == 0):
                        m.d.sync += load_word[:8].eq(uart_rx.data)
                    with m.Elif(load_byte_idx == 1):
                        m.d.sync += load_word[8:16].eq(uart_rx.data)
                    with m.Elif(load_byte_idx == 2):
                        m.d.sync += load_word[16:24].eq(uart_rx.data)
                    with m.Else():
                        m.d.sync += load_word[24:32].eq(uart_rx.data)

                    with m.If(load_byte_idx == 3):
                        m.next = "WRITE_WORD"
                    with m.Else():
                        m.d.sync += load_byte_idx.eq(load_byte_idx + 1)

            with m.State("WRITE_WORD"):
                with m.If(is_imem_word):
                    m.d.comb += [
                        loader_imem_addr.eq(load_words_received[:14]),
                        loader_imem_data.eq(load_word),
                        loader_imem_wr.eq(1),
                    ]
                with m.Else():
                    m.d.comb += [
                        loader_dmem_addr.eq(dmem_offset[:14]),
                        loader_dmem_data.eq(load_word),
                        loader_dmem_wr.eq(1),
                    ]
                m.d.sync += [
                    load_words_received.eq(load_words_received + 1),
                    load_byte_idx.eq(0),
                    load_word.eq(0),
                ]
                with m.If(load_words_received + 1 >= load_word_count):
                    m.d.sync += load_done.eq(1)
                    m.next = "IDLE"
                with m.Else():
                    m.next = "RECV_DATA"

            with m.State("IDLE"):
                pass

        boot_delay = Signal(4)
        boot_triggered = Signal()
        with m.If(~boot_triggered & load_done):
            with m.If(boot_delay < 0xF):
                m.d.sync += boot_delay.eq(boot_delay + 1)
            with m.Else():
                m.d.comb += core.boot_start.eq(1)
                m.d.sync += boot_triggered.eq(1)

        halted = Signal(init=1)
        stepping = Signal()

        prev_nia = Signal(32)
        m.d.sync += prev_nia.eq(core.nia)
        nia_changed = Signal()
        m.d.comb += nia_changed.eq(core.nia != prev_nia)
        step_complete = Signal()
        m.d.comb += step_complete.eq(stepping & nia_changed)

        m.d.comb += core.imem_valid.eq(~halted | (stepping & ~step_complete))
        m.d.comb += core.gc_start.eq(0)

        btn_sync = Signal(3)
        btn_prev = Signal()
        m.d.sync += [
            btn_sync[0].eq(self.push_button),
            btn_sync[1].eq(btn_sync[0]),
            btn_sync[2].eq(btn_sync[1]),
            btn_prev.eq(btn_sync[2]),
        ]
        btn_press = Signal()
        m.d.comb += btn_press.eq(btn_prev & ~btn_sync[2])

        heartbeat_ctr = Signal(23)
        m.d.sync += heartbeat_ctr.eq(heartbeat_ctr + 1)
        heartbeat_blink = Signal()
        m.d.comb += heartbeat_blink.eq(heartbeat_ctr[-1])

        led_loading = Signal()
        led_boot = Signal()
        led_run = Signal()
        led_fault = Signal()
        led_halted_blink = Signal()
        m.d.comb += [
            led_loading.eq(~load_done),
            led_boot.eq(load_done & ~core.boot_complete),
            led_run.eq(core.boot_complete & ~core.fault_valid & ~halted),
            led_fault.eq(core.fault_valid),
            led_halted_blink.eq(core.boot_complete & halted & ~core.fault_valid & heartbeat_blink),
        ]

        BANNER = Array([C(ord(c), 8) for c in "CHURCH v1.0\r\n"])
        banner_idx = Signal(range(len(BANNER) + 1))
        banner_byte = Signal(8)
        m.d.comb += banner_byte.eq(BANNER[banner_idx])

        HALT_MSG = Array([C(ord(c), 8) for c in "HALT\r\n"])
        halt_idx = Signal(range(len(HALT_MSG) + 1))
        halt_byte = Signal(8)
        m.d.comb += halt_byte.eq(HALT_MSG[halt_idx])

        STEP_MSG = Array([C(ord(c), 8) for c in "S:"])
        step_idx = Signal(range(len(STEP_MSG) + 1))
        step_byte = Signal(8)
        m.d.comb += step_byte.eq(STEP_MSG[step_idx])

        FAULT_MSG = Array([C(ord(c), 8) for c in "F:"])
        fault_msg_idx = Signal(range(len(FAULT_MSG) + 1))
        fault_byte = Signal(8)
        m.d.comb += fault_byte.eq(FAULT_MSG[fault_msg_idx])

        step_nia = Signal(32)
        step_fault = Signal(4)
        step_had_fault = Signal()

        startup_target = (self.clk_freq * 3) - 1
        startup_ctr = Signal(26)

        with m.FSM(name="debug_fsm"):
            with m.State("STARTUP_DELAY"):
                m.d.sync += startup_ctr.eq(startup_ctr + 1)
                with m.If(startup_ctr == startup_target):
                    m.next = "WAIT_BOOT"

            with m.State("WAIT_BOOT"):
                with m.If(core.boot_complete):
                    m.d.sync += [banner_idx.eq(0), halted.eq(1)]
                    m.next = "SEND_BANNER"

            with m.State("SEND_BANNER"):
                with m.If(~debug.busy):
                    with m.If(banner_idx < len(BANNER)):
                        m.d.comb += [
                            debug.byte_data.eq(banner_byte),
                            debug.send_byte.eq(1),
                        ]
                        m.d.sync += banner_idx.eq(banner_idx + 1)
                    with m.Else():
                        m.next = "DUMP_NIA"

            with m.State("DUMP_NIA"):
                with m.If(~debug.busy):
                    m.d.comb += [
                        debug.data.eq(core.nia),
                        debug.send.eq(1),
                    ]
                    m.next = "SEND_HALT"

            with m.State("SEND_HALT"):
                with m.If(~debug.busy):
                    with m.If(halt_idx < len(HALT_MSG)):
                        m.d.comb += [
                            debug.byte_data.eq(halt_byte),
                            debug.send_byte.eq(1),
                        ]
                        m.d.sync += halt_idx.eq(halt_idx + 1)
                    with m.Else():
                        m.d.sync += halt_idx.eq(0)
                        m.next = "HALTED"

            with m.State("HALTED"):
                with m.If(btn_press):
                    m.d.sync += stepping.eq(1)
                    m.next = "STEP_WAIT"

            with m.State("STEP_WAIT"):
                with m.If(step_complete):
                    m.d.sync += [
                        stepping.eq(0),
                        step_nia.eq(core.nia),
                        step_fault.eq(core.fault),
                        step_had_fault.eq(core.fault_valid),
                        step_idx.eq(0),
                    ]
                    m.next = "STEP_LABEL"

            with m.State("STEP_LABEL"):
                with m.If(~debug.busy):
                    with m.If(step_idx < len(STEP_MSG)):
                        m.d.comb += [
                            debug.byte_data.eq(step_byte),
                            debug.send_byte.eq(1),
                        ]
                        m.d.sync += step_idx.eq(step_idx + 1)
                    with m.Else():
                        m.d.sync += step_idx.eq(0)
                        m.next = "STEP_DUMP_NIA"

            with m.State("STEP_DUMP_NIA"):
                with m.If(~debug.busy):
                    m.d.comb += [
                        debug.data.eq(step_nia),
                        debug.send.eq(1),
                    ]
                    with m.If(step_had_fault):
                        m.d.sync += fault_msg_idx.eq(0)
                        m.next = "STEP_FAULT_LABEL"
                    with m.Else():
                        m.d.sync += halt_idx.eq(0)
                        m.next = "SEND_HALT"

            with m.State("STEP_FAULT_LABEL"):
                with m.If(~debug.busy):
                    with m.If(fault_msg_idx < len(FAULT_MSG)):
                        m.d.comb += [
                            debug.byte_data.eq(fault_byte),
                            debug.send_byte.eq(1),
                        ]
                        m.d.sync += fault_msg_idx.eq(fault_msg_idx + 1)
                    with m.Else():
                        m.d.sync += fault_msg_idx.eq(0)
                        m.next = "STEP_DUMP_FAULT"

            with m.State("STEP_DUMP_FAULT"):
                with m.If(~debug.busy):
                    fault_word = Signal(32)
                    m.d.comb += [
                        fault_word.eq(Cat(step_fault, C(0, 28))),
                        debug.data.eq(fault_word),
                        debug.send.eq(1),
                    ]
                    m.d.sync += halt_idx.eq(0)
                    m.next = "SEND_HALT"

        m.d.comb += [
            rgb.r.eq(led_fault),
            rgb.g.eq(led_run | led_halted_blink),
            rgb.b.eq(led_loading | led_boot),
        ]
        m.d.comb += [
            self.led_r.eq(led_fault),
            self.led_g.eq(led_run | led_halted_blink),
            self.led_b.eq(led_loading | led_boot),
        ]

        return m
