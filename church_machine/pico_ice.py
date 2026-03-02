from amaranth import *
from amaranth.lib.data import View

from .types import *
from .layouts import GT_LAYOUT, CAP_REG_LAYOUT
from .core import ChurchCore
from .boot_rom import BOOT_PROGRAM, DEMO_NAMESPACE, DEMO_CLIST
from .uart_tx import DebugPrinter
from .uart_rx import UartRx

IMEM_WORDS = 256
NS_WORDS = 192
CLIST_WORDS = 64
DMEM_WORDS = NS_WORDS + CLIST_WORDS
TOTAL_WORDS = IMEM_WORDS + DMEM_WORDS


class ICE40SPRAM(Elaboratable):
    """iCE40UP5K SPRAM wrapper — two SB_SPRAM256KA blocks for 32-bit width.

    Each SPRAM block: 16-bit wide x 16384 deep = 256Kbit.
    Two blocks side-by-side give 32-bit x 16384 = 64KB usable.
    Church Machine uses 4KB (1024 words) — fits easily.
    """

    def __init__(self):
        self.addr = Signal(14)
        self.wr_data = Signal(32)
        self.rd_data = Signal(32)
        self.wr_en = Signal()
        self.cs = Signal(init=1)

    def elaborate(self, platform):
        m = Module()

        maskwren = Signal(4)
        m.d.comb += maskwren.eq(Mux(self.wr_en, 0b1111, 0b0000))

        maskwren_hi = Signal(4)
        m.d.comb += maskwren_hi.eq(Mux(self.wr_en, 0b1111, 0b0000))

        m.submodules.spram_lo = Instance("SB_SPRAM256KA",
            i_ADDRESS=self.addr,
            i_DATAIN=self.wr_data[:16],
            i_MASKWREN=maskwren,
            i_WREN=self.wr_en,
            i_CHIPSELECT=self.cs,
            i_CLOCK=ClockSignal(),
            i_STANDBY=Const(0),
            i_SLEEP=Const(0),
            i_POWEROFF=Const(1),
            o_DATAOUT=self.rd_data[:16],
        )

        m.submodules.spram_hi = Instance("SB_SPRAM256KA",
            i_ADDRESS=self.addr,
            i_DATAIN=self.wr_data[16:32],
            i_MASKWREN=maskwren_hi,
            i_WREN=self.wr_en,
            i_CHIPSELECT=self.cs,
            i_CLOCK=ClockSignal(),
            i_STANDBY=Const(0),
            i_SLEEP=Const(0),
            i_POWEROFF=Const(1),
            o_DATAOUT=self.rd_data[16:32],
        )

        return m


class ICE40RGBLED(Elaboratable):
    """iCE40UP5K RGB LED driver using SB_RGBA_DRV primitive.

    The pico-ice RGB LED uses the dedicated LED driver block.
    Active-low: 0 = LED on, 1 = LED off.
    """

    def __init__(self):
        self.r = Signal()
        self.g = Signal()
        self.b = Signal()

    def elaborate(self, platform):
        m = Module()

        m.submodules.rgb_drv = Instance("SB_RGBA_DRV",
            p_CURRENT_MODE="0b1",
            p_RGB0_CURRENT="0b000001",
            p_RGB1_CURRENT="0b000001",
            p_RGB2_CURRENT="0b000001",
            i_CURREN=Const(1),
            i_RGBLEDEN=Const(1),
            i_RGB0PWM=self.g,
            i_RGB1PWM=self.b,
            i_RGB2PWM=self.r,
            o_RGB0=Signal(name="led_g_pin"),
            o_RGB1=Signal(name="led_b_pin"),
            o_RGB2=Signal(name="led_r_pin"),
        )

        return m


class ChurchPicoIce(Elaboratable):
    """Pico-ice top-level wrapper for the Pure Church Machine.

    Adapts ChurchTop for the pico-ice board (iCE40UP5K + RP2040):
    - Clock from RP2040 via pin 35 (default 12 MHz)
    - UART TX on pin 25 -> RP2040 -> USB serial
    - UART RX on pin 27 <- RP2040 <- USB serial
    - RGB LED via SB_RGBA_DRV (loading=blue, run=green, fault=red)
    - Push button on pin 10 (active-low)
    - Instruction memory in SPRAM pair 1 (blocks 3+4, 32-bit x 16K)
    - Data memory in SPRAM pair 2 (blocks 1+2, 32-bit x 16K)
    - All memory loaded via UART — no Boot ROM

    Memory map:
      IMEM SPRAM: words 0..255 = boot program (1KB)
      DMEM SPRAM: words 0..191 = namespace, 192..255 = c-list
    """

    def __init__(self, clk_freq=12_000_000, baud=115200, sim_mode=False):
        self.clk_freq = clk_freq
        self.baud = baud
        self.sim_mode = sim_mode

        self.uart_tx = Signal(init=1)
        self.uart_rx = Signal(init=1)
        self.push_button = Signal(init=1)

        self.led_r = Signal()
        self.led_g = Signal()
        self.led_b = Signal()

        self.dbg_nia = Signal(32)
        self.dbg_fault = Signal(4)
        self.dbg_fault_valid = Signal()
        self.dbg_boot_complete = Signal()

    def elaborate(self, platform):
        m = Module()

        core = ChurchCore()
        m.submodules.core = core

        debug = DebugPrinter(self.clk_freq, self.baud)
        m.submodules.debug = debug

        uart_rx = UartRx(self.clk_freq, self.baud)
        m.submodules.uart_rx = uart_rx
        m.d.comb += uart_rx.rx.eq(self.uart_rx)

        if not self.sim_mode:
            rgb = ICE40RGBLED()
            m.submodules.rgb = rgb

        load_done = Signal()

        if self.sim_mode:
            imem_init = list(BOOT_PROGRAM[:IMEM_WORDS])
            while len(imem_init) < IMEM_WORDS:
                imem_init.append(0)
            imem = Memory(width=32, depth=IMEM_WORDS, init=imem_init)
            m.submodules.imem = imem
            imem_rd = imem.read_port(transparent=True)
            imem_wr = imem.write_port()

            ns_flat = []
            for i in range(0, len(DEMO_NAMESPACE), 3):
                if i + 2 < len(DEMO_NAMESPACE):
                    ns_flat.extend([DEMO_NAMESPACE[i], DEMO_NAMESPACE[i+1], DEMO_NAMESPACE[i+2]])
            while len(ns_flat) < NS_WORDS:
                ns_flat.append(0)
            clist_init = list(DEMO_CLIST[:CLIST_WORDS])
            while len(clist_init) < CLIST_WORDS:
                clist_init.append(0)
            dmem_init = ns_flat + clist_init
            while len(dmem_init) < 1024:
                dmem_init.append(0)

            dmem = Memory(width=32, depth=1024, init=dmem_init)
            m.submodules.dmem = dmem
            dmem_rd = dmem.read_port(transparent=True)
            dmem_wr = dmem.write_port()

            imem_core_addr = Signal(8)
            m.d.comb += imem_core_addr.eq(core.imem_addr[2:10])
            m.d.comb += [
                imem_rd.addr.eq(imem_core_addr),
                core.imem_data.eq(imem_rd.data),
                imem_wr.en.eq(0),
            ]

            dmem_core_addr = Signal(10)
            any_ns_access = Signal()
            any_clist_access = Signal()
            m.d.comb += [
                any_ns_access.eq(core.ns_rd_en | core.ns_wr_en),
                any_clist_access.eq(core.clist_rd_en | core.clist_wr_en),
            ]
            with m.If(any_ns_access):
                m.d.comb += dmem_core_addr.eq(core.ns_addr[2:12])
            with m.Elif(any_clist_access):
                m.d.comb += dmem_core_addr.eq(core.clist_addr[2:12])
            with m.Else():
                m.d.comb += dmem_core_addr.eq(core.dmem_addr[2:12])

            m.d.comb += [
                dmem_rd.addr.eq(dmem_core_addr),
                core.dmem_rd_data.eq(dmem_rd.data),
                core.ns_rd_data.eq(Cat(dmem_rd.data, C(0, 64))),
                core.clist_rd_data.eq(dmem_rd.data),
            ]

            wr_data = Signal(32)
            wr_en = Signal()
            with m.If(core.ns_wr_en):
                m.d.comb += [wr_data.eq(core.ns_wr_data[:32]), wr_en.eq(1)]
            with m.Elif(core.clist_wr_en):
                m.d.comb += [wr_data.eq(core.clist_wr_data), wr_en.eq(1)]
            with m.Else():
                m.d.comb += [wr_data.eq(core.dmem_wr_data), wr_en.eq(core.dmem_wr_en)]

            m.d.comb += [
                dmem_wr.addr.eq(dmem_core_addr),
                dmem_wr.data.eq(wr_data),
                dmem_wr.en.eq(wr_en),
            ]

            m.d.comb += load_done.eq(1)
        else:
            imem_spram = ICE40SPRAM()
            m.submodules.imem_spram = imem_spram
            dmem_spram = ICE40SPRAM()
            m.submodules.dmem_spram = dmem_spram

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

            m.d.comb += [
                core.dmem_rd_data.eq(dmem_spram.rd_data),
                core.ns_rd_data.eq(Cat(dmem_spram.rd_data, C(0, 64))),
                core.clist_rd_data.eq(dmem_spram.rd_data),
                core.imem_data.eq(imem_spram.rd_data),
            ]

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

            load_word = Signal(32)
            load_byte_idx = Signal(2)
            load_word_count = Signal(range(TOTAL_WORDS + 1))
            load_words_received = Signal(range(TOTAL_WORDS + 1))

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

        halted = Signal(init=1)
        stepping = Signal()

        prev_nia = Signal(32)
        m.d.sync += prev_nia.eq(core.nia)
        nia_changed = Signal()
        m.d.comb += nia_changed.eq(core.nia != prev_nia)

        step_complete = Signal()
        m.d.comb += step_complete.eq(stepping & nia_changed)

        m.d.comb += core.imem_valid.eq(~halted | (stepping & ~step_complete))

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

        m.d.comb += self.uart_tx.eq(debug.tx)

        m.d.comb += [
            self.dbg_nia.eq(core.nia),
            self.dbg_fault.eq(core.fault),
            self.dbg_fault_valid.eq(core.fault_valid),
            self.dbg_boot_complete.eq(core.boot_complete),
        ]

        heartbeat_ctr = Signal(range(self.clk_freq))
        heartbeat_blink = Signal()
        m.d.sync += heartbeat_ctr.eq(heartbeat_ctr + 1)
        with m.If(heartbeat_ctr == self.clk_freq - 1):
            m.d.sync += [heartbeat_ctr.eq(0), heartbeat_blink.eq(~heartbeat_blink)]

        led_loading = Signal()
        led_boot = Signal()
        led_run = Signal()
        led_halted_blink = Signal()
        led_fault = Signal()
        m.d.comb += [
            led_loading.eq(~load_done),
            led_boot.eq(load_done & ~core.boot_complete),
            led_run.eq(core.boot_complete & ~core.fault_valid & ~halted),
            led_halted_blink.eq(core.boot_complete & halted & ~core.fault_valid & heartbeat_blink),
            led_fault.eq(core.fault_valid),
        ]

        m.d.comb += [
            self.led_b.eq(led_loading | led_boot),
            self.led_g.eq(led_run | led_halted_blink),
            self.led_r.eq(led_fault),
        ]

        if not self.sim_mode:
            m.d.comb += [
                rgb.r.eq(led_fault),
                rgb.g.eq(led_run | led_halted_blink),
                rgb.b.eq(led_loading | led_boot),
            ]

        boot_delay = Signal(4, init=0)
        boot_triggered = Signal()

        with m.If(~boot_triggered & load_done):
            m.d.sync += boot_delay.eq(boot_delay + 1)
            with m.If(boot_delay == 0xF):
                m.d.sync += boot_triggered.eq(1)
                m.d.comb += core.boot_start.eq(1)
        with m.Else():
            m.d.comb += core.boot_start.eq(0)

        m.d.comb += core.gc_start.eq(0)

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

        if self.sim_mode:
            startup_target = 4
        else:
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

        return m
