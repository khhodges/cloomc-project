"""Integration test for the UART program loader.

Tests the loader FSM by simulating UART byte reception and verifying
that words are correctly written to instruction and data memory.
Uses a simplified test harness with Memory instead of SPRAM.
Uses a tiny IMEM boundary (4 words) to keep simulation fast.
"""

from amaranth import *
from amaranth.sim import *
import struct

from .uart_rx import UartRx

BAUD = 115200
CLK_FREQ = 12_000_000
DIVISOR = CLK_FREQ // BAUD
SYNC_BYTE = 0xAA
TEST_IMEM_WORDS = 4


class LoaderTestHarness(Elaboratable):
    def __init__(self):
        self.uart_rx_pin = Signal(init=1)
        self.load_complete = Signal()
        self.words_received = Signal(16)
        self.imem_probe_addr = Signal(9)
        self.imem_probe_data = Signal(32)
        self.dmem_probe_addr = Signal(9)
        self.dmem_probe_data = Signal(32)

    def elaborate(self, platform):
        m = Module()

        uart_rx = UartRx(CLK_FREQ, BAUD)
        m.submodules.uart_rx = uart_rx
        m.d.comb += uart_rx.rx.eq(self.uart_rx_pin)

        imem = Memory(width=32, depth=64, init=[0]*64)
        m.submodules.imem = imem
        imem_wr = imem.write_port()
        imem_rd = imem.read_port()

        dmem = Memory(width=32, depth=64, init=[0]*64)
        m.submodules.dmem = dmem
        dmem_wr = dmem.write_port()
        dmem_rd = dmem.read_port()

        m.d.comb += [
            imem_rd.addr.eq(self.imem_probe_addr),
            self.imem_probe_data.eq(imem_rd.data),
            dmem_rd.addr.eq(self.dmem_probe_addr),
            self.dmem_probe_data.eq(dmem_rd.data),
        ]

        load_complete = Signal()
        m.d.comb += self.load_complete.eq(load_complete)

        load_word = Signal(32)
        load_byte_idx = Signal(2)
        load_word_count = Signal(16)
        load_words_received = Signal(16)
        load_is_imem = Signal()
        m.d.comb += self.words_received.eq(load_words_received)

        with m.FSM(name="loader_fsm"):
            with m.State("WAIT_SYNC"):
                with m.If(uart_rx.valid):
                    with m.If(uart_rx.data == SYNC_BYTE):
                        m.d.sync += [load_byte_idx.eq(0), load_word.eq(0)]
                        m.next = "RECV_HEADER"

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
                m.d.sync += [
                    load_word_count.eq(load_word[:16]),
                    load_word.eq(0),
                ]
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
                        m.d.sync += load_is_imem.eq(load_words_received < TEST_IMEM_WORDS)
                        m.next = "WRITE_WORD"
                    with m.Else():
                        m.d.sync += load_byte_idx.eq(load_byte_idx + 1)

            with m.State("WRITE_WORD"):
                with m.If(load_is_imem):
                    m.d.comb += [
                        imem_wr.addr.eq(load_words_received[:6]),
                        imem_wr.data.eq(load_word),
                        imem_wr.en.eq(1),
                    ]
                with m.Else():
                    m.d.comb += [
                        dmem_wr.addr.eq((load_words_received - TEST_IMEM_WORDS)[:6]),
                        dmem_wr.data.eq(load_word),
                        dmem_wr.en.eq(1),
                    ]
                m.d.sync += [
                    load_words_received.eq(load_words_received + 1),
                    load_byte_idx.eq(0),
                    load_word.eq(0),
                ]
                with m.If(load_words_received + 1 >= load_word_count):
                    m.next = "LOAD_DONE"
                with m.Else():
                    m.next = "RECV_DATA"

            with m.State("LOAD_DONE"):
                m.d.sync += load_complete.eq(1)
                m.next = "IDLE"

            with m.State("IDLE"):
                pass

        return m


def send_uart_byte(pin, val):
    yield pin.eq(0)
    for _ in range(DIVISOR):
        yield Tick()
    for bit in range(8):
        yield pin.eq((val >> bit) & 1)
        for _ in range(DIVISOR):
            yield Tick()
    yield pin.eq(1)
    for _ in range(DIVISOR):
        yield Tick()


def send_image_over_uart(pin, image):
    yield from send_uart_byte(pin, SYNC_BYTE)

    header = struct.pack('<I', len(image))
    for b in header:
        yield from send_uart_byte(pin, b)

    for word in image:
        word_bytes = struct.pack('<I', word)
        for b in word_bytes:
            yield from send_uart_byte(pin, b)


def test_loader():
    top = LoaderTestHarness()
    sim = Simulator(top)
    sim.add_clock(1 / CLK_FREQ)

    imem_words = [0xDEADBEEF, 0x12345678, 0xCAFEBABE, 0xAAAA5555]
    dmem_words = [0x11111111, 0x22222222, 0x33333333, 0x44444444]
    full_image = imem_words + dmem_words

    def testbench():
        print("=== UART Loader Test ===")
        print(f"  Image: {len(full_image)} words ({len(imem_words)} imem + {len(dmem_words)} dmem)")
        print()

        print("--- Phase 1: Send image over UART ---")
        yield from send_image_over_uart(top.uart_rx_pin, full_image)

        for _ in range(100):
            yield Tick()

        lc = yield top.load_complete
        wr = yield top.words_received
        print(f"  load_complete={lc}, words_received={wr}")
        assert lc == 1, f"FAIL: load_complete not asserted (got {lc})"
        assert wr == len(full_image), f"FAIL: words_received={wr}, expected {len(full_image)}"
        print("  PASS: Load complete, correct word count")

        print()
        print("--- Phase 2: Verify IMEM contents ---")
        all_ok = True
        for i, expected in enumerate(imem_words):
            yield top.imem_probe_addr.eq(i)
            yield Tick()
            yield Tick()
            got = yield top.imem_probe_data
            ok = got == expected
            status = "PASS" if ok else "FAIL"
            print(f"  {status}: imem[{i}] = 0x{got:08X} (expected 0x{expected:08X})")
            if not ok:
                all_ok = False
        assert all_ok, "FAIL: IMEM contents mismatch"

        print()
        print("--- Phase 3: Verify DMEM contents ---")
        for i, expected in enumerate(dmem_words):
            yield top.dmem_probe_addr.eq(i)
            yield Tick()
            yield Tick()
            got = yield top.dmem_probe_data
            ok = got == expected
            status = "PASS" if ok else "FAIL"
            print(f"  {status}: dmem[{i}] = 0x{got:08X} (expected 0x{expected:08X})")
            if not ok:
                all_ok = False
        assert all_ok, "FAIL: DMEM contents mismatch"

        print()
        print("=== All loader tests passed! ===")

    sim.add_process(testbench)

    with sim.write_vcd("build/test_loader.vcd"):
        sim.run()


if __name__ == "__main__":
    test_loader()
