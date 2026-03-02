"""Integration test for the UART data loader.

Tests the loader FSM by simulating UART byte reception and verifying
that words are correctly written to data memory (SPRAM).
Uses a simplified test harness with Memory instead of SPRAM.
"""

from amaranth import *
from amaranth.sim import *
import struct

from .uart_rx import UartRx
from .upload import NS_WORDS, CLIST_WORDS, TOTAL_WORDS

BAUD = 115200
CLK_FREQ = 12_000_000
DIVISOR = CLK_FREQ // BAUD


class LoaderTestHarness(Elaboratable):
    def __init__(self):
        self.uart_rx_pin = Signal(init=1)
        self.load_done = Signal()
        self.words_received = Signal(16)
        self.dmem_probe_addr = Signal(9)
        self.dmem_probe_data = Signal(32)

    def elaborate(self, platform):
        m = Module()

        uart_rx = UartRx(CLK_FREQ, BAUD)
        m.submodules.uart_rx = uart_rx
        m.d.comb += uart_rx.rx.eq(self.uart_rx_pin)

        dmem = Memory(width=32, depth=TOTAL_WORDS, init=[0]*TOTAL_WORDS)
        m.submodules.dmem = dmem
        dmem_wr = dmem.write_port()
        dmem_rd = dmem.read_port()

        m.d.comb += [
            dmem_rd.addr.eq(self.dmem_probe_addr),
            self.dmem_probe_data.eq(dmem_rd.data),
        ]

        load_done = Signal()
        m.d.comb += self.load_done.eq(load_done)

        load_word = Signal(32)
        load_byte_idx = Signal(2)
        load_word_count = Signal(range(TOTAL_WORDS + 1))
        load_words_received = Signal(range(TOTAL_WORDS + 1))
        m.d.comb += self.words_received.eq(load_words_received)

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
                with m.If(load_word[:9] <= TOTAL_WORDS):
                    m.d.sync += load_word_count.eq(load_word[:9])
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
                m.d.comb += [
                    dmem_wr.addr.eq(load_words_received[:9]),
                    dmem_wr.data.eq(load_word),
                    dmem_wr.en.eq(1),
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

    test_words = [0xDEADBEEF, 0x12345678, 0xCAFEBABE, 0xAAAA5555,
                  0x11111111, 0x22222222, 0x33333333, 0x44444444]

    def testbench():
        print("=== UART Data Loader Test ===")
        print(f"  Image: {len(test_words)} words")
        print()

        print("--- Phase 1: Send data over UART ---")
        yield from send_image_over_uart(top.uart_rx_pin, test_words)

        for _ in range(100):
            yield Tick()

        ld = yield top.load_done
        wr = yield top.words_received
        print(f"  load_done={ld}, words_received={wr}")
        assert ld == 1, f"FAIL: load_done not asserted (got {ld})"
        assert wr == len(test_words), f"FAIL: words_received={wr}, expected {len(test_words)}"
        print("  PASS: Load complete, correct word count")

        print()
        print("--- Phase 2: Verify DMEM contents ---")
        all_ok = True
        for i, expected in enumerate(test_words):
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
