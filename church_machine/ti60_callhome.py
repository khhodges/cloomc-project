"""
church_machine/ti60_callhome.py
================================
Efinix Ti60 F225 call-home module.

At power-up this module:
  1. Drives a soft JTAG master sequence to read the Ti60's 64-bit die serial
     via USERID instruction 0x08.  The ``EFX_USR_JTAG`` Efinix primitive
     connects fabric-driven TCK/TMS/TDI to the chip's own TAP; TDO is
     captured back into the fabric.
  2. Transmits the 23-byte call-home packet over UART TX.
  3. Waits for the 2-byte ACK ``0xCE 0x22`` on UART RX.
  4. Asserts ``callhome_done``, releasing the top-level boot gate.

Call-home packet layout (23 bytes):
  [0..1]   0xCE 0x11          magic
  [2]      0x03               board type: Ti60-Full
  [3]      0x01               FW major
  [4]      0x00               FW minor
  [5..8]   0x00000000         build_sig (zero, future use)
  [9..16]  uid[8]             64-bit die serial, LSB-first bytes
  [17]     0x00               boot reason: cold
  [18]     0x00               last fault: none
  [19..22] 0x00000000         fault NIA: none

JTAG note:
  ``EFX_USR_JTAG`` is the Efinix primitive that exposes the Ti60 TAP to the
  FPGA fabric in master mode (fabric drives TCK/TMS/TDI, samples TDO).
  Verify the exact port names in the Efinity Titanium Primitive Guide for
  your toolchain version; the primitive may also appear as ``EFX_JTAG_CTRL``.

  The Ti60 TAP instruction register is 8 bits wide.
  USERID instruction (0x08) selects a 64-bit die serial data register.
  TDO shifts out LSB first.

Ti60 top-level integration example::

    callhome = Ti60CallHome(clk_freq=100_000_000, baud=115200)
    m.submodules.callhome = callhome
    m.d.comb += [
        callhome.uart_rx.eq(self.uart_rx),
        self.uart_tx.eq(callhome.uart_tx),   # mux with normal debug UART after done
    ]
    # Gate normal boot FSM until call-home is complete:
    boot_gate = callhome.callhome_done
"""

from amaranth import *
from .uart_tx import UartTx
from .uart_rx import UartRx

# ── Protocol constants ────────────────────────────────────────────────────────

BOARD_TYPE_TI60  = 0x03
FW_MAJOR         = 1
FW_MINOR         = 0
PKT_LEN          = 23

PKT_HEADER = [
    0xCE, 0x11,                           # magic
    BOARD_TYPE_TI60,                      # board type
    FW_MAJOR,                             # fw_major
    FW_MINOR,                             # fw_minor
    0x00, 0x00, 0x00, 0x00,              # build_sig
]  # bytes 0-8; bytes 9-16 = uid; bytes 17-22 = zero

# ── JTAG constants ────────────────────────────────────────────────────────────

USERID_INSTR   = 0x08   # Efinix Ti60 die serial instruction
IR_LEN         = 8      # Ti60 instruction register width (bits)
UID_BITS       = 64     # die serial data register width (bits)

# TCK divisor: system_clock / TCK_DIV = TCK frequency.
# At 100 MHz, TCK_DIV=32 gives 3.125 MHz TCK — well within JTAG spec.
TCK_DIV        = 32

# ── JTAG sequence builder ─────────────────────────────────────────────────────

def _build_jtag_seq():
    """Return list of (tms, tdi) tuples that drive the full USERID read.

    JTAG TAP state path:
      Test-Logic-Reset (5 × TMS=1)
      → Run-Test/Idle (TMS=0)
      → Select-DR-Scan (TMS=1)
      → Select-IR-Scan (TMS=1)
      → Capture-IR (TMS=0)
      → Shift-IR × IR_LEN  (TMS=0 for bits 0..N-2, TMS=1 for last bit)
      → Exit1-IR → Update-IR (TMS=1)
      → Select-DR-Scan (TMS=1)
      → Capture-DR (TMS=0)
      → Shift-DR × UID_BITS  (TMS=0 for bits 0..N-2, TMS=1 for last bit)
      → Exit1-DR → Update-DR (TMS=1)
      → Run-Test/Idle (TMS=0)  [clean exit]

    TDO sampling happens only during the UID_BITS Shift-DR cycles.
    """
    seq = []

    # Test-Logic-Reset: 5 × TMS=1 (works from any TAP state)
    for _ in range(5):
        seq.append((1, 0))

    # Run-Test/Idle
    seq.append((0, 0))

    # Select-DR-Scan
    seq.append((1, 0))

    # Select-IR-Scan
    seq.append((1, 0))

    # Capture-IR
    seq.append((0, 0))

    # Shift-IR: IR_LEN bits of USERID_INSTR, LSB first.
    # TMS=0 to stay in Shift-IR; TMS=1 on the last bit to exit to Exit1-IR.
    for i in range(IR_LEN):
        tdi = (USERID_INSTR >> i) & 1
        tms = 1 if i == IR_LEN - 1 else 0
        seq.append((tms, tdi))

    # Exit1-IR → Update-IR
    seq.append((1, 0))

    # Update-IR → Select-DR-Scan
    seq.append((1, 0))

    # Select-DR-Scan → Capture-DR
    seq.append((0, 0))

    # Capture-DR → Shift-DR (first shift cycle)
    seq.append((0, 0))

    # Shift-DR: UID_BITS bits, TDI=0, TDO captured.
    # TMS=0 to stay in Shift-DR; TMS=1 on the last bit to exit to Exit1-DR.
    for i in range(UID_BITS):
        tms = 1 if i == UID_BITS - 1 else 0
        seq.append((tms, 0))

    # Exit1-DR → Update-DR
    seq.append((1, 0))

    # Update-DR → Run-Test/Idle
    seq.append((0, 0))

    return seq


_JTAG_SEQ = _build_jtag_seq()

# Index of the first Shift-DR cycle within _JTAG_SEQ:
#   5 (reset) + 1 (RTI) + 1 (sel-DR) + 1 (sel-IR) + 1 (cap-IR)
#   + IR_LEN (shift-IR) + 1 (upd-IR) + 1 (sel-DR) + 1 (cap-DR) + 1 (shift-DR first)
_TDO_START  = 5 + 1 + 1 + 1 + 1 + IR_LEN + 1 + 1 + 1 + 1
_TDO_END    = _TDO_START + UID_BITS      # exclusive
_SEQ_TOTAL  = len(_JTAG_SEQ)


# ── Module ────────────────────────────────────────────────────────────────────

class Ti60CallHome(Elaboratable):
    """Efinix Ti60 F225 call-home sequencer.

    Parameters
    ----------
    clk_freq : int
        System clock frequency in Hz (e.g. 100_000_000 for 100 MHz).
    baud : int
        UART baud rate (default 115200).
    ack_timeout : int or None
        Maximum cycles to wait for the 0xCE 0x22 ACK in WAIT_ACK before
        asserting callhome_done anyway so the board boots standalone.
        Defaults to clk_freq * 2 (2 seconds at 100 MHz).  Pass 0 to
        disable the timeout (wait forever — original behaviour).

    Ports
    -----
    uart_tx : Signal, out, init=1
        UART TX pin — connect to board's UART TX output.
    uart_rx : Signal, in, init=1
        UART RX pin — connect to board's UART RX input.
    callhome_done : Signal, out
        Asserted once the call-home sequence is complete (ACK received or
        ACK timeout elapsed).  Use this to gate the normal boot FSM.
    """

    def __init__(self, clk_freq=100_000_000, baud=115200, ack_timeout=None):
        self.clk_freq    = clk_freq
        self.baud        = baud
        self.ack_timeout = clk_freq * 2 if ack_timeout is None else ack_timeout

        self.uart_tx       = Signal(init=1)
        self.uart_rx       = Signal(init=1)
        self.callhome_done = Signal()

    def elaborate(self, platform):
        m = Module()

        # ── UART submodules ───────────────────────────────────────────────────
        m.submodules.tx = tx = UartTx(self.clk_freq, self.baud)
        m.submodules.rx = rx = UartRx(self.clk_freq, self.baud)

        m.d.comb += [
            self.uart_tx.eq(tx.tx),
            rx.rx.eq(self.uart_rx),
        ]

        # ── JTAG primitive signals ────────────────────────────────────────────
        jtag_tck = Signal()
        jtag_tms = Signal(init=1)   # idle high = TAP reset
        jtag_tdi = Signal()
        jtag_tdo = Signal()

        # Efinix EFX_USR_JTAG — connects fabric JTAG master to the Ti60 TAP.
        # i_TCK, i_TMS, i_TDI: fabric drives the TAP clock and control.
        # o_TDO: TAP drives data back to the fabric during DR shifts.
        # See: Efinix Titanium FPGA Primitive Guide, "User JTAG" section.
        m.submodules.efx_usr_jtag = Instance("EFX_USR_JTAG",
            i_TCK = jtag_tck,
            i_TMS = jtag_tms,
            i_TDI = jtag_tdi,
            o_TDO = jtag_tdo,
        )

        # ── JTAG sequence ROM ─────────────────────────────────────────────────
        tms_rom = Array([C(tms, 1) for tms, _   in _JTAG_SEQ])
        tdi_rom = Array([C(tdi, 1) for _,   tdi in _JTAG_SEQ])

        # ── TCK generator ─────────────────────────────────────────────────────
        # tck_ctr counts 0 .. TCK_DIV-1 and wraps.
        # TCK is HIGH when tck_ctr >= TCK_DIV//2.
        # Rising edge: tck_ctr transitions to TCK_DIV//2 (first high cycle).
        # Falling edge: tck_ctr wraps to 0 (first low cycle).
        #
        # Initialise at TCK_DIV//2 so that TCK is HIGH at power-up and the
        # first falling edge (which advances the sequencer) happens only after
        # a full half-period.  This guarantees step 0 (first reset TMS=1 pulse)
        # is seen at a rising edge before the step counter advances.
        tck_ctr = Signal(range(TCK_DIV), init=TCK_DIV // 2)
        m.d.sync += tck_ctr.eq(Mux(tck_ctr == TCK_DIV - 1, 0, tck_ctr + 1))

        tck_high  = tck_ctr >= TCK_DIV // 2
        tck_rise  = (tck_ctr == TCK_DIV // 2)   # first cycle of high phase
        tck_fall  = (tck_ctr == 0)               # first cycle of low phase
        m.d.comb += jtag_tck.eq(tck_high)

        # ── JTAG sequencer ────────────────────────────────────────────────────
        jtag_step = Signal(range(_SEQ_TOTAL))
        jtag_done = Signal()

        # Drive TMS/TDI from ROM at the current step.
        # We advance the step on each falling edge so the new TMS/TDI values
        # are stable for the full preceding high phase (ample setup time).
        m.d.comb += [
            jtag_tms.eq(Mux(jtag_done, 1, tms_rom[jtag_step])),
            jtag_tdi.eq(Mux(jtag_done, 0, tdi_rom[jtag_step])),
        ]

        with m.If(~jtag_done & tck_fall):
            with m.If(jtag_step == _SEQ_TOTAL - 1):
                m.d.sync += jtag_done.eq(1)
            with m.Else():
                m.d.sync += jtag_step.eq(jtag_step + 1)

        # ── UID shift register ────────────────────────────────────────────────
        # Sample TDO on each rising edge during the Shift-DR phase.
        # New bits are shifted into the MSB so that after UID_BITS shifts:
        #   uid[63] = bit 63 (last received = MSB of serial)
        #   uid[0]  = bit 0  (first received = LSB of serial)
        # uid.word_select(N, 8) then gives byte N in natural (LSB-first) order.
        uid = Signal(UID_BITS)

        tdo_phase = Signal()
        m.d.comb += tdo_phase.eq(
            (jtag_step >= _TDO_START) & (jtag_step < _TDO_END) & ~jtag_done
        )

        with m.If(tck_rise & tdo_phase):
            m.d.sync += uid.eq(Cat(uid[1:], jtag_tdo))

        # ── Call-home packet sender ───────────────────────────────────────────
        pkt_idx  = Signal(range(PKT_LEN + 1))
        pkt_sent = Signal()

        pkt_hdr_rom = Array([C(b, 8) for b in PKT_HEADER])
        uid_byte_idx = Signal(3)           # which byte of uid (0-7)
        m.d.comb += uid_byte_idx.eq(pkt_idx - 9)

        pkt_byte = Signal(8)
        with m.If(pkt_idx <= 8):
            m.d.comb += pkt_byte.eq(pkt_hdr_rom[pkt_idx])
        with m.Elif(pkt_idx <= 16):
            m.d.comb += pkt_byte.eq(uid.word_select(uid_byte_idx, 8))
        with m.Else():
            m.d.comb += pkt_byte.eq(0)    # bytes 17-22: all zero

        # ── ACK receiver ─────────────────────────────────────────────────────
        # Wait for two bytes: 0xCE then 0x22 on UART RX.
        ack_want_ce  = Signal(init=1)     # waiting for first byte (0xCE)
        ack_want_22  = Signal()           # received 0xCE, waiting for 0x22
        ack_received = Signal()

        with m.If(rx.valid & ~ack_received):
            with m.If(ack_want_ce):
                with m.If(rx.data == 0xCE):
                    m.d.sync += [ack_want_ce.eq(0), ack_want_22.eq(1)]
            with m.Elif(ack_want_22):
                with m.If(rx.data == 0x22):
                    m.d.sync += [ack_want_22.eq(0), ack_received.eq(1)]
                with m.Else():
                    # Not 0x22 after 0xCE — restart match
                    m.d.sync += [ack_want_ce.eq(1), ack_want_22.eq(0)]

        m.d.comb += self.callhome_done.eq(ack_received | ack_timed_out)

        # ── ACK timeout counter ───────────────────────────────────────────────
        # Counts up from 0 while in WAIT_ACK.  If ack_timeout > 0 and the
        # counter reaches ack_timeout, callhome_done is asserted anyway so
        # the board boots even when no host bridge is present.
        ack_timeout_cnt = Signal(range(max(self.ack_timeout + 1, 1)))
        ack_timed_out   = Signal()

        # ── Main FSM ──────────────────────────────────────────────────────────
        with m.FSM(name="callhome_fsm"):

            with m.State("JTAG_READ"):
                # Wait for the JTAG sequencer to finish reading the UID.
                with m.If(jtag_done):
                    m.d.sync += pkt_idx.eq(0)
                    m.next = "SEND_PKT"

            with m.State("SEND_PKT"):
                # Send packet bytes one at a time, advancing pkt_idx after
                # each successful handoff to the UART TX.
                with m.If(~tx.busy & ~pkt_sent):
                    with m.If(pkt_idx < PKT_LEN):
                        m.d.comb += [
                            tx.data.eq(pkt_byte),
                            tx.start.eq(1),
                        ]
                        m.d.sync += pkt_idx.eq(pkt_idx + 1)
                    with m.Else():
                        m.d.sync += pkt_sent.eq(1)
                        m.next = "WAIT_TX_DONE"

            with m.State("WAIT_TX_DONE"):
                # Drain any in-flight byte before moving to ACK wait.
                with m.If(~tx.busy):
                    m.d.sync += ack_timeout_cnt.eq(0)
                    m.next = "WAIT_ACK"

            with m.State("WAIT_ACK"):
                # Wait for the host bridge to send 0xCE 0x22 ACK.
                # If ack_timeout > 0 and the timer fires, proceed to DONE
                # anyway so the board boots standalone (no host required).
                with m.If(ack_received):
                    m.next = "DONE"
                with m.Elif(self.ack_timeout > 0):
                    with m.If(ack_timeout_cnt + 1 >= self.ack_timeout):
                        m.d.sync += ack_timed_out.eq(1)
                        m.next = "DONE"
                    with m.Else():
                        m.d.sync += ack_timeout_cnt.eq(ack_timeout_cnt + 1)

            with m.State("DONE"):
                # callhome_done remains asserted; top-level unblocks boot FSM.
                pass

        return m
