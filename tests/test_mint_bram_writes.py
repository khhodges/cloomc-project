"""
tests/test_mint_bram_writes.py

Simulate the Mint FSM inside ChurchCore(iot_profile=True) and verify that
the correct 3-word NS entry and E-GT are actually written to the NS and
clist BRAMs by the hardware FSM.

test_mint_bram_writes
  - Instantiates ChurchCoreMintHarness, which wraps ChurchCore(iot_profile=True)
    with a stub IMEM, a combinatorial DMEM (LibMemory), a separate NS memory,
    and a separate clist memory.
  - The harness exposes the live NS and clist write buses so the test can
    observe write events (proxy for mint_call arrival and MINT_DONE sequencing)
    as well as read back BRAM contents after the FSM completes.
  - Fires the outform FSM via the test-injection port (outform_start_in).
  - Drives the IoT protocol: connect byte + lean header + 256-byte lump payload.
  - Waits for outform_busy to de-assert, then checks both:
      * Live write events (NS writes appeared → Mint FSM processed mint_call)
      * BRAM readback:
          ns[slot*3 + 0] == alloc_base   (lump base pointer)
          ns[slot*3 + 1] == W2           (gt_seq=1, limit_offset=63)
          ns[slot*3 + 2] == CRC-16 seal  (CCITT-FALSE over e_gt||base||w2)
          clist[caller]  == E-GT         (E-perm, Inform, seq=1, slot_id)
"""

import sys
import os
import struct
import zlib

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from amaranth import *
from amaranth.lib.memory import Memory as LibMemory
from amaranth.sim import Simulator

from hardware.core import ChurchCore
from hardware.outform_iot import TUNNEL_REQ_LEN
from hardware.hw_types import GT_TYPE_INFORM, CRC16_POLY, CRC16_INIT

MAX_TICKS = 20_000


# ── Reference computations (mirror core.py Mint FSM formulas) ─────────────────

def ref_e_gt(slot_id: int) -> int:
    """E-GT: perms=E(bit30) | typ=Inform(01<<23) | gt_seq=1(<<16) | slot_id."""
    return (1 << 30) | (GT_TYPE_INFORM << 23) | (1 << 16) | (slot_id & 0xFFFF)


def ref_w2(lump_size_words: int) -> int:
    """W2: gt_seq=1(<<21) | limit_offset=(lump_size-1)[20:0]."""
    return (1 << 21) | ((lump_size_words - 1) & 0x1FFFFF)


def ref_crc16(e_gt: int, base: int, w2: int) -> int:
    """CRC16/CCITT-FALSE over e_gt[24:0] || base[31:0] || w2[31:0], MSB-first."""
    crc  = CRC16_INIT & 0xFFFF
    bits = []
    for i in range(24, -1, -1):
        bits.append((e_gt >> i) & 1)
    for i in range(31, -1, -1):
        bits.append((base >> i) & 1)
    for i in range(31, -1, -1):
        bits.append((w2 >> i) & 1)
    for bit in bits:
        top = (crc >> 15) & 1
        crc = ((crc << 1) & 0xFFFF) ^ (CRC16_POLY if (top ^ bit) else 0)
    return crc & 0xFFFF


# ── Lump construction ─────────────────────────────────────────────────────────

def build_lump_payload():
    """256-byte lump: magic=0x1F, n_minus_6=0, cw=2, typ=0, cc=0.

    Layout (little-endian words):
      word[0]     : header — magic=0x1F, n_minus_6=0, cw=2, typ=0, cc=0
      word[1..2]  : non-zero code words
      word[3..63] : zero free-space (MINT_SCAN_FS must see 0)
    """
    header_word = (0x1F << 27) | (0 << 23) | (2 << 10) | (0 << 8) | 0
    payload  = struct.pack("<I", header_word)
    payload += struct.pack("<I", 0x00000001)
    payload += struct.pack("<I", 0x00000002)
    payload += bytes(64 * 4 - 3 * 4)
    assert len(payload) == 256
    return payload


def build_lean_header(payload: bytes) -> bytes:
    """8-byte IoT tunnel header: payload_len (4 B LE) + CRC-32 (4 B LE)."""
    crc = zlib.crc32(payload) & 0xFFFFFFFF
    return struct.pack("<II", len(payload), crc)


# ── Harness ───────────────────────────────────────────────────────────────────

class ChurchCoreMintHarness(Elaboratable):
    """Wraps ChurchCore(iot_profile=True) with stub IMEM, combinatorial DMEM,
    a separate NS memory, and a separate clist memory.

    The harness wires the NS and clist write buses from ChurchCore into actual
    LibMemory instances so the test can read back the written values after the
    Mint FSM has completed.  The live write buses are also surfaced as output
    signals so the test can observe write events directly (acting as a proxy
    for mint_call arrival and MINT_DONE sequencing without modifying ChurchCore's
    public interface).

    DMEM layout (word-addressed):
      words 0..255   : zero-initialised (not used by Mint FSM in this harness)
      words 256..511 : lump payload written by outform during simulation

    NS memory (192 words = 64 slots × 3 words):
      slot s → word indices 3s, 3s+1, 3s+2
      (When cr15.word1_location=0 at reset, mint_ns_entry_base = slot_id*12)

    clist memory (64 words):
      E-GT written at word index core.clist_addr >> 2
    """

    DMEM_DEPTH  = 2048
    NS_DEPTH    = 192
    CLIST_DEPTH = 64

    def __init__(self, slot_id: int, clist_slot_baddr: int):
        self._slot_id           = slot_id
        self._clist_slot_baddr  = clist_slot_baddr

        # Outform I/O (drive from test)
        self.outform_start = Signal()
        self.rx_valid      = Signal()
        self.rx_data       = Signal(8)
        self.tx_ack        = Signal()

        # Outform outputs (observe from test)
        self.tx_valid      = Signal()
        self.tx_data       = Signal(8)
        self.outform_busy  = Signal()

        # Live NS write bus (proxy for mint_call / MINT_WRITE_NS0/1/2 events)
        self.ns_wr_en   = Signal()
        self.ns_wr_addr = Signal(32)
        self.ns_wr_data = Signal(32)

        # Live clist write bus (proxy for MINT_WRITE_CLIST event)
        self.clist_wr_en   = Signal()
        self.clist_wr_addr = Signal(32)
        self.clist_wr_data = Signal(32)

        # NS memory readback ports
        self.ns_rd_addr = Signal(range(self.NS_DEPTH))
        self.ns_rd_data = Signal(32)

        # clist memory readback ports
        self.clist_rd_addr = Signal(range(self.CLIST_DEPTH))
        self.clist_rd_data = Signal(32)

    def elaborate(self, platform):
        m = Module()

        core = ChurchCore(iot_profile=True)
        m.submodules.core = core

        # ── Stub IMEM ─────────────────────────────────────────────────────────
        # The CPU is never booted; imem_valid=0 suppresses all instruction fetch.
        m.d.comb += [
            core.imem_data .eq(0),
            core.imem_valid.eq(0),
        ]

        # ── DMEM (combinatorial read port; outform writes, Mint reads) ────────
        dmem = LibMemory(shape=unsigned(32), depth=self.DMEM_DEPTH,
                         init=[0] * self.DMEM_DEPTH)
        m.submodules.dmem = dmem

        dmem_rd = dmem.read_port(domain="comb")
        dmem_wr = dmem.write_port()

        m.d.comb += [
            dmem_rd.addr     .eq(core.dmem_addr[2:]),
            core.dmem_rd_data.eq(dmem_rd.data),
            dmem_wr.addr     .eq(core.dmem_addr[2:]),
            dmem_wr.data     .eq(core.dmem_wr_data),
            dmem_wr.en       .eq(core.dmem_wr_en),
        ]

        # ── NS memory (separate from DMEM; receives Mint NS writes) ───────────
        ns_mem = LibMemory(shape=unsigned(32), depth=self.NS_DEPTH,
                           init=[0] * self.NS_DEPTH)
        m.submodules.ns_mem = ns_mem

        ns_rd = ns_mem.read_port(domain="comb")
        ns_wr = ns_mem.write_port()

        m.d.comb += [
            ns_wr.addr      .eq(core.ns_addr[2:]),
            ns_wr.data      .eq(core.ns_wr_data[:32]),
            ns_wr.en        .eq(core.ns_wr_en),
            ns_rd.addr      .eq(self.ns_rd_addr),
            self.ns_rd_data .eq(ns_rd.data),
            # Mint FSM never reads back NS; tie the read bus to silence warnings.
            core.ns_rd_data .eq(0),
        ]

        # Surface live NS write signals so the test can observe Mint sequencing.
        m.d.comb += [
            self.ns_wr_en  .eq(core.ns_wr_en),
            self.ns_wr_addr.eq(core.ns_addr),
            self.ns_wr_data.eq(core.ns_wr_data[:32]),
        ]

        # ── clist memory (separate; receives Mint clist writes) ────────────────
        cl_mem = LibMemory(shape=unsigned(32), depth=self.CLIST_DEPTH,
                           init=[0] * self.CLIST_DEPTH)
        m.submodules.cl_mem = cl_mem

        cl_rd = cl_mem.read_port(domain="comb")
        cl_wr = cl_mem.write_port()

        m.d.comb += [
            cl_wr.addr         .eq(core.clist_addr[2:]),
            cl_wr.data         .eq(core.clist_wr_data),
            cl_wr.en           .eq(core.clist_wr_en),
            cl_rd.addr         .eq(self.clist_rd_addr),
            self.clist_rd_data .eq(cl_rd.data),
            # Mint FSM never reads back clist; tie the read bus to silence warnings.
            core.clist_rd_data .eq(0),
        ]

        # Surface live clist write signals for Mint sequencing observation.
        m.d.comb += [
            self.clist_wr_en  .eq(core.clist_wr_en),
            self.clist_wr_addr.eq(core.clist_addr),
            self.clist_wr_data.eq(core.clist_wr_data),
        ]

        # ── Outform wiring ────────────────────────────────────────────────────
        m.d.comb += [
            core.outform_start_in      .eq(self.outform_start),
            core.outform_slot_id_in    .eq(self._slot_id),
            core.outform_clist_addr_in .eq(self._clist_slot_baddr),
            core.outform_gt_raw_in     .eq(0),
            core.outform_rx_valid      .eq(self.rx_valid),
            core.outform_rx_data       .eq(self.rx_data),
            core.outform_tx_ack        .eq(self.tx_ack),
            self.tx_valid              .eq(core.outform_tx_valid),
            self.tx_data               .eq(core.outform_tx_data),
            self.outform_busy          .eq(core.outform_busy),
        ]

        # Stub unused inputs
        m.d.comb += [
            core.boot_start    .eq(0),
            core.gc_start      .eq(0),
            core.free_run_start.eq(0),
            core.free_run_nia  .eq(0),
        ]

        return m


# ── Simulation helpers ────────────────────────────────────────────────────────

async def send_byte(ctx, dut, byte: int):
    ctx.set(dut.rx_valid, 1)
    ctx.set(dut.rx_data,  int(byte))
    await ctx.tick()
    ctx.set(dut.rx_valid, 0)
    await ctx.tick()


async def send_bytes(ctx, dut, data: bytes):
    for b in data:
        await send_byte(ctx, dut, b)


async def ack_tx_bytes(ctx, dut, count: int):
    """Ack `count` outform TX bytes, polling tx_valid before each ack."""
    acked = 0
    for _ in range(MAX_TICKS):
        if ctx.get(dut.tx_valid):
            ctx.set(dut.tx_ack, 1)
            await ctx.tick()
            ctx.set(dut.tx_ack, 0)
            await ctx.tick()
            acked += 1
            if acked >= count:
                return
        else:
            await ctx.tick()
    raise AssertionError(f"ack_tx_bytes: timed out after acking {acked}/{count}")


async def wait_for_alloc(ctx, dut, extra_ticks: int = 6):
    """Wait for the watermark allocator to resolve and the FSM to enter RECV_PAYLOAD.

    After the last lean-header byte the outform FSM transitions:
      LEAN_HDR → DERIVE_N (1 cycle) → ALLOC (alloc_done is combinatorial in
      ChurchCore, so it fires and is seen in the same cycle) → RECV_PAYLOAD.

    Two cycles would technically suffice, but a small fixed margin guards against
    any additional registered pipeline stages without requiring a dedicated
    observable alloc_done_out signal on ChurchCore's public interface.
    """
    for _ in range(extra_ticks):
        await ctx.tick()


async def drain_ns_clist_writes(ctx, dut):
    """Poll until outform_busy de-asserts, capturing every NS and clist write.

    Capturing writes while polling serves two purposes:
      1. Verifies that the Mint FSM actually issued NS writes (proxy for
         mint_call being processed and the FSM advancing through WRITE_NS0/1/2).
      2. Verifies that a clist write occurred (proxy for MINT_WRITE_CLIST /
         MINT_DONE sequence completing before outform_busy dropped).

    Returns (ns_writes, clist_writes) dicts mapping byte-address → data.
    """
    ns_writes    = {}
    clist_writes = {}
    for _ in range(MAX_TICKS):
        await ctx.tick()
        if ctx.get(dut.ns_wr_en):
            addr = ctx.get(dut.ns_wr_addr)
            data = ctx.get(dut.ns_wr_data)
            ns_writes[addr] = data
        if ctx.get(dut.clist_wr_en):
            addr = ctx.get(dut.clist_wr_addr)
            data = ctx.get(dut.clist_wr_data)
            clist_writes[addr] = data
        if not ctx.get(dut.outform_busy):
            return ns_writes, clist_writes
    raise AssertionError(
        f"drain_ns_clist_writes: timed out; "
        f"ns_writes={ns_writes}  clist_writes={clist_writes}"
    )


# ── Main test ─────────────────────────────────────────────────────────────────

def test_mint_bram_writes():
    """ChurchCore(iot_profile=True) Mint FSM writes correct NS entry and E-GT.

    Procedure
    ---------
    1. Fire the outform via outform_start_in (test injection, no CPU needed).
    2. Ack the 6-byte tunnel-request TX burst.
    3. Send connect byte + 8-byte lean header.
    4. Wait for the watermark allocator (automatic, combinatorial in ChurchCore).
    5. Send 256-byte lump payload; outform writes it into DMEM.
    6. Poll ticks, capturing NS/clist write events until outform_busy de-asserts.
       - NS write events confirm mint_call was processed (MINT_WRITE_NS0→NS1→NS2).
       - clist write event confirms MINT_WRITE_CLIST / MINT_DONE completed.
    7. Assert captured live-write values match references.
    8. Read back from NS and clist memories (actual BRAM contents post-Mint).
    9. Assert BRAM readback matches the same references.

    Assertions (both live-write and BRAM readback paths)
    -----------------------------------------------------
    ns[slot*3+0]  == alloc_base               (lump pointer)
    ns[slot*3+1]  == W2 = (1<<21)|(63)        (gt_seq=1, limit_offset=63)
    ns[slot*3+2]  == CRC16 seal               (CCITT-FALSE, 89-bit input)
    clist[caller] == E-GT                      (E-perm, Inform, seq=1, slot)
    """
    SLOT_ID         = 0
    # clist byte address for the caller's slot; word index = CLIST_BADDR >> 2.
    CLIST_BADDR     = 4    # byte 4 → word 1 in clist memory
    LUMP_WORDS      = 64   # 2^(n_minus_6=0 + 6)
    WATERMARK_INIT  = 256  # first free DMEM word in ChurchCore iot_profile
    ALLOC_BASE      = WATERMARK_INIT * 4   # = 0x400 (1024 bytes)

    payload  = build_lump_payload()
    lean_hdr = build_lean_header(payload)

    dut = ChurchCoreMintHarness(slot_id=SLOT_ID, clist_slot_baddr=CLIST_BADDR)

    async def process(ctx):
        # ── Fire outform FSM via test-injection port ──────────────────────────
        ctx.set(dut.outform_start, 1)
        await ctx.tick()
        ctx.set(dut.outform_start, 0)

        # ── Ack the 6-byte tunnel-request TX burst ────────────────────────────
        await ack_tx_bytes(ctx, dut, TUNNEL_REQ_LEN)

        # ── Send connect byte + lean header ───────────────────────────────────
        await send_byte(ctx, dut, 0xAC)      # connect byte
        await send_bytes(ctx, dut, lean_hdr)

        # ── Wait for allocator + DERIVE_N → RECV_PAYLOAD transition ──────────
        await wait_for_alloc(ctx, dut)

        # ── Send 256-byte payload (outform writes it into DMEM) ───────────────
        await send_bytes(ctx, dut, payload)

        # ── Poll until outform_busy de-asserts, capturing Mint write events ───
        # Live write captures serve as sequencing verification:
        #   - Any ns_wr_en pulse confirms the Mint FSM received mint_call and
        #     advanced past MINT_READ_HDR / MINT_CHECK_HDR / MINT_SCAN_FS into
        #     MINT_WRITE_NS0 / MINT_WRITE_NS1 / MINT_WRITE_NS2.
        #   - Any clist_wr_en pulse at CLIST_BADDR confirms MINT_WRITE_CLIST
        #     fired and MINT_DONE was reached (after which mint_done=1 drives
        #     the outform FSM to COMPLETE → outform_busy=0).
        ns_writes, clist_writes = await drain_ns_clist_writes(ctx, dut)

        # ── Compute reference values ──────────────────────────────────────────
        exp_e_gt = ref_e_gt(SLOT_ID)
        exp_ns0  = ALLOC_BASE
        exp_ns1  = ref_w2(LUMP_WORDS)
        exp_ns2  = ref_crc16(exp_e_gt, ALLOC_BASE, exp_ns1)

        # NS entry byte addresses for slot_id=0 with cr15.word1_location=0:
        #   mint_ns_entry_base = 0 + (0<<3) + (0<<2) = 0
        ns_ba0 = 0 + SLOT_ID * 12 + 0   # = 0
        ns_ba1 = 0 + SLOT_ID * 12 + 4   # = 4
        ns_ba2 = 0 + SLOT_ID * 12 + 8   # = 8

        # ── Assertions on live write-event captures ───────────────────────────
        assert len(ns_writes) == 3, (
            f"Expected 3 NS writes (WRITE_NS0/1/2), got {len(ns_writes)}: {ns_writes}"
        )
        assert ns_ba0 in ns_writes, (
            f"No NS write at byte addr {ns_ba0}; got: {ns_writes}"
        )
        assert ns_ba1 in ns_writes, (
            f"No NS write at byte addr {ns_ba1}; got: {ns_writes}"
        )
        assert ns_ba2 in ns_writes, (
            f"No NS write at byte addr {ns_ba2}; got: {ns_writes}"
        )
        assert ns_writes[ns_ba0] == exp_ns0, (
            f"live ns[0]=0x{ns_writes[ns_ba0]:08X}  expected=0x{exp_ns0:08X}"
        )
        assert ns_writes[ns_ba1] == exp_ns1, (
            f"live ns[1]=0x{ns_writes[ns_ba1]:08X}  expected W2=0x{exp_ns1:08X}"
        )
        assert ns_writes[ns_ba2] == exp_ns2, (
            f"live ns[2]=0x{ns_writes[ns_ba2]:04X}  expected CRC16=0x{exp_ns2:04X}"
        )
        assert CLIST_BADDR in clist_writes, (
            f"No clist write at byte addr {CLIST_BADDR}; got: {clist_writes}"
        )
        assert clist_writes[CLIST_BADDR] == exp_e_gt, (
            f"live clist=0x{clist_writes[CLIST_BADDR]:08X}  expected=0x{exp_e_gt:08X}"
        )

        # ── Allow one extra tick so memory read port reflects the last write ──
        await ctx.tick()

        # ── Read NS memory (BRAM readback) ────────────────────────────────────
        ns_word0_idx = ns_ba0 >> 2   # = 0
        ns_word1_idx = ns_ba1 >> 2   # = 1
        ns_word2_idx = ns_ba2 >> 2   # = 2

        ctx.set(dut.ns_rd_addr, ns_word0_idx)
        await ctx.tick()
        ns_word0 = ctx.get(dut.ns_rd_data)

        ctx.set(dut.ns_rd_addr, ns_word1_idx)
        await ctx.tick()
        ns_word1 = ctx.get(dut.ns_rd_data)

        ctx.set(dut.ns_rd_addr, ns_word2_idx)
        await ctx.tick()
        ns_word2 = ctx.get(dut.ns_rd_data)

        # ── Read clist memory (BRAM readback) ─────────────────────────────────
        ctx.set(dut.clist_rd_addr, CLIST_BADDR >> 2)
        await ctx.tick()
        clist_e_gt = ctx.get(dut.clist_rd_data)

        # ── Assertions on BRAM readback contents ─────────────────────────────
        assert ns_word0 == exp_ns0, (
            f"bram ns[slot*3+0]=0x{ns_word0:08X}  expected alloc_base=0x{exp_ns0:08X}"
        )
        assert ns_word1 == exp_ns1, (
            f"bram ns[slot*3+1]=0x{ns_word1:08X}  expected W2=0x{exp_ns1:08X}"
        )
        assert ns_word2 == exp_ns2, (
            f"bram ns[slot*3+2]=0x{ns_word2:04X}  expected CRC16=0x{exp_ns2:04X}"
        )
        assert clist_e_gt == exp_e_gt, (
            f"bram clist[caller]=0x{clist_e_gt:08X}  expected E-GT=0x{exp_e_gt:08X}"
        )

        # E-GT structural checks
        assert (clist_e_gt >> 30) & 1, (
            f"E-GT bit30 (perms=E) not set: 0x{clist_e_gt:08X}"
        )
        assert ((clist_e_gt >> 23) & 0x3) == GT_TYPE_INFORM, (
            f"E-GT type={(( clist_e_gt >> 23) & 3)} != GT_TYPE_INFORM={GT_TYPE_INFORM}"
        )
        assert (clist_e_gt & 0xFFFF) == SLOT_ID, (
            f"E-GT slot_id={(clist_e_gt & 0xFFFF)} != SLOT_ID={SLOT_ID}"
        )

        print(
            f"\n  [live write events]\n"
            f"    ns write @  0: 0x{ns_writes[ns_ba0]:08X}  (alloc_base)\n"
            f"    ns write @  4: 0x{ns_writes[ns_ba1]:08X}  "
            f"(W2: gt_seq=1, limit_offset={LUMP_WORDS-1})\n"
            f"    ns write @  8: 0x{ns_writes[ns_ba2]:04X}      (CRC16 seal)\n"
            f"    clist write @ {CLIST_BADDR}: 0x{clist_writes[CLIST_BADDR]:08X}  "
            f"(E-GT: E-perm, Inform, seq=1, slot={SLOT_ID})\n"
            f"\n  [BRAM readback]\n"
            f"    ns[slot*3+0]         = 0x{ns_word0:08X}  (alloc_base)\n"
            f"    ns[slot*3+1]  (W2)   = 0x{ns_word1:08X}  "
            f"(gt_seq=1, limit_offset={LUMP_WORDS-1})\n"
            f"    ns[slot*3+2]  (W3)   = 0x{ns_word2:04X}      (CRC16 seal)\n"
            f"    clist[caller] (E-GT) = 0x{clist_e_gt:08X}  "
            f"(E-perm, Inform, seq=1, slot={SLOT_ID})"
        )

    sim = Simulator(dut)
    sim.add_clock(1e-6)
    sim.add_testbench(process)
    with sim.write_vcd("/tmp/mint_bram_writes.vcd"):
        sim.run()
    print("PASS: test_mint_bram_writes")


if __name__ == "__main__":
    test_mint_bram_writes()
    print("\nAll ChurchCore Mint BRAM write tests passed.")
