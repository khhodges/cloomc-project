"""hardware/outform.py — ChurchOutform: Absent-Outform lazy-load handler.

Triggered when mLoad encounters a c-list Golden Token whose typ field == 11
(Outform), indicating the lump is not yet resident in physical memory.

Protocol (all confirmed in task spec):
  1. Send a tunnel request: 6 bytes (GT raw 32-bit LE, slot_id 16-bit LE).
  2. Receive a ZIP Local File Header (30 bytes fixed, field-by-field).
  3. Validate: signature, flags bit-3, compression method, uncompressed size.
  4. Derive n from uncompressed_size (must be power-of-2 multiple of 4 words,
     6 <= n <= 14).
  5. Request memory allocation (size = 2^n words).
  6. Skip filename (L bytes, independent counter).
  7. Skip extra field (E bytes, independent counter — never combined with L).
  8. Inflate data into allocated base (STORE: direct copy; DEFLATE/RLE: stubbed).
  9. Verify CRC-32 of inflated data against ZIP header value.
 10. Call Mint.Lump(base, n) via single-cycle pulse; await mint_done / mint_fault.

Out of scope:
  - DEFLATE Huffman+LZ77 internals (stubbed; separate hardware task).
  - RLE internals (stubbed; separate hardware task).
  - Mint.Lump internal implementation (called via interface ports).
  - Integration into core.py.

Fault types defined at module scope (OUTFORM_FAULT_* constants below).
"""

from amaranth import *

# ── Outform-specific fault codes (extend the 5-bit FaultType space) ──────────
OUTFORM_FAULT_SIG    = 0x11   # ZIP signature != 0x04034B50
OUTFORM_FAULT_FLAGS  = 0x12   # General purpose bit 3 = 1 (data descriptor / streaming)
OUTFORM_FAULT_METHOD = 0x13   # Compression method not STORE (0) or DEFLATE (8)
OUTFORM_FAULT_N      = 0x14   # n < 6, n > 14, or size not power-of-2 word count
OUTFORM_FAULT_CRC32  = 0x15   # CRC-32 of inflated data does not match ZIP header
OUTFORM_FAULT_ALLOC  = 0x16   # Memory allocator returned fault
OUTFORM_FAULT_MINT   = 0x17   # Mint.Lump rejected the inflated lump

# ── ZIP constants ─────────────────────────────────────────────────────────────
ZIP_SIGNATURE   = 0x04034B50  # Local File Header magic (little-endian wire: 50 4B 03 04)
METHOD_STORE    = 0           # Compression method: no compression
METHOD_DEFLATE  = 8           # Compression method: DEFLATE (stubbed)

# ZIP LFH fixed region is 30 bytes (indices 0..29).
ZIP_HDR_LEN = 30

# CRC-32 polynomial (IEEE 802.3, bit-reflected)
CRC32_POLY  = 0xEDB88320
CRC32_INIT  = 0xFFFFFFFF     # initial accumulator value
CRC32_FINAL = 0xFFFFFFFF     # final XOR mask

# Tunnel request: 6 bytes — gt_raw[31:0] LE then slot_id[15:0] LE
TUNNEL_REQ_LEN = 6


class ChurchOutform(Elaboratable):
    """Absent-Outform lazy-load handler.

    All tunnel I/O is byte-granular (rx_data / tx_data).
    Memory writes for the inflate path are word-granular (mem_wr_*).
    Mint.Lump is invoked via a single-cycle mint_call pulse; the result GT
    is returned on mint_result_gt (valid when mint_done is asserted).
    """

    def __init__(self):
        # ── trigger / status ─────────────────────────────────────────────────
        self.outform_start      = Signal()    # single-cycle start pulse
        self.outform_busy       = Signal()    # asserted throughout operation
        self.outform_done       = Signal()    # single-cycle completion
        self.outform_fault      = Signal()    # single-cycle fault indication
        self.outform_fault_type = Signal(5)   # 5-bit fault code (OUTFORM_FAULT_*)

        # ── incoming Outform identity ─────────────────────────────────────────
        self.gt_raw  = Signal(32)  # raw 32-bit E-GT from NS entry (tunnel request)
        self.slot_id = Signal(16)  # NS slot index (tunnel request)

        # ── tunnel byte stream ────────────────────────────────────────────────
        # TX: send request bytes to the Home Base IDE.
        self.tx_valid = Signal()   # assert for one cycle per byte
        self.tx_data  = Signal(8)  # byte to transmit

        # RX: receive ZIP response bytes from the IDE.
        self.rx_valid = Signal()   # byte available from tunnel
        self.rx_data  = Signal(8)  # received byte

        # ── memory allocator interface ────────────────────────────────────────
        self.alloc_req   = Signal()    # pulse: request 2^n-word block
        self.alloc_n     = Signal(5)   # log2(words) for allocation request
        self.alloc_done  = Signal()    # allocator has assigned address
        self.alloc_fault = Signal()    # allocator failed
        self.alloc_base  = Signal(32)  # byte base of allocated block

        # ── memory write port (inflate destination) ───────────────────────────
        self.mem_wr_addr = Signal(32)  # byte address (word-aligned)
        self.mem_wr_data = Signal(32)  # 32-bit word to write
        self.mem_wr_en   = Signal()    # write strobe (one cycle per word)

        # ── Mint.Lump interface ───────────────────────────────────────────────
        self.mint_call      = Signal()    # single-cycle pulse: invoke Mint.Lump(base, n)
        self.mint_base      = Signal(32)  # byte base of inflated lump
        self.mint_n         = Signal(5)   # log2(words) = n
        self.mint_done      = Signal()    # Mint completed successfully
        self.mint_fault     = Signal()    # Mint rejected the lump
        self.mint_result_gt = Signal(32)  # E-GT issued by Mint (valid when mint_done=1)

        # ── output from completed load ────────────────────────────────────────
        self.result_gt = Signal(32)   # latched Mint E-GT, valid when outform_done=1

    # ── CRC-32 update — byte-serial, combinatorial, 8 stages ─────────────────
    @staticmethod
    def _crc32_byte(m, crc_in, byte_in, crc_out):
        """Wire an 8-stage combinatorial CRC-32 update: crc_in + byte_in -> crc_out.

        Uses the reflected IEEE 802.3 polynomial 0xEDB88320 (ZIP / Ethernet).
        Bits of byte_in are processed LSB-first. All logic is purely combinatorial;
        the caller is responsible for registering crc_out on the clock edge.
        """
        stage = crc_in
        for i in range(8):
            xb  = Signal(name=f"_crc_xb{i}")
            nxt = Signal(32, name=f"_crc_s{i}")
            m.d.comb += xb.eq(stage[0] ^ byte_in[i])
            m.d.comb += nxt.eq((stage >> 1) ^ Mux(xb, CRC32_POLY, 0))
            stage = nxt
        m.d.comb += crc_out.eq(stage)

    def elaborate(self, platform):
        m = Module()

        # ── Internal registers ────────────────────────────────────────────────
        hdr_byte_cnt  = Signal(5)    # position within 30-byte ZIP fixed header
        tx_byte_cnt   = Signal(3)    # position within 6-byte tunnel request

        # Independent skip counters (NEVER combined in arithmetic)
        fname_cnt     = Signal(16)   # L-counter: bytes remaining in filename
        extra_cnt     = Signal(16)   # E-counter: bytes remaining in extra field

        # ZIP header fields, assembled byte-by-byte (little-endian)
        sig_reg         = Signal(32)  # bytes  0-3  : Local File Header signature
        flags_reg       = Signal(16)  # bytes  6-7  : general purpose bit flag
        method_reg      = Signal(16)  # bytes  8-9  : compression method
        crc32_stored    = Signal(32)  # bytes 14-17 : stored CRC-32 from ZIP header
        comp_size_reg   = Signal(32)  # bytes 18-21 : compressed size (informational)
        ucomp_size_reg  = Signal(32)  # bytes 22-25 : uncompressed size
        fname_len_reg   = Signal(16)  # bytes 26-27 : filename length (L)
        extra_len_reg   = Signal(16)  # bytes 28-29 : extra field length (E)

        # Derived sizing
        word_count_reg  = Signal(32)  # ucomp_size / 4 — number of 32-bit words
        n_reg           = Signal(5)   # log2(word_count), valid range 6..14
        total_words     = Signal(32)  # latched word_count for inflate countdown

        # Memory / inflate state
        base_reg        = Signal(32)  # byte base of allocated block
        wr_word_cnt     = Signal(32)  # words written so far
        byte_buf        = Signal(24)  # accumulates bytes 0..2 of the current word
        byte_buf_cnt    = Signal(2)   # 0..3: how many bytes of current word received

        # CRC-32 accumulator (IEEE 802.3, updated per inflated byte)
        crc_acc         = Signal(32, init=CRC32_INIT)
        crc_next        = Signal(32)  # combinatorial result of _crc32_byte

        # Mint result latch
        result_gt_reg   = Signal(32)

        # ── CRC-32 combinatorial update wiring ────────────────────────────────
        self._crc32_byte(m, crc_acc, self.rx_data, crc_next)

        # ── Tunnel TX byte multiplexor ────────────────────────────────────────
        tx_byte = Signal(8)
        with m.Switch(tx_byte_cnt):
            with m.Case(0): m.d.comb += tx_byte.eq(self.gt_raw[ 0: 8])
            with m.Case(1): m.d.comb += tx_byte.eq(self.gt_raw[ 8:16])
            with m.Case(2): m.d.comb += tx_byte.eq(self.gt_raw[16:24])
            with m.Case(3): m.d.comb += tx_byte.eq(self.gt_raw[24:32])
            with m.Case(4): m.d.comb += tx_byte.eq(self.slot_id[0:8])
            with m.Case(5): m.d.comb += tx_byte.eq(self.slot_id[8:16])
            with m.Default(): m.d.comb += tx_byte.eq(0)

        # ── Constant combinatorial drives ─────────────────────────────────────
        m.d.comb += [
            self.tx_data.eq(tx_byte),
            self.mint_base.eq(base_reg),
            self.mint_n.eq(n_reg),
            self.result_gt.eq(result_gt_reg),
        ]

        # ── Full combinatorial word to write in INFLATE (STORE mode) ──────────
        # When byte_buf_cnt==3 and rx_valid, the incoming rx_data is byte 3
        # (bits [31:24]).  byte_buf holds bytes 0..2 already registered.
        # Build the full word combinatorially so the write is not stale.
        inflate_word = Signal(32)
        m.d.comb += inflate_word.eq(Cat(byte_buf, self.rx_data))

        # ── FSM ───────────────────────────────────────────────────────────────
        with m.FSM(name="outform"):

            # ── IDLE ─────────────────────────────────────────────────────────
            with m.State("IDLE"):
                m.d.comb += self.outform_busy.eq(0)
                with m.If(self.outform_start):
                    m.d.sync += [
                        hdr_byte_cnt  .eq(0),
                        tx_byte_cnt   .eq(0),
                        fname_cnt     .eq(0),
                        extra_cnt     .eq(0),
                        sig_reg       .eq(0),
                        flags_reg     .eq(0),
                        method_reg    .eq(0),
                        crc32_stored  .eq(0),
                        comp_size_reg .eq(0),
                        ucomp_size_reg.eq(0),
                        fname_len_reg .eq(0),
                        extra_len_reg .eq(0),
                        word_count_reg.eq(0),
                        n_reg         .eq(0),
                        total_words   .eq(0),
                        crc_acc       .eq(CRC32_INIT),
                        wr_word_cnt   .eq(0),
                        byte_buf      .eq(0),
                        byte_buf_cnt  .eq(0),
                        result_gt_reg .eq(0),
                    ]
                    m.next = "TUNNEL_CONNECT"

            # ── TUNNEL_CONNECT ────────────────────────────────────────────────
            # Send 6-byte tunnel request (gt_raw LE, slot_id LE) one byte per cycle.
            with m.State("TUNNEL_CONNECT"):
                m.d.comb += [
                    self.outform_busy.eq(1),
                    self.tx_valid.eq(1),
                ]
                with m.If(tx_byte_cnt == TUNNEL_REQ_LEN - 1):
                    m.d.sync += tx_byte_cnt.eq(0)
                    m.next = "RECV_HDR"
                with m.Else():
                    m.d.sync += tx_byte_cnt.eq(tx_byte_cnt + 1)

            # ── RECV_HDR ──────────────────────────────────────────────────────
            # Receive ZIP LFH fixed region (30 bytes) byte-by-byte.
            # Each field is extracted from its canonical ZIP LFH byte offset.
            with m.State("RECV_HDR"):
                m.d.comb += self.outform_busy.eq(1)
                with m.If(self.rx_valid):
                    with m.Switch(hdr_byte_cnt):
                        # Signature: bytes 0-3 (little-endian)
                        with m.Case( 0): m.d.sync += sig_reg[ 0: 8].eq(self.rx_data)
                        with m.Case( 1): m.d.sync += sig_reg[ 8:16].eq(self.rx_data)
                        with m.Case( 2): m.d.sync += sig_reg[16:24].eq(self.rx_data)
                        with m.Case( 3): m.d.sync += sig_reg[24:32].eq(self.rx_data)
                        # General purpose bit flag: bytes 6-7
                        with m.Case( 6): m.d.sync += flags_reg[ 0: 8].eq(self.rx_data)
                        with m.Case( 7): m.d.sync += flags_reg[ 8:16].eq(self.rx_data)
                        # Compression method: bytes 8-9
                        with m.Case( 8): m.d.sync += method_reg[ 0: 8].eq(self.rx_data)
                        with m.Case( 9): m.d.sync += method_reg[ 8:16].eq(self.rx_data)
                        # CRC-32: bytes 14-17
                        with m.Case(14): m.d.sync += crc32_stored[ 0: 8].eq(self.rx_data)
                        with m.Case(15): m.d.sync += crc32_stored[ 8:16].eq(self.rx_data)
                        with m.Case(16): m.d.sync += crc32_stored[16:24].eq(self.rx_data)
                        with m.Case(17): m.d.sync += crc32_stored[24:32].eq(self.rx_data)
                        # Compressed size: bytes 18-21 (informational)
                        with m.Case(18): m.d.sync += comp_size_reg[ 0: 8].eq(self.rx_data)
                        with m.Case(19): m.d.sync += comp_size_reg[ 8:16].eq(self.rx_data)
                        with m.Case(20): m.d.sync += comp_size_reg[16:24].eq(self.rx_data)
                        with m.Case(21): m.d.sync += comp_size_reg[24:32].eq(self.rx_data)
                        # Uncompressed size: bytes 22-25
                        with m.Case(22): m.d.sync += ucomp_size_reg[ 0: 8].eq(self.rx_data)
                        with m.Case(23): m.d.sync += ucomp_size_reg[ 8:16].eq(self.rx_data)
                        with m.Case(24): m.d.sync += ucomp_size_reg[16:24].eq(self.rx_data)
                        with m.Case(25): m.d.sync += ucomp_size_reg[24:32].eq(self.rx_data)
                        # Filename length L: bytes 26-27 (independent from E)
                        with m.Case(26): m.d.sync += fname_len_reg[ 0: 8].eq(self.rx_data)
                        with m.Case(27): m.d.sync += fname_len_reg[ 8:16].eq(self.rx_data)
                        # Extra field length E: bytes 28-29 (independent from L)
                        with m.Case(28): m.d.sync += extra_len_reg[ 0: 8].eq(self.rx_data)
                        with m.Case(29): m.d.sync += extra_len_reg[ 8:16].eq(self.rx_data)
                    m.d.sync += hdr_byte_cnt.eq(hdr_byte_cnt + 1)
                    with m.If(hdr_byte_cnt == ZIP_HDR_LEN - 1):
                        m.next = "CHECK_SIG"

            # ── CHECK_SIG ─────────────────────────────────────────────────────
            # Verify ZIP Local File Header signature == 0x04034B50.
            with m.State("CHECK_SIG"):
                m.d.comb += self.outform_busy.eq(1)
                with m.If(sig_reg != ZIP_SIGNATURE):
                    m.d.sync += self.outform_fault_type.eq(OUTFORM_FAULT_SIG)
                    m.next = "FAULT"
                with m.Else():
                    m.next = "CHECK_FLAGS"

            # ── CHECK_FLAGS ───────────────────────────────────────────────────
            # Bit 3 of general purpose flag = 1 means data descriptor / streaming.
            # Streaming mode is rejected: CRC-32 and sizes must be in the header.
            with m.State("CHECK_FLAGS"):
                m.d.comb += self.outform_busy.eq(1)
                with m.If(flags_reg[3]):
                    m.d.sync += self.outform_fault_type.eq(OUTFORM_FAULT_FLAGS)
                    m.next = "FAULT"
                with m.Else():
                    m.next = "READ_UCSIZE"

            # ── READ_UCSIZE ───────────────────────────────────────────────────
            # Uncompressed size must be divisible by 4 (lump is word-aligned).
            with m.State("READ_UCSIZE"):
                m.d.comb += self.outform_busy.eq(1)
                with m.If(ucomp_size_reg[0:2] != 0):
                    m.d.sync += self.outform_fault_type.eq(OUTFORM_FAULT_N)
                    m.next = "FAULT"
                with m.Else():
                    m.d.sync += word_count_reg.eq(ucomp_size_reg >> 2)
                    m.next = "DERIVE_N"

            # ── DERIVE_N ──────────────────────────────────────────────────────
            # word_count must be a power of 2 in [64, 16384]; n = log2(word_count).
            # Valid: 6 <= n <= 14.
            with m.State("DERIVE_N"):
                m.d.comb += self.outform_busy.eq(1)

                # Power-of-2 check: x > 0 and (x & (x-1)) == 0
                is_pow2 = Signal()
                m.d.comb += is_pow2.eq(
                    (word_count_reg != 0) &
                    ((word_count_reg & (word_count_reg - 1)) == 0)
                )

                # log2 via one-hot match; 2^6=64 .. 2^14=16384
                n_computed = Signal(5)
                with m.Switch(word_count_reg):
                    for bit in range(6, 15):
                        with m.Case(1 << bit):
                            m.d.comb += n_computed.eq(bit)
                    with m.Default():
                        m.d.comb += n_computed.eq(0)

                with m.If(~is_pow2 | (n_computed < 6) | (n_computed > 14)):
                    m.d.sync += self.outform_fault_type.eq(OUTFORM_FAULT_N)
                    m.next = "FAULT"
                with m.Else():
                    m.d.sync += [
                        n_reg.eq(n_computed),
                        total_words.eq(word_count_reg),
                    ]
                    m.next = "ALLOC"

            # ── ALLOC ─────────────────────────────────────────────────────────
            # Assert alloc_req / alloc_n every cycle until alloc_done or alloc_fault.
            with m.State("ALLOC"):
                m.d.comb += [
                    self.outform_busy.eq(1),
                    self.alloc_req.eq(1),
                    self.alloc_n.eq(n_reg),
                ]
                with m.If(self.alloc_fault):
                    m.d.sync += self.outform_fault_type.eq(OUTFORM_FAULT_ALLOC)
                    m.next = "FAULT"
                with m.Elif(self.alloc_done):
                    m.d.sync += [
                        base_reg    .eq(self.alloc_base),
                        wr_word_cnt .eq(0),
                        byte_buf_cnt.eq(0),
                        byte_buf    .eq(0),
                        crc_acc     .eq(CRC32_INIT),
                        fname_cnt   .eq(fname_len_reg),   # prime L-counter
                    ]
                    m.next = "SKIP_FNAME"

            # ── SKIP_FNAME ────────────────────────────────────────────────────
            # Consume exactly L bytes.  fname_cnt is an independent Signal(16).
            # It is NEVER used in extra-field arithmetic at any point.
            with m.State("SKIP_FNAME"):
                m.d.comb += self.outform_busy.eq(1)
                with m.If(fname_cnt == 0):
                    m.d.sync += extra_cnt.eq(extra_len_reg)  # prime E-counter
                    m.next = "SKIP_EXTRA"
                with m.Elif(self.rx_valid):
                    m.d.sync += fname_cnt.eq(fname_cnt - 1)

            # ── SKIP_EXTRA ────────────────────────────────────────────────────
            # Consume exactly E bytes.  extra_cnt is an independent Signal(16).
            # It is NEVER derived from or fed into fname_cnt arithmetic.
            with m.State("SKIP_EXTRA"):
                m.d.comb += self.outform_busy.eq(1)
                with m.If(extra_cnt == 0):
                    m.next = "INFLATE"
                with m.Elif(self.rx_valid):
                    m.d.sync += extra_cnt.eq(extra_cnt - 1)

            # ── INFLATE ───────────────────────────────────────────────────────
            # Dispatch on compression method.
            #
            # STORE (method=0):
            #   Bytes arrive from tunnel; bytes 0..2 are registered into byte_buf.
            #   On byte 3 (byte_buf_cnt==3 AND rx_valid):
            #     - Full word built combinatorially as Cat(byte_buf[0:24], rx_data)
            #     - Word written to memory (no stale-data issue)
            #     - CRC-32 accumulator updated for this byte
            #     - byte_buf_cnt reset to 0
            #   Transition to CHECK_CRC32 after wr_word_cnt+1 == total_words.
            #
            # DEFLATE (method=8):
            #   TODO: full Huffman+LZ77 sub-FSM is a separate hardware task.
            #   Faults immediately to prevent silent data corruption.
            #
            # Other / RLE:
            #   TODO: custom RLE sub-FSM when method code is standardised.
            #   Faults immediately.
            with m.State("INFLATE"):
                m.d.comb += self.outform_busy.eq(1)

                with m.If(method_reg == METHOD_STORE):
                    with m.If(self.rx_valid):
                        # Always update CRC-32 for every incoming byte
                        m.d.sync += crc_acc.eq(crc_next)

                        with m.If(byte_buf_cnt == 3):
                            # Byte 3 of current word — write the complete word now.
                            # inflate_word = Cat(byte_buf[0:24], rx_data) is wired
                            # combinatorially above; mem_wr_data picks it up intact.
                            m.d.comb += [
                                self.mem_wr_addr.eq(base_reg + (wr_word_cnt << 2)),
                                self.mem_wr_data.eq(inflate_word),
                                self.mem_wr_en  .eq(1),
                            ]
                            m.d.sync += [
                                wr_word_cnt .eq(wr_word_cnt + 1),
                                byte_buf_cnt.eq(0),
                                byte_buf    .eq(0),
                            ]
                            with m.If(wr_word_cnt + 1 == total_words):
                                m.next = "CHECK_CRC32"
                        with m.Else():
                            # Bytes 0..2: register into byte_buf
                            with m.Switch(byte_buf_cnt):
                                with m.Case(0):
                                    m.d.sync += byte_buf[ 0: 8].eq(self.rx_data)
                                with m.Case(1):
                                    m.d.sync += byte_buf[ 8:16].eq(self.rx_data)
                                with m.Case(2):
                                    m.d.sync += byte_buf[16:24].eq(self.rx_data)
                            m.d.sync += byte_buf_cnt.eq(byte_buf_cnt + 1)

                with m.Elif(method_reg == METHOD_DEFLATE):
                    # TODO: DEFLATE (Huffman+LZ77) sub-FSM — separate hardware task
                    m.d.sync += self.outform_fault_type.eq(OUTFORM_FAULT_METHOD)
                    m.next = "FAULT"

                with m.Else():
                    # Unsupported method (incl. custom RLE)
                    # TODO: RLE sub-FSM when method code is standardised
                    m.d.sync += self.outform_fault_type.eq(OUTFORM_FAULT_METHOD)
                    m.next = "FAULT"

            # ── CHECK_CRC32 ───────────────────────────────────────────────────
            # Apply final XOR and compare against the stored ZIP CRC-32.
            with m.State("CHECK_CRC32"):
                m.d.comb += self.outform_busy.eq(1)
                crc_final = Signal(32)
                m.d.comb += crc_final.eq(crc_acc ^ CRC32_FINAL)
                with m.If(crc_final != crc32_stored):
                    m.d.sync += self.outform_fault_type.eq(OUTFORM_FAULT_CRC32)
                    m.next = "FAULT"
                with m.Else():
                    m.next = "MINT"

            # ── MINT ─────────────────────────────────────────────────────────
            # Issue a SINGLE-CYCLE pulse on mint_call, then immediately enter
            # MINT_WAIT.  mint_base and mint_n are held stable throughout.
            with m.State("MINT"):
                m.d.comb += [
                    self.outform_busy.eq(1),
                    self.mint_call.eq(1),        # one-cycle pulse only
                ]
                m.next = "MINT_WAIT"

            # ── MINT_WAIT ─────────────────────────────────────────────────────
            # Wait for Mint.Lump to complete or fault.  mint_call is LOW here.
            # Latch the E-GT from mint_result_gt on success.
            with m.State("MINT_WAIT"):
                m.d.comb += self.outform_busy.eq(1)
                with m.If(self.mint_fault):
                    m.d.sync += self.outform_fault_type.eq(OUTFORM_FAULT_MINT)
                    m.next = "FAULT"
                with m.Elif(self.mint_done):
                    m.d.sync += result_gt_reg.eq(self.mint_result_gt)
                    m.next = "COMPLETE"

            # ── COMPLETE ─────────────────────────────────────────────────────
            # Single-cycle done pulse; result_gt holds the issued E-GT.
            with m.State("COMPLETE"):
                m.d.comb += [
                    self.outform_busy.eq(0),
                    self.outform_done.eq(1),
                ]
                m.next = "IDLE"

            # ── FAULT ─────────────────────────────────────────────────────────
            # Single-cycle fault pulse; outform_fault_type holds the reason.
            with m.State("FAULT"):
                m.d.comb += [
                    self.outform_busy.eq(0),
                    self.outform_fault.eq(1),
                ]
                m.next = "IDLE"

        return m
