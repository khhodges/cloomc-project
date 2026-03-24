# Church Machine Lump Architecture

## Overview

A **lump** is the fundamental deployable unit of the Church Machine. It is a
contiguous, capability-secured memory region containing an executable code
section and an optional capability list (c-list). Every abstraction compiles
to exactly one lump.

---

## Lump Size Rule

Lump size is always a power of 2, minimum 64 words, maximum 16384 words (32-bit each):

    lumpSize = 2^n   where 6 <= n <= 14
    freespace = lumpSize - 1 - cw - cc   (must be all-zero; Mint verifies at load time)

The maximum is 2^14 = 16384 words. The header `cw` field (13 bits, max 8191) and `cc`
field (8 bits, max 255) together cap the maximum useful payload at 1+8191+255 = 8447
words — any lump larger than 2^14 = 16384 words would carry mandatory unverifiable
freespace with no mechanism to fill it. Mint hard-rejects n-6 > 8 (lumpSize > 16K).

| Abstraction | Code words (cw) | C-list slots (cc) | Lump size         | Freespace  |
|-------------|-----------------|-------------------|-------------------|------------|
| Decimal     | 107             | 0                 | 2^7 = 128 words   | 20 words   |
| SlideRule   | 525             | 1                 | 2^10 = 1024 words | 497 words  |
| TestSR      | 604             | 1                 | 2^10 = 1024 words | 418 words  |

---

## Lump Memory Layout

```
Word 0:                Header word (metadata, never executed)
Words 1..cw:           Code section — dispatcher at PC=1, then methods
Words cw+1..lumpSize-cc-1:  Freespace (all zeros, verified by Mint at load time)
Words lumpSize-cc..end:     C-list slots (cc×1 word each, compiler-populated, top of lump)
```

Hardware entry point is **PC = 1** — Word 0 is the header (never executed);
code begins at Word 1. The c-list is pre-populated by the compiler at build time.

---

## The Header Word (Word 0)

The first word of every lump binary is a metadata descriptor. It uses opcode
`0x1F` (`11111b`) — an undefined instruction on the Church Machine ISA. If
Word 0 were accidentally executed, the hardware traps rather than silently
corrupting state.

```
31      27 26    23 22                10 9   8 7              0
+──────────+────────+──────────────────+──────+────────────────+
│ 0x1F [5] │ n-6[4] │     cw [13]      │typ[2]│    cc [8]      │
+──────────+────────+──────────────────+──────+────────────────+
```

| Field | Bits  | Meaning |
|-------|-------|---------|
| magic | 31:27 | Always `11111` (0x1F). Traps if executed. |
| n-6   | 26:23 | lumpSize = 2^(val+6). Valid range 0..8 → 64..16384 words. Values 9..15 rejected by Mint. |
| cw    | 22:10 | Code word count (0..8191). Words 1..cw are code; words cw+1..lumpSize-cc-1 must be zero. |
| typ   | 9:8   | Object type: `00`=lump, `01`=data, `10`=clist-only, `11`=Outform. |
| cc    | 7:0   | C-list slot count (0..255). |

32 bits total. No spare bits. No dead fields. `code_base = base + 4` always. `PC = 1` always.

### Example Header Words

Encoding formula: `(0x1F << 27) | ((n-6) << 23) | (cw << 10) | (typ << 8) | cc`

```
Decimal    (n=7,  cw=107, cc=0, typ=00):  0xF881_AC00
SlideRule  (n=10, cw=525, cc=1, typ=00):  0xFA08_3401
```

### Mint Validation Sequence

`Mint.Lump(base, n)` receives a lump already inflated into physical memory.
It validates the header and binary before issuing any GT.

```
Step 1  Read Mem[base] — the header word.
Step 2  magic[31:27] == 0x1F — reject if not.
Step 3  n-6[26:23] <= 8   — reject if n-6 > 8 (lump would exceed 16K words).
Step 4  lumpSize = 2^(n-6+6).
Step 5  cw[22:10] <= lumpSize - cc - 2  — reject if header is self-contradictory.
Step 6  cc[7:0]   <= lumpSize - 2       — reject if c-list overflows lump.
Step 7  Scan words cw+1 .. lumpSize-cc-1: reject if any word is non-zero.
          Freespace must be all-zero — this is enforced, not assumed.
Step 8  Validate c-list slots (each must be a well-formed GT Word 0).
Step 9  Issue E-GT, write NS slot.
```

Steps 2–6 are pure arithmetic on the 32-bit header — no memory access beyond
the header word. Step 7 is the freespace scan, protected by the cheap
consistency gates in steps 3–6. A malformed or malicious header is caught
before Mint touches the binary body.

---

## Instruction Set Mutual Exclusion

The Church Machine's 20 instructions divide into **two completely independent
groups** with **mutually exclusive access rights**. A memory region carries
rights from one group only — never both. This is enforced at the hardware
instruction-decode level, not by software policy.

```
+────────────────────────────────────────+  +────────────────────────────────────────+
│       TURING instructions              │  │       CHURCH instructions              │
│       (Data side)                      │  │       (GT / Capability side)           │
+────────────────────────────────────────+  +────────────────────────────────────────+
│  DREAD   DWRITE                        │  │  LOAD    SAVE                          │
│  IADD    ISUB    SHL    SHR            │  │  CALL    RETURN                        │
│  MCMP    BRANCH                        │  │  LAMBDA  TPERM                         │
│  BFEXT   BFINS                         │  │  ELOADCALL  XLOADLAMBDA                │
│                                        │  │  CHANGE  SWITCH                        │
│  Access rights:  R  W  X               │  │  Access rights:  L  S  E               │
+────────────────────────────────────────+  +────────────────────────────────────────+
         operate on DATA memory                    operate on CAPABILITIES only
         cannot reach GTs                          cannot reach data memory
```

### Permission definitions

Word 0 of every GT encodes the TPERM-controllable permission bits at [31:25],
the GT class at [24:23], and identity fields below that:

```
Word 0 [31:25]  B  R  W  X  L  S  E   ← TPERM-changeable, excluded from CRC
Word 0 [24:23]  typ[2]                 ← GT class, CRC covered
```

| Bits  | Field | Side   | Instruction  | Meaning |
|-------|-------|--------|--------------|---------|
| 31    | B     | Church | SAVE         | Bind — B=1 allows SAVE; B=0 causes SAVE to fault |
| 30    | R     | Turing | DREAD        | Read data words from this region |
| 29    | W     | Turing | DWRITE       | Write data words to this region |
| 28    | X     | Turing | —            | Region is executable (PC may enter) |
| 27    | L     | Church | LOAD         | Load a capability out of this region |
| 26    | S     | Church | SAVE         | Save a capability into this region |
| 25    | E     | Church | CALL         | This GT is a valid CALL target |
| 24:23 | typ   | —      | —            | GT class: 00=NULL, 01=Real, 10=Abstract, 11=Outform — CRC covered |

**B is in Word 0 bit 31 and is stored with the GT.** It survives context switch
naturally. TPERM can clear B (reducing delegation depth) but never add it.

**R, W, X and L, S, E are mutually exclusive groups.** Any GT with bits from
more than one group is rejected by Mint as malformed. **NULL GT** is all 128
bits zero (typ=00, all rights clear). **Abstract GT** is typ=10 — hardware
returns a constant on DREAD; no RAM region exists.

### Standard GT combinations

| GT type           | typ | permissions [31:25] | Object NS slot? | Description |
|-------------------|-----|---------------------|-----------------|-------------|
| E-GT (lump gate)  | 01  | B E                 | Yes             | Church: callable lump — the only issued lump GT |
| RW-GT (data)      | 01  | B R W               | Depends on use  | Turing: full data read/write |
| R-GT (read-only)  | 01  | B R                 | Depends on use  | Turing: read-only data |
| LS-GT (MintCL)    | 01  | B L S               | Depends on use  | Church: full capability read/write |
| **NULL GT**       | 00  | 0 (all clear)       | No              | All bits zero — faults on any use |
| **ABSTRACT GT**   | 10  | 0 (no rights)       | **No**          | Self-defining constant or PassKey — no RAM |
| **OUTFORM GT**    | 11  | (any)               | Yes (outform)   | Lump registered but not yet resident — fires Absent event on LOAD |
| *(CR14 transient)*| 01  | B X                 | —               | Derived from NS slot on CALL; never issued or stored |
| *(CR6 transient)* | 01  | B L                 | —               | Derived from NS slot on CALL; never issued or stored |

### What mutual exclusion eliminates — flawlessly

| Threat | Why architecturally impossible |
|--------|-------------------------------|
| GT forgery via data write | DWRITE is Turing — cannot reach an L/S region |
| Buffer overrun into c-list | DWRITE cannot cross into a Church-rights region |
| Code injection via DWRITE | DWRITE cannot touch an X region |
| GT inspection via DREAD | DREAD is Turing — cannot read a Church-rights region |
| Capability smuggling through data | SAVE is Church — cannot write to an R/W data region |
| ROP gadget scanning | Code is X-only — DREAD cannot read instructions |
| Confused deputy via integer trick | R/W integers cannot be reinterpreted as Church capabilities |

---

## GT Taxonomy — Three Fundamental Cases

Every GT belongs to exactly one of three fundamental classes, identified by
`typ[2]` in Word 0 bits [24:23]. This is CRC-covered and visible to hardware at
instruction-decode time without inspecting Word 2.

### NULL GT (typ = 00)

All 128 bits zero. Faults on any CALL, LOAD, or DREAD. Occupies every
unoccupied c-list slot. Never issued by Mint — it is the absence of a GT.

### Real GT (typ = 01)

Issued by Mint. References a physical memory region. The R/W/X or L/S/E
permission bits describe what the holder may do with that region. All specific
GT subtypes (X-GT, L-GT, RW-GT, E-GT, LS-GT) are Real GTs.

### Abstract GT (typ = 10)

Self-defining. No memory region, no Object NS slot. Hardware maps
`object_id → value` internally. Covers physical constants (DREAD returns a
fixed value) and PassKey credentials (opaque identity tokens).
Abstract GTs are distributed by writing the full CR directly into c-list slots —
no NS slot consumed.

**`typ[2]` (Word 0 bits [24:23])** is the hardware-visible GT class field:
00=NULL, 01=Real, 10=Abstract, 11=Outform. This is CRC-covered and visible at
decode time without reading Word 2.

---

## Context Register (CR) — 128-bit Structure

A CR is four 32-bit words stored in a hardware register file (CR0..CR15 per
thread). It defines a protected memory region, the holder's permission set,
and the CRC validated on every LOAD.

```
+──────────────────────────────────────────────────────────────+
│  Word 3 [127:96]  CRC and GC  (spare[15] | G[1] | CRC[16]) │
+──────────────────────────────────────────────────────────────+
│  Word 2 [95:64]   Limit and revocation                      │
│                   (spare[4] | gt_seq[7] | limit_offset[21]) │
+──────────────────────────────────────────────────────────────+
│  Word 1 [63:32]   Base address [32]                         │
+──────────────────────────────────────────────────────────────+
│  Word 0 [31:0]    GT — the holder's credential (per-holder) │
│                   SAVE copies this word only                 │
+──────────────────────────────────────────────────────────────+
```

**B** (Word 0 bit 31) and **G** (Word 3 bit 112) are stored in the 128-bit
value. B is TPERM-controllable and survives context switch. G is hardware-managed
during GC sweeps and is re-initialised by LOAD, not preserved by SAVE.

### Word 0 — The GT (per-holder credential)

Word 0 is user-specific. Two holders of GTs for the same object have different
Word 0 values because their permission sets differ. SAVE copies Word 0 only.

```
31      25 24  23 22      16 15            0
+─────────┬──────┬──────────┬──────────────+
│B R W X  │ typ  │  gt_seq  │  object_id   │
│ L S E   │ [2]  │   [7]    │    [16]      │
│  [7]    │      │          │              │
+─────────┴──────┴──────────┴──────────────+
```

| Field     | Bits  | Meaning |
|-----------|-------|---------|
| B R W X L S E | 31:25 | Permissions (TPERM-changeable, **excluded from CRC**) |
| typ       | 24:23 | GT class: 00=NULL, 01=Real, 10=Abstract, 11=Outform — CRC covered |
| gt_seq    | 22:16 | Revocation sequence number — CRC covered |
| object_id | 15:0  | Object index, unique per lump issuance — CRC covered |

**TPERM** clears any subset of bits [31:25] to produce a weaker GT.
Because bits [31:25] are excluded from the CRC, TPERM is a pure hardware
operation — no Mint round-trip, no new CRC computation required. Permission
escalation is architecturally impossible.

### Word 1 — Base Address

Physical base address of the memory region this GT describes. CRC covered.

```
63                                              32
+────────────────────────────────────────────────+
│                   base [32]                    │
+────────────────────────────────────────────────+
```

| Field | Meaning |
|-------|---------|
| base  | Physical base address of the region (32-bit word address) |

The region spans `base` to `base + limit_offset` (Word 2). Both base and
limit_offset are CRC-covered and cannot be tampered with independently of
the CRC check in Word 3.

### Word 2 — Limit and Revocation

Encodes the region limit and the per-object revocation sequence number. CRC
covered. Same value for all holders of GTs for the same object.

```
95  92 91      85 84                          64
+──────+──────────+────────────────────────────+
│spare │  gt_seq  │       limit_offset [21]     │
│ [4]  │   [7]    │                             │
+──────+──────────+────────────────────────────+
```

| Field        | Bits  | Meaning |
|--------------|-------|---------|
| spare        | 95:92 | Reserved zero |
| gt_seq       | 91:85 | Revocation sequence — must match Word 0 gt_seq at LOAD |
| limit_offset | 84:64 | Region limit = base + limit_offset |

**Revocation:** Mint increments gt_seq in the Object NS slot. On LOAD, hardware
checks Word 0 gt_seq against Word 2 gt_seq — a mismatch means the GT has been
revoked and the LOAD faults. The stored GT (Word 0) may be stale in any c-list;
the Object NS slot is authoritative.

### Word 3 — CRC and GC

Contains the 16-bit CRC and the GC mark bit G. Validated by hardware on
every `LOAD CR_d, CR6, #slot`. If the CRC check fails the LOAD faults and
the destination CR is not populated.

```
127         113 112 111                     96
+─────────────┬───┬──────────────────────────+
│  spare [15] │ G │        CRC [16]          │
+─────────────┴───┴──────────────────────────+
```

| Field | Bits    | Meaning |
|-------|---------|---------|
| spare | 127:113 | Reserved zero |
| G     | 112     | GC mark bit — hardware-managed, re-initialised by LOAD, not preserved by SAVE |
| CRC   | 111:96  | CRC-16/CCITT (poly 0x1021) over Word 0[24:0] + Word 1[all] + Word 2[all] |

**CRC coverage:**

| Field | Covered? | Reason |
|-------|----------|--------|
| B R W X L S E (Word 0 bits 31:25) | **No** | TPERM-changeable — excluded so TPERM needs no CRC recomputation |
| typ, gt_seq, object_id (Word 0 bits 24:0) | Yes | Core identity — tamper-evident |
| base (Word 1) | Yes | Region address — tamper-evident |
| spare, gt_seq, limit_offset (Word 2) | Yes | Limit and revocation — tamper-evident |

---

## CR Hardware — B and G

### B — Bind (Word 0 bit 31, TPERM-controllable)

B is a stored permission bit in Word 0. It controls whether the GT can be
delegated via SAVE.

**B=1** — SAVE allowed. The holder may copy this GT into a c-list slot.
Mint sets B=1 at issuance time for all transferable GTs.

**B=0** — SAVE faults. The GT is non-transferable. Any attempt to SAVE a GT
with B=0 causes an immediate hardware fault. This prevents theft of private GTs
(e.g. PassKey credentials, private session GTs).

Because B is in Word 0 (bits [31:25]), it:
- Survives context switch with no special mechanism
- Is excluded from CRC — TPERM can clear B without triggering a CRC recomputation
- Can be cleared by TPERM but **never set** — delegation depth can only decrease

Mint sets B at issuance. TPERM clears B. There is no instruction that adds B.

### G — GC Mark Bit (Word 3 bit 112, hardware-managed)

G is the hardware mark bit for the Church Machine's garbage collector. The GC
uses **mark-and-sweep**, not reference counting (reference counting fails on
cycles and incurs per-operation overhead).

**LOAD** re-initialises G when populating a CR slot — a new live reference is
recorded for the GC sweep. **SAVE** does not copy G — it is not part of the
Word 0 snapshot written to the c-list.

A sweep pass works as follows:

```
Sweep pass N:   traverse all live CR slots
                flip G: 0 → 1 for every reachable slot
                anything still at G=0 was unreachable → nullified

Sweep pass N+1: traverse all live CR slots
                flip G: 1 → 0 for every reachable slot
                anything still at G=1 was unreachable → nullified

(alternates indefinitely: 0→1, 1→0, 0→1, ...)
```

Anything whose G did **not** change during a sweep pass was not reached —
those CR slots are nullified (written with NULL GT, 128 bits zero).

The sweep is triggered deliberately by an explicit GC instruction, not by a
continuous background process.

---

## NULL GT

The NULL GT is the canonical representation of **no capability** — the all-zeros
128-bit value. It occupies every unoccupied c-list slot and every NS slot Word 2
when `cc = 0`.

```
+──────────────────────────────────────────────────────────────+
│  Word 3 [127:96]   0x0000_0000                              │
+──────────────────────────────────────────────────────────────+
│  Word 2 [95:64]    0x0000_0000                              │
+──────────────────────────────────────────────────────────────+
│  Word 1 [63:32]    0x0000_0000                              │
+──────────────────────────────────────────────────────────────+
│  Word 0 [31:0]     0x0000_0000   (typ=00, all bits zero)    │
+──────────────────────────────────────────────────────────────+
```

### Properties

| Property | Value |
|----------|-------|
| Encoding | All 128 bits zero |
| typ | 00 — NULL class; hardware treats any typ=00 GT as null regardless of other bits |
| Namespace slot | None issued for it — but NULL values appear inside NS slot words |
| On use   | Any CALL, DREAD, or LOAD through a NULL GT faults immediately |
| Issued by | Never — it is the absence of a GT, not a GT Mint creates |

### Where NULL GTs Appear

| Location | Meaning |
|----------|---------|
| Unoccupied c-list slot | Slot has no capability installed |
| NS slot Word 2 when cc=0 | Set to `0x00000000` — lump has no c-list, hardware skips CR6 setup |
| NS slot Words 1–3 when NULL | All zero — lump absent with no recovery token; LOAD faults |
| NS slot Words 1–3 when Outform absent | IDE 96-bit token — LOAD fires Absent event; IDE fetches binary and promotes slot to Live |
| NS slot Words 1–3 when Live | base / gt_seq+limit / CRC — lump resident in RAM; LOAD reconstructs GT |
| Revoked GT in c-list | LOAD detects gt_seq mismatch against Object NS slot — LOAD nullifies the c-list slot (writes NULL GT) and faults |
| Uninitialised CR | Hardware zero-fills CR registers at context creation |

---

## ABSTRACT GT

Abstract GTs represent **hardware-defined immutable values** — physical constants
and PassKey credentials. They carry `typ=10` in Word 0 bits [24:23].

Unlike every other GT type, an Abstract GT is **self-defining**. The CR is the
complete definition. There is no lump to find, no RAM to read, and no Object NS
slot. Hardware maps `object_id → value` internally. Physical constants are
immutable and never revoked. PassKey credentials are managed entirely by the
Abstraction handler — revocation is handled at that layer, not via the Object NS
slot mechanism used by Real GTs.

```
+──────────────────────────────────────────────────────────────+
│  Word 3 [127:96]   spare[15] | G[1] | CRC[16]              │
+──────────────────────────────────────────────────────────────+
│  Word 2 [95:64]    spare[4] | gt_seq[7]=0 | limit_offset=0  │
+──────────────────────────────────────────────────────────────+
│  Word 1 [63:32]    base = 0  (no RAM region)                 │
+──────────────────────────────────────────────────────────────+
│  Word 0 [31:0]     B=0 R=0 W=0 X=0 L=0 S=0 E=0 | typ=10   │
│                    | gt_seq=0 | object_id=<id>              │
+──────────────────────────────────────────────────────────────+
```

### Constant ID Space

| Range       | Sub-type           | Meaning |
|-------------|--------------------|---------|
| 0x000–0x0FF | Physical constants | Hardware returns a fixed 32-bit value on DREAD |
| 0x100–0x1FF | PassKey credentials| Opaque identity tokens for Mint caller authentication |
| 0x200–0xFFF | Reserved           | Future use |

### Physical Constants (0x000–0x0FF)

All values in fixed-point ×1000 (e.g. PI = 3141 representing 3.141).

| Constant ID | Name  | Value (×1000) | Decimal |
|-------------|-------|---------------|---------|
| 0x000       | ZERO  | 0             | 0.000 |
| 0x001       | ONE   | 1000          | 1.000 |
| 0x002       | PI    | 3141          | 3.141 |
| 0x003       | E     | 2718          | 2.718 |
| 0x004       | LN2   | 693           | 0.693 |
| 0x005       | LN10  | 2303          | 2.303 |
| 0x006       | SQRT2 | 1414          | 1.414 |

```
DREAD DR2, CR_pi, #0   -> DR2 = 3141   (no RAM access, no address needed)
DREAD DR3, CR_e,  #0   -> DR3 = 2718
```

### Self-Defining — No Namespace Slot

A holder of an Abstract GT holds the CR itself. There is no lump to locate or
fetch. The full access lifecycle:

```
Lump GT lifecycle:
  NS slot → Locator → fetch → Mint.Lump() → E-GT → CALL
  (CALL derives transient CR14 (X) and CR6 (L) from the NS slot)

Abstract GT lifecycle:
  CR is the GT → DREAD → hardware returns value
  (no NS slot, no fetch, no Mint step, no memory region)
```

For the full step-by-step absent-lump fetch protocol (Absent event → Locator
subroutine → zip inflate → Mint → Live NS slot → LOAD retry), see
**`CM_LAZY_LOAD.md`**.

Abstract GTs are distributed by being **written directly into c-list slots** at
abstraction installation time. Mint writes the CR into the slot — no namespace
slot is consumed.

```
SlideRule c-list (example):
  Slot 0: Decimal E-GT      <- regular lump GT, one NS slot consumed
  Slot 1: PI abstract GT    <- self-defining, zero NS slots consumed
  Slot 2: LN10 abstract GT  <- self-defining, zero NS slots consumed
```

### PassKey Credentials (0x100–0x1FF)

A PassKey is an opaque identity token. It is used by privileged abstractions
(primarily Mint) to verify that a caller is authorised before servicing a
privileged request.

| Property | Detail |
|----------|--------|
| typ      | 10 (Abstract) — Word 0 bits [24:23] |
| permissions | B=0, all rights clear — non-transferable (SAVE faults) |
| object_id | 0x100–0x1FF — identifies which PassKey |
| Issued by | Hardware at boot — written directly into the Loader's c-list |
| Non-transferable | B=0 in Word 0 — SAVE always faults; PassKey cannot be copied to any c-list |
| Revocation | Managed by the Abstraction handler — no Object NS slot involved |

#### MINT Verification Flow

```
Caller holds PassKey CR in slot N
  -> CALL Mint
  -> Mint: LOAD its own c-list slot for the expected PassKey
  -> Mint: compare object_id of caller's PassKey to approved list
  -> Match: service the request
  -> No match: fault — caller identity not recognised
```

#### Bootstrap Chain

```
Hardware boot
  -> CPU writes PassKey CRs directly into Loader's c-list slots
  -> No Mint involvement — these are hardware-issued
  -> Loader uses PassKey to authenticate itself to first Mint instance
  -> Mint issues subsequent PassKeys via its own privileged interface
  -> Software-issued PassKeys: object_id >= 0x140 (convention)
```

---

## Mint.Lump — One E-GT, One NS Slot

`Mint.Lump(base, n)` receives a pre-inflated lump binary already in physical
memory at `base`, validates the header word, and issues **one E-GT** and writes
**one NS slot** using the 128-bit CR structure. Inflation and pre-allocation are
performed by the Locator before Mint is called (see "ZIP Local File Header and
Pre-Allocation" in the Zip File Formats section).

| Token    | Region              | permissions | Mounted as | Issued? |
|----------|---------------------|-------------|------------|---------|
| **E-GT** | Entire lump (word 0..lumpSize-1) | B E | held by caller | Yes — only issued GT |
| CR14 (X) | Words 1..lumpSize-cc-1          | B X | CR14 on CALL   | No — transient only |
| CR6  (L) | Words lumpSize-cc..lumpSize-1   | B L | CR6 on CALL    | No — transient only |

If `cc = 0` (e.g. Decimal): CR6 is NULL GT after CALL; derived X view still valid.

### Mint validation steps

Mint receives `(base, n)` from the Locator — the lump binary has already been
inflated into physical memory at `base` by the pre-allocation sequence above.
Mint does not inflate; it validates and issues.

```
1. Read header word W0 = Mem[base]       (Word 0 of lump binary, already in memory)
2. Check W0[31:27] == 0x1F              -> valid magic field? Reject if not.
3. Extract ver = W0[26:23]              -> version check
4. Extract n_hdr = W0[22:19] + 6        -> header-derived lump size exponent
   Cross-check: n_hdr must equal n (transport-derived)
   Mismatch → corrupted binary; free region, reject.
5. lumpSize = 1 << n                    -> confirmed from both transport and header
6. Extract cc  = W0[18:11]             -> c-list starts at word (lumpSize - cc)
7. Extract typ = W0[10:9]              -> must be 0x00 for a callable lump
8. Assign object_id from pool; assign gt_seq from pool counter
9. Build E-GT CR (4 words) — the only issued GT:
     Word 0: B=1 E=1 | typ=01 | gt_seq[7] | object_id[16]
     Word 1: base [32]  (physical byte address of lump word 0)
     Word 2: spare[4] | gt_seq[7] | (lumpSize-1) [limit_offset, 21 bits]
     Word 3: spare[15] | G=0 | CRC-16/CCITT(Word0[24:0] + Word1 + Word2) [16]
10. Write single Object NS slot (3 words):
     Word 1: base [32]
     Word 2: spare[4] | gt_seq[7] | (lumpSize-1) [21]
     Word 3: spare[15] | G=0 | CRC[16]
     (CALL reads n_minus_6 and cc directly from Mem[base] — no cached Word 4)
11. Return E-GT to caller
     (CR14 and CR6 are derived transiently on every CALL — not returned here)
```

---

## The Object NS Slot

Each lump occupies exactly **one Object NS slot** consisting of three 32-bit
words. Words 1–3 mirror the E-GT CR Words 1–3 exactly. Word 0 (the Golden
Token) is held privately by the owner in their c-list slot — it is never
stored in the NS slot.

CALL reads the lump header word directly from `Mem[base]` to obtain
`n_minus_6` and `cc`. No cached copy is held in the NS slot.

```
NS Word 1  base [32]
             Mirrors CR Word 1. Physical byte address of lump word 0.

NS Word 2  spare[4] | gt_seq[7] | limit_offset[21]
             Mirrors CR Word 2. limit_offset = lumpSize-1 for the E-GT.
             Revocation: LOAD checks Word 0 gt_seq [22:16] against NS Word 2 gt_seq [27:21].

NS Word 3  spare[15] | G[1] | CRC[16]
             Mirrors CR Word 3. LOAD re-computes and verifies CRC from this.
```

**Read cost:** LOAD reads Words 1–3 (3 reads). CALL reads Words 1–3 then
`Mem[base]` (3 NS reads + 1 lump-header read).
Old two-slot design: 6 reads (NS_X Words 1–3 + NS_L Words 1–3). **50% reduction.**

### Concrete NS Slot Values

```
Decimal    (gt_seq=0x01, base=0x20000000, lumpSize=128)
  Word 1:  0x20000000          (lump base)
  Word 2:  0x0020007F          (gt_seq=0x01, limit_offset=127)
  Word 3:  0x00004CEF          (E-GT CRC — CRC-16/CCITT-FALSE of EW0[24:0]+EW1+EW2)

SlideRule  (gt_seq=0x01, base=0x10000000, lumpSize=1024)
  Word 1:  0x10000000          (lump base)
  Word 2:  0x002003FF          (gt_seq=0x01, limit_offset=1023)
  Word 3:  0x000048F3          (E-GT CRC — illustrative; E-GT covers whole lump)
```

On CALL, hardware reads NS Words 1–3, then reads `Mem[base]` (the lump
header word) to extract `n_minus_6` and `cc`. It then derives `code_base = base + 4`
and `clist_base = base + (lumpSize - cc) * 4` and constructs CR14 and CR6
transiently. The CR is computed fresh on every CALL — no GT store index needed.

**LOAD** reads NS Words 1–3 to reconstruct the E-GT directly — Word 0 comes
from the c-list slot, Words 1–3 come from the NS slot, CRC is re-computed and
verified against NS Word 3.

**Revocation:** Mint increments gt_seq in NS Word 2. LOAD checks Word 0 gt_seq
[22:16] against NS Word 2 gt_seq [27:21] — mismatch nullifies the c-list slot
and faults. Stale E-GTs self-clean on first encounter.

---

## CALL Execution Flow

```
CALL CR_s   (CR_s holds the E-GT for the target lump)
  1. Validate E-GT CRC — FAULT if mismatch
  2. Read object_id and gt_seq from E-GT Word 0
  3. Fetch NS[object_id] — 3 words:
       base       = NS.Word1
       gt_seq_ns  = NS.Word2[27:21]
       Read Mem[base] → lump header word:
       n_minus_6  = Mem[base][22:19]    ← lump header field
       cc         = Mem[base][18:11]    ← lump header field
     If lump not present locally (evicted or first access via Outform GT):
       -> CALL Locator abstraction — secure subroutine in the same thread
       -> Locator fetches, inflates, validates, and Mints the lump
       -> Locator RETURNs; CALL retries from step 1 with a Live NS slot
       (Full absent-lump protocol: see CM_LAZY_LOAD.md)
  4. Revocation check: if E-GT gt_seq != NS gt_seq_ns -> FAULT
  5. Derive lumpSize = 1 << (n_minus_6 + 6)
  6. Derive and build transient CR14 (X):
       XW1 = base + 4  (always — no manifest offset)
       XW2 = (gt_seq<<21)|(lumpSize-cc-2), XW3 = CRC(...)
       CR14 ← { B=1 X=1 typ=01 ..., XW1, XW2, XW3 }
  7. If cc > 0: derive and build transient CR6 (L):
       LW1 = base + (lumpSize-cc)*4, LW2 = (gt_seq<<21)|(cc-1), LW3 = CRC(...)
       CR6 ← { B=1 L=1 typ=01 ..., LW1, LW2, LW3 }
     else: CR6 ← NULL GT
  8. PC ← 1  (code begins at word 1)
  9. Execute dispatcher
```

### Locator and Flag Pool

> **Note:** This section describes the Flag pool concurrency mechanism used
> *inside* the Locator. For the top-level absent-lump protocol (the thread-
> level view), see **`CM_LAZY_LOAD.md`**.

The absent-lump path uses a **Flag pool** to bound thread count and minimise CHANGE
instructions. All Flags in the pool are identical when idle — they are homogeneous
resources with no identity until allocated.

**Thread State Block (TSB)** is the CHANGE data structure. It is not a separate
bookkeeping record — it is what CHANGE operates on. When a thread issues CHANGE,
its full live state is captured into its TSB:

```
TSB fields:
  All register conditions       — general-purpose registers
  DR                            — data register (holds a GT or a data value)
  Stack                         — full call stack state
  Indicators                    — condition flags / status bits
  Flag pointer                  — which Flag this thread is waiting on
```

No kernel stack. No virtual memory descriptor. No privilege level.
The GC marks all GT-holding fields in every live TSB during a sweep so that
capabilities held by parked threads are never incorrectly collected.

```
Free Flag pool:  [ F0  F1  F2  ...  Fn ]   — all identical when idle

Inside Locator (awaiting network I/O):
  Locator picks one free Flag (e.g. F3)
  F3 removed from free pool               — now owned by this I/O job
  Locator queues network request, binds F3 to the job
  Locator issues CHANGE                   — Locator thread parks, another thread runs
  (The Locator subroutine parks here internally — the calling thread is already
   on the Locator's call stack and will resume when the Locator RETURNs)

I/O event thread (single, always running):
  Receives network completion for F3's job
  Validates SHA256
  Calls Mint.Lump() → NS slot written (Live state)
  Signals F3                              — Locator thread becomes runnable
  F3 returned to free pool

Locator thread resumes from CHANGE:
  Verifies NS slot is Live
  RETURNs to calling thread              — calling thread retries LOAD/CALL normally
```

**Why this design:**

| Property | Benefit |
|---|---|
| Flag pool is homogeneous | No per-download state — any free Flag serves any I/O job |
| Locator picks Flag on demand | Pool shrinks only under load; idles at full size |
| I/O event thread is single | No concurrent NS slot writes; no locking needed on NS updates |
| CHANGE to known-free thread | No scheduler guessing — CHANGE target is confirmed runnable at the moment of issue |
| Flag returned after job | Pool self-replenishes; bounded by pool size, not download count |
| One CHANGE per download | Absent-lump path costs exactly one CHANGE in, one resume out |

---

## Zip File Formats

Lump binaries are distributed as zip files. The ZIP container format is
standard; the compression method inside varies by context.

### Case 1 — Single Lump Upload

```
SlideRule.lump.zip
+-- SlideRule.bin    <- raw lump binary, 2^n x 4 bytes
```

Filename encodes the abstraction label. DEFLATE compression is highly effective
on the freespace region (consecutive zero words).

### Case 2 — Namespace Bundle

```
namespace.zip
+-- manifest.json    <- install order + dependency declarations
+-- Decimal.bin
+-- SlideRule.bin
+-- TestSR.bin
```

Each `.bin` is independently self-describing via its header word. Mint.Lump
is called once per entry, in dependency order declared by the manifest.

### Case 3 — Network-Cached Lump

```
cm://domain/SlideRule@sha256:a3f9c2...
```

Same format as Case 1. The SHA256 hash covers the lump binary (header +
code + c-list). Any node that holds the cached binary (or zip wrapping it)
can serve it. Mint extracts the binary, re-validates the hash; the Locator
inflates if the ZIP local file header's compression method field is non-zero
(DEFLATE, RLE, etc.) — the lump binary itself carries no compression flag.

### Compression Methods

| Method        | CM method field | Use case                    | Inflate cost in CM ISA       |
|---------------|-----------------|-----------------------------|------------------------------|
| STORE         | 0               | Integrity check only         | Trivial — copy words directly |
| DEFLATE       | 8               | Offline compiler tools       | High — Huffman + LZ77        |
| RLE (custom)  | n/a             | CM-native inflate abstraction| Low — ~50 instructions       |

The freespace region (all zeros) compresses near-perfectly under any method.
A 419-word freespace block compresses to a handful of bytes under RLE.

### ZIP Local File Header and Pre-Allocation

The Locator reads the ZIP local file header before inflating to derive `n`
and pre-allocate the physical memory region. All multi-byte fields are
little-endian.

```
Offset  Size  Field
──────  ────  ─────────────────────────────────────────────────────────────
 0      4     Signature = 0x04034B50
 4      2     Version needed to extract
 6      2     General purpose bit flags          ← bit 3: data descriptor flag
 8      2     Compression method (0=STORE, 8=DEFLATE)
10      2     Last-mod time
12      2     Last-mod date
16      4     CRC-32
20      4     Compressed size
24      4     Uncompressed size                  ← n = log2(value / 4)
28      2     File name length (L)
30      2     Extra field length (E)
32+L+E        [compressed data begins here]
```

**Bit 3 constraint — lump zips must not use data descriptors.**
When bit 3 of the general-purpose flags is 1 (streaming / data descriptor
mode), the compressor did not know the output size when writing the header.
Fields at offsets 16–27 (CRC-32, compressed size, uncompressed size) are
written as zero; the true values appear in a Data Descriptor record after the
compressed payload. In that case the Locator cannot derive `n` until the
entire compressed stream has been received, defeating streaming pre-allocation.

A lump binary has a fixed, known size at compile time — the compiler chose
`n` before emitting a single word. Lump zips **must** be produced with bit 3
clear so the uncompressed size is present in the local file header.

**Loader MUST reject** any lump zip where:
- bit 3 of the general-purpose flags is 1, or
- the uncompressed size field is zero.

**Pre-allocation sequence:**

```
1. Receive first ~32 bytes of the network response
2. Verify signature = 0x04034B50 — reject if mismatch
3. Assert bit 3 of flags = 0 — reject if streaming mode
4. Read uncompressed_size from offset 24
5. Derive n = log2(uncompressed_size / 4)
     Reject if uncompressed_size is not a power-of-2 multiple of 4
     Reject if n < 6 (minimum lump size is 2^6 = 64 words)
6. Call Memory Manager with n → receive base
7. Inflate compressed payload into [base, base + uncompressed_size)
8. Verify zip CRC-32 against offset 16 — reject on mismatch
9. Hand (base, n) to Mint.Lump() for header validation and GT issuance
```

Step 9 passes `n` explicitly so Mint can cross-check the lump header's `n-6`
field against the transport-derived value. A mismatch means the binary is
corrupted; Mint rejects it and the Memory Manager frees the region.

---

## C-List — Compiler-Populated

The IDE toolchain pre-fills every c-list slot at compile time. There is no
embedded manifest block and no loader dep-resolution loop.

Each c-list slot is **one 32-bit word** — Word 0 of the GT (the Golden Token).
This is identical to the existing c-list format: LOAD reads Word 0 from the
c-list slot, then fetches Words 1–3 from the NS table. The c-list stores only
the per-holder credential; the NS table provides the shared region descriptor.

`cc` is the number of c-list slots. The c-list occupies `cc` words at the top
of the lump binary, starting at word address `lumpSize - cc`.

| Slot Word 0 value | typ | Meaning |
|---|---|---|
| B\|perms\|typ=01\|gt_seq\|object_id | 01=Real | Compiler writes the current golden token for this dep |
| typ=10\|object_id | 10=Abstract | Hardware constant or PassKey — self-defining, no NS slot |
| typ=11\|object_id | 11=Outform | IDE-managed dep — Absent event fires on first LOAD |
| 0x00000000 | 00=NULL | Unused slot |

```
SlideRule.lump.zip        <- optional zip wrapper (compression only)
+-- SlideRule.bin         <- lump binary: header + code + freespace + c-list
                                          (no manifest block)
```

Mint verifies Real GTs in the c-list against the NS table (gt_seq check)
before issuing the E-GT for the new lump. See LUMP_MANIFEST.md for the full
trust-but-verify policy and phase progression.

### Naming and Obfuscation

Content-addressed URLs reveal nothing about content or origin beyond the hash.
The abstraction name in the URL (`SlideRule`, `Decimal`) is a convention — it
could equally be a hash-only identifier. This is the Church Machine equivalent
of the PP250's flat `namespace.object` name lookup, extended to global
cyberspace with content-addressed, obfuscatable identifiers.

---

## The Locator Abstraction

URL resolution, network fetching, and cache policy are handled by a separate
**Locator abstraction** — referenced by `loc[12]` in the NS slot descriptor.
This isolates fetch policy from the fixed NS slot format, allowing the URL
scheme and network topology to evolve without hardware or slot-format changes.

### Why A Separate Abstraction

| Concern               | NS Slot     | Locator Abstraction |
|-----------------------|-------------|---------------------|
| Call setup            | Its job     |                     |
| URL storage           |             | Its job             |
| Fetch policy          |             | Can evolve freely   |
| Cache management      |             | Its job             |
| DHT vs CDN vs local   |             | Swappable           |
| Access control on fetch|            | Via c-list          |

### Interface

| Method     | Parameters    | Returns      | Description |
|------------|--------------|-------------|-------------|
| `resolve`  | label        | url (encoded)| Maps label to canonical URL |
| `fetch`    | url          | zip RW-GT    | Downloads zip bytes |
| `install`  | label, zip   | —            | Calls Mint.Lump + NamespaceWrite |
| `evict`    | label        | —            | Revokes GTs, clears NS slot to NULL (or restores Outform IDE token) |
| `register` | label, url   | —            | Adds label to URL mapping |

### C-List

| Slot | Capability          | Rights |
|------|---------------------|--------|
| 0    | NetworkIO E-GT      | Fetch from network |
| 1    | Mint E-GT           | Mint fetched lumps |
| 2    | NamespaceWrite E-GT | Install into namespace |

### Evolution Path

Because Locator is just another lump, it can be replaced without touching the
NS slot format. The `loc[12]` index in Word 0 is updated to point to the new
version — zero hardware changes required.

| Generation | Behaviour |
|------------|-----------|
| 1          | Label-to-URL table, plain HTTP |
| 2          | DHT lookup, peer-to-peer fetch |
| 3          | Policy routing: LAN cache -> CDN -> origin |
| 4          | Signed manifests, publisher verification chains |

---

## Content-Addressed URLs

Lumps are identified by content hash, following a content-addressed scheme:

```
cm://domain/SlideRule@sha256:a3f9c2...
```

| Property | Explanation |
|----------|-------------|
| Tamper detection | Hash covers entire zip — any modification breaks it before Mint sees it |
| Location independence | Any node with the zip can serve it — no single origin dependency |
| Cache correctness | Same hash always yields the same E-GT identity |
| Version pinning | Different hash = different abstraction version = different NS slot |
| Eviction safety | Evict locally, re-fetch from any peer on next CALL |

This is functionally equivalent to IPFS content addressing, with the Church
Machine Golden Token system providing the access-control layer that IPFS lacks.

---

## Contact Abstraction

A **Contact** is the local representative of a remote object. Every remote
capability in the system is held by a Contact instance. The caller interacts
with the Contact exactly as if it were the remote object directly — the network
boundary is invisible.

### C-List

| Slot | Capability           | Description |
|------|----------------------|-------------|
| 0    | NetworkIO E-GT       | Reach the remote node |
| 1    | rendezvous E-GT      | Locate the target if it moves |
| 2    | target descriptor    | Current content-addressed URL (obfuscated) |
| 3    | policy GT            | Heartbeat / reconnect strategy (swappable) |
| 4    | cached state GT      | Last known response — optional, used during outage |

### Active Behaviour

A Contact is not passive. It maintains the connection independently:

| Behaviour  | Description |
|------------|-------------|
| Heartbeat  | Periodically verifies remote is reachable (frequency set by policy GT) |
| Reconnect  | On failure, re-queries rendezvous for new location and retries |
| Update     | Absorbs new URL from rendezvous silently — target can move without caller awareness |
| Cache      | Holds last known response for brief outage resilience |
| Fault      | After exhausting reconnect strategy, faults cleanly to caller |

### Generic and Parameterised

One Contact abstraction, many instances — each parameterised by its policy GT:

```
mymother_contact  <- Contact(target=cm://..., policy=HighFrequency)
mybank_contact    <- Contact(target=cm://..., policy=OnDemand)
```

### Permission Levels

| GT | Rights | Held by |
|----|--------|---------|
| invoke-only GT   | E        | Callers |
| management GT    | E+L+S    | Owner |
| heartbeat GT     | E scoped | Policy lump |

### Methods

**Group 1 — Forwarding (primary)**

| Method | Parameters | Returns | Description |
|--------|-----------|---------|-------------|
| `invoke` | method_id, args | result | Forward call transparently to remote |

**Group 2 — Lifecycle**

| Method | Parameters | Returns | Description |
|--------|-----------|---------|-------------|
| `connect` | — | — | Explicitly establish connection |
| `disconnect` | — | — | Cleanly terminate connection |
| `status` | — | state | `connected` / `reconnecting` / `unreachable` / `cached-only` |
| `register_callback` | GT | — | Give remote a GT to call back |

**Group 3 — Management**

| Method | Parameters | Returns | Description |
|--------|-----------|---------|-------------|
| `update_policy` | policy_GT | — | Swap heartbeat/reconnect strategy |
| `update_target` | url | — | Accept new location from rendezvous |
| `cached` | — | state GT | Return last state without network round-trip |
| `flush` | — | — | Invalidate cache, force fresh fetch on next invoke |

### Policy Abstraction

The policy GT in c-list slot 3 controls Contact behaviour. It is swappable
independently — two Contact instances can share the same policy GT.

| Policy | Heartbeat | Reconnect | Cache TTL |
|--------|-----------|-----------|-----------|
| HighFrequency | 30 s | Immediate | Live state |
| OnDemand | None | On-call | Last response |
| Periodic | 1 hr | Lazy | Long TTL |

Only holders of the management GT can swap the policy. The swap is atomic —
the Contact adopts the new policy on the next heartbeat cycle.

---

## Session Abstraction

`Contact.establish(modality_GT)` returns a **Session GT**.

A Session is a temporary lump whose lifetime equals the duration of a single
communication. When the session ends, the Session lump's G bit is cleared by
the TSB — the next GC sweep nullifies all Session resources deterministically.
No explicit deallocation is required.

```
Contact  ->  establish(modality_GT)  ->  Session GT
Session  ->  open()                  ->  Channel GT
Channel  ->  send / recv / close
```

---

## Modalities

A modality is a first-class lump GT that defines the communication protocol
for a Session. Passing a different modality GT to `Contact.establish()` produces
a different kind of Session.

### The Five Modalities

| Modality       | Transfers       | Channel type              | Primary use |
|----------------|-----------------|---------------------------|-------------|
| API            | Structured data | Single CALL/RETURN        | RPC, queries |
| Text           | Messages        | Mailbox (async delivery)  | Chat, notifications |
| Voice          | Audio stream    | Real-time bidirectional   | Voice calls |
| Video          | A/V stream      | High-bandwidth real-time  | Video calls |
| Church Tunnel  | GTs             | Capability channel        | GT transfer, delegation |

### Three-Layer Stack

```
Contact  → where is the remote?    (location, routing, heartbeat)
Session  → how are we talking?     (protocol, lifetime, modality)
Channel  → the live pipe            (stream, mailbox, CALL, GT transfer)
```

### Church Tunnel

The Church Tunnel modality transfers **GTs**, not data bytes. It is the
substrate all other modalities can be built on.

**Wire format:** same zip structure as lump distribution.

```
Church Tunnel transfer of GT:
  1. Sender: TPERM -> attenuate perms to what receiver should hold
  2. Sender: package lump zip + GT credential (Word 0)
  3. If receiver already has lump cached (same SHA256): credential only
  4. Both ends: CRC validate GT structure before delivery
  5. Both ends: validate gt_seq — Word 0 gt_seq must match Object NS slot gt_seq;
               mismatch means GT revoked, transfer rejected
  6. Receiver: Mint validates + installs into receiver's namespace
```

**Mutual authentication:** both endpoints present PassKey credentials at
handshake before any GT crosses the tunnel. Neither side accepts GTs from
an unauthenticated peer.

**Permission attenuation:** the sender applies TPERM before transmission.
The receiver cannot hold stronger rights than the sender chose to grant.
This is enforced structurally — the GT itself carries only the attenuated perms,
and its CRC reflects that state.

---

## Security Properties

### Architectural (hardware-enforced, not bypassable)

| Property | Mechanism |
|----------|-----------|
| Turing/Church mutual exclusion | Data and capability instructions operate on strictly separate rights — see section above |
| GT unforgeable | Only Mint can issue GTs — raw bytes cannot be reinterpreted as capabilities |
| Execute isolation | Transient CR14 (X) grants X only — code is execute-only, DREAD cannot reach it, CALL requires a separate E-GT |
| C-list isolation | Transient CR6 (L) grants L only — callers can load capabilities out but cannot SAVE into slots |
| Permission non-escalation | TPERM can only remove permission bits, never add them; perms excluded from CRC enables pure-hardware TPERM |
| Entry point integrity | PC always starts at 1 — the header word cannot be executed |
| CRC check | Every LOAD validates CRC-16/CCITT (poly 0x1021) over Word 0[24:0] + Word 1 + Word 2 — tampered GTs fault immediately |
| SAVE gating | B=0 in Word 0 bit 31 causes SAVE to fault — private GTs (PassKeys, session GTs) cannot be copied; B is TPERM-controllable |
| GC correctness | Mark-and-sweep via G bit — cycles collected, no per-operation overhead, no reference count drift |

### Policy (Mint + Namespace enforced)

| Property | Mechanism |
|----------|-----------|
| Tamper detection | Mint binds GT to exact zip bytes — any modification invalidates the GT |
| Type safety | `typ` field in header word and NS slot — lump vs data vs clist-only |
| Slot isolation | MintCL issues a fresh, empty c-list — no leftover capabilities from prior use |
| Install authority | NamespaceWrite E-GT held only by Locator — callers cannot self-install lumps |
| Eviction safety | NamespaceWrite E-GT held only by Locator — no per-slot flag needed |
| Content integrity | SHA256 hash in URL verified before inflate — tamper detected before Mint sees bytes |
| Revocation | gt_seq in Word 0 matched against Object NS slot at LOAD — stale GTs in any c-list rejected on use |
| Import resolution | Import table resolved at install time by Mint — callers never see unresolved slot indices |
