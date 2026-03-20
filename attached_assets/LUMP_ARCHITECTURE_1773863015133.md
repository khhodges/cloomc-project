# Church Machine Lump Architecture

## Overview

A **lump** is the fundamental deployable unit of the Church Machine. It is a
contiguous, capability-secured memory region containing an executable code
section and an optional capability list (c-list). Every abstraction compiles
to exactly one lump.

---

## Lump Size Rule

Lump size is always a power of 2, minimum 64 words (32-bit each):

    lumpSize = 2^n   where n >= 6
    freespace = lumpSize - codeWords - clistCount

| Abstraction | Code words | C-list slots | Lump size         | Freespace  |
|-------------|-----------|-------------|-------------------|------------|
| Decimal     | 107       | 0           | 2^7 = 128 words   | 21 words   |
| SlideRule   | 525       | 1           | 2^10 = 1024 words | 498 words  |
| TestSR      | 604       | 1           | 2^10 = 1024 words | 419 words  |

---

## Lump Memory Layout

```
Word 0:           Header word (metadata, never executed)
Words 1..N:       Code section — dispatcher at PC=1, then methods
Words N+1..M:     Freespace (all zeros, compresses efficiently)
Words M+1..end:   C-list slots (one word each, top of lump)
```

Hardware entry point is always **PC = 1** — Word 0 is the self-describing
header, not an instruction.

---

## The Header Word (Word 0)

The first word of every lump binary is a metadata descriptor. It uses opcode
`0x1F` (`11111b`) — an undefined instruction on the Church Machine ISA. If
Word 0 were accidentally executed, the hardware traps rather than silently
corrupting state.

```
31      27 26    23 22      19 18        11 10    8 7        0
+----------+--------+----------+------------+-------+----------+
|  0x1F[5] | ver[4] |  n-6[4]  |   cc[8]    | typ[3]| flags[8] |
+----------+--------+----------+------------+-------+----------+
```

| Field | Bits  | Meaning |
|-------|-------|---------|
| magic | 31:27 | Always `11111` (0x1F). Traps if executed. |
| ver   | 26:23 | Header format version. Currently 0. |
| n-6   | 22:19 | Lump size = 2^(val+6). Range: 2^6=64 to 2^21=2M words. |
| cc    | 18:11 | clistCount — number of c-list slots (0..255). |
| typ   | 10:8  | Object type: `000`=lump, `001`=data, `010`=clist-only. |
| flags | 7:0   | See below. |

### Flags (bits 7:0)

| Bit | Name       | Meaning |
|-----|------------|---------|
|  7  | network    | 1 = canonical URL exists for this lump |
|  6  | compressed | 1 = was delivered as zip (informational, post-inflate) |
| 5:0 | reserved   | Must be zero |

### Example Header Words

Using formula: `(0x1F << 27) | (ver << 23) | ((n-6) << 19) | (cc << 11) | (typ << 8) | flags`

```
Decimal    (n=7,  cc=0):  0xF808_0000
SlideRule  (n=10, cc=1):  0xF820_0800
TestSR     (n=10, cc=1):  0xF820_0800   (same descriptor shape, different code)
```

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
| 24:23 | typ   | —      | —            | GT class: 00=NULL, 01=Real, 10=Abstract, 11=reserved — CRC covered |

**B is in Word 0 bit 31 and is stored with the GT.** It survives context switch
naturally. TPERM can clear B (reducing delegation depth) but never add it.

**R, W, X and L, S, E are mutually exclusive groups.** Any GT with bits from
more than one group is rejected by Mint as malformed. **NULL GT** is all 128
bits zero (typ=00, all rights clear). **Abstract GT** is typ=10 — hardware
returns a constant on DREAD; no RAM region exists.

### Standard GT combinations

| GT type           | typ | permissions [31:25] | Object NS slot? | Description |
|-------------------|-----|---------------------|-----------------|-------------|
| X-GT (code)       | 01  | B X                 | Yes             | Turing: execute-only — PC may enter |
| L-GT (c-list)     | 01  | B L                 | Yes             | Church: load capabilities out |
| RW-GT (data)      | 01  | B R W               | Depends on use  | Turing: full data read/write |
| R-GT (read-only)  | 01  | B R                 | Depends on use  | Turing: read-only data |
| LS-GT (MintCL)    | 01  | B L S               | Depends on use  | Church: full capability read/write |
| E-GT (call gate)  | 01  | B E                 | Depends on use  | Church: callable via CALL |
| **NULL GT**       | 00  | 0 (all clear)       | No              | All bits zero — faults on any use |
| **ABSTRACT GT**   | 10  | 0 (no rights)       | **No**          | Self-defining constant or PassKey — no RAM |

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
00=NULL, 01=Real, 10=Abstract, 11=reserved. This is CRC-covered and visible at
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
| typ       | 24:23 | GT class: 00=NULL, 01=Real, 10=Abstract, 11=reserved — CRC covered |
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
| NS slot Words 1 & 2 when PR=0 | Set to `0x00000000` — lump evicted, GT indices invalid until re-fetched |
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
  NS slot → Locator → fetch → Mint.Lump() → X-GT + L-GT → CALL

Abstract GT lifecycle:
  CR is the GT → DREAD → hardware returns value
  (no NS slot, no fetch, no Mint step, no memory region)
```

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

## Golden Token Split — Mint.Lump

`Mint.Lump(zipFile)` inflates the zip, reads and validates the header word,
then issues two Golden Tokens using the 128-bit CR structure:

| Token    | Region                          | permissions | Mounted as |
|----------|---------------------------------|-------------|------------|
| **X-GT** | Words 1..lumpSize-cc-1          | B X         | CR14       |
| **L-GT** | Words lumpSize-cc..lumpSize-1   | B L         | CR6        |

If `cc = 0` (e.g. Decimal): no L-GT is issued and CR6 is not populated.

### Mint validation steps

```
1. Inflate zip -> raw words
2. Read header word (Word 0 of lump binary)
3. Check bits[31:27] == 0x1F            -> valid lump header?
4. Extract ver                           -> version check
5. Extract n-6 -> lumpSize = 2^(n-6+6)  -> verify inflated_length == lumpSize
6. Extract cc  -> c-list starts at word (lumpSize - cc)
7. Extract typ -> must be 0x0 for a callable lump
8. Assign object_id from pool; assign gt_seq from pool counter
9. Compute physical base addresses: code_base, clist_base
10. Build X-GT CR (4 words):
     Word 0: B=1 X=1 | typ=01 | gt_seq[7] | object_id[16]
     Word 1: code_base [32]
     Word 2: spare[4] | gt_seq[7] | (lumpSize-cc-2) [limit_offset, 21 bits]
     Word 3: spare[15] | G=0 | CRC-16/CCITT(Word0[24:0] + Word1 + Word2) [16]
11. Build L-GT CR (4 words, only if cc > 0):
     Word 0: B=1 L=1 | typ=01 | gt_seq[7] | object_id[16]
     Word 1: clist_base [32]
     Word 2: spare[4] | gt_seq[7] | (cc-1) [limit_offset, 21 bits]
     Word 3: spare[15] | G=0 | CRC-16/CCITT(Word0[24:0] + Word1 + Word2) [16]
12. Write Object NS slot (Words 1+2 mirror CR Words 1+2 for LOAD reconstruction)
13. Return X-GT, L-GT, and the 3-word Lump NS slot entry
```

---

## The 3-Word Namespace Slot

Each lump occupies exactly **one namespace slot** consisting of three 32-bit
words. This is everything the hardware needs to service a CALL.

### Word 0 — Slot Descriptor

```
31    28 27    24 23    20 19          12 11           0
+-------+--------+--------+--------------+-------------+
|typ[4] | ver[4] | n-6[4] |    cc[8]     |   loc[12]   |
+-------+--------+--------+--------------+-------------+
```

| Field | Meaning |
|-------|---------|
| typ   | 0x1 = lump slot |
| ver   | NS slot format version |
| n-6   | Lump size exponent — used for eviction/swap sizing decisions |
| cc    | clistCount (0..255) — 0 means Word 2 is null, hardware skips CR6 setup |
| loc   | Locator abstraction's namespace slot index (0 = local-only, no network) |

PR (Present) and SR (Swap Required) have moved to the high bits of Words 1 and
2 — they describe the validity of the GT indices in those words, so they sit
adjacent to those indices.

### Word 1 — X-GT Reference

```
31  30  29                                            0
+---+---+----------------------------------------------+
|PR |SR |              X-GT index  [30 bits]           |
+---+---+----------------------------------------------+
```

| Field | Meaning |
|-------|---------|
| PR    | Present — lump is in local memory, X-GT index in bits 29:0 is valid |
| SR    | Swap Required — lump evicted, raise fault to Locator before CALL can proceed |
| X-GT index | 30-bit index into the GT store for the execute-only code region |

Hardware fetches the 128-bit GT using this index and mounts it as CR14.
When PR=0: both bits 29:0 are `0x00000000`.

### Word 2 — L-GT Reference

```
31  30                                                0
+---+--------------------------------------------------+
|PR |              L-GT index  [31 bits]               |
+---+--------------------------------------------------+
```

| Field | Meaning |
|-------|---------|
| PR    | Present — L-GT index in bits 30:0 is valid (only meaningful when cc > 0) |
| L-GT index | 31-bit index into the GT store for the load-only c-list region |

Hardware fetches the 128-bit GT and mounts it as CR6.
When cc=0 or PR=0: all bits are `0x00000000`.

### Concrete NS Slot Values

```
Decimal    typ=1, ver=0, n-6=1, cc=0, loc=0, PR=1, SR=0
  Word 0:  0x1010_0000
  Word 1:  0x8000_0000 | <decimal_x_gt_index>   (PR=1, SR=0)
  Word 2:  0x0000_0000                           (no c-list)

SlideRule  typ=1, ver=0, n-6=4, cc=1, loc=0, PR=1, SR=0
  Word 0:  0x1040_1000
  Word 1:  0x8000_0000 | <sliderule_x_gt_index>  (PR=1, SR=0)
  Word 2:  0x8000_0000 | <sliderule_l_gt_index>  (PR=1)

TestSR     typ=1, ver=0, n-6=4, cc=1, loc=0, PR=1, SR=0
  Word 0:  0x1040_1000
  Word 1:  0x8000_0000 | <testsr_x_gt_index>     (PR=1, SR=0)
  Word 2:  0x8000_0000 | <testsr_l_gt_index>     (PR=1)
```

GT store entries pointed to by Words 1 and 2 carry the full 128-bit CR format:
base in CR Word 1, limit_offset in CR Word 2, CRC+G in CR Word 3.

### Object NS Slot

Distinct from the 3-word Lump NS slot above. The Object NS slot is a per-GT
issuance record used by LOAD to reconstruct the full 128-bit CR from the 32-bit
Word 0 stored in a c-list slot.

**Why it exists:** SAVE copies only Word 0. When LOAD reads that Word 0 from the
c-list, it must obtain Words 1, 2, and 3 from somewhere authoritative. The Object
NS slot provides them.

**Format:** Object NS slot Words 1+2 mirror CR Words 1+2 exactly:

```
Object NS slot Word 1:  base [32]
Object NS slot Word 2:  spare[4] | gt_seq[7] | limit_offset[21]
```

**Revocation** is performed here: Mint increments gt_seq in Object NS slot Word 2.
On LOAD, hardware compares Word 0 gt_seq against Object NS slot gt_seq. A mismatch
means the GT is revoked — LOAD nullifies the c-list slot (writes NULL GT) and faults;
the destination CR is not populated. Stale GTs in any c-list are self-cleaning:
each one is nullified the first time LOAD encounters it.

---

## CALL Execution Flow

```
CALL namespace[i]
  1. Read NS slot Word 0 -> check typ=0x1, read cc, loc; compute lump size from n-6
  2. Read NS slot Word 1 -> check PR bit (bit 31)
     If PR=0 and SR=1:
       -> raise "lump absent" fault
       -> Locator abstraction (namespace[loc]) handles fetch:
            resolve label -> URL
            fetch URL -> zip bytes
            Mint.Lump(zip) -> X-GT + L-GT (full 128-bit CRs)
            write NS slot Words 1 & 2 (with PR=1, SR=0 in high bits)
       -> retry CALL
  3. Read NS slot Word 1 bits 29:0 -> fetch 128-bit X-GT from GT store -> CR14
  4. If cc > 0: read NS slot Word 2 bits 30:0 -> fetch 128-bit L-GT -> CR6
  5. Set PC = 1 (skip lump header word)
  6. Execute dispatcher
```

---

## Zip File Formats

Lump binaries are distributed as zip files. The ZIP container format is
standard; the compression method inside varies by context.

### Case 1 — Single Lump Upload

```
SlideRule@v1.lump.zip
+-- SlideRule@v1.bin    <- raw lump binary, 2^n x 4 bytes
```

Filename encodes label and version. DEFLATE compression is highly effective
on the freespace region (consecutive zero words).

### Case 2 — Namespace Bundle

```
namespace.zip
+-- manifest.json       <- install order + dependency declarations
+-- Decimal@v1.bin
+-- SlideRule@v1.bin
+-- TestSR@v1.bin
```

Each `.bin` is independently self-describing via its header word. Mint.Lump
is called once per entry, in dependency order declared by the manifest.

### Case 3 — Network-Cached Lump

```
cm://domain/SlideRule@sha256:a3f9c2.../v1
```

Same format as Case 1. The SHA256 hash covers the entire zip file. Any node
that holds the cached zip can serve it. Mint re-validates the hash before
inflating.

### Compression Methods

| Method        | CM method field | Use case                    | Inflate cost in CM ISA       |
|---------------|-----------------|-----------------------------|------------------------------|
| STORE         | 0               | Integrity check only         | Trivial — copy words directly |
| DEFLATE       | 8               | Offline compiler tools       | High — Huffman + LZ77        |
| RLE (custom)  | n/a             | CM-native inflate abstraction| Low — ~50 instructions       |

The freespace region (all zeros) compresses near-perfectly under any method.
A 419-word freespace block compresses to a handful of bytes under RLE.

---

## Import Table

At compile time the compiler knows which remote abstractions will fill specific
c-list slots but cannot yet know the GT index (the GT is not issued until
install time). The import table bridges this gap.

### Format

The import table is a separate file inside the lump zip, alongside the binary:

```
SlideRule@v1.lump.zip
+-- SlideRule@v1.bin              <- raw lump binary
+-- SlideRule@v1.imports.json     <- import table
```

The import table maps c-list slot index to a content-addressed URL:

```json
{
  "0": "cm://decimal.net/Decimal@sha256:a3f9c2.../v1",
  "1": null,
  "2": null
}
```

A `null` entry means the slot holds an Abstract GT issued directly by hardware —
no URL is needed or possible.

### Install-Time Resolution

At install time Mint reads the import table, resolves each non-null URL through
the Locator, and populates the c-list slots with the resolved GTs. Local objects
(same namespace) reference by slot index directly — no import table entry.

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
| `evict`    | label        | —            | Revokes GTs, sets PR=0, SR=1 in NS slot |
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

| Version | Behaviour |
|---------|-----------|
| v1      | Label-to-URL table, plain HTTP |
| v2      | DHT lookup, peer-to-peer fetch |
| v3      | Policy routing: LAN cache -> CDN -> origin |
| v4      | Signed manifests, publisher verification chains |

---

## Content-Addressed URLs

Lumps are identified by content hash, following a content-addressed scheme:

```
cm://domain/SlideRule@sha256:a3f9c2.../v1
```

| Property | Explanation |
|----------|-------------|
| Tamper detection | Hash covers entire zip — any modification breaks it before Mint sees it |
| Location independence | Any node with the zip can serve it — no single origin dependency |
| Cache correctness | Same hash always yields the same X-GT + L-GT identity |
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
| Execute isolation | X-GT grants X only — code is execute-only, DREAD cannot reach it, CALL requires a separate E-GT |
| C-list isolation | L-GT grants L only — callers can load capabilities out but cannot SAVE into slots |
| Permission non-escalation | TPERM can only remove permission bits, never add them; perms excluded from CRC enables pure-hardware TPERM |
| Entry point integrity | PC always starts at 1 — header word cannot be executed |
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
