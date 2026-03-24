# Church Machine — Lump Specification

## Overview

A **lump** is the fundamental deployable unit of the Church Machine. It is a
contiguous, capability-secured memory region containing an executable code
section and an optional capability list (c-list). Every function abstraction
compiles to exactly one lump. The Thread is a specialised lump whose code
section is replaced by a live execution context — see Appendix A.

---

## Lump Size Rule

Lump size is always a power of 2, minimum 64 words, maximum 16 384 words
(32-bit each):

```
lumpSize = 2^n   where 6 ≤ n ≤ 14
freespace = lumpSize - 1 - cw - cc   (must be all-zero; Mint verifies at load time)
```

The maximum is 2^14 = 16 384 words. The header `cw` field (13 bits, max 8 191)
and `cc` field (8 bits, max 255) together cap the maximum useful payload at
1 + 8 191 + 255 = 8 447 words. Mint hard-rejects n-6 > 8 (lumpSize > 16 K).

| Abstraction | Code words (cw) | C-list slots (cc) | Lump size         | Freespace  |
|-------------|-----------------|-------------------|-------------------|------------|
| Decimal     | 107             | 0                 | 2^7 = 128 words   | 20 words   |
| SlideRule   | 525             | 1                 | 2^10 = 1 024 words | 497 words |
| TestSR      | 604             | 1                 | 2^10 = 1 024 words | 418 words |
| Boot.Abstr  | 0               | 46                | 2^8 = 256 words   | 209 words  |

---

## Lump Memory Layout — Function Abstraction

```
┌─────────────────────────────────────────────────────────┐  ← base (word 0)
│  Word 0     Header word   [metadata — never executed]   │
├─────────────────────────────────────────────────────────┤  ← word 1  (PC = 1)
│  Words 1 … cw   Code section                           │
│                 Dispatcher at PC = 1, then methods      │
├─────────────────────────────────────────────────────────┤  ← word cw + 1
│  Words cw+1 … lumpSize-cc-1   Freespace                │
│                 All zeros — verified by Mint at load    │
├─────────────────────────────────────────────────────────┤  ← word lumpSize - cc
│  Words lumpSize-cc … lumpSize-1   C-list               │
│                 cc × 1-word GT slots (Word 0 only)      │
└─────────────────────────────────────────────────────────┘  ← word lumpSize - 1
```

Hardware entry point is **PC = 1** — Word 0 is the header and is never
executed. The c-list is pre-populated by the compiler at build time and
anchors at the tail of the lump.

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
| n-6   | 26:23 | lumpSize = 2^(val+6). Valid range 0..8 → 64..16 384 words. Values 9..15 rejected by Mint. |
| cw    | 22:10 | Code word count (0..8191). Words 1..cw are code; words cw+1..lumpSize-cc-1 must be zero. |
| typ   | 9:8   | Object type: `00`=lump, `01`=data, `10`=clist-only, `11`=Outform. |
| cc    | 7:0   | C-list slot count (0..255). |

32 bits total. No spare bits. No dead fields. `code_base = base + 4` always.
`PC = 1` always.

### Example Header Words

Encoding formula: `(0x1F << 27) | ((n-6) << 23) | (cw << 10) | (typ << 8) | cc`

```
Decimal    (n=7,  cw=107, cc=0, typ=00):  0xF881_AC00
SlideRule  (n=10, cw=525, cc=1, typ=00):  0xFA08_3401
Boot.Abstr (n=8,  cw=0,   cc=46, typ=00): 0xF900_002E
```

---

## Mint Validation Sequence

`Mint.Lump(base, n)` receives a lump already inflated into physical memory.
It validates the header and binary before issuing any GT.

```
Step 1  Read Mem[base] — the header word.
Step 2  magic[31:27] == 0x1F — reject if not.
Step 3  n-6[26:23] <= 8   — reject if n-6 > 8 (lump would exceed 16 K words).
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

The Church Machine's 20 instructions divide into two completely independent
groups with mutually exclusive access rights. A memory region carries rights
from one group only — never both. This is enforced at the hardware
instruction-decode level, not by software policy.

```
┌────────────────────────────────────────┐  ┌────────────────────────────────────────┐
│       TURING instructions              │  │       CHURCH instructions              │
│       (Data side)                      │  │       (GT / Capability side)           │
├────────────────────────────────────────┤  ├────────────────────────────────────────┤
│  DREAD   DWRITE                        │  │  LOAD    SAVE                          │
│  IADD    ISUB    SHL    SHR            │  │  CALL    RETURN                        │
│  MCMP    BRANCH                        │  │  LAMBDA  TPERM                         │
│  BFEXT   BFINS                         │  │  ELOADCALL  XLOADLAMBDA                │
│                                        │  │  CHANGE  SWITCH                        │
│  Access rights:  R  W  X               │  │  Access rights:  L  S  E               │
└────────────────────────────────────────┘  └────────────────────────────────────────┘
         operate on DATA memory                    operate on CAPABILITIES only
         cannot reach GTs                          cannot reach data memory
```

### Permission Bit Definitions

Word 0 of every GT encodes the TPERM-controllable permission bits at [31:25],
the GT class at [24:23], and identity fields below that:

| Bits  | Field | Side   | Instruction | Meaning |
|-------|-------|--------|-------------|---------|
| 31    | B     | Church | SAVE        | Bind — B=1 allows SAVE; B=0 causes SAVE to fault |
| 30    | R     | Turing | DREAD       | Read data words from this region |
| 29    | W     | Turing | DWRITE      | Write data words to this region |
| 28    | X     | Turing | —           | Region is executable (PC may enter) |
| 27    | L     | Church | LOAD        | Load a capability out of this region |
| 26    | S     | Church | SAVE        | Save a capability into this region |
| 25    | E     | Church | CALL        | This GT is a valid CALL target |
| 24:23 | typ   | —      | —           | GT class: 00=NULL, 01=Real, 10=Abstract, 11=Outform — CRC covered |

**R, W, X and L, S, E are mutually exclusive groups.** Any GT with bits from
more than one group is rejected by Mint as malformed.

### Standard GT Combinations

| GT type           | typ | permissions [31:25] | Description |
|-------------------|-----|---------------------|-------------|
| E-GT (lump gate)  | 01  | B E                 | Church: callable lump — the only issued lump GT |
| RW-GT (data)      | 01  | B R W               | Turing: full data read/write |
| R-GT (read-only)  | 01  | B R                 | Turing: read-only data |
| LS-GT (MintCL)    | 01  | B L S               | Church: full capability read/write |
| NULL GT           | 00  | 0 (all clear)       | All bits zero — faults on any use |
| ABSTRACT GT       | 10  | 0 (no rights)       | Self-defining constant or PassKey — no RAM |
| OUTFORM GT        | 11  | (any)               | Lump registered but not yet resident — fires Absent event on LOAD |
| *(CR14 transient)*| 01  | B X                 | Derived from NS slot on CALL; never issued or stored |
| *(CR6 transient)* | 01  | B L                 | Derived from NS slot on CALL; never issued or stored |

---

## GT Taxonomy — Three Fundamental Classes

Every GT belongs to exactly one of three fundamental classes, identified by
`typ[2]` in Word 0 bits [24:23]. This is CRC-covered and visible to hardware
at instruction-decode time.

### NULL GT (typ = 00)

All 128 bits zero. Faults on any CALL, LOAD, or DREAD. Occupies every
unoccupied c-list slot. Never issued by Mint.

### Real GT (typ = 01)

Issued by Mint. References a physical memory region. The R/W/X or L/S/E
permission bits describe what the holder may do with that region.

### Abstract GT (typ = 10)

Self-defining. No memory region, no Object NS slot. Hardware maps
`object_id → value` internally. Covers physical constants (DREAD returns a
fixed value) and PassKey credentials (opaque identity tokens). Abstract GTs
are distributed by writing the full CR directly into c-list slots — no NS
slot consumed.

### Outform GT (typ = 11)

Lump registered in the namespace but not resident. An IDE-managed 96-bit
token is held in NS Words 1–3. LOAD fires an Absent event; the Locator
fetches, inflates, and Mints the lump, promoting the slot to Live.

---

## Context Register (CR) — 128-bit Structure

A CR is four 32-bit words stored in a hardware register file (CR0..CR15 per
thread).

```
┌──────────────────────────────────────────────────────────────┐
│  Word 3 [127:96]  CRC and GC  (spare[15] | G[1] | CRC[16]) │
├──────────────────────────────────────────────────────────────┤
│  Word 2 [95:64]   Limit and revocation                      │
│                   (spare[4] | gt_seq[7] | limit_offset[21]) │
├──────────────────────────────────────────────────────────────┤
│  Word 1 [63:32]   Base address [32]                         │
├──────────────────────────────────────────────────────────────┤
│  Word 0 [31:0]    GT — the holder's credential (per-holder) │
│                   SAVE copies this word only                 │
└──────────────────────────────────────────────────────────────┘
```

### Word 0 — The Golden Token (per-holder credential)

```
31      25 24  23 22      16 15            0
+─────────┬──────┬──────────┬──────────────+
│B R W X  │ typ  │  gt_seq  │  object_id   │
│ L S E   │ [2]  │   [7]    │    [16]      │
│  [7]    │      │          │              │
+─────────┴──────┴──────────┴──────────────+
```

| Field         | Bits  | Meaning |
|---------------|-------|---------|
| B R W X L S E | 31:25 | Permissions — TPERM-changeable, **excluded from CRC** |
| typ           | 24:23 | GT class: 00=NULL, 01=Real, 10=Abstract, 11=Outform — CRC covered |
| gt_seq        | 22:16 | Revocation sequence number — CRC covered |
| object_id     | 15:0  | Object index, unique per lump issuance — CRC covered |

TPERM clears any subset of bits [31:25] to produce a weaker GT. Permission
escalation is architecturally impossible.

### Word 1 — Base Address

Physical base address of the memory region. CRC covered.

### Word 2 — Limit and Revocation

```
95  92 91      85 84                          64
+──────+──────────+────────────────────────────+
│spare │  gt_seq  │       limit_offset [21]     │
│ [4]  │   [7]    │                             │
+──────+──────────+────────────────────────────+
```

**Revocation:** Mint increments gt_seq in the Object NS slot. On LOAD,
hardware checks Word 0 gt_seq against Word 2 gt_seq — a mismatch means the
GT has been revoked and the LOAD faults.

### Word 3 — CRC and GC

```
127         113 112 111                     96
+─────────────┬───┬──────────────────────────+
│  spare [15] │ G │        CRC [16]          │
+─────────────┴───┴──────────────────────────+
```

CRC is CRC-16/CCITT (poly 0x1021) over Word 0[24:0] + Word 1[all] +
Word 2[all]. Permission bits [31:25] are **excluded** — TPERM requires no
CRC recomputation.

---

## Mint.Lump — One E-GT, One NS Slot

`Mint.Lump(base, n)` issues exactly **one E-GT** and writes **one NS slot**.
Transient CR14 and CR6 are derived fresh on every CALL — they are never
issued or stored.

| Token    | Region                         | Permissions | Mounted as   | Issued? |
|----------|--------------------------------|-------------|--------------|---------|
| **E-GT** | Entire lump (word 0..size-1)   | B E         | held by caller | Yes — only issued GT |
| CR14 (X) | Words 1..lumpSize-cc-1         | B X         | CR14 on CALL | No — transient only |
| CR6  (L) | Words lumpSize-cc..lumpSize-1  | B L         | CR6 on CALL  | No — transient only |

If `cc = 0`: CR6 is NULL GT after CALL; the derived X view still covers the
full code section.

---

## The Object NS Slot

Each lump occupies exactly one Object NS slot (three 32-bit words). Word 0
(the Golden Token) is held privately by the owner — it is never stored in
the NS slot.

```
NS Word 1  base [32]               — physical byte address of lump word 0
NS Word 2  spare[4] | gt_seq[7] | limit_offset[21]
NS Word 3  spare[15] | G[1] | CRC[16]
```

CALL reads the lump header word directly from `Mem[base]` to obtain
`n_minus_6` and `cc`. No cached copy is held in the NS slot.

---

## CALL Execution Flow

```
CALL CR_s   (CR_s holds the E-GT for the target lump)
  1. Validate E-GT CRC — FAULT if mismatch
  2. Read object_id and gt_seq from E-GT Word 0
  3. Fetch NS[object_id] — 3 words: base, gt_seq_ns, limit_offset
     Read Mem[base] → lump header word:
       n_minus_6 = Mem[base][22:19]
       cc        = Mem[base][18:11]
     If lump not present (evicted / Outform): invoke Locator, retry
  4. Revocation check: if E-GT gt_seq != NS gt_seq_ns -> FAULT
  5. Derive lumpSize = 1 << (n_minus_6 + 6)
  6. Build transient CR14 (X):
       base+4, limit = lumpSize-cc-2, gt_seq, CRC
  7. If cc > 0: build transient CR6 (L):
       base+(lumpSize-cc)*4, limit = cc-1, gt_seq, CRC
     Else: CR6 ← NULL GT
  8. PC ← 1
  9. Execute dispatcher
```

---

## C-List — Compiler-Populated

The IDE toolchain pre-fills every c-list slot at compile time. Each c-list
slot is **one 32-bit word** — Word 0 of the GT only. LOAD reads Word 0 from
the c-list, then fetches Words 1–3 from the NS table.

| Slot Word 0 value       | typ | Meaning |
|-------------------------|-----|---------|
| B\|perms\|typ=01\|gt_seq\|object_id | 01=Real | Regular lump or data GT |
| typ=10\|object_id       | 10=Abstract | Physical constant or PassKey — self-defining |
| typ=11\|object_id       | 11=Outform | IDE-managed dep — Absent event fires on first LOAD |
| 0x00000000              | 00=NULL | Unused slot |

`cc` is the slot count. The c-list occupies the last `cc` words of the lump.

---

## Zip Distribution Format

Lump binaries are distributed as zip files.

### Single Lump Upload

```
SlideRule.lump.zip
+-- SlideRule.bin    ← raw lump binary: header + code + freespace + c-list
```

### Namespace Bundle

```
namespace.zip
+-- manifest.json   ← install order + dependency declarations
+-- Decimal.bin
+-- SlideRule.bin
+-- TestSR.bin
```

### Network-Cached Lump

```
cm://domain/SlideRule@sha256:a3f9c2...
```

The SHA256 hash covers the lump binary. Any node holding the binary can
serve it. **Bit 3** of the ZIP general-purpose flags must be 0 (no data
descriptor). The Locator rejects any lump zip where bit 3 is 1 or the
uncompressed-size field is zero.

### ZIP Pre-Allocation Sequence

```
1. Verify signature = 0x04034B50
2. Assert bit 3 of flags = 0 — reject if streaming mode
3. Read uncompressed_size at offset 24
4. Derive n = log2(uncompressed_size / 4)
   Reject if not power-of-2 multiple of 4, or n < 6
5. Call Memory Manager with n → receive base
6. Inflate compressed payload into [base, base + uncompressed_size)
7. Verify ZIP CRC-32 — reject on mismatch
8. Hand (base, n) to Mint.Lump()
```

---

## Security Properties

### Architectural (hardware-enforced, not bypassable)

| Property | Mechanism |
|----------|-----------|
| Turing/Church mutual exclusion | Data and capability instructions operate on strictly separate rights |
| GT unforgeable | Only Mint issues GTs — raw bytes cannot be reinterpreted as capabilities |
| Execute isolation | Transient CR14 grants X only — code is execute-only, DREAD cannot reach it |
| C-list isolation | Transient CR6 grants L only — callers can load capabilities out but cannot SAVE into slots |
| Permission non-escalation | TPERM can only remove bits, never add; perms excluded from CRC enables pure-hardware TPERM |
| Entry point integrity | PC always starts at 1 — the header word cannot be executed |
| CRC check | Every LOAD validates CRC-16/CCITT over Word 0[24:0] + Word 1 + Word 2 |
| SAVE gating | B=0 in Word 0 bit 31 causes SAVE to fault — PassKeys and session GTs cannot be copied |
| GC correctness | Mark-and-sweep via G bit — cycles collected, no per-operation overhead |

### Policy (Mint + Namespace enforced)

| Property | Mechanism |
|----------|-----------|
| Tamper detection | Mint binds GT to exact zip bytes — any modification invalidates the GT |
| Type safety | `typ` field in header word and NS slot |
| Slot isolation | MintCL issues a fresh, empty c-list — no leftover capabilities |
| Install authority | NamespaceWrite E-GT held only by Locator |
| Content integrity | SHA256 hash in URL verified before inflate |
| Revocation | gt_seq in Word 0 matched against Object NS slot at LOAD |

---

## Concrete Lump Examples

### Decimal (n=7, cw=107, cc=0)

```
Header:  0xF881_AC00
  magic=0x1F  n-6=1 (2^7=128)  cw=107  typ=00  cc=0

Layout (128 words):
  Word 0:         0xF881_AC00  [header]
  Words 1..107:   CLOOMC code  [107 words]
  Words 108..127: freespace    [20 zeros]
  C-list:         (none)

NS Slot (gt_seq=0x01, base=0x20000000):
  Word 1:  0x20000000
  Word 2:  0x0020007F  (gt_seq=0x01, limit_offset=127)
  Word 3:  0x00004CEF  (E-GT CRC)
```

### SlideRule (n=10, cw=525, cc=1)

```
Header:  0xFA08_3401
  magic=0x1F  n-6=4 (2^10=1024)  cw=525  typ=00  cc=1

Layout (1024 words):
  Word 0:          0xFA08_3401  [header]
  Words 1..525:    CLOOMC code  [525 words]
  Words 526..1022: freespace    [497 zeros]
  Word 1023:       PI abstract GT (Word 0 only)  [c-list, cc=1]

NS Slot (gt_seq=0x01, base=0x10000000):
  Word 1:  0x10000000
  Word 2:  0x002003FF  (gt_seq=0x01, limit_offset=1023)
  Word 3:  0x000048F3  (E-GT CRC — illustrative)
```

### Boot.Abstr (n=8, cw=0, cc=46) — simulator boot-time lump

```
Header:  0xF900_002E
  magic=0x1F  n-6=2 (2^8=256)  cw=0  typ=00  cc=46

Layout (256 words):
  Word 0:          0xF900_002E  [header]
  Words 1..0:      (no code — cw=0)
  Words 1..209:    freespace    [209 zeros]
  Words 210..255:  c-list       [46 GT words, one per NS slot 0..45]

Note: clistStart = lumpSize - cc = 256 - 46 = 210
```

---

---

# Appendix A — Thread as a Lump

## Overview

The Thread is a lump in the sense that it occupies a capability-secured,
power-of-2 memory region assigned a single E-GT by Mint at creation time.
However, the Thread lump is **not a function abstraction** — its memory
region is not organised as header + code + freespace + c-list. Instead it
holds the live execution state of a running computation. The Thread lump has
**no header word** and **no code section**. PC never enters it. It is a
purely data structure from the hardware's perspective, accessed only via
transient data and capability views set up by CALL and RETURN.

The Church Machine has two fundamentally different lump types:

| Property          | Function Abstraction lump       | Thread lump                  |
|-------------------|---------------------------------|------------------------------|
| Word 0            | Header (magic 0x1F, metadata)   | CR0 — first capability register |
| Entry point       | PC = 1 on every CALL            | Never — Thread is data, not code |
| Freespace         | Fixed at compile time, verified | Zone ③ — dynamic, grows/shrinks at runtime |
| C-list            | Tail of lump, compiler-fixed    | Zone ① — CR0..CR11, runtime-managed |
| Issued GT         | E-GT (callable, B E)            | E-GT scoped to full region (B E) |
| Transient CR14    | Code view (X), words 1..cw      | Not derived — Thread is not called |
| Transient CR6     | C-list view (L), tail words     | Not derived — Thread owns its own CR file |

---

## Thread Lump Memory Layout

The Thread lump is 256 words (SLOT_SIZE). Word addresses increase downward
from the base.

```
┌─────────────────────────────────────────────┐  ← base  (+0)
│  ① Capabilities                             │  [12 words]
│     CR0 … CR11 — Golden Token words         │  (one 32-bit GT Word 0 per slot)
│     Fixed zone — mLoad keeps this zone      │
├─────────────────────────────────────────────┤  ← base  (+12)  ← stack top
│  ② LIFO Stack  ↓                            │  [32 words]
│     CALL: 2-word frame  [E-GT · frame word] │
│     LAMBDA: 1-word frame  [frame word]      │
│     Grows downward; SP tracks depth         │
├─────────────────────────────────────────────┤  ← base  (+44)
│  ③ Freespace                                │  [132 words]
│     Unallocated — dynamic                   │
│     Shrinks as Stack grows ↓                │
│     Shrinks as Heap grows ↑                 │
│     Mint verifies all-zero at creation time │
├─────────────────────────────────────────────┤  ← base  (+176)  ← heap base
│  ④ Heap  ↑                                  │  [64 words]
│     Fixed size — set by IDE at design time  │
│     Objects allocated from heap base upward │
│     Grows toward Freespace                  │
├─────────────────────────────────────────────┤  ← base  (+240)  ← DR base
│  ⑤ Data Registers                           │  [16 words]
│     DR0 … DR15 — 32-bit registers           │
│     Fixed zone — always at lump base end    │
└─────────────────────────────────────────────┘  ← base  (+255)
```

### Zone Constants (all offsets from Thread lump base)

| Zone | Identifier | Offset range | Words | Bytes |
|------|-----------|--------------|-------|-------|
| ① Capabilities | CAPS  | +0 … +11   | 12  | 48   |
| ② LIFO Stack   | STACK | +12 … +43  | 32  | 128  |
| ③ Freespace    | FREE  | +44 … +175 | 132 | 528  |
| ④ Heap         | HEAP  | +176 … +239| 64  | 256  |
| ⑤ Data Regs    | DR    | +240 … +255| 16  | 64   |
| **Total**      |       | 0 … 255    | **256** | **1 024** |

---

## Zone ① — Capabilities (CR0–CR11)

Twelve 32-bit words occupying the base of the Thread lump. Each word is
**GT Word 0** — the per-holder credential. Words 1–3 for each GT are held in
the hardware CR file (the full 128-bit CR), not in the lump memory. Only
Word 0 is saved to / loaded from lump memory via SAVE/LOAD.

```
+0   CR0    — typically the Thread's own E-GT or NULL
+1   CR1    — caller's return capability (CALL pushes here)
+2   CR2    — Scheduler E-GT
+3   CR3    — Mint E-GT
+4   CR4    — NS write authority
+5   CR5    — (general — working capability)
+6   CR6    — transient C-list view (set by CALL, not stored permanently)
+7   CR7    — (general — working capability)
+8   CR8    — (general — working capability)
+9   CR9    — (general — working capability)
+10  CR10   — (general — working capability)
+11  CR11   — (general — working capability)
```

### CR12–CR15 — Privileged Zone

CR12–CR15 are not stored in Zone ① of the Thread lump. They are held
exclusively in the hardware CR file and are loaded via `mLoad(NS Slot 1)`
at boot step B:02. They carry zero permissions in their stored GT Word 0
and are of Inform-type — the hardware returns a constant on DREAD. They
are never written to lump memory and are never accessible to DREAD.

---

## Zone ② — LIFO Stack

32 words starting at base+12. The stack grows downward (toward higher
offsets). SP points to the next free slot.

### Frame Formats

```
CALL frame (SZ=1 — 2 words):
  SP+0:  E-GT Word 0 of the callee  (Golden Token, Church-side)
  SP+1:  Frame word: SZ[1] | return_PC[15] | prev_SP[16]

LAMBDA frame (SZ=0 — 1 word):
  SP+0:  Frame word: SZ[1]=0 | lambda_arg[15] | prev_SP[16]
```

The RETURN instruction pops the frame, restores SP, and jumps to
return_PC in the caller's code section. No kernel involvement.

### Stack Depth

With 32 words and 2-word CALL frames, the maximum call depth is **16
nested calls** before the stack overflows into Freespace. The hardware
detects overflow when SP would enter Zone ③.

---

## Zone ③ — Freespace

132 words at base+44..base+175. This is the collision zone between the
downward-growing Stack and the upward-growing Heap. At Thread creation
Mint verifies all 132 words are zero (consistent with the function
abstraction freespace rule).

At runtime, Stack frames below the initial high-water mark and Heap
objects above the initial heap base both consume words from this zone.
The sum of live Stack depth and live Heap allocation must not exceed
132 words — the hardware generates an overflow fault if it does.

This is the only zone in any Church Machine lump that is dynamically
variable at runtime. The function abstraction freespace zone is fixed
at compile time and never changes; the Thread freespace zone is live.

---

## Zone ④ — Heap

64 words at base+176..base+239. Fixed size set by the IDE slot metadata
at design time. Objects are allocated from base+176 upward (toward the
Freespace zone). The heap is managed by the GC abstraction — the G bit
in Word 3 of each live GT enables mark-and-sweep collection of
unreachable heap objects.

---

## Zone ⑤ — Data Registers

16 words at base+240..base+255. DR0–DR15 are 32-bit general-purpose
data registers. This zone is always at the physical tail of the Thread
lump — the last 16 words.

DR contents are raw 32-bit integers — they are subject to DREAD/DWRITE
via a Turing-rights view, never to LOAD/SAVE. A data value cannot be
reinterpreted as a GT.

---

## Thread Lump and the Header Rule

A function abstraction lump always has `Mem[base] = header word` with
magic `0x1F`. The Thread lump does **not** follow this convention:

```
Function abstraction:   Mem[base] = 0xF8xx_xxxx  (magic 0x1F — header)
Thread lump:            Mem[base] = CR0 Word 0    (a Golden Token or zero)
```

The CALL instruction reads `Mem[base]` to extract `n_minus_6` and `cc`
for function abstraction lumps. CALL is never issued against a Thread
E-GT — the Thread is not callable. The Thread E-GT grants `B E` rights
to the Scheduler so it can context-switch the thread. The Scheduler does
not CALL the Thread; it uses the Thread's E-GT to access the lump as a
data region for state save/restore.

Mint must therefore handle Thread lump creation differently from function
abstraction lump creation. For Thread lumps `Mint.Thread(base, n)` is
used rather than `Mint.Lump(base, n)`:

- `Mint.Thread` does **not** check for magic `0x1F` at Word 0.
- `Mint.Thread` does **not** scan for freespace violations at creation —
  Zone ① (Capabilities) will contain GT Word 0 values at boot, which are
  non-zero; Zone ③ (Freespace) must be all-zero.
- `Mint.Thread` issues an E-GT over the entire 256-word region with `B E`
  permissions for the Scheduler, and a separate RW-GT for the Thread's own
  data access.
- No transient CR14 or CR6 is derived on access — the Scheduler uses
  DREAD/DWRITE against an RW-GT to save and restore thread state.

---

## Thread Lump vs Function Abstraction — Summary

| Property | Function Abstraction | Thread |
|----------|---------------------|--------|
| Word 0 | Header (magic 0x1F) | CR0 Word 0 (GT or zero) |
| `Mint` entry | `Mint.Lump(base, n)` | `Mint.Thread(base, n)` |
| Header validation | Yes — magic, cw, cc, freespace scan | No |
| Freespace zone | Compile-time fixed, all-zero, immutable | Runtime dynamic (Stack ↓ Heap ↑ collision zone) |
| C-list zone | Compiler-populated tail, LOAD-only | Zone ① — CR0..CR11, runtime LOAD/SAVE |
| Entry point | PC = 1 on every CALL | Never — not callable |
| Transient CR14 | Code view (X) derived on CALL | Not applicable |
| Transient CR6 | C-list view (L) derived on CALL | Not applicable |
| Issued GTs | One E-GT (caller holds) | E-GT (Scheduler) + RW-GT (Thread data) |
| GC interaction | G bit in lump's NS slot | G bits in all live CRs in Zone ① |
| Zip format | `*.lump.zip` with header word | Created in-place by Mint.Thread; not distributed as zip |
| lumpSize | 2^n, compiler-chosen | 2^8 = 256 words (fixed by architecture) |

---

*Document applies to: Church Machine IDE simulator · Boot.NS slots 1 (Boot.Thread), 2 (Boot.Abstr), 45 (Thread) · Tang Nano 20 K + Efinix Ti60 F225 targets.*
