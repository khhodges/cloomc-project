# CR Hardware Format — Reference Summary

Church Machine · 128-bit Capability Register · 4 × 32-bit Words

---

```
127      113 112 111          96   ← Word 3
┌──────────┬─────┬──────────────┐
│ spare[15]│ G[1]│   CRC [16]   │
└──────────┴─────┴──────────────┘
  95   92 91      85 84         64  ← Word 2
┌──────┬──────────┬───────────────┐
│spare │  gt_seq  │ limit_offset  │
│ [4]  │   [7]    │    [21]       │
└──────┴──────────┴───────────────┘
  63                           32   ← Word 1
┌────────────────────────────────┐
│           base [32]            │
└────────────────────────────────┘
  31      25 24  23 22      16 15           0   ← Word 0  (SAVE copies this word only)
┌─────────┬──────┬──────────┬─────────────┐
│B R W X  │ typ  │  gt_seq  │  object_id  │
│ L S E   │ [2]  │   [7]    │    [16]     │
│  [7]    │      │          │             │
└─────────┴──────┴──────────┴─────────────┘
```

---

## Field Reference

| Word | Bits    | Field         | W | Description |
|------|---------|--------------|---|-------------|
| 0    | 31:25   | B R W X L S E | 7 | Permissions — TPERM-changeable, **excluded from CRC** |
| 0    | 24:23   | typ           | 2 | GT class (see below) — CRC covered |
| 0    | 22:16   | gt_seq        | 7 | Revocation sequence number — CRC covered |
| 0    | 15:0    | object_id     | 16| Object index from 0 — CRC covered |
| 1    | 63:32   | base          | 32| Physical base address — CRC covered |
| 2    | 84:64   | limit_offset  | 21| limit = base + limit_offset — CRC covered |
| 2    | 91:85   | gt_seq        | 7 | Mirrors Word 0 gt_seq; mismatch on LOAD → revoked |
| 2    | 95:92   | spare         | 4 | Reserved zero |
| 3    | 111:96  | CRC           | 16| CRC-16/CCITT (poly 0x1021) over Word 0[24:0] + Word 1[all] + Word 2[all] |
| 3    | 112     | G             | 1 | GC mark bit — hardware managed, not stored on SAVE |
| 3    | 127:113 | spare         | 15| Reserved zero |

**Permission bits (Word 0 [31:25]):**
B=bind/SAVE-allowed · R=DREAD · W=DWRITE · X=execute · L=LOAD · S=SAVE-into · E=CALL-target

**typ[2] (Word 0 [24:23]):**
00=NULL (all-zero, faults on use) · 01=Real (physical region) · 10=Abstract (self-defining, no RAM) · 11=reserved

---

## Object NS Slot — 4 × 32-bit Words

Every lump GT (E-GT) has exactly one Object NS slot indexed by `object_id`.
Words 1–3 mirror the CR Words 1–3 exactly. Word 4 carries the lump header
word cached from physical memory.

```
NS Word 4  [31:0]   lump header word  (= Mem[base])   ← CALL only
┌──────────┬────────┬──────────┬──────────┬──────┬───────────┬──────────┐
│magic [5] │ ver[4] │ n-6 [4]  │  cc [8]  │typ[2]│  mw [6]   │h│c│  r  │
└──────────┴────────┴──────────┴──────────┴──────┴───────────┴──────────┘
NS Word 3  [31:0]   spare[15] | G[1] | CRC[16]         ← mirrors CR Word 3
NS Word 2  [31:0]   spare[4] | gt_seq[7] | limit_offset[21]  ← mirrors CR Word 2
NS Word 1  [31:0]   base [32]                           ← mirrors CR Word 1
```

| NS Word | Source        | Used by      | Content |
|---------|---------------|--------------|---------|
| 1       | CR Word 1     | LOAD + CALL  | `base` — physical byte address of lump word 0 |
| 2       | CR Word 2     | LOAD + CALL  | `spare[4] \| gt_seq[7] \| limit_offset[21]` — limit = lumpSize-1 for E-GT |
| 3       | CR Word 3     | LOAD         | `spare[15] \| G[1] \| CRC[16]` — LOAD re-computes CRC and verifies |
| 4       | Mem[base]     | CALL only    | Lump header: `magic[5] \| ver[4] \| n_minus_6[4] \| cc[8] \| typ[2] \| mw[6] \| h[1] \| c[1] \| r[1]` |

**Read cost per operation:**

| Operation | NS words read | Reason |
|-----------|---------------|--------|
| LOAD      | 1, 2, 3       | Reconstruct full CR; verify CRC; check revocation |
| CALL      | 1, 2, 3, 4    | Base from W1; revocation from W2; n_minus_6, cc, and **mw** from W4 to derive CR14 and CR6 |

Old two-slot design (NS_X + NS_L, 3 words each): **6 reads**.
New single-slot design: **4 reads** for CALL, **3 reads** for LOAD.

---

## Semantics

**SAVE** — copies Word 0 only; faults if B=0. B lives in Word 0 so it survives context switch with no special mechanism.

**TPERM** — clears any subset of bits [31:25]; cannot set bits; CRC unaffected. Sole mechanism for controlling delegation depth.

**Revocation** — Mint increments gt_seq in NS Word 2. LOAD checks Word 0 gt_seq [22:16] against NS Word 2 gt_seq [27:21]; mismatch nullifies the c-list slot and faults.

**G (GC)** — mark-and-sweep; hardware alternates 0→1→0 each sweep pass. Unreached slots nullified. Re-initialised by LOAD, not preserved by SAVE.

**CALL (E-GT)** — reads all 4 NS words. Extracts `n_minus_6`, `cc`, and `mw`
from NS Word 4 to derive transient CR14 (X, code region) and CR6 (L, c-list
region). `mw` shifts the code base: `code_base = base + (1+mw)*4`. When
`mw=0` the result is `base+4`, identical to the pre-manifest layout. Transient
CRs are never stored and exist only for the duration of that thread's execution
inside the lump.
