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

## Semantics

**SAVE** — copies Word 0 only; faults if B=0. B lives in Word 0 so it survives context switch with no special mechanism.

**TPERM** — clears any subset of bits [31:25]; cannot set bits; CRC unaffected. Sole mechanism for controlling delegation depth.

**Revocation** — Mint increments gt_seq in the NS slot. LOAD checks Word 0 gt_seq against Word 2 gt_seq; mismatch faults.

**G (GC)** — mark-and-sweep; hardware alternates 0→1→0 each sweep pass. Unreached slots nullified. Re-initialised by LOAD, not preserved by SAVE.
