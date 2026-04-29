# Memory Manager Design

**v1.0 — 2026-04-29**
**CONFIDENTIAL**

## Domain-Separated Allocation with Passkey Billing

**Status: DRAFT — awaiting approval before implementation**
**Author: design session 2026-04-29**
**Depends on: `docs/golden-tokens.md`, `docs/abstractions.md`**

---

## 1. Motivation

The current `Memory` abstraction (NS[7]) is a flat bump-pointer allocator. It has three
problems that this design addresses:

| Problem | Consequence |
|---|---|
| Returns raw addresses, not GTs | Caller can construct arbitrary memory references — no capability discipline |
| No power-of-2 quantization | Lump sizes are arbitrary; hardware bounds model is not enforced at allocation time |
| No domain separation | A caller asking for a code region gets the same kind of handle as one asking for a c-list region — the hardware cannot distinguish them at use time |
| `Mint.Create` ignores the `perms` argument | GT permission bits are never encoded; every issued GT is identical and meaningless |
| No identity or quota on memory requests | Any caller with an E-GT to Memory can exhaust physical RAM without limit or attribution |

This document specifies a replacement consisting of four cooperating abstractions:
`PhysicalPool`, `TuringMemory`, `ChurchMemory`, and `Billing`, unified by a thin
orchestration layer `LumpFactory`.

---

## 2. Reference: GT Word Bit Layout

Every GT is a single 32-bit word. The hardware checks specific bits on every
instruction that uses it. No instruction reveals the physical address behind the token.

```
Bit  31   30   29   28   27   26   25   24─23   22────16   15────────0
     ┌────┬────┬────┬────┬────┬────┬────┬──────┬──────────┬───────────┐
     │ B  │ E  │ S  │ L  │ X  │ W  │ R  │ type │  gt_seq  │  slot_id  │
     └────┴────┴────┴────┴────┴────┴────┴──────┴──────────┴───────────┘
      [31]  [30] [29] [28] [27] [26] [25] [24:23]  [22:16]    [15:0]
```

| Field | Width | Meaning |
|---|---|---|
| `slot_id` | 16 | Index into the Namespace table (hardware looks up base + limit) |
| `gt_seq` | 7 | Revocation freshness counter; must match NS Entry Word 1 `gt_seq` |
| `type` | 2 | `00`=NULL `01`=Inform `10`=Outform `11`=Abstract |
| `perms` | 6 | R W X L S E — checked individually by hardware on each use |
| `B` | 1 | Bind flag — 1 = GT may be propagated via `mSave` to another c-list |

### 2.1 Permission rules

```
Turing domain:  R (bit 25)   W (bit 26)   X (bit 27)
Church domain:  L (bit 28)   S (bit 29)   E (bit 30)
```

**Domain purity**: A GT may carry Turing bits OR Church bits, never both. Mixed bits
raise `DOMAIN_PURITY` at mint time.

**E isolation**: Within the Church domain, E must be standalone. `LE`, `SE`, and `LSE`
are all invalid. An Enter token is the key to call an abstraction; Load/Save tokens are
the keys to the capability list inside it. They must never be the same key.

```
Valid Turing:   R  W  X  RW  RX  WX  RWX
Valid Church:   L  S  E  LS          (E alone only)
Invalid:        RL WL XE RE LSE LE SE   (any cross-domain or E+L/S combination)
```

**Instruction–bit mapping** (the only bits the hardware checks per instruction):

| Instruction | Bit checked | Fault if 0 |
|---|---|---|
| `DREAD  DR, GT, offset` | R (25) | `DATA_PERM` |
| `DWRITE GT, offset, DR` | W (26) | `DATA_PERM` |
| execute via CR14 | X (27) | `EXEC_PERM` |
| `mLoad  CR, GT, offset` | L (28) | `CAP_LOAD` |
| `mSave  CR, GT, offset` | S (29) | `CAP_SAVE` |
| `CALL   GT` | E (30) | `ENTER_PERM` |
| `mSave  CR, dest, GT` (propagate GT itself) | B (31) | `BIND` |

### 2.2 Abstract GT word layout

The Abstract type (`type=11`) treats the GT word itself as the value — no NS lookup,
no lump, nothing callable. Used for hardware device handles and for the Passkey P-GT.

```
Bit  31──27   26  25   24─23   22────16   15──────0
     ┌────────┬───┬───┬──────┬──────────┬──────────┐
     │ab_type │ R │ W │  11  │  gt_seq  │  ab_data │
     │ 5 bits │   │   │(Abs) │  7 bits  │  16 bits │
     └────────┴───┴───┴──────┴──────────┴──────────┘
```

---

## 3. Layer 0 — `PhysicalPool`

The existing `Memory` abstraction is refactored into a low-level page dispenser. It has
no opinion about domain, permissions, or GTs. It is **kernel-internal only** — no
user-visible E-GT is ever issued for PhysicalPool. Only `TuringMemory` and
`ChurchMemory` hold a capability to it.

### 3.1 Size quantization

All lumps must be exactly 2ⁿ words, where 6 ≤ n ≤ 14 (64 words through 16 384 words).
This matches the hardware `n-6` field in the lump header and the ZIP-derivation rule
in `docs/abstractions.md §ZIP → Header Word`.

```
Claim(requestedWords):
    exp = 6
    while (1 << exp) < requestedWords:
        exp = exp + 1
    if exp > 14:
        fault LUMP_TOO_LARGE
    quantizedWords = 1 << exp

    base    = read(freePointer)
    newFree = base + quantizedWords
    write(freePointer, newFree)
    return (base, quantizedWords, exp)
```

Quantization happens **before** the Billing charge (§6), so the charge is always the
real committed size — a caller cannot request 63 words to get 64 words for the price
of 63.

### 3.2 Methods

| Method | Parameters | Returns | Notes |
|---|---|---|---|
| `Claim(requestedWords)` | words ≥ 1 | `(base, quantizedWords, exp)` | Quantizes then commits |
| `Release(base)` | physical base | `0` | Returns block to free pool |

---

## 4. Layer 1a — `TuringMemory`

Issues **only Turing-domain GTs**. A caller holding an E-GT to TuringMemory
can never receive an L-, S-, or E-permissioned region regardless of what they request.
The encoding is structural — `Mint.Encode` is called with Turing bits only.

### 4.1 GT patterns issued

| Method | B | E | S | L | X | W | R | type | Meaning |
|---|---|---|---|---|---|---|---|---|---|
| `Allocate` | 0 | 0 | 0 | 0 | 0 | 1 | 1 | Inform | General read/write data region |
| `AllocCode` | 0 | 0 | 0 | 0 | 1 | 0 | 1 | Inform | Read + execute; no write after load |

`AllocCode` withholds W deliberately: code is immutable once installed. The hardware
will allow the instruction pointer to enter the region (X) and allow a debugger to read
it (R), but writing new instructions into a live code region is structurally prevented.

### 4.2 Methods

```
abstraction TuringMemory {
    capabilities {
        PhysicalPool        // c-list[0]: E-GT — raw allocator
        Mint                // c-list[1]: E-GT — GT encoder
        Navana              // c-list[2]: E-GT — NS table writer
        Billing             // c-list[3]: E-GT — quota enforcer
    }

    public method Allocate(requestedWords, pgt):
        // Charge quota (quantized size — see §6.3)
        (exp, quantized) = PhysicalPool.quantizeOnly(requestedWords)
        Billing.Charge(pgt, quantized)          // faults QUOTA_EXCEEDED if over budget
        (base, size, exp) = PhysicalPool.Claim(requestedWords)
        gt = Mint.Encode(base, exp, perms=RW, bindable=0, far=0)
        nsSlot = Navana.Add(base, size)
        gt[15:0] = nsSlot
        return gt

    public method AllocCode(requestedWords, pgt):
        (exp, quantized) = PhysicalPool.quantizeOnly(requestedWords)
        Billing.Charge(pgt, quantized)
        (base, size, exp) = PhysicalPool.Claim(requestedWords)
        gt = Mint.Encode(base, exp, perms=RX, bindable=0, far=0)
        nsSlot = Navana.Add(base, size)
        gt[15:0] = nsSlot
        return gt

    public method Free(gt, pgt):
        nsSlot = gt[15:0]
        (base, size) = Navana.Remove(nsSlot)
        PhysicalPool.Release(base)
        Billing.TopUp(pgt, size)
        return 0
}
```

---

## 5. Layer 1b — `ChurchMemory`

Issues **only Church-domain GTs**. A caller holding an E-GT to ChurchMemory
can never receive an R-, W-, or X-permissioned region.

The E isolation rule (§2.1) means E-GTs are issued only by `AllocAbstract`, which
does not allocate a new physical region — it wraps an existing NS entry in an
Enter-capable handle. L and S are never combined with E.

### 5.1 GT patterns issued

| Method | B | E | S | L | X | W | R | type | Meaning |
|---|---|---|---|---|---|---|---|---|---|
| `Allocate` | 1 | 0 | 1 | 1 | 0 | 0 | 0 | Inform | Read/write capability list region |
| `AllocCList` | 1 | 0 | 0 | 1 | 0 | 0 | 0 | Inform | Read-only sealed c-list |
| `AllocAbstract` | 1 | 1 | 0 | 0 | 0 | 0 | 0 | Inform | Enter-only handle (E standalone) |

B=1 is the default for Church GTs because capabilities are designed to propagate: you
obtain a service reference and pass it to collaborators via `mSave`.

`AllocCList` withholds S: the capability list was written at construction time and is
sealed. The caller can inspect it (L) but cannot install new capabilities into it.

`AllocAbstract` issues a standalone E-GT for an existing NS entry. No new physical
memory is claimed — it wraps an already-installed lump. This is how user code obtains a
callable reference to a resident service.

### 5.2 Methods

```
abstraction ChurchMemory {
    capabilities {
        PhysicalPool        // c-list[0]
        Mint                // c-list[1]
        Navana              // c-list[2]
        Billing             // c-list[3]
    }

    public method Allocate(requestedWords, pgt):
        (exp, quantized) = PhysicalPool.quantizeOnly(requestedWords)
        Billing.Charge(pgt, quantized)
        (base, size, exp) = PhysicalPool.Claim(requestedWords)
        gt = Mint.Encode(base, exp, perms=LS, bindable=1, far=0)
        nsSlot = Navana.Add(base, size)
        gt[15:0] = nsSlot
        return gt

    public method AllocCList(requestedWords, pgt):
        (exp, quantized) = PhysicalPool.quantizeOnly(requestedWords)
        Billing.Charge(pgt, quantized)
        (base, size, exp) = PhysicalPool.Claim(requestedWords)
        gt = Mint.Encode(base, exp, perms=L, bindable=1, far=0)
        nsSlot = Navana.Add(base, size)
        gt[15:0] = nsSlot
        return gt

    public method AllocAbstract(existingNsSlot):
        // No physical allocation — wraps an already-installed NS entry.
        // No Billing charge — memory was charged at AllocCode/Allocate time.
        gt = Mint.Encode(nsBase=0, exp=0, perms=E, bindable=1, far=0)
        gt[15:0] = existingNsSlot
        return gt

    public method Free(gt, pgt):
        nsSlot = gt[15:0]
        (base, size) = Navana.Remove(nsSlot)
        PhysicalPool.Release(base)
        Billing.TopUp(pgt, size)
        return 0
}
```

---

## 6. Layer 2 — `Billing` and the Passkey P-GT

### 6.1 What the P-GT is

The Passkey Golden Token (P-GT) is an **Abstract-type GT** that acts as an unforgeable
identity credential and quota key. It is passive: it cannot be `CALL`ed, `mLoad`ed
from, or `DREAD`ed. The hardware enforces this because the Abstract type performs no
NS lookup and the R/W bits are both zero. The caller can only hold it in a register and
pass it as a parameter to memory APIs.

### 6.2 P-GT word layout

```
Bit  31──27        26  25   24─23   22────16   15──────────0
     ┌────────────┬───┬───┬──────┬──────────┬──────────────┐
     │ 0b00001    │ 0 │ 0 │  11  │  gt_seq  │  account_id  │
     │ (PID class)│ R │ W │(Abs) │ freshness│  (16 bits)   │
     └────────────┴───┴───┴──────┴──────────┴──────────────┘
```

`ab_type = 0b00001` is the PID class identifier. `account_id` is a 16-bit opaque key
that Billing uses to look up the quota record. `gt_seq` is a freshness counter that
increments on each reissue, invalidating any copies of an old P-GT without requiring
the caller to return it.

The P-GT word itself costs zero words of physical memory. It is pure state in the
32-bit register — no lump, no NS entry, no allocation.

### 6.3 Quota records (inside Billing, never visible to callers)

Billing maintains a private table of quota records. Callers have no GT to this table.

| Field | Type | Meaning |
|---|---|---|
| `account_id` | 16-bit | Key; matches P-GT `ab_data` field |
| `words_remaining` | 32-bit | Current allocation budget in words |
| `words_used` | 32-bit | Cumulative words ever committed |
| `quota_class` | 4-bit | Controls default limits and B-bit policy |

**Quota classes:**

| Class | Default words | Bindable P-GT | Typical holder |
|---|---|---|---|
| `basic` (0) | 4 096 | No (B=0) | Student, single program |
| `standard` (1) | 65 536 | No (B=0) | Application developer |
| `premium` (2) | 524 288 | Yes (B=1) | Trusted service abstraction |
| `system` (3) | Unlimited | Yes (B=1) | Navana, boot chain only |

The system-class P-GT is issued by Billing during `Navana.Init` before any user code
runs. It is never placed in a user-accessible c-list.

### 6.4 Billing methods

```
abstraction Billing {
    capabilities {
        Mint                // c-list[0]: to build P-GT words
    }

    // Issue — create a new P-GT for an account.
    // Returns the 32-bit Abstract GT word; no physical memory allocated.
    public method Issue(account_id, initial_words, quota_class):
        write(quotaTable[account_id], initial_words)
        write(classTable[account_id], quota_class)
        pgt = build_abstract_gt(ab_type=0b00001,
                                R=0, W=0,
                                gt_seq=freshSeq(),
                                ab_data=account_id)
        pgt[31] = (quota_class >= premium) ? 1 : 0   // B-bit
        return pgt

    // Charge — validate and atomically decrement quota.
    // Faults QUOTA_EXCEEDED if budget insufficient; faults BAD_PGT if malformed.
    public method Charge(pgt, requested_words):
        if pgt[24:23] != 0b11        → fault BAD_PGT_TYPE
        if pgt[31:27] != 0b00001     → fault BAD_PGT_CLASS
        account_id = pgt[15:0]
        remaining  = read(quotaTable[account_id])
        if remaining < requested_words → fault QUOTA_EXCEEDED
        write(quotaTable[account_id], remaining - requested_words)
        return 1

    // TopUp — return words to the quota (called by TuringMemory.Free / ChurchMemory.Free).
    public method TopUp(pgt, words):
        account_id = pgt[15:0]
        remaining  = read(quotaTable[account_id])
        write(quotaTable[account_id], remaining + words)
        return remaining + words

    // Revoke — zero the quota; future Charge calls fault QUOTA_EXCEEDED.
    // The P-GT word is not recalled — it becomes a dead key.
    public method Revoke(account_id):
        write(quotaTable[account_id], 0)
        return 0

    // Reissue — bump gt_seq to invalidate existing P-GT copies; return new P-GT.
    public method Reissue(account_id):
        pgt = build_abstract_gt(ab_type=0b00001,
                                R=0, W=0,
                                gt_seq=freshSeq(),     // new seq — old copies rejected
                                ab_data=account_id)
        return pgt

    // Balance — read remaining quota (for diagnostics; caller must hold a valid P-GT).
    public method Balance(pgt):
        account_id = pgt[15:0]
        return read(quotaTable[account_id])
}
```

### 6.5 Why revocation works without recalling the P-GT

The P-GT is an Abstract GT — no NS entry, no `gt_seq` check. Billing validates the
P-GT by inspecting the `ab_type` and `ab_data` fields directly and then consulting its
internal table. When `Revoke(account_id)` zeros the quota, every subsequent
`Billing.Charge(pgt, n)` for any P-GT carrying that `account_id` will fault
`QUOTA_EXCEEDED` regardless of how many copies of the P-GT exist in how many c-lists.

To invalidate copies entirely (not just block usage), `Reissue` bumps the `gt_seq`
field of newly issued P-GTs and Billing records the current valid sequence per account.
Old copies with a stale `gt_seq` are then additionally rejected at `Charge` time.

---

## 7. Layer 3 — `LumpFactory` (unified entry point)

`LumpFactory` is a thin orchestration layer for callers who want a single method rather
than calling TuringMemory and ChurchMemory separately. It enforces domain purity as a
pre-flight check before touching either allocator.

```
abstraction LumpFactory {
    capabilities {
        TuringMemory        // c-list[0]
        ChurchMemory        // c-list[1]
        Billing             // c-list[2]
    }

    // AllocAndMint — allocate and issue a GT in one call.
    //   domain:    0 = Turing  1 = Church
    //   permsBits: 6-bit mask [R,W,X,L,S,E] aligned to domain
    //   bindable:  only meaningful for Turing (Church is always B=1)
    //   far:       1 = Outform type (lazy placeholder)
    //   pgt:       Passkey P-GT for quota
    public method AllocAndMint(requestedWords, domain, permsBits, bindable, far, pgt):
        turingBits = permsBits & 0b000111
        churchBits = permsBits & 0b111000
        if turingBits != 0 and churchBits != 0 → fault DOMAIN_PURITY

        if domain == 0:
            if permsBits & 0b000100:   // X bit set → code region
                return TuringMemory.AllocCode(requestedWords, pgt)
            else:
                return TuringMemory.Allocate(requestedWords, pgt)
        else:
            eBit = permsBits & 0b100000
            lsBit = permsBits & 0b011000
            if eBit and lsBit → fault E_ISOLATION
            if eBit:
                return ChurchMemory.AllocAbstract(...)
            elif lsBit == 0b010000:   // L only → sealed c-list
                return ChurchMemory.AllocCList(requestedWords, pgt)
            else:
                return ChurchMemory.Allocate(requestedWords, pgt)

    // Free — revoke GT and return memory.
    public method Free(gt, pgt):
        domain = (gt[28] | gt[29] | gt[30]) ? 1 : 0   // any Church bit = Church domain
        if domain == 1:
            return ChurchMemory.Free(gt, pgt)
        else:
            return TuringMemory.Free(gt, pgt)
}
```

---

## 8. How the GT word differs by source

A caller receiving a GT from either allocator sees the domain encoded directly in bits
25–30. This is the only information they have — no physical address is ever visible.

```
Source                   │B │E │S │L │X │W │R │type      │ Hardware use
─────────────────────────┼──┼──┼──┼──┼──┼──┼──┼──────────┼──────────────────────────
TuringMemory.Allocate    │0 │0 │0 │0 │0 │1 │1 │01 Inform │ DREAD + DWRITE
TuringMemory.AllocCode   │0 │0 │0 │0 │1 │0 │1 │01 Inform │ DREAD + execute (CR14)
ChurchMemory.Allocate    │1 │0 │1 │1 │0 │0 │0 │01 Inform │ mLoad + mSave (c-list RW)
ChurchMemory.AllocCList  │1 │0 │0 │1 │0 │0 │0 │01 Inform │ mLoad only (c-list RO)
ChurchMemory.AllocAbst.  │1 │1 │0 │0 │0 │0 │0 │01 Inform │ CALL (enter abstraction)
Billing.Issue (P-GT)     │* │0 │0 │0 │0 │0 │0 │11 Abstr  │ none (passive credential)
```

*P-GT B-bit = 0 for basic/standard, 1 for premium/system.

**Hardware enforcement of the table above:**
- Row 1 (R,W): `DREAD` and `DWRITE` succeed; `CALL` faults (E=0); `mLoad` faults (L=0)
- Row 2 (R,X): `DREAD` succeeds (debugger readable); `DWRITE` faults (W=0 — code immutable); execute succeeds
- Row 3 (L,S): `mLoad` and `mSave` succeed; `DREAD` faults (R=0); `CALL` faults (E=0)
- Row 4 (L): `mLoad` succeeds; `mSave` faults (S=0); c-list is sealed after construction
- Row 5 (E): `CALL` succeeds; `mLoad` faults (L=0); `mSave` faults (S=0); E is standalone
- Row 6 (P-GT): no instruction succeeds (Abstract, R=0, W=0); hardware treats it as a bare value

---

## 9. NS slot assignment

The boot-order constraint is strict: a service cannot hold a capability to something
installed after it. The proposed NS assignments preserve the existing kernel layout
and insert new services between existing entries.

| NS Slot | Name | Layer | Notes |
|---|---|---|---|
| 0 | Boot.NS | 0 | Unchanged |
| 1 | Boot.Thread | 0 | Unchanged |
| 2 | Startup.Config | 0 | Unchanged |
| 3 | LED flash | 0 | Unchanged |
| 4 | Salvation | 1 | Unchanged |
| 5 | Navana | 1 | Updated: holds P-GT for system-class; updated c-list |
| 6 | Mint | 1 | Updated: `Create` replaced by `Encode` |
| 7 | **PhysicalPool** | 1 | Renamed from `Memory`; kernel-internal only |
| 8 | **Billing** | 1 | New |
| 9 | **TuringMemory** | 1 | New |
| 10 | **ChurchMemory** | 1 | New |
| 11 | **LumpFactory** | 1 | New |
| 12 | Scheduler | 1 | Renumbered from 8 |
| 13 | Stack | 1 | Renumbered from 9 |
| 14 | DijkstraFlag | 1 | Renumbered from 10 |
| 15+ | UART, LED, Button… | 2 | Renumbered accordingly |

---

## 10. Boot sequence changes

`Navana.Init` currently calls `Memory.Allocate` and `Mint.Create` directly. After this
change it:

1. Receives a system-class P-GT from Billing (issued during Billing's own init, before
   Navana.Init runs — Billing is installed first).
2. Uses that P-GT for every allocation in Init.
3. Calls `TuringMemory.AllocCode` for code lumps (SlideRule, Constants).
4. Calls `TuringMemory.Allocate` for working-memory buffers (LED, UART, Stack, DijkstraFlag).
5. Calls `ChurchMemory.AllocAbstract` to wrap each installed lump in an E-GT for the Scheduler.

```
Before (current navana.cloomc):
    srBase    = call(Memory.Allocate(16384))     // raw address, no domain
    srGT      = call(Mint.Create(16384, 5))      // perms ignored, no GT encoding

After:
    srGT      = call(TuringMemory.AllocCode(16384, systemPgt))    // GT(R,X, Inform)
    srEnterGT = call(ChurchMemory.AllocAbstract(srGT.nsSlot))     // GT(E, Inform)
```

---

## 11. Migration of existing callers

| Current call in navana.cloomc | Migrated to | GT issued |
|---|---|---|
| `Memory.Allocate(16384)` — SlideRule code | `TuringMemory.AllocCode(16384, pgt)` | R, X |
| `Memory.Allocate(256)` — Constants | `TuringMemory.AllocCode(256, pgt)` | R, X |
| `Memory.Allocate(1024)` — Scheduler | `TuringMemory.Allocate(1024, pgt)` | R, W |
| `Memory.Allocate(512)` — Stack | `TuringMemory.Allocate(512, pgt)` | R, W |
| `Memory.Allocate(256)` — DijkstraFlag | `TuringMemory.Allocate(256, pgt)` | R, W |
| `Memory.Allocate(64)` — LED buffer | `TuringMemory.Allocate(64, pgt)` | R, W |
| `Memory.Allocate(512)` — UART buffer | `TuringMemory.Allocate(512, pgt)` | R, W |
| `Mint.Create(16384, 5)` — SlideRule GT | `ChurchMemory.AllocAbstract(slot)` | E |
| `Mint.Create(256, 5)` — Constants GT | `ChurchMemory.AllocAbstract(slot)` | E |

---

## 12. Files to create or change

| File | Action | Description |
|---|---|---|
| `simulator/cloomc/memory.cloomc` | Rename + rework | Becomes `physical_pool.cloomc`; only `Claim` and `Release`; no perms |
| `simulator/cloomc/mint.cloomc` | Rework | Replace `Create` with `Encode(base, exp, permsBits, bindable, far)` |
| `simulator/cloomc/navana.cloomc` | Update | Use TuringMemory + ChurchMemory + system P-GT in `Init` |
| `simulator/cloomc/billing.cloomc` | **New** | Billing abstraction as specified in §6 |
| `simulator/cloomc/turing_memory.cloomc` | **New** | TuringMemory as specified in §4 |
| `simulator/cloomc/church_memory.cloomc` | **New** | ChurchMemory as specified in §5 |
| `simulator/cloomc/lump_factory.cloomc` | **New** | LumpFactory as specified in §7 |
| `simulator/abstractions.js` | Update | Register four new abstractions; renumber existing NS slots 8–14+ |

---

## 13. Open questions — decisions required before implementation

The following items need an answer before any code changes are made:

**Q1 — NS slot renumbering**
Renumbering existing abstractions from slot 8 upward will break any hardcoded slot
references in boot firmware, tutorials, and tests. Should the new services be inserted
at high slot numbers (e.g., 100+) to avoid renumbering, or should the clean layout
in §9 be used with a coordinated renumber pass?

**Q2 — Billing table storage**
The quota table needs persistent storage across reboots (so a system-class P-GT issued
at boot can recover the same account IDs). Should this live in a reserved memory
region provisioned at boot, or should Billing always reissue fresh accounts at every
boot (simpler, but quotas reset on power cycle)?

**Q3 — P-GT freshness enforcement**
The current design uses `gt_seq` in the P-GT for reissue invalidation but Abstract GTs
have no NS entry so the hardware does not check `gt_seq` automatically. Should Billing
maintain a per-account `current_seq` field and reject P-GTs with a stale sequence, or
is Revoke-by-zeroing-quota sufficient without sequence tracking?

**Q4 — `AllocAbstract` memory charging**
`ChurchMemory.AllocAbstract` wraps an existing NS entry without allocating new memory,
so no Billing charge is made. Is this correct, or should issuing a new E-GT for an
existing lump carry a small flat fee (to prevent unbounded E-GT proliferation)?

**Q5 — Compatibility shim**
Should the old `Memory` NS slot remain as a backward-compatible shim that internally
calls `TuringMemory.Allocate` with a system-class P-GT, to avoid breaking existing
tutorials and boot_uploads.js? Or is a clean break preferred?

**Q6 — LumpFactory necessity**
Given that TuringMemory and ChurchMemory already cover all cases, is `LumpFactory`
worth the extra NS slot and boot-time allocation? It is convenient but not essential.

---

*This document describes design intent only. No source files have been modified.
Implementation begins only after all open questions in §13 are resolved and this
document is approved.*
---
*Confidential — Kenneth Hamer-Hodges — April 2026*
