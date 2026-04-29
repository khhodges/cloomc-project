# Foundation Lump Design

**v1.0 — 2026-04-29**
**CONFIDENTIAL**

This document records the architectural agreements established in April 2026
design sessions covering how the IDE initialises a Church Machine system,
how the simulator must behave, and what the programmer controls.

It is the single authoritative reference for foundation-lump design. If
another document conflicts with the rules below, this document wins and the
other should be corrected.

---

## 1. C-list rule

A C-list is strictly an array of 32-bit Golden Token (GT) words — one GT per
slot. No other value type (raw address, scalar, data word) may occupy a C-list
slot. The hardware (and the simulator) refuses to load anything but a
MAC-validated GT from a C-list slot.

A null word (`0x00000000`) is a valid GT encoding meaning *empty / invalid* —
it is structurally a GT with no permissions and no valid NS index. It is
semantically unusable but does not violate the C-list rule.

This constraint is what gives the capability system its security property: a
program cannot smuggle a raw address or arbitrary data through a C-list slot.

See also: [`golden-tokens.md`](golden-tokens.md), [`architecture.md`](architecture.md).

---

## 2. NS slot 0 — namespace physical memory descriptor

Slot 0 of the Namespace table (`Boot.NS`) is the NS entry that describes the
**total physical memory** allocated to the namespace. It is a *descriptor*,
not a GT container.

The Namespace table never holds GTs. GTs live only in C-lists. Slot 0 of the
NS table tells you *what physical memory exists and where*; it does not grant
the right to act on that memory.

The right to act on the namespace's memory is held by the **memory manager**
for the namespace. The memory manager has, in its own C-list, a GT covering
the full namespace memory region. It uses that GT to allocate lumps on
demand: when a thread requests new memory (e.g. a new digital object,
heap growth, or a lazy-loaded lump), the memory manager carves a lump out of
the region described by NS slot 0 using the GT in its C-list as the
authority.

This separation is fundamental:

| Holds what | Where |
|------------|-------|
| Description of "this memory exists" | NS slot 0 in the Namespace table |
| Authority to allocate from it | A GT in the memory manager's C-list |

See also: [`namespace-security.md`](namespace-security.md).

---

## 3. IDE role — design-time only

The IDE's responsibility ends at boot image generation.

Once the binary boot image is produced and the device boots, the IDE may
disconnect at any time — network failure, programmer closing the browser,
device powered off and back on, or the device deployed to a remote site that
never sees the IDE again. Every Church Machine deployment must be designed
so that **after boot, the system is fully self-supporting**.

That means the following services are not IDE features — they are runtime
system services that must be part of the boot image itself:

- **Lazy loading** — fetching and resident-loading lumps on first CALL
- **NS slot allocation** — claiming empty slots from the reserved pool
- **Memory management** — allocating and reclaiming lump-sized regions
- **Garbage collection** — sweeping unreachable NS entries and lumps (see [`garbage-collection.md`](garbage-collection.md))
- **Error recovery** — handling faults, thread crashes, and resource exhaustion

The IDE produces the image. The runtime owns the system from boot onwards.
This rule has hard implications for the boot image design described in
section 4 — every service the running system needs must be either resident
in the boot image or reachable through the lazy load mechanism the boot
image installs.

---

## 4. Boot image initialisation — three programmer-controlled steps

The IDE walks the programmer through three sequential steps when generating
a boot image. The IDE provides hardware information (memory budget, address
map for the chosen target board) so the programmer can make informed
decisions; it never derives sizes automatically.

### Step 1 — Foundational lumps (always present)

The programmer specifies the total namespace physical memory and the sizes
of the three foundational lumps:

- **Namespace Lump** — anchors the namespace; defines NS slot 0 and the
  initial NS table layout.
- **Thread Lump** — physical region for the initial thread (its registers,
  stack, heap).
- **Abstraction Lump** — physical region for the initial abstraction the
  thread runs in (its code and C-list).

All three lump sizes are programmer choices, informed by the target hardware
profile shown by the IDE. The Namespace Lump is sized based on how many NS
entries the design will need over its lifetime (resident + lazy + reserved
empty slots, plus headroom for digital-object slots).

### Step 2 — Resident lumps (zero or many)

The programmer declares which additional lumps are baked into the boot image
at fixed physical addresses. There may be none or many — it is the
programmer's call which abstractions need to be resident from the first
clock cycle (e.g. memory manager, lazy loader, garbage collector, fault
handler) versus which can be lazy-loaded.

For each resident lump, the IDE places its body in the binary image at a
fixed address inside the namespace memory region.

Lumps not declared resident use the **lazy load mechanism**: their NS entry
exists in the namespace table (so GTs can be minted against them from the
start), but the lump body is fetched into memory at first CALL.

### Step 3 — Empty NS slots (open-ended growth)

The programmer reserves a number of empty NS slots in the namespace table
for lumps that do not exist at design time. The IDE cannot know what those
lumps will contain — only how much headroom to leave.

These slots are filled at runtime by the lazy loader when new lumps are
created (a new abstraction installed, a new digital object minted, a remote
lump cached locally).

### What the IDE produces

A self-contained binary boot image whose byte layout is:

- The NS table (foundational + resident + reserved empty slots)
- The three foundational lump bodies at fixed addresses
- All resident lump bodies at fixed addresses
- The memory manager's C-list, including a GT covering the full namespace
  memory region (per section 2)

After the device boots from this image, the IDE plays no further role.

---

## 5. Digital object lifecycle

Thread-internal variables — short-lived scalars, intermediate calculations,
local arrays — use the thread's own heap. No NS machinery is needed and no
GTs are minted; the thread's own region (its Thread Lump from step 1, plus
any heap pages it has been granted) is sufficient.

**Digital objects** are different. An image, a document, a piece of work
output created by the running system has an independent existence and may
outlive any single thread interaction. While a digital object is *online*,
it requires:

- An **NS slot** — so the namespace knows it exists
- A **memory window** — the physical region it occupies (allocated by the
  memory manager from the namespace's memory)
- **GTs** in any C-list that needs to reference it

All three are dynamic — minted when the object is created, evolving as the
object changes (a document being edited, an image being processed),
referenced from one or many threads simultaneously, and revoked when the
object goes away.

When a digital object is **exported** — packaged as a lump and moved offline
(written to disk, sent over the network, archived) — it leaves the namespace
entirely. Its NS slot, memory window, and outstanding GTs all become
redundant. They are now eligible for garbage collection:

1. Revoke the GTs (bump the NS entry's `gt_seq` so all stale references
   fault)
2. Release the memory window back to the memory manager for re-allocation
3. Free the NS slot for reuse by the next dynamic object

The garbage collector runs without IDE involvement (per section 3); see
[`garbage-collection.md`](garbage-collection.md) for the mark-scan-sweep
mechanism.

---

## Glossary

| Term | Meaning |
|------|---------|
| **Namespace Lump** | The boot lump anchoring the namespace; defines NS slot 0 |
| **Thread Lump** | Physical region for a thread's registers, stack, heap |
| **Abstraction Lump** | Physical region containing an abstraction's code and C-list |
| **Resident lump** | A lump baked into the boot image at a fixed address |
| **Lazy lump** | A lump with an NS entry but no body until first CALL |
| **Memory manager** | Runtime service holding the GT for the namespace's full memory; allocates lumps on demand |
| **Digital object** | A piece of work output (image, document, etc.) with its own NS slot, memory window, and GTs while online |
| **Exported lump** | A digital object packaged offline; its NS slot, memory, and GTs are now redundant |

---

## Note on the existing demo boot image

The demo boot image documented in [`boot-rom-layout.md`](boot-rom-layout.md)
predates this design and does not yet follow the rules above. In particular,
the `Boot.Abstr` C-list in that demo contains GTs at slots that the rules in
section 2 would assign to the memory manager's C-list, and the simulator has
a known divergence around `Boot.Abstr` C-list slot 0 (initialised to NULL
even though the DEMO_CLIST table grants it `R|X` to NS slot 3). The NS slot
0 descriptor semantics in section 2 of this document do *not* require any
particular `Boot.Abstr` C-list slot to be NULL — they only forbid a GT for
the namespace memory region from living in the NS table itself. The demo
image is preserved as-is for now; the programmer-authored boot image work
(Tasks #214–#217) will produce a clean image that fully obeys these rules.

---

## Cross-references

- [`architecture.md`](architecture.md) — Overall Church Machine architecture
- [`boot-rom-layout.md`](boot-rom-layout.md) — Specific demo boot ROM layout
- [`golden-tokens.md`](golden-tokens.md) — GT format and rules
- [`namespace-security.md`](namespace-security.md) — Namespace integrity model
- [`garbage-collection.md`](garbage-collection.md) — GC mechanism
- [`plan-lazy-load.md`](plan-lazy-load.md) — Lazy loading design
---
*Confidential — Kenneth Hamer-Hodges — April 2026*
