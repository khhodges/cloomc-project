# Transparent Thread Suspension — NULL GT Lazy-Resolve (Task #1519)

## Overview

When a Church Machine program references a named capability (by pet name in its
c-list) that has not yet been linked to a live NS entry, the simulator **suspends
the thread transparently** instead of faulting immediately. The IDE then has a
wall-clock window to supply the missing Golden Token. Only after the deadline
expires does the simulator escalate to a `NULL_CAP` fault.

This is an extension of the 0xFEED pending-GT sentinel mechanism introduced in
Task #1446. Task #1519 adds the same transparent behaviour for **NULL GT** slots
— where the c-list entry is genuinely 0x00000000 at run-time.

---

## Trigger Conditions

All three capability-loading instructions check for NULL GT in a named c-list
slot:

| Instruction  | Trigger condition |
|---|---|
| `LOAD`        | `memory[clistBase + row] === 0` and pet name exists for that row |
| `ELOADCALL`   | `memory[clistBase + ecRow] === 0` and pet name exists for ecRow |
| `XLOADLAMBDA` | `memory[clistBase + imm] === 0` and pet name exists for imm |

"Pet name" means the slot is named in `programCapabilities[slot]`.

---

## Instant Resolution

Before suspending, each instruction tries **inline resolution**: it walks
`sim.nsLabels` for a label matching the pet name (case-insensitive). If a match
is found and `isNSEntryValid(nsIdx)` is true, the GT is written directly into
the c-list slot and execution continues normally — no suspension, no fault.

```
memory[clistBase + slot] ← createGT(0, nsIdx, {E:1}, 1)
output += "[LAZY-RESOLVE] NULL Slot … → NS[nsIdx] resolved instantly."
// falls through to normal instruction execution
```

---

## Suspension Path

If instant resolution fails (no matching nsLabel, or NS entry not yet valid):

1. A `_pendingResolves` entry is created for the slot:
   ```js
   sim._pendingResolves.set(slotIdx, {
       petName, slot, instrName, kind: 'NULL_GT',
       pc, savedDRs, savedCRs, savedFlags, savedSto
   });
   ```
2. `sim._lazySuspended = true` — `step()` and `run()` return early without
   advancing the program counter.
3. A `lazyResolvePending` event is emitted with `{ petName, slot, instrName, kind }`.
4. `step()` / `run()` return a sentinel object:
   ```js
   { lazySuspended: true, petName, slot, instrName, kind: 'NULL_GT', pc, desc }
   ```

The calling thread remains at the same PC; no fault is raised.

---

## Resuming the Thread

The IDE calls `sim.resolvePendingSlot(slotIdx, nsIdx)` to supply the GT:

```js
const result = sim.resolvePendingSlot(0, 6);
// result: { ok: true, nsIdx: 6, pendingName: 'NavanaService', gt: 0x... }
```

`resolvePendingSlot` handles both the 0xFEED sentinel path (Task #1446) and the
NULL GT path (Task #1519):

- **NULL GT path** (`existing === 0 && _pendingResolves.has(slotIdx)`):
  creates a GT pointing to `nsIdx`, writes it to memory, clears the
  `_pendingResolves` entry, and — when no more entries remain — clears
  `_lazySuspended` and optionally restores thread context so execution can
  continue.

After resolution, the next `step()` re-executes the same instruction (the PC
was not advanced during suspension) and succeeds.

---

## Escalation

If the IDE does not supply the GT within the configured timeout, it calls:

```js
sim.escalateLazyResolve(slotIdx);
```

This restores the saved thread context and fires a `NULL_CAP` fault with full
details, halting the machine in the normal fault-display path.

The default timeout is **30 days** (stored in localStorage key
`church_lazy_resolve_timeout_ms`) — effectively infinite in practice. The IDE
UI may expose this as a configurable setting.

---

## IDE Integration (app-run.js)

`stepSim()` and `runBatch()` both detect `result.lazySuspended` and call
`_registerLazyResolvePending(result)`, which:

1. Adds the item to the `_lazyResolvePending` Map (keyed by slot index).
2. Renders the **Pending Capabilities** collapsible panel in the IDE console
   area, listing each pending pet name with a resolve button.

The console displays:
```
⏸ Thread suspended — waiting for 'NavanaService' (c-list slot 0)
  Link it via the Pending Capabilities panel below to resume.
```

---

## Comparison: 0xFEED Sentinel vs. NULL GT Suspension

| Aspect | 0xFEED sentinel (Task #1446) | NULL GT suspension (Task #1519) |
|---|---|---|
| Trigger | `isPendingGT(word)` | `word === 0 && petName` |
| Origin | `_injectClistNow` at compile time | Runtime — c-list was never populated |
| Instant resolve | Yes (same nsLabel walk) | Yes (same nsLabel walk) |
| Suspension flag | `sim.awaitingLump` / `sim.suspended` | `sim._lazySuspended` |
| Resume API | `resolvePendingSlot(slot, nsIdx)` | `resolvePendingSlot(slot, nsIdx)` |
| Escalation | `escalateLazyResolve(slot)` | `escalateLazyResolve(slot)` |

---

## Test Coverage

`simulator/test_lazy_resolve_pending.js` — T006 through T011:

| Test | Description |
|---|---|
| T006 | LOAD on NULL slot → `_lazySuspended=true`, no fault, event emitted |
| T007 | ELOADCALL on NULL slot → transparent suspension |
| T008 | NULL slot with no pet name → immediate `NULL_CAP` fault (no suspension) |
| T009 | `resolvePendingSlot` on a NULL-path entry → ok, GT written, flag cleared |
| T010 | `escalateLazyResolve` → `NULL_CAP` fault fired |
| T011 | NULL GT with matching valid NS entry → inline resolution, no suspension |

---

*Kenneth Hamer-Hodges — May 2026*
