# Deterministic Garbage Collection

## The G Permission Bit

The G (Garbage) permission bit is a flag on Golden Tokens that participates in the garbage collection process. Its role differs between the two simulators:

- **Sim-64**: The G bit is cleared on namespace access during LOAD, signaling to the GC system that the entry has been recently accessed.
- **Sim-32**: The G bit indicates that a capability is subject to garbage collection management. It is used during the Mark-Scan-Sweep cycle to track reachability.

---

## Three-Phase Mark-Scan-Sweep Cycle

Both simulators implement deterministic garbage collection through a three-phase cycle. Unlike traditional GC systems that rely on non-deterministic tracing, this cycle is explicit and predictable.

### Phase 1: Mark

The Mark phase flags all non-empty namespace entries as potentially reclaimable.

- Every namespace entry that contains valid data is marked.
- This is a conservative starting point: all entries are assumed unreachable until proven otherwise.

### Phase 2: Scan

The Scan phase walks all live references to identify entries that are still in use.

- The scan examines all capability registers (CR0-CR15) and the call stack.
- For each reference found, if the referenced entry's capability has L (Load) or M (Machine) permission, the mark on that namespace entry is cleared.
- An entry with its mark cleared is considered reachable and will not be reclaimed.

### Phase 3: Sweep

The Sweep phase reclaims entries that are still marked after scanning.

- Any namespace entry that still has its mark set after the Scan phase is unreachable -- no live capability register or call stack frame references it.
- The entry is cleared (reclaimed).
- The entry's **version is bumped**, incrementing the version number in the namespace entry.

---

## Version Bumping and Token Invalidation

Version bumping is the mechanism that prevents use-after-free vulnerabilities. When a namespace entry is reclaimed during Sweep:

1. The entry's version number is incremented.
2. Any outstanding Golden Token that references this entry still contains the old version number.
3. When that stale token is later used in a LOAD or CALL, the version check fails (the token's version does not match the entry's new version).
4. The operation FAULTs, preventing access to the recycled entry.

This provides strong temporal safety: even if an old token is retained in a register or data structure, it cannot be used to access a namespace entry that has been reclaimed and potentially reassigned to a different resource.

---

## Sim-64: GC Integration

In Sim-64, garbage collection is integrated directly into the LOAD instruction:

- When a LOAD accesses a namespace entry, the G bit on that entry is cleared.
- This means that actively used entries automatically signal their liveness through normal program execution.
- The GC operates on the namespace hierarchy, traversing the tree of namespaces rooted at CR15.
- The Mark-Scan-Sweep cycle can identify unreferenced entries by checking which entries still have their G bit set (indicating they have not been accessed since the last Mark phase).

This approach is tightly coupled with normal execution: no separate GC pause or cycle is required for tracking liveness.

---

## Sim-32: Separate GC Cycle

In Sim-32, garbage collection is a separate, explicitly triggered cycle that operates on the flat namespace table:

- The GC does not run automatically during normal instruction execution.
- Instead, the three phases (Mark, Scan, Sweep) are triggered independently through the Dashboard UI.
- The namespace table is a flat array of up to 32,768 entries, and the GC processes all entries in this table.

### GC Operation Details (Sim-32)

**Mark**: Iterates over all namespace entries. Any entry with non-zero content is flagged as potentially reclaimable.

**Scan**: Examines all 16 capability registers (CR0-CR15) and all frames on the call stack. For each Golden Token found:
- If the token has L (Load) or M (Machine) permission, the corresponding namespace entry's mark is cleared.
- This identifies entries that are reachable through live capabilities.

**Sweep**: Iterates over all namespace entries. Any entry still marked after scanning:
- Has its content cleared (Location, Limit, Seal reset to zero).
- Has its version incremented (bumped by 1, wrapping within the 5-bit field).
- All Golden Tokens referencing this entry at the old version are now invalid.

---

## Dashboard UI (Sim-32)

The Sim-32 Dashboard provides four GC control buttons:

| Button | Action |
|--------|--------|
| **Mark** | Executes the Mark phase only. Flags all non-empty namespace entries. |
| **Scan** | Executes the Scan phase only. Clears marks on reachable entries. |
| **Sweep** | Executes the Sweep phase only. Reclaims still-marked entries and bumps versions. |
| **GC Cycle** | Executes all three phases in sequence (Mark, then Scan, then Sweep). |

The Dashboard also displays GC status information, showing the state of the namespace table and the results of each phase.

---

## Comparison

| Aspect | Sim-64 (CTMM) | Sim-32 (RV32-Cap) |
|--------|---------------|-------------------|
| **GC Trigger** | Integrated into LOAD (G-bit cleared on access) | Explicit via Dashboard buttons |
| **Namespace Structure** | Hierarchical (tree of namespaces) | Flat table (up to 32,768 entries) |
| **Liveness Tracking** | G-bit: cleared on access, set entries are unreferenced | Mark flag: set during Mark, cleared during Scan |
| **Version Bumping** | Not used (different invalidation mechanism) | 5-bit version incremented on Sweep |
| **Token Invalidation** | Through G-bit and capability integrity | Through version mismatch detection |
| **User Control** | Automatic during execution | Manual via Mark, Scan, Sweep, GC Cycle buttons |
