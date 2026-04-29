# Method Access Control in CLOOMC

**v1.0 — 2026-04-29**
**CONFIDENTIAL**

**Status**: Architectural specification. April 27, 2026.

## Overview

CLOOMC abstractions support two visibility qualifiers on method declarations:

```
public method Foo(args) { ... }
private method Bar(args) { ... }
```

Omitting the qualifier defaults to `public` — existing source files without qualifiers compile identically to before.

These qualifiers have a specific, structural meaning tied to the lump seal and the dispatch mechanism. This document explains what they mean, how they are enforced, and why the design is sound.

---

## What `public` and `private` Mean

### `public`

A public method is externally callable. It appears in the auto-generated dispatch table at M00. Any caller holding a GT to this abstraction can invoke a public method by passing its selector in DR0.

### `private`

A private method is an internal implementation detail. It is compiled into the lump binary at its assigned offset and is fully reachable from within the abstraction via a direct `BRANCH` instruction. However, it does **not** appear in the dispatch table at M00. Because M00 never routes to it, and the lump seal prevents modification of the code region from outside, private methods are **structurally unreachable** from external callers — not merely hidden by convention.

---

## Structural Enforcement: Why "Unreachable" Is the Right Word

The security property comes from two orthogonal mechanisms working together:

1. **Dispatch exclusion**: The compiler-generated M00 only emits MCMP+BRANCHEQ entries for public methods. There is no code path in M00 that reaches a private method's offset.

2. **Lump seal**: The lump is a sealed binary object. Once committed to the namespace, its code region cannot be patched or extended from outside. An attacker cannot inject new branch instructions into M00 to reach the private method.

Together these mean: a private method's byte offset exists in the lump binary, but no externally-reachable code path leads to it. It is unreachable in the same sense that dead code is unreachable in a conventional binary — structurally, not probabilistically.

---

## Why Not a Separate GT?

A natural alternative would be to give each method its own GT (capability token) so that private methods simply never have a GT issued. This is the capability approach used for object-level facets. It was considered and rejected for the following reasons:

**NS table amplification**: Every new GT requires a namespace table entry. An abstraction with 20 methods would require 20 NS entries under this model, growing the NS table and the trusted computing base.

**Trust boundary fragmentation**: Each GT creates a new security boundary with its own c-list. A 20-method abstraction would have 20 separate c-lists, each needing its own lump seal verification. The atomic simplicity of a single sealed lump is lost.

**LAMBDA semantics mismatch**: In this architecture, LAMBDA means a well-defined entry point *within* a sealed lump — not a separate GT with its own c-list. Introducing per-method GTs would require LAMBDA instructions to load GTs from a caller-supplied c-list, re-introducing the exact amplification problem that capability architectures are designed to avoid.

**The correct design**: Method dispatch is a macro, not a trust boundary. The auto-generated dispatch code at M00 is inside the existing lump seal. Private methods are excluded from the macro. No new GT or NS entry is created.

This is the same design principle that makes OS kernels compile internal functions without exporting them from the symbol table: the security is provided by the binary boundary (the lump seal), not by the capability mechanism.

---

## Auto-Generated Dispatch

When a CLOOMC abstraction uses at least one explicit `public` or `private` qualifier, and has no hand-written `Dispatch` or `M00` method, the compiler automatically generates M00.

The generated dispatch is a linear scan identical in structure to the traditional hand-written MCMP+BRANCHEQ loop:

```
M00 (auto-generated Dispatch):
  ISUB DR15, DR0, DR0        ; initialize counter to 0
  IADD DR15, DR15, #1        ; counter = 1
  MCMP DR0, DR15             ; compare selector with 1
  BRANCHEQ → PublicMethod1   ; branch if selector == 1
  IADD DR15, DR15, #1        ; counter = 2
  MCMP DR0, DR15
  BRANCHEQ → PublicMethod2
  ...
  RETURN AL                  ; unknown selector: return
```

Private methods are compiled into the lump after the public methods and are assigned word offsets, but no BRANCHEQ entry points to them from M00.

**Condition code encoding:** The auto-generated dispatch uses `cond=AL` (value 14, always-execute) for all data-processing instructions (ISUB, IADD, RETURN). Hand-written RAW ISA code in this project has historically used `cond=NV` (value 15) for the same instructions. The difference is intentional and inconsequential:

- `cond=AL` (14): the ISA specification defines this as "always execute" — instruction executes on every cycle unconditionally.
- `cond=NV` (15): historically used as a second always-execute encoding in this IoT-profile processor's microarchitecture; no instruction in the RAW ISA method bodies is ever skipped due to a condition=15 code in practice.

The BRANCHEQ instruction (`cond=EQ`, value 0) and MCMP instruction (`cond=AL`, value 14) are encoded identically between hand-written and auto-generated forms. The branch target offsets are recomputed from scratch by the compiler for each abstraction and are therefore exact. The only difference is the condition field of the ISUB/IADD/RETURN words (14 vs 15), which have no observable semantic difference on IoT-profile hardware.

For WordString specifically: the auto-generated 41-word M00 (for 13 public methods) has BRANCHEQ targets at the same absolute word offsets as the original hand-written 41-word Dispatch, verified analytically by computing selector offsets from method code lengths.

---

## Trigger Condition

Auto-dispatch is only generated when **at least one method has an explicit visibility qualifier**. Abstractions that use no qualifiers (all `method Foo(...)` without prefix) compile identically to before — no dispatch method is prepended. This preserves full backward compatibility.

**All-private abstractions:** If all methods are private (zero public methods), the generated M00 is a single `RETURN` instruction. The lump entrypoint immediately returns without dispatching to any private method body. This ensures private methods are structurally unreachable even in degenerate cases.

---

## Worked Example: Mint

The `mint.cloomc` abstraction manages memory allocation and capability revocation:

```
abstraction Mint {
    capabilities { Memory }

    public method Create(size, perms) {
        result = call(Memory.Allocate(size))
        return(result)
    }

    private method Revoke(index) {
        var word2 = read(CR7, 2)
        var version = bfext(word2, 25, 7)
        var newVersion = version + 1
        bfins(word2, newVersion, 25, 7)
        write(CR7, 2, word2)
        return(newVersion)
    }

    public method Transfer(gt) {
        return(gt)
    }
}
```

### Why Revoke is private

`Revoke` modifies the version number embedded in a capability word (at CR7 offset 2). This operation is an internal bookkeeping step — it increments the version counter that a revocation check compares against. External callers should never be able to trigger version bumps directly; doing so would allow them to revoke capabilities they don't own.

By marking `Revoke` as `private`, the lump seal guarantees that no external caller can reach `Revoke`. The version bump can only occur when `Create` internally decides to call it (via a `BRANCH` instruction to `Revoke`'s offset) — a design decision that the abstraction author controls and that is verified by the lump seal at build time.

### Compiled layout

With the auto-generated dispatch, the Mint lump looks like:

```
M00  Dispatch     — auto-generated (8 words: ISUB + 2×(IADD+MCMP+BRANCHEQ) + RETURN)
M01  Create       — public, selector 1
M02  Revoke       — private: compiled at its offset; absent from dispatch table
M03  Transfer     — public, selector 2
```

The dispatch table has two entries (selectors 1 and 2 for Create and Transfer). Selector 3 would hit the RETURN fallthrough and return immediately. There is no selector that routes to Revoke.

---

## Selector Numbering

Public methods are assigned selectors in source order, starting at 1. Private methods are compiled at their offset but have no selector. AliasOf methods share the offset (and selector) of their target method.

| Selector | Method     | Visibility |
|----------|------------|------------|
| 1        | Create     | public     |
| —        | Revoke     | private    |
| 2        | Transfer   | public     |

---

## Summary

| Property | Value |
|----------|-------|
| Qualifier `public` | Method appears in auto-generated dispatch table |
| Qualifier `private` | Method is compiled but excluded from dispatch; unreachable from outside |
| Default (no qualifier) | Treated as `public` for backward compatibility; no auto-dispatch generated unless at least one qualifier is present |
| Enforcement mechanism | Structural: compiler exclusion from M00 + lump seal |
| GT count | Unchanged — no new GT or NS entry per method |
| Binary compatibility | Generated dispatch is structurally identical to hand-written; condition encoding uses AL (14) |

---

## Worked Example: WordString

`WordString.cloomc` implements string operations for a UTF-8 lump object. It has 27 named methods: 13 externally callable (matching the original hand-written 13-selector dispatch) and 14 internal helpers.

### Original hand-written dispatch

The original source contained an explicit `method Dispatch [RAW ISA]` with 41 words of ISUB+MCMP+BRANCHEQ code exposing selectors 1–13. Methods like `StringOp`, `Classify`, `CheckNonZero`, `ToUppercase`, and `Offset` were present in the lump binary but had no BRANCHEQ entry in the dispatch — they were structurally unreachable from outside.

### New auto-dispatch design

Removing the hand-written dispatch and adding `public`/`private` qualifiers preserves the same external interface:

- **13 public** (selectors 1–13): `GetWordCount`, `GetCharCount`, `GetByteCount`, `GetCharByte`, `IsUppercase`, `IsLowercase`, `ReturnFalse`, `IsDigit`, `IsAlpha`, `IsUpperExt`, `IsPunct`, `IsLowerExt`, `IsSymbol`.
- **14 private** (no selectors): `IsSpace`, `IsAlphaNum`, `ToUppercase`, `Stub`, `ToLowercase`, `StubExt`, `NormaliseDigit`, `IsHex`, `Offset`, `StringOp`, `CheckNonZero`, `CheckPositive`, `ComputeBase`, `Classify`.

The auto-generated M00 is 41 words (3×13 + 2) — exactly the same word count as the hand-written dispatch, preserving the existing lump layout.

---

## Language Target Notes

### Pet-name (`[pet name]`) sources

Pet-name abstractions are expression-oriented — they describe data-register naming aliases rather than method implementations. They do not use `method` declarations and therefore cannot carry `public`/`private` qualifiers. Pet-name compilation is unaffected by this change; visibility qualifiers are simply inapplicable to that target.

---

See also: [dispatch-styles.md](dispatch-styles.md) for how auto-dispatch fits into the three existing dispatch styles.
---
*Confidential — Kenneth Hamer-Hodges — April 2026*
