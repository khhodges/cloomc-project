> **ARCHIVED — February 14, 2026**
>
> This proposal has been **superseded** by the simplified LAMBDA instruction design. The lambda register window (CR12-CR15 aliased to x28-x31) has been set aside in favor of a clean domain separation: capability registers hold capabilities exclusively, data registers hold values exclusively. The LAMBDA instruction now operates as a lightweight in-scope call using X permission, with arguments and results passed through data registers. See [LAMBDA Instruction Specification](lambda-instruction.md) for the current design.
>
> This document is retained for historical reference only. None of the concepts below are part of the current architecture.

---

# DESIGN PROPOSAL: Lambda Register Window

**v1.0 — 2026-04-29**
**CONFIDENTIAL**

**Status**: ARCHIVED — Superseded by simplified LAMBDA instruction design (February 14, 2026). Retained for historical reference.

**Date**: February 13, 2026

**Author**: Kenneth James Hamer-Hodges

**Fallback**: If this proposal encounters roadblocks, it can be abandoned with zero impact on the existing architecture. The opt-in CSR mechanism (LAMBDA_MODE) provides an additional safety net at the hardware level — the feature exists in silicon but does not activate unless explicitly enabled by the Nucleus at boot time.

---

## 1. Motivation

The GT-Literal and LAMBDA instruction design (see docs/gt-literals.md) provides a 2-3× performance improvement for pure computation by bypassing the mLoad validation path. However, the current design requires explicit bridge instructions (LDL, STL) to move values between the Turing domain (data registers) and the Church domain (capability registers). Every arithmetic operation on GT-Literals requires:

```asm
STL  x10, CR1         ; extract value from CR — 1 cycle
STL  x11, CR2         ; extract value from CR — 1 cycle
ADD  x12, x10, x11    ; compute — 1 cycle
LDL  CR0, x12         ; wrap result as GT-Literal — 1 cycle
```

Four instructions and four cycles for a single addition. The bridge overhead consumes half the cycles. This proposal eliminates that overhead entirely through register aliasing.

---

## 2. Proposed CR Layout Change

### Current Layout

| Register | Assignment | Addressable |
|----------|-----------|-------------|
| CR0-CR7 | User capability registers | Yes (instruction-addressable) |
| CR8 | Thread identity | No (system) |
| CR9-CR10 | Reserved | No (system) |
| CR11-CR14 | Unused | No |
| CR15 | Namespace root | No (system) |

### Proposed Layout

| Register | Assignment | Addressable |
|----------|-----------|-------------|
| CR0-CR7 | User capability registers | Yes (instruction-addressable) |
| CR8 | Thread identity | No (system) |
| CR9-CR10 | Reserved | No (system) |
| CR11 | Namespace root (moved from CR15) | No (system) |
| CR12-CR15 | Lambda registers (aliased to x28-x31) | Via DR alias when LAMBDA_MODE=1 |

### Rationale for CR11 as Namespace Root

- CR11 remains outside the instruction-addressable range (CR0-CR7), so user code still cannot directly manipulate the Namespace root — same security as CR15
- CR11 is currently unused, so no functionality is displaced
- Freeing CR12-CR15 as a contiguous 4-register block creates a clean 2-bit address space (binary 11xx) for the lambda register window

---

## 3. Register Aliasing Mechanism

When LAMBDA_MODE is enabled, the top four data registers are aliased to capability registers:

| Data Register | Capability Register | Direction |
|--------------|-------------------|-----------|
| x28 (t3) | CR12 | Bidirectional |
| x29 (t4) | CR13 | Bidirectional |
| x30 (t5) | CR14 | Bidirectional |
| x31 (t6) | CR15 | Bidirectional |

### Hardware Behavior

**When a Turing instruction reads from x28-x31** (LAMBDA_MODE=1):
1. Hardware routes the read to CR12-CR15 instead of the DR file
2. Checks Type = 10 (Literal) → FAULT if not a GT-Literal
3. Extracts the 30-bit value from bits [31:2]
4. Delivers the value to the ALU as a normal 32-bit operand (zero-extended)

This is an **implicit STL** — same security check, same value extraction, no explicit instruction needed.

**When a Turing instruction writes to x28-x31** (LAMBDA_MODE=1):
1. Hardware routes the write to CR12-CR15 instead of the DR file
2. Takes the 32-bit result, truncates to 30 bits
3. Sets bits [1:0] = 10 (Literal type)
4. Writes the GT-Literal to the destination CR

This is an **implicit LDL** — same type tagging, same value wrapping, no explicit instruction needed.

**When LAMBDA_MODE=0**:
- x28-x31 behave as normal data registers
- CR12-CR15 are not accessible
- All existing RV32I code runs unchanged

### Hardware Decode

The aliasing decision is a simple multiplexer controlled by two signals:
- `lambda_mode` (from the LAMBDA_MODE CSR)
- `reg_addr[4:3] == 2'b11` (register number 28-31, i.e., top 4)

```
if (lambda_mode && reg_addr[4:3] == 2'b11)
    route to CR[reg_addr[1:0] + 12]   // CR12-CR15
else
    route to DR[reg_addr]               // x0-x31
```

Minimal additional hardware: one AND gate per register access port, plus 4 multiplexers.

---

## 4. Opt-In Activation via LAMBDA_MODE CSR

### CSR Definition

```
CSR address: 0x800 (machine-level custom CSR range)
Name:        LAMBDA_MODE
Width:       1 bit
Access:      M-level write, any-level read
Default:     0 (disabled)
```

### Activation

Only the Nucleus (M-level / Meta-level code) can enable lambda registers:

```asm
; During boot or thread creation:
LI    t0, 1
CSRW  LAMBDA_MODE, t0    ; enable lambda register aliasing
```

User-level code cannot change the mode — attempting to write LAMBDA_MODE from U-level or S-level causes a privilege FAULT. User-level code can read the CSR to discover whether lambda registers are available.

### Per-Thread Context

The LAMBDA_MODE state is part of the thread context:
- **CHANGE save**: LAMBDA_MODE CSR value is saved as part of the thread's context (alongside DRs and call stack)
- **CHANGE restore**: LAMBDA_MODE is restored from the incoming thread's context
- Different threads can run with different modes — legacy threads with LAMBDA_MODE=0, capability-aware threads with LAMBDA_MODE=1

This enables gradual adoption: old code runs in threads with LAMBDA_MODE=0, new code opts in per-thread.

---

## 5. Examples

### 5.1 A = B + C (Simplest Case)

```asm
; Setup: B=5 in x5, C=3 in x6. LAMBDA_MODE=1.

; Load values into lambda registers
LDL   CR12, x5         ; CR12 = GT-Literal(5) — 1 cycle
LDL   CR13, x6         ; CR13 = GT-Literal(3) — 1 cycle

; Compute directly — no bridge instructions needed
ADD   x28, x29, x30    ; x28/CR12 = 5 + 3 = GT-Literal(8) — 1 cycle
                        ; hardware reads CR13(5), CR14(3), writes CR12(8)
```

**3 cycles total** vs. 6 cycles with explicit LDL/STL, vs. 15+ cycles with mLoad path.

Note: The first two LDL instructions could also be eliminated if the values were already in CR12-CR13 from a prior CAP.LOAD of Literal-type namespace entries or prior computation.

### 5.2 Dot Product: A*B + C*D

```asm
; CR12=A, CR13=B, CR14=C, CR15=D (loaded previously)
; LAMBDA_MODE=1

MUL   x28, x28, x29    ; CR12 = A*B — 1 cycle
MUL   x30, x30, x31    ; CR14 = C*D — 1 cycle
ADD   x28, x28, x30    ; CR12 = A*B + C*D — 1 cycle
```

**3 cycles** for a dot product on GT-Literal values. No bridge instructions at all.

### 5.3 Mixed DR and CR Computation

```asm
; x5 holds a loop counter (data register, not a GT-Literal)
; CR12 holds an accumulator as GT-Literal
; LAMBDA_MODE=1

; Read accumulator, add loop counter, write back
STL   x10, CR12         ; extract accumulator value — 1 cycle (explicit, x10 is a DR)
ADD   x10, x10, x5      ; add loop counter — 1 cycle
LDL   CR12, x10         ; wrap result — 1 cycle
; Or equivalently, if we allow mixed DR/CR sources:
; This case still needs explicit LDL/STL for mixing domains
```

### 5.4 LAMBDA with Lambda Registers

```asm
; CR3 holds a GT with X permission pointing to a "square" function body
; LAMBDA_MODE=1

LDL   CR12, x7           ; CR12 = GT-Literal(7) — 1 cycle
LAMBDA CR12, CR3, CR12    ; CR12 = square(7) = GT-Literal(49) — 2 cycles + body

; The body can use x28-x31 as its working registers:
;   MUL  x28, x28, x28   ; square the argument using aliased registers
;   (result automatically written as GT-Literal to CR12)
```

### 5.5 Functional Map with Lambda Registers

```asm
; Apply "double" function to each element of an array
; CR3 = GT with X permission pointing to "double" body
; x25 = input array pointer, x26 = output array pointer
; LAMBDA_MODE=1

loop:
  LW    x28, 0(x25)        ; load element directly into CR12 as GT-Literal
  LAMBDA CR12, CR3, CR12   ; apply double — 2 cycles + body
  STL   x10, CR12          ; extract result — 1 cycle
  SW    x10, 0(x26)        ; store to output — 1 cycle
  ADDI  x25, x25, 4
  ADDI  x26, x26, 4
  BNE   x25, x27, loop
```

---

## 6. Security Analysis

### GT-Literal Immutability Is Preserved

The aliased register mechanism does not mutate GT-Literals. Every Turing instruction that writes to x28-x31 (CR12-CR15) creates a **new** GT-Literal. The operation is:

1. **Read** source CR values (implicit STL — read-only extraction with Type check)
2. **Compute** in the Turing ALU (pure arithmetic on extracted values)
3. **Create** a fresh GT-Literal in the destination CR (implicit LDL — write new value with Type=10)

This is functionally identical to the explicit STL → compute → LDL sequence. The data flow is the same. The security properties are the same. Only the instruction count is reduced.

### Type Enforcement

- Reading from x28-x31 when CR12-CR15 does not hold a GT-Literal (Type ≠ 10) → **FAULT**
- This prevents extracting bits from capability references through the alias — same protection as STL's type check
- A capability reference (Inform, Outform, Abstract) in CR12-CR15 cannot be read through the alias

### Non-Escalation

- Writing to x28-x31 always produces a GT-Literal (Type=10) — never a capability reference
- A GT-Literal cannot be used as a namespace reference (CAP.LOAD, CAP.CALL will FAULT on Type=10)
- The alias path cannot create, modify, or forge capability references

### Non-Forgery

- The Type field (bits [1:0] = 10) is set by hardware on every write through the alias
- Software cannot control the Type bits through the alias path
- Only LDL (explicit) or the alias write (implicit) can set Type=10

### Privilege Protection

- LAMBDA_MODE is M-level writable only — user code cannot toggle the aliasing
- If LAMBDA_MODE=0, x28-x31 are ordinary data registers — no capability register access
- If a malicious user-level program runs with LAMBDA_MODE=0, it cannot reach CR12-CR15 through the data register path

### Functional Programming Parallel

In lambda calculus, values are immutable. The expression `(+ 3 5)` does not modify 3 or 5 — it produces a new value 8. The aliased register mechanism implements exactly this semantics: source values are read (never modified), a new result value is created. Church's value immutability is preserved by the hardware.

---

## 7. LAMBDA Instruction: Continued Role

The lambda register aliasing handles **inline arithmetic** on GT-Literal values. It does not replace the LAMBDA instruction, which handles **function application**:

| Mechanism | What It Does | Example |
|-----------|-------------|---------|
| Lambda registers (x28-x31) | Single arithmetic operation on values | `ADD x28, x29, x30` |
| LAMBDA instruction | Apply a multi-instruction code body to a value | `LAMBDA CR0, CR3, CR12` |

The LAMBDA instruction branches to a reusable code body (with X permission), executes it, and returns. The lambda registers provide the working space within that body:

```asm
; LAMBDA body for "square" function:
;   Input: CRarg (CR12/x28) holds the argument
;   Output: CRd (CR12/x28) holds the result
square_body:
    MUL  x28, x28, x28    ; x28² using aliased register — 1 cycle
    ; implicit return (hardware restores PC)
```

Without LAMBDA, applying a reusable function requires CALL/RETURN — 10+ cycles of domain-crossing overhead. LAMBDA stays within the same domain at 2 cycles setup.

---

## 8. LAMBDA Return Address

With lambda registers providing the arithmetic working space, the LAMBDA instruction's return mechanism should be clean and secure:

**Proposed**: LAMBDA saves the return PC in an **internal hardware micro-return register**, not visible to any software instruction. The body terminates with a new `LRET` instruction (or reuses the existing RET with a hardware-internal flag). The hardware pops the saved PC from the micro-return register.

This is secure because:
- The return address is not in any software-accessible register — cannot be tampered with
- The micro-return register is cleared on LAMBDA entry and restored on LRET
- Nested LAMBDA requires a small hardware micro-return stack (depth 4-8 is sufficient for practical lambda chains)
- If the micro-return stack overflows (deeply nested lambdas), the hardware can TRAP and fall back to the CALL/RETURN path

---

## 9. Trade-Offs

### What We Gain

1. **Zero-overhead Church-Turing bridging**: Turing instructions operate directly on GT-Literal values in CRs, no explicit LDL/STL needed
2. **No new instruction encoding**: Uses existing RV32I register numbering — no prefix, no fused opcodes
3. **Backward compatible**: LAMBDA_MODE=0 is standard RISC-V, no registers lost
4. **Clean hardware**: Simple multiplexer at register file ports, one CSR bit
5. **Per-thread opt-in**: Legacy and lambda-aware code coexist in different threads
6. **Performance**: A=B+C drops from 6 cycles to 3 (or 1 if values already in lambda registers)

### What We Give Up

1. **Four data registers** (x28-x31) when LAMBDA_MODE=1 — but these are caller-saved temporaries (t3-t6), the least valuable in the RISC-V calling convention
2. **CR15 changes meaning** — moves from Namespace root to Lambda register. Requires updating all four implementations (simulator, Verilog, Amaranth, Church Machine)
3. **Complexity in CHANGE**: Thread context must save/restore LAMBDA_MODE and the 4 lambda register contents
4. **Toolchain awareness**: Compilers targeting LAMBDA_MODE=1 must know x28-x31 are aliased

### Comparison with LPREFIX Alternative

| Property | Lambda Register Window | LPREFIX |
|----------|----------------------|---------|
| Instructions needed | 0 (register numbering does it) | 1 prefix per modified instruction |
| Code density | Best (no extra instructions) | Moderate (prefix adds 4 bytes) |
| Encoding space | 0 new opcodes | 1 new opcode |
| Pipeline complexity | Multiplexer at register ports | Inter-instruction state (prefix flip-flops) |
| Registers affected | x28-x31 only (when enabled) | Any register (per-instruction) |
| Flexibility | Fixed 4 lambda registers | Any DR↔CR mapping per instruction |
| Hardware cost | Minimal (MUX + CSR) | Moderate (state register + auto-clear) |

**Recommendation**: Lambda Register Window as the primary mechanism (covers the common case with zero overhead), with LPREFIX as a possible future extension for cases needing more than 4 lambda registers.

---

## 10. Proposed Patent Claims (Draft — Not Yet Filed)

These claims would supplement the existing 12 claims in docs/patent-ctmm-lambda.md if the proposal is adopted.

### Proposed Claim 13 — Lambda Register Window

The architecture of Claim 1, further comprising a lambda register window wherein a contiguous subset of data registers (x28-x31 in the RISC-V implementation) are aliased to a corresponding contiguous subset of capability registers (CR12-CR15) when a configuration register (LAMBDA_MODE) is enabled; wherein Turing-domain instructions reading from said data registers implicitly extract the value from the corresponding capability register's GT-Literal with Type field verification; and wherein Turing-domain instructions writing to said data registers implicitly create a new GT-Literal in the corresponding capability register with the Type field set to Literal (10); thereby enabling Turing-domain arithmetic instructions to operate directly on Church-domain values without explicit bridge instructions.

### Proposed Claim 14 — Opt-In CSR Activation

The architecture of Proposed Claim 13, wherein the lambda register window is activated by a machine-level control and status register (LAMBDA_MODE CSR) writable only by the most privileged execution level (Nucleus/Meta); wherein user-level code cannot enable or disable the aliasing; and wherein the LAMBDA_MODE state is saved and restored as part of the thread context during context switches (CHANGE instruction), enabling per-thread configuration where legacy threads operate with full data register access and lambda-aware threads operate with the aliased register window.

### Proposed Claim 15 — Implicit Type Enforcement via Register Alias

The architecture of Proposed Claim 13, wherein the implicit value extraction on read verifies that the capability register holds a GT-Literal (Type = 10) and generates a hardware fault if the capability register holds any reference type (Inform, Outform, or Abstract); and wherein the implicit value creation on write unconditionally sets the Type field to Literal (10), preventing the alias path from creating, modifying, or forging capability references; thereby maintaining the structural security model of Claims 8 and 9 through the aliased register path.

---

## 11. Impact Assessment

### Code Changes Required (If Adopted)

| Component | Change | Scope |
|-----------|--------|-------|
| riscv_cap/simulator.js | CR15 → CR11 for Namespace; add LAMBDA_MODE CSR; alias x28-x31 to CR12-CR15 | Moderate |
| riscv_cap/assembler.js | No change (register numbering unchanged) | None |
| verilog/ctmm_pkg.sv | Update CR_NS constant; add LAMBDA_MODE CSR | Small |
| verilog/ctmm_mload.sv | Update namespace root reference | Small |
| ctmm_amaranth/types.py | Update NS_ROOT constant | Small |
| ctmm_amaranth/mload.py | Update namespace root reference | Small |
| docs/gt-literals.md | Add lambda register section (if adopted) | Additive |
| docs/patent-ctmm-lambda.md | Add Claims 13-15 (if adopted) | Additive |
| All test files | Update CR15 references to CR11 | Moderate |

### Reversibility

- **Documentation level**: This proposal is in a separate file. If abandoned, delete this file. No other docs are modified.
- **Hardware level**: LAMBDA_MODE defaults to 0 (disabled). Even if built into silicon, the feature never activates unless the Nucleus enables it. Standard RISC-V operation is the default.
- **Code level**: No code changes have been made. All changes are listed above for future reference if the proposal is adopted.

---

## 12. Open Questions

1. **LW/SW through aliased registers**: Should `LW x28, 0(x25)` load a memory word directly into CR12 as a GT-Literal? This would be powerful for array operations but mixes memory access with type tagging.

2. **Nested LAMBDA depth**: What is the practical maximum depth for the micro-return stack? Lambda calculus theoretically allows unbounded nesting, but practical code rarely exceeds 4-8 levels.

3. **Interaction with CALL/RETURN**: If a CALL occurs while LAMBDA_MODE=1, should x28-x31 in the saved context be the CR12-CR15 GT-Literals or the underlying DR values? The thread table shadow already tracks CR0-CR7; should it extend to CR12-CR15?

4. **Lambda register window**: The Church Machine uses DR0-DR15. Should the lambda register window apply to all implementations?

5. **GC interaction**: CR12-CR15 hold direct GT-Literals (no namespace entry). They don't participate in GC. But if one holds an indirect GT-Literal (from CAP.LOAD), should the GC scan walk through them? The current scan walks CR0-CR7 via the thread table shadow.

6. **Interrupts/exceptions under LAMBDA_MODE**: If an interrupt or exception occurs while LAMBDA_MODE=1, the context save must preserve CR12-CR15 contents and the micro-return stack state. What is the save/restore protocol? Should the interrupt handler run with LAMBDA_MODE=0 (safe default) or inherit the interrupted thread's mode?

7. **mLoad/mLoadByIndex namespace root reference**: If CR11 becomes the Namespace root, all paths that currently reference CR15 for namespace access — mLoad, mLoadByIndex, RETURN revalidation (direct mode), GC scan root — must be updated. Should the namespace root register be a configurable constant (allowing either CR11 or CR15) or a hard architectural change?
---
*Confidential — Kenneth Hamer-Hodges — April 2026*
