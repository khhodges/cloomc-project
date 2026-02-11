# Golden Tokens

## What Are Golden Tokens?

Golden Tokens (GTs) are the fundamental unit of access control in the CTMM architecture. Every access to a resource -- whether loading data, calling a service, or switching privilege levels -- requires a valid Golden Token that grants the necessary permissions. Golden Tokens are unforgeable: they cannot be fabricated by software, only created and managed through hardware-enforced mechanisms.

A Golden Token encodes three things:
1. **What** resource it refers to (via an index or offset into the namespace)
2. **What operations** are permitted (via permission bits)
3. **Whether it is authentic** (via MAC seal validation)

Without a valid Golden Token, no operation proceeds. Any attempt to use an invalid, expired, or insufficient token results in a FAULT.

---

## GT Format: Sim-64 (CTMM)

Sim-64 uses a 64-bit Golden Token with the following structure:

| Field | Description |
|-------|-------------|
| **Offset** | Index into the namespace identifying the target resource |
| **Permissions** | 10-bit permission field (R, W, X, L, S, E, B, M, F, G) |
| **Spare** | Reserved bits for future use |

The 64-bit GT is stored directly in capability registers CR0-CR15.

---

## GT Format: Sim-32 (RV32-Cap)

Sim-32 uses a 32-bit Golden Token with a precisely defined bit layout:

```
[31:27] Version     (5 bits)  -- Version tag for GC invalidation
[26:12] Index       (15 bits) -- Namespace entry index (0-32,767)
[11:2]  Permissions (10 bits) -- G, F, M, B, S, E, L, X, W, R
[1:0]   Type        (2 bits)  -- Token type classification
```

Each capability register in Sim-32 is 128 bits wide (4 x 32-bit words):

| Word | Content |
|------|---------|
| word0 | The 32-bit Golden Token |
| word1 | Location (from namespace entry) |
| word2 | Limit (from namespace entry) |
| word3 | VersionSeals (from namespace entry) |

---

## Permission Bits

Both simulators use the same 10 permission bits. Each bit independently enables a specific operation:

| Bit | Name | Description |
|-----|------|-------------|
| R | Read | Read data from the referenced resource |
| W | Write | Write data to the referenced resource |
| X | Execute | Execute code at the referenced location |
| L | Load | Load a Golden Token from a C-List |
| S | Save | Save a Golden Token to a C-List |
| E | Enter | Enter an abstraction (call a service) |
| B | Bind | Bind a capability to a context |
| M | Machine | Machine-level privileged operations |
| F | Foreign | Foreign/remote capability (proxy) |
| G | Garbage | Garbage collection management flag |

---

## Permission Domains

Permissions are organized into four mutually exclusive domains. A single operation context uses permissions from only one domain:

| Domain | Bits | Purpose |
|--------|------|---------|
| **Church** | L, S | Capability management -- loading and saving Golden Tokens through C-Lists |
| **Turing** | R, W, X | Data processing -- reading, writing, and executing computational resources |
| **Lambda** | E | Abstraction entry -- invoking protected services and functions |
| **Meta** | B, M, F, G | System management -- binding, machine privilege, foreign proxies, garbage collection |

This separation ensures that a single token cannot simultaneously grant data access and capability management, preventing privilege confusion attacks.

---

## GT Type Field (Sim-32 Only)

Sim-32 includes a 2-bit type field in bits [1:0] of the Golden Token, classifying the nature of the referenced resource:

| Value | Type | Description |
|-------|------|-------------|
| 00 | Inform | Local resource -- data or code residing in the local namespace |
| 01 | Outform | Remote resource -- data or service accessible through a network proxy |
| 10 | Literal | Literal value -- the token encodes a direct value, not a reference |
| 11 | Abstract | Abstract service -- a callable abstraction (function or service entry point) |

Sim-64 does not have an explicit type field; the type is implicit in how the token is used.

---

## GT Version Field (Sim-32 Only)

Sim-32 includes a 5-bit version field in bits [31:27] of the Golden Token. This version tag is critical for garbage collection safety:

- Each namespace entry has a corresponding version number stored in the VersionSeals word.
- When a GT is used (LOAD or CALL), the version in the GT must match the version in the namespace entry. A mismatch triggers a FAULT.
- During garbage collection sweep, reclaimed entries have their version bumped. This automatically invalidates all outstanding GTs that reference the old version, preventing use-after-free vulnerabilities.

Sim-64 does not use a version field; it relies on different GC mechanisms (G-bit clearing on access).

---

## Capability Registers

Both simulators provide 16 capability registers (CR0-CR15), divided into two groups:

### Instruction-Addressable Registers (CR0-CR7)

These registers are directly accessible by Church instructions through a 3-bit register encoding field. Software can freely read and manipulate GTs in these registers using LOAD, SAVE, and other Church instructions.

### System Registers (CR8-CR15)

These registers are protected from direct instruction access. The only way to write to a system register is through the SWITCH instruction, which requires appropriate permissions. This architectural constraint prevents privilege escalation through direct register manipulation.

### Special Register Roles

| Register | Role | Description |
|----------|------|-------------|
| **CR6** | C-List | Points to the current Capability List -- the set of capabilities available to the running code |
| **CR7** | Nucleus | Points to the code (access code or nucleus) of the current abstraction |
| **CR8** | Thread | Identifies the current thread of execution |
| **CR15** | Namespace | Points to the root of the namespace hierarchy -- the master directory of all resources |

These roles are consistent across both simulators. CR6 and CR7 are saved and restored during CALL/RETURN operations. CR8 is updated during CHANGE (thread switching). CR15 defines the security boundary of the entire system.
