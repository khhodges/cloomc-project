# Namespace and Security Model

## Namespace Table Structure

The namespace is the master directory of all resources in the system. Every Golden Token references an entry in the namespace table. Each entry describes a resource with three fields:

| Field | Sim-64 (CTMM) | Sim-32 (RV32-Cap) |
|-------|---------------|-------------------|
| **Word 1** | Location | Location (32-bit) |
| **Word 2** | Limit | Limit (32-bit) |
| **Word 3** | Seals (MAC hash) | VersionSeals (32-bit) |

### Sim-64 Namespace Entries

Sim-64 uses 3-word entries where the Seals word contains a hardware-enforced MAC (Message Authentication Code) hash. This hash is computed from the Location and Limit fields and serves as an integrity check -- any tampering with the entry's data will cause the MAC to fail validation.

### Sim-32 Namespace Entries

Sim-32 uses 3 x 32-bit word entries. The VersionSeals word combines two pieces of information:

```
VersionSeals [31:0]:
  [31:27] Version  (5 bits)  -- Current version of this entry
  [26:0]  Seal     (27 bits) -- FNV hash of Location + Limit
```

The 27-bit FNV seal serves the same purpose as the Sim-64 MAC hash: it provides integrity verification for the namespace entry. The 5-bit version field enables garbage collection by allowing stale tokens to be detected and invalidated.

The namespace table in Sim-32 supports up to 32,768 entries (limited by the 15-bit index field in the Golden Token). Each entry occupies 3 words, so the slot address is calculated as `Index x 3`.

---

## MAC Seal Validation

MAC seal validation is the mechanism that ensures Golden Tokens and namespace entries have not been corrupted or forged.

### When Validation Occurs

| Operation | Sim-64 | Sim-32 |
|-----------|--------|--------|
| **LOAD** | MAC hash checked on loaded GT | Version match + FNV seal checked on source GT and target namespace entry |
| **CALL** | Implicit capability integrity check | Version match + FNV seal checked on both source GT and target namespace entry |
| **SAVE** | N/A | FNV seal recomputed from Location + Limit, preserving existing version |

### How Validation Works (Sim-32)

When a LOAD or CALL instruction accesses a namespace entry:

1. The **version** in the Golden Token (bits [31:27]) is compared against the version in the namespace entry's VersionSeals word (bits [31:27]). If they do not match, the token is stale and a FAULT is triggered.
2. The **FNV seal** is recomputed from the entry's Location and Limit values and compared against the stored seal in VersionSeals (bits [26:0]). If they do not match, the entry has been corrupted and a FAULT is triggered.

When a SAVE instruction writes to a namespace entry:

1. The Location and Limit values from the source capability register are written to the namespace entry.
2. A new FNV seal is computed from the written Location and Limit values.
3. The VersionSeals word is constructed by combining the existing version with the new seal.

---

## Validation Flow

Every Church instruction that accesses the namespace follows a strict validation sequence. Any failure at any step triggers an immediate FAULT:

```
1. Permission Check
   Does the source/destination CR have the required permission bit?
   (L for LOAD, S for SAVE, E for CALL, M for SWITCH)
   Failure -> FAULT

2. GT Validation
   Is the Golden Token authentic?
   - Sim-64: Capability object integrity check
   - Sim-32: Version match + MAC seal validation
   Failure -> FAULT

3. Bounds Check
   Is the index within the valid range of the namespace table?
   - Sim-64: Against source C-List entry count
   - Sim-32: Against namespace table length (max 32,768)
   Failure -> FAULT

4. Namespace Entry MAC Check
   Is the target namespace entry intact?
   - Sim-64: Hardware MAC hash verification
   - Sim-32: FNV seal recomputation and comparison
   Failure -> FAULT

5. Operation
   Execute the requested operation (load, save, call, etc.)
```

---

## Failsafe Principle

The CTMM architecture follows a strict failsafe design: **any validation failure triggers a FAULT, handled by a single fault handler**. There are no partial failures, no silent degradation, and no undefined behaviors. The system is either operating correctly or it is faulted.

This applies uniformly to:
- Permission violations (missing required permission bit)
- Version mismatches (stale Golden Token)
- MAC/seal failures (corrupted namespace entry)
- Bounds violations (index out of range)
- Stack overflows (call stack full)
- Stack underflows (return with empty stack)

The fault handler is the single point of error management, ensuring consistent and predictable behavior regardless of the failure mode.

---

## Security Invariants

Both simulators enforce the following invariants at all times:

### No Direct System Register Access

Only CR0-CR7 are addressable through the 3-bit register encoding in Church instructions. System registers CR8-CR15 are physically unreachable through instruction encoding. This is an architectural constraint, not a software convention.

### Privilege Through SWITCH Only

The SWITCH instruction is the sole mechanism for writing to system registers CR8-CR15. It requires appropriate permissions:
- Sim-64: L or E permission on the source capability
- Sim-32: M (Machine) permission on the source capability

### Capability-Mediated Access

All resource access goes through capability-mediated C-Lists. LOAD reads from a C-List entry. SAVE writes to a C-List entry. There is no instruction that can access raw memory without a valid Golden Token authorizing the operation.

### Mutually Exclusive Permission Domains

The four permission domains (Church, Turing, Lambda, Meta) cannot be mixed within a single operation context. This prevents confused deputy attacks where a capability intended for one purpose is misused for another.

| Domain | Permissions | Operations |
|--------|-------------|------------|
| Church | L, S | Load/Save Golden Tokens through C-Lists |
| Turing | R, W, X | Read/Write data, Execute code |
| Lambda | E | Enter abstractions (protected calls) |
| Meta | B, M, F, G | Binding, machine privilege, foreign proxies, GC |
