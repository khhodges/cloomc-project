# GT-Literals and Network Transparency

> **Status**: Architectural design document. The GT Type field (Inform, Outform, Literal, Abstract) exists in the simulator's 32-bit GT format. GT-Literal namespace entries can be constructed via boot_builder.py. Outform behavior, TRAP handling, and tunnel/credential operations are not yet implemented in the simulator. This document specifies the planned design.

## Overview

Network transparency is the capability-secured mechanism by which the Meta Machine accesses remote objects and services as if they were local. The GT **Type field** determines whether a resource is local (Inform) or remote (Outform). The existing 6-bit permissions (R, W, X, L, S, E) control access identically regardless of location — this is what makes it *transparent*.

The architecture is **symmetrical**: the Meta Machine can both consume remote resources and serve its own resources to authorized remote parties.

Object fetch and flush (R/W on Outform) use **standard HTTPS** — the same browser mechanisms the web already uses (TLS, ETag, Cache-Control). This ensures interoperability with existing web infrastructure. RPC calls (E on Outform Abstract) between Meta Machines use an encrypted **point-to-point tunnel** keyed by a **Literal GT**, since both endpoints understand the capability protocol.

---

## GT Type Field

The 2-bit Type field in the 32-bit Golden Token classifies each resource:

| Value | Type | Description |
|-------|------|-------------|
| 00 | **Inform** | Local resource — data or code in the local namespace |
| 01 | **Outform** | Remote resource — data or service at a network URL, accessed transparently |
| 10 | **Literal** | GT-Literal — capability-secured handle to a secret value in the namespace entry |
| 11 | **Abstract** | Abstract service — a callable abstraction (function or service entry point) |

The Type field combines with permissions to determine behavior:

| Type + Permission | Behavior |
|-------------------|----------|
| Outform + R | **Object fetch** — HTTPS GET from remote URL into local cache |
| Outform + W | **Object flush** — HTTPS PUT dirty cached object back to home URL |
| Outform + E (Abstract) | **RPC call** — invoke remote Meta Machine service through Literal GT encrypted tunnel |
| Outform + L | **TRAP** — future extension for remote service discovery |
| Outform + S | **TRAP** — future extension for remote capability delegation |
| Outform + X | **TRAP** — nonsense case, safe |

---

## TRAP vs FAULT

Network transparency introduces a distinction between two kinds of exceptional events:

| Event | Meaning | Recovery |
|-------|---------|----------|
| **FAULT** | Security violation — forged MAC, permission denied, version mismatch, bounds error | Unrecoverable. Execution stops. |
| **TRAP** | Architectural event requiring software handling — cache miss, deferred Outform operation | Recoverable. Handle and retry. |

TRAP codes for network transparency:

| Code | Name | Trigger |
|------|------|---------|
| TRAP_CACHE_MISS | Cache Miss | R access on uncached Outform object |
| TRAP_OUTFORM_L | Outform Load | L permission on Outform entry (future) |
| TRAP_OUTFORM_S | Outform Save | S permission on Outform entry (future) |
| TRAP_OUTFORM_X | Outform Execute | X permission on Outform entry (future) |

---

## Case 1: Object Fetch and Flush (R/W on Outform)

### Object Fetch (R — Read)

When mLoad encounters an Outform entry that is not in the local cache:

```
1. mLoad detects Type = Outform (01) on namespace entry
2. Check R permission on GT → FAULT if denied
3. Check local cache for this namespace index
4. Cache hit → return cached object (same as Inform access)
5. Cache miss → TRAP: CACHE_MISS
6. Trap handler initiates HTTPS GET from home URL (standard browser mechanism)
7. Response stored in local cache with ETag/Last-Modified/Cache-Control metadata
8. Instruction retried → cache hit → succeeds
```

Standard browser caching semantics apply: conditional GET with `If-None-Match` / `If-Modified-Since`, `Cache-Control` directives honored, TLS provides transport security. The Meta Machine adds capability-based access control on top of what the browser already provides.

In auto-run mode, the simulator automatically retries after the fetch completes, making the network access transparent to the running program.

### Object Flush (W — Write)

When a cached Outform object is modified:

```
1. W permission checked on GT → FAULT if denied
2. Modification applied to local cached copy
3. Dirty bit set on namespace entry metadata
4. On eviction, GC sweep, or explicit flush:
   a. Dirty cached object written back to home URL via HTTPS PUT/POST
   b. Server validates ETag / version for conflict detection
   c. Dirty bit cleared after successful flush
   d. Safe to invalidate / bump version
```

This is analogous to write-back caching in virtual memory, but for named objects rather than fixed-size pages. The URL is the "home address" and the local cache is the working copy. Standard HTTP conflict detection (ETag, `If-Match`) prevents lost updates.

---

## Case 2: RPC Tunnel (E on Outform Abstract)

When CALL targets an Outform entry with Abstract type:

```
1. E permission checked on GT → FAULT if denied
2. Data registers (x0-x31) serialized as argument payload
3. Payload encrypted using Literal GT tunnel key
4. Sent to remote Meta Machine's /api/invoke endpoint
5. Remote machine:
   a. Decrypts using its copy of the Literal GT key
   b. Validates the incoming GT (MAC, version, permissions)
   c. Executes the abstraction locally
   d. Serializes result (data registers + condition flags)
   e. Encrypts and returns response
6. Local machine decrypts response, updates data registers
7. RETURN resumes locally — the program never knew it left
```

---

## Symmetrical Operation

Network transparency is **symmetrical** — the Meta Machine is both client and server:

### Outbound (as client)

| Operation | Permission | Network Action |
|-----------|-----------|---------------|
| Object fetch | R | HTTPS GET from remote URL (standard browser mechanism) |
| Object flush | W | HTTPS PUT to remote URL (standard browser mechanism) |
| RPC call | E | POST to remote /api/invoke via Literal GT encrypted tunnel |

### Inbound (as server)

| Endpoint | Permission Validated | Action |
|----------|---------------------|--------|
| /api/serve | R on requested GT | Serve local namespace object to authorized remote requestor |
| /api/invoke | E on target GT | Execute local abstraction on behalf of remote caller, return results |
| /api/accept | W on target GT | Accept modified object written back by remote party |

All inbound requests carry a GT that is validated through the same mLoad path — MAC check, version check, permission check — before any access is granted.

---

## GT-Literals

The GT-Literal (Type = 10) is a **capability-secured handle to a secret value**. The GT itself uses the standard 32-bit format:

```
GT-Literal [31:0]:
  [31:27] Version    (7 bits)  — Revocable via GC version bump
  [26:8]  Index      (17 bits) — Namespace index of the value entry
  [7:2]   Permissions (6 bits) — Access control on the value itself
  [1:0]   Type = 10  (2 bits)  — Literal
```

The **secret value** is not stored in the GT itself — it resides in the **namespace entry** that the GT references. The entry's Location and Limit fields hold the actual value (a crypto key, a credential, a token, etc.), and the entry's MAC seal provides integrity protection. This is consistent with the capability model: the GT is a validated handle, the namespace entry holds the actual resource.

### Core Properties

Every GT-Literal, regardless of its specific use, shares these properties:

1. **Capability-secured**: Access to the secret value requires a valid GT with correct permissions, MAC, and version — the same mLoad validation path as any other namespace access
2. **Revocable**: Sweeping the GT-Literal during garbage collection bumps its version, instantly invalidating the handle. Anyone still holding the old GT gets FAULT: VERSION
3. **Compartmentalized**: Each GT-Literal is independent. Compromise of one does not affect any other
4. **GC-managed lifecycle**: The secret value follows the same lifecycle as every other namespace object — created, used, garbage-collected
5. **MAC-protected**: The namespace entry's MAC seal ensures the value has not been tampered with

### Use Case 1: RPC Tunnel Key

A GT-Literal whose namespace entry holds a **symmetric encryption key** for point-to-point RPC tunnels between Meta Machines. Both communicating namespaces hold matching entries with the same key material (Location + Limit values).

- The Literal GT is loaded into a CR before a `CAP.CALL` on an Outform Abstract GT
- The CALL instruction uses the key material to encrypt the serialized register payload
- The remote Meta Machine decrypts using its matching entry
- **Revocation**: GC sweep bumps the version, killing the tunnel instantly
- **No PKI required**: The capability *is* the trust — no certificate authorities needed

### Use Case 2: Transparent Login (Conceptual)

A GT-Literal whose namespace entry holds **authentication credentials** (a password hash, a token, a certificate fingerprint). Presenting the GT-Literal to a remote service proves identity without exposing the underlying secret to the calling program. The exact handshake mechanism is a future design decision; the GT-Literal infrastructure is in place.

- The program holds a GT-Literal for "my login to service X" — it never sees the raw credential
- On CALL to the remote service, the system reads the credential from the namespace entry and presents it in the authentication handshake
- The credential is protected by MAC and version — cannot be forged or replayed after revocation
- **Revocation**: GC sweep invalidates the login credential. The program's GT becomes stale, login fails cleanly with FAULT: VERSION
- **Rotation**: Create a new namespace entry with updated credentials, issue a new GT-Literal. Old GT dies on next GC cycle

### Use Case 3: Session Tokens (Conceptual)

A GT-Literal whose namespace entry holds a **session identifier** for an ongoing interaction with a remote service. The session token is managed entirely within the capability model. The exact session protocol is a future design decision.

- Created when a session is established (e.g., after successful transparent login)
- The program passes the GT-Literal on each subsequent request — the system extracts the session token from the namespace entry automatically
- **Timeout/expiry**: The remote service can invalidate the session. Locally, GC sweep of the GT-Literal cleans up the stale session handle
- **Compartmentalized**: Each session with each service has its own GT-Literal. Compromise of one session does not affect others

### Use Case 4: Encryption Keys for Data at Rest (Conceptual)

A GT-Literal whose namespace entry holds a **symmetric key for encrypting stored objects**. This extends the crypto key concept beyond network tunnels to local data protection. The encryption/decryption mechanism is a future design decision.

- Objects written to persistent storage are encrypted using the key material from the GT-Literal's namespace entry
- Reading the object requires the same GT-Literal (or one pointing to the same namespace entry)
- **Key rotation**: Create a new entry with a new key, re-encrypt data, sweep the old GT-Literal
- **Access control**: R permission on the GT-Literal controls who can decrypt; W controls who can re-encrypt

### Use Case 5: API Key / Bearer Token Wrapping (Conceptual)

A GT-Literal whose namespace entry holds a **third-party API key or bearer token**. This wraps external credentials in the capability model so they are governed by the same security rules as everything else. The injection mechanism is a future design decision.

- The program holds a GT-Literal for "my API key to service X" — it never sees the raw key
- On outbound HTTPS requests, the system reads the API key from the namespace entry and injects it into the request header
- **Revocation**: GC sweep invalidates the GT-Literal. Even if the underlying API key is still valid, the program can no longer access it
- **Rotation**: Update the namespace entry with a new API key, or issue a new GT-Literal pointing to a new entry. Old GT dies on GC

### Common Pattern

All GT-Literal use cases follow the same pattern:

```
GT-Literal (handle)  →  Namespace Entry (secret value)
     │                        │
     ├─ Version (revocable)   ├─ Location (value part 1)
     ├─ Index (points to →)   ├─ Limit    (value part 2)
     ├─ Permissions (access)  ├─ MAC seal (integrity)
     └─ Type = 10 (Literal)   └─ gBit     (GC metadata)
```

The GT-Literal never exposes the secret. The program holds the handle, the system holds the value. Revocation is instant (GC version bump), compartmentalization is automatic (each use case gets its own GT-Literal), and integrity is guaranteed (MAC seal on the namespace entry).

---

## Example: CALL(CONNECT(me, mymother))

This example demonstrates how a single CLOOMC instruction establishes a cryptographically secured connection between two namespaces and invokes a remote service.

### Namespace Setup

Two Meta Machines ("me" and "mymother") each have their own namespace:

**"me" namespace:**

| Index | Name | Type | Description |
|-------|------|------|-------------|
| 0 | Root | Inform | Local namespace root |
| 1 | C-List | Inform | Local capability list |
| 2 | Code | Inform | Local code segment |
| 3 | Thread | Inform | Current thread object |
| 4 | TunnelKey_Mother | Literal | Crypto key for tunnel to "mymother" |
| 5 | Mother_CList | Outform | Remote: mymother's published C-List |
| 6 | Mother_Service | Outform/Abstract | Remote: a callable service on mymother |

**"mymother" namespace:**

| Index | Name | Type | Description |
|-------|------|------|-------------|
| 0 | Root | Inform | Local namespace root |
| 1 | C-List | Inform | Local capability list |
| 2 | Code | Inform | Local code segment |
| 3 | Thread | Inform | Current thread object |
| 4 | TunnelKey_Child | Literal | Crypto key for tunnel to "me" (same key material) |
| 5 | Published_CList | Inform | Services available to authorized remote callers |
| 6 | MyService | Abstract | The service implementation |

### The Instruction Sequence

```asm
; CALL(CONNECT(me, mymother))
; One fail-safe CLOOMC instruction sequence:

; Step 1: Load the RPC tunnel key (Literal GT) from our C-List
CAP.LOAD  CR0, CR6, 4      ; CR0 = TunnelKey_Mother (Literal GT, index 4)
                            ;   → mLoad validates: L perm on CR6, MAC, version
                            ;   → Result: CR0 holds the RPC tunnel key for mymother

; Step 2: Load the remote service GT from our C-List
CAP.LOAD  CR1, CR6, 6      ; CR1 = Mother_Service (Outform/Abstract GT, index 6)
                            ;   → mLoad validates: L perm on CR6, MAC, version
                            ;   → Result: CR1 holds GT pointing to mymother's service

; Step 3: Call the remote service — network-transparent RPC
CAP.CALL  CR1              ; CALL on Outform Abstract GT
                            ;   → E permission checked on CR1 → FAULT if denied
                            ;   → Data registers serialized as arguments
                            ;   → Payload encrypted using TunnelKey_Mother (CR0)
                            ;   → Sent to mymother via encrypted tunnel
                            ;   → mymother validates, executes, returns result
                            ;   → Response decrypted, data registers updated
                            ;   → RETURN resumes here
```

### What Happens Step by Step

1. **CAP.LOAD CR0, CR6, 4**: The Literal GT at C-List index 4 is loaded into CR0. This establishes the RPC tunnel key for communicating with "mymother". Object fetch/flush would use standard HTTPS and not need this key — only RPC requires the tunnel. mLoad validates L permission on CR6, checks MAC and version.

2. **CAP.LOAD CR1, CR6, 6**: The Outform Abstract GT at C-List index 6 is loaded into CR1. This is the handle to "mymother's" remote service. mLoad validates as usual.

3. **CAP.CALL CR1**: The CALL instruction detects that CR1 holds an Outform Abstract GT:
   - Checks E (Enter) permission — FAULT if missing
   - Looks up the Literal GT in CR0 as the tunnel key for this peer relationship
   - Serializes x0-x31 as the argument payload
   - Encrypts with the tunnel key
   - Sends to mymother's endpoint
   - mymother decrypts, validates the incoming GT through her own mLoad, executes her service, encrypts the result
   - "me" decrypts the response, updates data registers and condition flags
   - Execution continues after CALL as if it were a local function call

### Security Guarantees

Every step is fail-safe:

| Check | What Happens on Failure |
|-------|------------------------|
| L permission on C-List (Steps 1, 2) | FAULT: PERMISSION — cannot access C-List entries |
| MAC validation on any GT | FAULT: MAC — capability has been tampered with |
| Version mismatch | FAULT: VERSION — capability refers to recycled namespace entry |
| E permission on CALL (Step 3) | FAULT: PERMISSION — not authorized to invoke this service |
| Tunnel key revoked (GC swept) | FAULT: VERSION — tunnel key GT has been invalidated |
| Remote validation fails | Remote FAULT — mymother refuses the request |
| Man-in-the-middle attempt | Decryption fails — wrong key, message rejected |

### Revocation

To cut the connection between "me" and "mymother":

```
; Garbage collection sweeps TunnelKey_Mother (index 4)
; → Version bumped: old Literal GT in CR0 becomes invalid
; → Any subsequent CALL using this tunnel key → FAULT: VERSION
; → The tunnel is dead — no new communication possible
; → mymother's copy (TunnelKey_Child) is independently GC-managed
```

---

## Future Extensions (Trapped Today)

The following Outform operations are trapped today and will be implemented as software abstractions when the architecture matures:

### L on Outform — Remote Service Discovery

```
; Load a GT from mymother's published C-List
CAP.LOAD CR2, CR1, 3       ; CR1 = Outform C-List, index 3
                            ; Today: TRAP: OUTFORM_L
                            ; Future: fetch GT from remote C-List via tunnel
                            ;   → Enables browsing foreign namespace trees
                            ;   → "What services does mymother offer?"
```

### S on Outform — Remote Capability Delegation

```
; Save one of our GTs into mymother's C-List
CAP.SAVE CR2, CR1, 5       ; CR1 = Outform C-List, index 5
                            ; Today: TRAP: OUTFORM_S
                            ; Future: write GT to remote C-List via tunnel
                            ;   → Enables delegation across network boundaries
                            ;   → "Here, mymother, you can now access my data"
```

### X on Outform — Remote Code Execution

```
; Execute code at a remote location
; Today: TRAP: OUTFORM_X
; This is architecturally nonsensical — you cannot directly
; execute remote memory. Use E (Enter/RPC) instead.
```

These traps are **future-safe**: implementing the abstraction behind the trap requires no hardware change, no GT format change, and no permission bit change. The instruction set is frozen.

---

## Integration with Existing Web

Network transparency is designed to interwork with the existing web, not replace it:

| Capability Operation | Maps To |
|---------------------|---------|
| R on Outform (fetch) | HTTPS GET to URL (standard browser mechanism, TLS, ETag, Cache-Control) |
| W on Outform (flush) | HTTPS PUT/POST to URL (standard browser mechanism, ETag/If-Match for conflicts) |
| E on Outform Abstract (RPC) | Literal GT encrypted tunnel to remote Meta Machine endpoint |
| GC version bump | Cache invalidation (aligned with ETag/Last-Modified) |
| MAC validation | Object integrity check (independent of transport, enforced by mLoad) |

The key insight: object fetch and flush use **standard HTTPS** — the same mechanisms browsers have used for decades. The Meta Machine adds capability-based access control (R/W permission checks, MAC validation, version tracking) on top of what the web already provides. This means Outform objects can be fetched from any existing web server, CDN, or REST API without requiring the remote end to understand capabilities.

RPC (E on Outform Abstract) is the only operation that requires both endpoints to be Meta Machines, since it involves serializing capability-secured register state through a Literal GT encrypted tunnel.

This makes the architecture **media-tight** (content-type enforced), **data-tight** (R/W permissions enforced on every access), and **function-tight** (E permission required for every RPC invocation).
