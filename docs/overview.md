# CTMM Project Overview

## What Is This Project?

This project implements simulators for Kenneth James Hamer-Hodges' **Church-Turing Meta-Machine (CTMM)** capability-based security architecture. The CTMM is a hardware architecture that enforces failsafe security through **Golden Tokens** -- unforgeable capability keys that mediate all access to system resources. Named after Alonzo Church and Alan Turing, the architecture integrates lambda calculus principles (controlled access through abstraction) with Turing's computational model (data processing and execution).

The project contains two independent simulators that share the same foundational security philosophy but differ in instruction set architecture, token width, and implementation details.

---

## The Two Simulators

### Sim-64 (CTMM)

Located in the `web/` directory, Sim-64 is the original CTMM simulator. It uses a custom ARM-style instruction set with 64-bit Golden Tokens. The web interface provides seven views: Dashboard, Namespace Browser, Assembly Editor, Capabilities Explorer, Instructions, Tutorial, and Code Browser. Hardware implementations exist in SystemVerilog (`verilog/`) and Amaranth HDL (`ctmm_amaranth/`).

### Sim-32 (RV32-Cap)

Located in the `riscv_cap/` directory, Sim-32 extends the standard RISC-V RV32I instruction set with capability-based security. It uses 32-bit Golden Tokens with explicit Version and Type fields. The web interface provides five views: Dashboard, Namespace Browser, Assembly Editor, Capabilities Explorer, and Instructions.

---

## Shared Architectural Principles

Both simulators enforce the same core security model:

- **Failsafe Security**: Every validation failure routes to a single FAULT handler. There are no silent failures or undefined behaviors.
- **Golden Tokens**: All access rights are embodied in unforgeable capability keys. No raw memory addressing is permitted.
- **Capability Registers (CR0-CR15)**: 16 capability registers hold Golden Tokens. CR0-CR7 are instruction-addressable via 3-bit encoding. CR8-CR15 are system registers, protected from direct instruction access.
- **Permission Domains**: Four mutually exclusive domains govern what operations a token authorizes:
  - **Church (L, S)**: Load and Save capabilities
  - **Turing (R, W, X)**: Read, Write, and Execute data
  - **Lambda (E)**: Enter an abstraction
  - **Meta (B, M, F, G)**: Bind, Machine, Foreign, Garbage collection
- **C-List Mediation**: LOAD and SAVE operations go through capability-mediated C-Lists, never through raw memory addresses.
- **SWITCH as Privilege Gate**: The only way to write to system registers CR8-CR15 is through the SWITCH instruction.

---

## Quick Comparison

| Feature | Sim-64 (CTMM) | Sim-32 (RV32-Cap) |
|---------|---------------|-------------------|
| **Directory** | `web/` | `riscv_cap/` |
| **Golden Token Width** | 64-bit | 32-bit |
| **GT Format** | Offset + Permissions + Spare | Version(5) + Index(15) + Permissions(10) + Type(2) |
| **Base ISA** | Custom ARM-style encoding | RISC-V RV32I |
| **Data Registers** | DR0-DR15 (64-bit each) | x0-x31 (32-bit each) |
| **Capability Registers** | CR0-CR15 (64-bit GTs) | CR0-CR15 (128-bit, 4x32-bit words) |
| **Church Instructions** | 11 (LOAD, SAVE, LOADX, SAVEX, LDM, STM, CALL, RETURN, CHANGE, SWITCH, TPERM) | 6 (CAP.LOAD, CAP.SAVE, CAP.CALL, CAP.RETURN, CAP.CHANGE, CAP.SWITCH) |
| **Condition Codes** | ARM-style (N, Z, C, V) on all instructions | None (RISC-V uses explicit branches) |
| **Namespace Entries** | 3 words (Location, Limit, Seals) | 3x32-bit words (Location, Limit, VersionSeals) |
| **Max Namespace Entries** | Offset-dependent | 32,768 (15-bit index) |
| **GT Version Field** | None | 5-bit version tag |
| **GT Type Field** | None (implicit) | 2-bit: Inform, Outform, Literal, Abstract |
| **MAC Validation** | Hardware-enforced hash | 27-bit FNV seal in VersionSeals |
| **Garbage Collection** | G-bit cleared on LOAD access | Separate Mark-Scan-Sweep cycle with version bump |
| **Web Views** | 7 | 5 |
| **Hardware Implementations** | SystemVerilog + Amaranth HDL | Software simulation only |

---

## Directory Structure

```
/
+-- web/                    Sim-64 (CTMM) web simulator
|   +-- index.html          Single-page application
|   +-- simulator.js        Core simulation engine
|   +-- app.js              UI controller
|   +-- styles.css           Dark-themed styling
|   +-- app.py / server.py  Python HTTP server
|   +-- images/             UI assets
|
+-- riscv_cap/              Sim-32 (RV32-Cap) web simulator
|   +-- index.html          Single-page application
|   +-- simulator.js        Core simulation engine (RV32I + Church)
|   +-- assembler.js        Two-pass assembler
|   +-- app.js              UI controller
|   +-- styles.css           Dark-themed styling
|   +-- main.py             Flask web server (port 5000)
|
+-- verilog/                SystemVerilog hardware implementation (Sim-64)
+-- ctmm_amaranth/          Amaranth HDL hardware implementation (Sim-64)
+-- CTMM/                   Haskell console simulator (Sim-64)
+-- docs/                   Project documentation
```

---

## Independence

The two simulators are fully independent codebases. Changes to one never affect the other. Only one web simulator can be active on port 5000 at a time.
