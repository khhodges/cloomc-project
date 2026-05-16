# CLOOMC — The Church Machine Educational Platform

> A capability-secured processor architecture with an educational IDE,
> designed for the Tang Nano 20K FPGA. Making computer architecture
> and secure programming accessible through hands-on learning.

**Website:** [cloomc.org](https://cloomc.org) | **Commercial & Curriculum Info:** [cloomc.com](https://cloomc.com)

---

## Licensing

### 1. Free for Everyone to Learn

The CLOOMC platform is **free and open source** under GPL-3.0. This includes the full IDE,
simulator, CLOOMC++ compiler, shared abstraction library, and CTMM hardware designs.

**Who can use it for free:**
- Students of all ages — primary school through postgraduate
- Parents and families, including homeschool
- Teachers and educators
- Schools (K-12), IB programmes, and universities
- Non-profit academic researchers

Clone it. Use it. Teach with it. Build student projects. No cost, no restrictions
beyond the GPL-3.0 license terms.

```bash
git clone https://github.com/khhodges/cloomc-project.git
cd cloomc-project/simulator
# Open index.html in a browser — that's it
```

### 2. Curriculum Packages (Paid Add-Ons)

Structured courseware aligned to specific examination boards is available as
paid add-on services. These curriculum packages build on the free platform:

- **UK O-Levels** — Computer Science
- **UK A-Levels** — Computer Science
- **International Baccalaureate (IB)** — Computer Science HL/SL
- **11+ Entrance Exams** — Logic and computational thinking
- **GCSE** — Computer Science
- **AP Computer Science** — Principles and A

These packages include lesson plans, exercises, mark schemes, and exam-style
questions mapped to the Church Machine architecture. Visit [cloomc.com](https://cloomc.com)
for details and availability.

### 3. Commercial License

Any use of the platform technology in commercial products, proprietary hardware,
paid services, or for-profit systems requires a separate commercial license from
CLOOMC Technologies LLC.

**Contact:** SIPanticINC@gmail.com

---

## What's Inside

| Directory | Contents |
|-----------|----------|
| `simulator/` | Web-based IDE — editor, compiler, pipeline viewer, math tools |
| `library/` | Shared abstraction library (Mum Tunnel) |
| `hardware/` | Amaranth HDL designs for Tang Nano 20K FPGA |

### The Church Machine

A 32-bit capability-secured processor with:
- **20 instructions** — 10 Church (capability) + 10 Turing (data)
- **Golden Tokens** — 32-bit unforgeable capability tokens with R/W/X/L/S/E permission bits
- **9 abstraction layers** — 45 abstractions, each a security block with MTBF tracking
- **Domain purity** — strict separation between capabilities and code/data

### The IDE

Nine integrated views: Math, Code, Tutorial, Dashboard, Namespace, Abstractions,
Pipeline, Reference, and Docs. Includes:

- **CLOOMC++ Compiler** — Multi-language: English, JavaScript, Haskell, Symbolic Math (Ada), Assembly
- **Interactive Math Tools** — HP-35 calculator, soroban abacus, slide rule
- **Math Challenge** — Grade-adaptive problems with Turing/Church dual explanations
- **WebSerial Deploy** — Flash compiled programs to the Tang Nano 20K FPGA
- **Mum Tunnel Library** — Share and discover abstractions via GitHub

### Hardware Target

**Tang Nano 20K** (Gowin GW2AR-18) with all features enabled:
CHANGE/SWITCH, SEAL_CHECK, FUSED_OPS, GC.

---

## Manifesto

Every child deserves to understand how computers actually work — not just how to
use them, but how they think. The Church Machine bridges the gap between theoretical
computer science and hands-on engineering. Named for Alonzo Church, whose lambda
calculus gave us the mathematical foundation of computation, this platform makes
capability-based security and processor architecture tangible.

We believe:
- **Security should be built in**, not bolted on. Golden Tokens make capability-based
  security a first-class concept from day one.
- **Theory and practice belong together.** Students write real code that runs on real
  hardware, seeing Church's lambda calculus and Turing's state machines working side by side.
- **Education should be free.** The platform is open source. Every student, everywhere,
  can learn.

## Call to Action

# CLOOMC: The Church Machine Reference Implementation

[![License](https://img.shields.io/badge/License-Source--Available-blue.svg)](LICENSE.md)
[![Status](https://img.shields.io/badge/Status-Beta--Simulator-orange.svg)]()

Welcome to the official repository for **CLOOMC** (`cloomc.org` / `cloomc.com`). This project delivers a clean-slate computing paradigm that replaces the fundamentally insecure, ambient-authority architecture of traditional von Neumann machines with a hardware-enforced **Capability Crust** derived from the Lambda Calculus.

By mechanizing mathematical reduction directly into the hardware substrate, CLOOMC completely isolates and tames the "Molten Core" of non-deterministic, AI-driven applications and autonomous agents.

---

## The Core Paradigm: Mathematical Containment

Traditional security paradigms rely on a reactionary "patch-and-pray" methodology—deploying software guardrails, regex filters, or hardware memory tagging to catch exploits after they have already bypassed system boundaries. 

CLOOMC changes the underlying geometry of cyberspace:
1. **No Ambient Authority:** A process owns absolutely zero implicit privileges. If a resource is not explicitly present in the active context's private namespace, it does not exist to that process.
2. **Immutable Golden Tokens:** Capabilities are unforgeable primitives managed natively at the microcode level. They cannot be derived, guessed, or synthesized from raw data inputs.
3. **Deterministic State Transitions:** The machine state changes exclusively through six atomic, invariant operations.

### The 6 Church Instructions
* **`LOAD`**: Unlocks an immutable Key from the current context register into an execution slot, executing all boundary and authorization checks.
* **`SAVE`**: Stores a valid Key into a C-List (Capability List), updating all native resource tracking and reference counts.
* **`CALL`**: Securely pushes the active context onto a hardware LIFO thread stack and transfers execution to an isolated target namespace[cite: 4].
* **`RETURN`**: Pops the previous context from the LIFO stack, instantly restoring the thread to its exact, untampered prior state[cite: 4].
* **`CHANGE`**: Atomically saves the comprehensive machine state of the active thread and context-switches to a separate Thread environment[cite: 4].
* **`SWITCH`**: Dynamically alters the accessible Namespace entirely, executing all structural validation filters and permission counts[cite: 4].

---

## Quickstart: Running the Architecture Simulator

Before our physical FPGA hardware implementation launches next month, you can explore, test, and attempt to exploit the CLOOMC architecture using our interactive software simulator[cite: 4].

### Prerequisites
* Python 3.8+

### Installation & Execution
Clone the repository and run the simulation gauntlet to see how the 6 Church Instructions natively neutralize memory injection and privilege escalation[cite: 4]:

```bash
git clone [https://github.com/cloomc/church-machine.git](https://github.com/cloomc/church-machine.git)
cd church-machine
python church_simulator.py

- **Students:** Clone the repo, launch the simulator, write your first Church Machine program.
- **Teachers:** Use it in your classroom. The curriculum packages give you structured lesson plans.
- **Makers:** Grab a Tang Nano 20K and flash your own capability-secured processor.
- **Contributors:** PRs welcome. Help us build the future of computer science education.

---

*CLOOMC Technologies LLC — Making secure computing accessible to every learner.*
