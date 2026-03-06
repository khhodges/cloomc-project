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

- **Students:** Clone the repo, launch the simulator, write your first Church Machine program.
- **Teachers:** Use it in your classroom. The curriculum packages give you structured lesson plans.
- **Makers:** Grab a Tang Nano 20K and flash your own capability-secured processor.
- **Contributors:** PRs welcome. Help us build the future of computer science education.

---

*CLOOMC Technologies LLC — Making secure computing accessible to every learner.*
