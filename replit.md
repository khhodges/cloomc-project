# Church Machine Educational Platform

## Overview

The Church Machine is a capability-secured processor architecture with an educational IDE, designed for the Tang Nano 20K FPGA and supporting the Efinix Ti60 F225. Its primary purpose is to provide a web-based integrated development environment for children to write, compile, test, and deploy Church Machine programs. The platform uses capability-based security with Golden Tokens, making secure programming and computer architecture accessible through hands-on learning for various educational settings.

## User Preferences

- Church Gold dark theme
- Mobile-responsive for parent mode on handsets
- All feature flags True for Tang build
- No separate dynamicObjects — all entries in namespaceObjects
- B (Bind) bit defaults to 0, auto-cleared by CALL
- C-Lists only have E permission, CLOOMC only X or RX
- Phase 1 + 1b + 1c + 1d + 1e: English, JS, Haskell, Symbolic Math (Ada), and Lambda Calculus front-ends implemented; auto-detected by compiler
- Pure Math "Compile Session" button: compiles interactive let-bindings to Church Machine code via symbolic math front-end

## System Architecture

The system comprises hardware (Amaranth HDL for FPGA), a web IDE (HTML/JS/CSS), and a Flask backend.

**UI/UX Decisions:**
The web IDE offers ten views (Math, Code, Tutorial, Dashboard, Namespace, Abstractions, Pipeline, Reference, Builder, Docs) with interactive learning tools:
- **Educational Tools:** Includes a Pure Math calculator with a "Compile Session" for converting let-bindings, an HP-35 Calculator (lambda calculus based), a soroban-style Abacus, and a logarithmic Slide Rule, all featuring Church Machine trace.
- **Learning Aids:** A "Math Challenge" sidebar provides grade-appropriate problems, "History Tab" offers contextual stories, and "Syntax Tab" provides language-specific cheat sheets.
- **Builder:** A Visual Namespace Builder for designing deployment topology, allowing drag-and-drop node positioning and dependency wiring.
- **Documentation:** The "Docs" view structures all documents as chapters of "The Church Machine" book, with Prologue and Parts I–X covering architecture, security, runtime, and other key areas.
- **Interactive Elements:** Features educational popups for guides and a global CSS tooltip system for consistent user feedback.
- **Responsive Design:** Panels use draggable dividers and adapt to narrow screens, with tab bars collapsing into a hamburger menu.
- **Tutorials:** Two interactive slide-based tutorials cover architecture, ISA, compiler pipeline, and multi-language abstraction implementations.
- **State Persistence:** Utilizes localStorage for editor state, settings, and progress.

**Technical Implementations:**
- **Abstraction Model:** A scale-free model where 45 abstractions across 9 layers function as security blocks.
- **Security Model:** Based on 32-bit unforgeable Golden Tokens with specific permission bits and a 7-step mLoad pipeline for capability handling. Domain purity strictly separates capabilities and code/data.
- **Compiler:** The CLOOMC++ Compiler is a multi-language compiler targeting the 20-instruction Church Machine instruction set, supporting English, JavaScript, Haskell, Symbolic Math, and Lambda Calculus. It auto-detects language and outputs `upload.json` for abstraction deployment.
- **Lump Header Format (hardware-accurate):** Every abstraction slot starts with a 32-bit header at word 0: `magic(5)|n_minus_6(4)|cw(13)|typ(2)|cc(8)`. Boot.Abstr header = `0xF800_440C` (64w slot, cw=17, cc=12). C-list is at the physical end of the slot (words 52–63), freespace fills words 18–51. Both the simulator BOOT_NUC step and `_execCall` derive CR14 (X capability, lim=50) and CR6 (L capability, base=loc+52, lim=11) simultaneously from this single header word, matching FPGA behaviour. `_fetchInstruction` applies +1 offset so PC=0 fetches the first instruction after the header.
- **Locator (Absent-Lump Protocol):** Handles on-demand lump loading triggered by `LOAD` instructions, fetching, verifying, and validating lumps securely.
- **Navana Master Controller:** Manages Namespace entries, abstraction creation, allocation, and secure deployment, acting as the sole writer and enforcing security constraints with PassKey access control for device drivers.
- **Instruction Set:** Comprises 20 instructions, evenly split between Church (capability-focused) and Turing (data manipulation) sets, supporting ARM-style conditional execution.
- **Hardware Target:** Primarily the Tang Nano 20K FPGA, with optional Efinix Ti60 F225 support, enabling features like CHANGE/SWITCH, SEAL_CHECK, FUSED_OPS, and GC.
- **WebSerial:** Used for deploying compiled programs to the Tang Nano 20K FPGA.
- **Mum Tunnel Library:** A GitHub-backed shared abstraction library with API endpoints for browsing, retrieving, and publishing abstractions.
- **Self-Documenting Abstractions:** Each `upload.json` includes a `doc` block with metadata, auto-generated from compiler output.

## External Dependencies

- **Python/Flask:** Backend web server.
- **SQLite:** Local database (`server/church_machine.db`).
- **Amaranth HDL:** Hardware synthesis for FPGA design.
- **localStorage:** Client-side state persistence.
- **oss-cad-suite:** FPGA toolchain (`yosys`, `nextpnr-gowin`, `gowin_pack`, `openFPGALoader`).
- **GitHub:** Integrated for the Mum Tunnel shared abstraction library and project management.