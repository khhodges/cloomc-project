# Church-Turing Meta-Machine (CTMM) Simulator

## Overview
This project is a comprehensive simulator for the Church-Turing Meta-Machine (CTMM) capability-based architecture. It implements Kenneth James Hamer-Hodges' failsafe security design, utilizing "Golden Tokens" (64-bit capability keys) for all access control. The system integrates concepts from Church's lambda calculus and Turing's computational model to provide a robust and secure execution environment. The simulator aims to provide a deep understanding of capability-based security, secure system design, and the foundational principles of computation through an interactive web interface.

## User Preferences
- Tooltips positioned below for elements near top of viewport
- Minimal UI with consolidated header controls
- Auto-switching to Dashboard when Reset/Step/Run clicked
- Code persistence in localStorage for session continuity

## System Architecture

The CTMM simulator provides both a Haskell console interface and a primary web-based visualization. The web interface is built with a Python HTTP server, HTML for structure, CSS for styling, and JavaScript for core simulation logic and UI interactions.

### Core Architectural Concepts

-   **Capability-based Security**: All access control is managed via "Golden Tokens" (GTs).
-   **Register Architecture**:
    -   **Context Registers (CR0-CR7)**: Hold Golden Tokens for access rights. CR15 serves as the Namespace root, CR8 for Thread identity, CR7 for the Nucleus (kernel capability), and CR6 for the current C-List.
    -   **Data Registers (DR0-DR15)**: Hold 64-bit numeric values for computation.
-   **Golden Token Structure**: A 64-bit key composed of:
    -   Offset (0-31): Index into the Namespace Table.
    -   Permissions (32-47): R, W, X, L, S, E, B, M bits defining access rights (Read, Write, Execute, Load, Save, Enter, Bind, Meta-Machine). M distinguishes hardware-level (Namespace, Threads) from software-level permissions.
    -   Spare (48-63): Reserved.
-   **Namespace Entry**: A 3-word descriptor for each object:
    -   Word 1: Location (Physical Address or URL).
    -   Word 2: Limit (Object Size).
    -   Word 3: Seals (MetaData, Type, MAC).
-   **MAC Validation**: Hardware-enforced security check during `LOAD` operations, comparing a calculated hash of the GT offset and namespace entry words against the stored MAC.
-   **Boot Sequence**: A 4-step process (Fault Restart, Load Namespace, Initialize Thread, Load Nucleus) to securely initialize the CTMM.

### Web Interface (UI/UX)

The web interface is composed of five distinct views, accessible via a dropdown:

1.  **Dashboard**: Thread View displaying registers (Church CR0-CR15 and Turing DR0-DR15), condition flags, and visual boot sequence banner. The current thread is defined by CR8.
2.  **Namespace Browser**: Visual exploration of the capability namespace, displaying objects, C-List hierarchy, and providing tools to manage objects and state.
3.  **Assembly Editor**: A syntax-highlighted code editor for CTMM assembly with example programs and output tabs for console, Turing registers, and Church registers. Enforces security by restricting direct namespace access from code.
4.  **Capabilities Explorer**: Provides a detailed view of Golden Token structure, allowing interactive editing of bit fields and namespace entries, including live MAC validation.
5.  **Tutorial**: Interactive lessons explaining CTMM concepts, Golden Tokens, permissions, and guided examples.

### Key Features

-   **Built-in Abstractions**: Includes `Boot` (root namespace), `Threads` (user identities like Kenneth, Matthew, Daniel), `SlideRule` (IEEE 754 float operations), `Abacus` (64-bit integer operations), and `Circle` (geometric calculations).
-   **Instruction Set**: Comprehensive set of instructions covering Arithmetic, Logic, Shifts, Compare, Permission Test (`TPERM`), Branching, and Church-specific Capability Operations (`LOAD`, `SAVE`, `CALL`, `RETURN`, `CHANGE`, `SWITCH`).
-   **Condition Codes**: ARM-style condition flags (EQ, NE, CS, CC, MI, PL, VS, VC, HI, LS, GE, LT, GT, LE, AL) for conditional execution.
-   **Tooltip Help System**: All interactive elements include detailed hover tooltips for user guidance.
-   **State Persistence**: Automatically saves and restores the simulator's state (namespace, editor content) using local storage.
-   **Export/Import**: Functionality to export and import the complete simulator state as a JSON file.

## External Dependencies

-   **Python HTTP Server**: Used to serve the web interface files.
-   **Haskell GHC**: For the console-based simulator backend.
-   **`localStorage`**: Browser API used for client-side state persistence.

## Recent Changes

- 2026-01-16: Redesigned Capabilities Explorer with vertically-stacked GT and NMD layout (GT, W1, W2, W3 rows aligned)
- 2026-01-16: GT fields reordered to Little-Endian: Offset [0:31] left, Spare [48:63] center, Perms [32:47] right
- 2026-01-16: Hex values shown via Hex button popup (Big-Endian, MSB first)
- 2026-01-16: Added LE button below Hex for Little-Endian byte order display (ARM format, LSB first)
- 2026-01-16: Each row shows: Key | Hex/LE buttons | Fields with explanatory tooltips on all labels
- 2026-01-16: Removed yellow gradient from GT row, kept gold border for distinction
- 2026-01-16: MAC validation now shows as hover popup tooltip instead of separate section
- 2026-01-16: Added F (Far) permission bit - indicates remote URL location vs local memory address (bit 0x0100)
- 2026-01-16: W1/W2 display adapts based on F bit: shows "Location (URL)" when F set, "Location (Address)" when clear
- 2026-01-16: MAC recalculate button moved inline to right of MAC value
- 2026-01-16: Added explanatory tooltips for all namespace objects (Kenneth, Matthew, Daniel, Boot, SlideRule, Abacus, Circle)
- 2026-01-16: Added M (Meta-Machine) permission bit to distinguish hardware-level access (Namespace, Threads) from software-level permissions
- 2026-01-16: M is now the ONLY permission on Namespace (offset 0) and all Thread entries (Kenneth, Matthew, Daniel) - no other permissions when M is set
- 2026-01-16: Removed Context section from Capabilities Explorer - now only System and C-List views
- 2026-01-16: Removed Permission Reference section from Capabilities Explorer
- 2026-01-16: Clicking CR6 now populates C-List section with all 7 GTs from the Boot C-List
- 2026-01-16: Fixed header/boot sequence overlap - increased main margin-top to 60px
- 2026-01-16: Header tooltips now show below (tooltip-bottom class) to be visible
- 2026-01-16: Back button simplified: removed arrow, just says "Back", has tooltip
- 2026-01-16: Made CR15, CR8, CR6 clickable: CR15→Namespace, CR8→Dashboard, CR6→Capabilities
- 2026-01-16: Added Save button to Assembly Editor toolbar to save code to localStorage
- 2026-01-16: Added Back button to left of view buttons for navigating to previous views
- 2026-01-16: View history tracking stores up to 20 previous views for back navigation
- 2026-01-16: View buttons (Dashboard, Namespace, etc.) properly centered using flex layout
- 2026-01-16: Namespace restructured with proper offsets: 0=Namespace (self-ref), 1=Boot C-List, 2=Kenneth, 3=Access code, etc.
- 2026-01-16: Boot C-List now contains GT entries with nsOffset pointers to namespace entries
- 2026-01-16: Boot Step 3 loads CR6 from NS offset 1 (Boot C-List), CR8 from offset 2 (Kenneth), shows [n] count
- 2026-01-16: Boot Step 4 loads CR7 from C-List[0] pointing to NS offset 3 (Boot/Access.asm)
- 2026-01-16: Namespace Browser now displays table format with Offset, Name, Type, Word1, Word2, Perms columns
- 2026-01-16: View button highlighting syncs with active view (Assembly button highlights when editor open)
- 2026-01-16: Editor content cleared and localStorage updated on Fault Restart
- 2026-01-16: Removed Reset button - "Fault Restart" (boot step 1) now handles state save and reset
- 2026-01-16: View selector changed from dropdown to horizontal row of buttons
- 2026-01-16: Simplified Dashboard - removed Command Input and Output Log, now pure Thread View
- 2026-01-17: Increased left column width in word rows from 60px to 80px for better label visibility
- 2026-01-17: Permission hex value now right-justified on header row using flex layout (field-label-row class)
- 2026-01-17: Clicking GT tokens in left panel updates right detail panel with hierarchy path and register assignments
- 2026-01-17: Added capability info bar showing: Path (hierarchy from Namespace root) and Loaded (register assignments)
- 2026-01-17: Hierarchy path shows clickable items with arrows (Namespace → Boot → Object), current item highlighted in blue
- 2026-01-17: Register assignments shown as green badges (CR15, CR8, CR6, etc.) or "Not loaded" if unassigned
- 2026-01-17: C-List entries now show descriptive type labels ([0] Code, [1] Thread, [2] Abstraction) instead of generic indices
- 2026-01-17: Removed h2 title from capability detail panel - hierarchy path is now the title
- 2026-01-17: Current capability name highlighted in blue in hierarchy path (larger font, bold)
- 2026-01-17: Changed "Loaded" to "Locked/Unlocked" status indicator with lock icons (🔒/🔓)
- 2026-01-17: Lock status now based on register assignment: loaded in register = Unlocked, not loaded = Locked
- 2026-01-17: Updated lock tooltips: Locked = "Navigate to the C-List parent and perform Load GT to unlock access rights", Unlocked = "Unlocked for use as Permissions allow"
- 2026-01-17: Replaced CSS tooltips with JavaScript floating tooltip system to avoid clipping in scrollable containers
- 2026-01-17: Renamed "System" section in Capabilities Explorer to "Context Register"
- 2026-01-17: Added 16 clickable CR buttons (CR0-CR15) in 4x4 grid layout with special labels (NS, TH, NU, CL)
- 2026-01-17: CR buttons show green highlight when loaded with GT, dimmed when empty
- 2026-01-17: Clicking loaded CR button shows GT details in right panel with proper hierarchy path
- 2026-01-17: Removed two-character labels (NS, TH, NU, CL) from CR buttons - now just numbers
- 2026-01-17: CR buttons now color-coded by type: brown=Namespace, purple=Thread, blue=C-List, orange=Code, green=Abstraction
- 2026-01-17: CR button grid now 8 per row (2 rows total) with smaller buttons
- 2026-01-17: Threads (Kenneth, Matthew, Daniel) shown directly under Namespace in hierarchy, not under Boot
- 2026-01-17: Removed dropdown and "Load Samples" button from Capabilities Explorer
- 2026-01-17: Empty CR buttons now clickable - shows NULL status with W1-W3 hidden and helpful message
- 2026-01-17: Added Add (+) button in C-List header to create new capabilities with modal form
- 2026-01-17: Added Delete (X) button on C-List items with impact analysis warning popup
- 2026-01-17: Description field required for new capabilities, shown as tooltip on hover
- 2026-01-17: Reduced modal padding/gaps for compact display ensuring Create button visibility
- 2026-01-17: Updated Tutorial with new "Golden Token Structure" lesson covering GT bit layout and 3-word Namespace Entry
- 2026-01-17: Tutorial now explains 9 permissions including M (Meta-Machine) and F (Far)
- 2026-01-17: Tutorial covers MAC validation, W3 Seals structure (Meta/Type/MAC), and Little-Endian format
- 2026-01-18: MINT operation now creates GT in next free Namespace slot and returns in CR0
- 2026-01-18: MINT allocates namespace offset, creates 3-word entry (W1, W2, W3), generates GT with source permissions
- 2026-01-18: New GT automatically linked to running Thread through CR0
- 2026-01-18: MINT modal shows explanation of operation steps (no C-List option - use SAVE programmatically)
- 2026-01-18: Renamed "Link" to "MINT" in toolbar and context menu with 🔑 icon
- 2026-01-18: GT in CR0 released on RETURN - must use SAVE instruction to persist to C-List
- 2026-01-18: Delete confirmation popup now shows comprehensive impact analysis with severity levels
- 2026-01-18: Delete warnings include: critical (cannot delete), danger (significant impact), warning (minor impact), safe
- 2026-01-18: Delete checks for register assignments (CR0-CR15), child objects, C-List references, and type-specific impacts
- 2026-01-18: MAJOR REFACTOR: Namespace objects now contain only raw 3-word entries (W1, W2, W3) without embedded permissions
- 2026-01-18: Boot C-List is now the AUTHORITATIVE source for all GT definitions (offset, permissions, metadata)
- 2026-01-18: Namespace offsets reorganized: 0=Namespace, 1=Access.asm, 2=Boot C-List, 3=Kenneth, 4=Matthew, 5=Daniel, 6-8=Abstractions
- 2026-01-18: Boot sequence refactored: Step 3a uses CHANGE for Thread (first), Step 3b uses CALL HWGT to load CR6+CR7+IP=0
- 2026-01-18: Added coldRestart flag to bypass SAVE operations on cold restart (prevents double-fault)
- 2026-01-18: Helper functions added: getBootGT(), getNSEntry(), buildGTFromCList() for GT lookups through C-List
- 2026-01-18: UI now resolves permissions through Boot C-List lookups instead of namespace object properties
- 2026-01-18: Added Instructions view with Church/Turing tabs showing all instruction sets with hover popup summaries
- 2026-01-18: Church instructions: LOAD, SAVE, CALL, RETURN, CHANGE, SWITCH, MINT, TPERM with detailed tooltips
- 2026-01-18: Turing instructions: Arithmetic, Logic, Shifts, Compare, Branch, Condition codes with tooltips
- 2026-01-18: Permission validation rules implemented: M clears software perms, Data/Capability mutually exclusive
- 2026-01-18: Permission categories defined: Data (R,W,X), Capability (E,B), Protected (L,S internal only), Meta (M)
- 2026-01-18: Boot C-List corrected: Access=X only, Threads=M only, Abstractions=E,B only
- 2026-01-18: Added getSafePerms() and formatPerms() helper functions for safe permission access/display
- 2026-01-18: Permission normalization added to ALL mutation paths: confirmObjectModal, confirmLinkModal, loadFromStorage, importNamespaceState, createObject, updateObject, addToCList