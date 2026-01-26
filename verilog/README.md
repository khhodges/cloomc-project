# CTMM Verilog Hardware Implementation

This directory contains a synthesizable Verilog (SystemVerilog) implementation of the Church-Turing Meta-Machine (CTMM) capability-based architecture designed by Kenneth James Hamer-Hodges.

## Architecture Overview

The CTMM hardware implements failsafe security through Golden Tokens (64-bit capability keys) for all access control. The design follows these core principles:

1. **Capability-Based Security**: All access is mediated by Golden Tokens
2. **Hardware-Enforced Permissions**: Permission checks are performed in dedicated hardware
3. **Deterministic Garbage Collection**: The G bit enables hardware-assisted GC
4. **Failsafe Design**: Single FAULT output for all security violations

## File Structure

```
verilog/
├── ctmm_pkg.sv        # Package with types, constants, and definitions
├── ctmm_registers.sv  # Register file (CR0-CR15, DR0-DR15, flags)
├── ctmm_perm_check.sv # Permission checking unit
├── ctmm_gc_unit.sv    # Garbage collection unit with G bit
├── ctmm_decoder.sv    # Instruction decoder
├── ctmm_core.sv       # Top-level processor core
├── ctmm_tb.sv         # Testbench
└── README.md          # This file
```

## Golden Token (GT) Format

```
Bits [63:48] - Permissions (16 bits)
  Bit 0: R - Read
  Bit 1: W - Write
  Bit 2: X - Execute
  Bit 3: L - Load (capability from C-List)
  Bit 4: S - Save/Store (capability to C-List)
  Bit 5: E - Enter (call procedure)
  Bit 6: B - Bind (save to namespace DNA)
  Bit 7: M - Meta-Machine (hardware-level access)
  Bit 8: F - Far (remote URL location)
  Bit 9: G - Garbage (deterministic GC flag)

Bits [47:32] - Spare (reserved)
Bits [31:0]  - Offset (index into Namespace Table)
```

## Register Architecture

### Context Registers (Church)
- **CR0-CR7**: General purpose capability registers
- **CR6**: Current C-List
- **CR7**: Nucleus (kernel capability)
- **CR8**: Thread identity
- **CR15**: Namespace root

### Data Registers (Turing)
- **DR0-DR15**: 64-bit data registers

### Condition Flags
- N (Negative), Z (Zero), C (Carry), V (Overflow)

## Instruction Set

### Church Instructions (Capability Operations)
| Opcode | Mnemonic | Description |
|--------|----------|-------------|
| 000001 | LOAD     | Load capability from C-List |
| 000010 | SAVE     | Save capability to C-List |
| 000011 | CALL     | Call procedure via capability |
| 000100 | RETURN   | Return from procedure |
| 000101 | CHANGE   | Change thread identity |
| 000110 | SWITCH   | Switch namespace |
| 000111 | TPERM    | Test permissions |

### Turing Instructions (Data Operations)
| Opcode | Mnemonic | Description |
|--------|----------|-------------|
| 010000 | MOV      | Move data |
| 010001 | ADD      | Add |
| 010010 | SUB      | Subtract |
| 010011 | MUL      | Multiply |
| 010100 | DIV      | Divide |
| 010101 | AND      | Bitwise AND |
| 010110 | ORR      | Bitwise OR |
| 010111 | EOR      | Bitwise XOR |
| 011000 | LSL      | Logical Shift Left |
| 011001 | LSR      | Logical Shift Right |
| 011010 | ASR      | Arithmetic Shift Right |
| 011011 | CMP      | Compare |
| 011100 | TST      | Test bits |
| 100000 | B        | Branch |
| 100001 | BL       | Branch with Link |

## Boot Sequence

1. **FAULT_RST**: Clear all registers (cold restart)
2. **LOAD_NS**: Load namespace root into CR15 with M+L permissions
3. **INIT_THRD**: Initialize thread identity in CR8
4. **LOAD_NUC**: Load CR6 (Boot C-List) and CR7 (Nucleus)

## Garbage Collection

The G bit enables deterministic garbage collection:

1. **Mark Phase**: GC sets G=1 on all namespace entries
2. **Scan Phase**: Valid key access (via LOAD) resets G=0
3. **Sweep Phase**: Entries with G=1 are unreachable garbage

## Simulation

Using Icarus Verilog:
```bash
iverilog -g2012 -o ctmm_sim ctmm_pkg.sv ctmm_registers.sv ctmm_perm_check.sv ctmm_gc_unit.sv ctmm_decoder.sv ctmm_core.sv ctmm_tb.sv
vvp ctmm_sim
```

Using Verilator:
```bash
verilator --binary -j 0 --top-module ctmm_tb ctmm_pkg.sv ctmm_registers.sv ctmm_perm_check.sv ctmm_gc_unit.sv ctmm_decoder.sv ctmm_core.sv ctmm_tb.sv
./obj_dir/Vctmm_tb
```

## Synthesis

The design is synthesizable for FPGA or ASIC targets. Key synthesis considerations:

- Target clock: 100 MHz (adjustable based on target technology)
- Memory interfaces are external (instantiate appropriate memory IPs)
- Single-cycle execution for most instructions
- Multi-cycle for memory operations and GC

## Security Features

1. **Permission Validation**: All capability access requires permission checks
2. **Bounds Checking**: Access index validated against namespace entry limits
3. **MAC Validation**: Optional hardware MAC check for integrity
4. **Null Capability Detection**: NULL GT access causes FAULT
5. **Single FAULT Path**: All security violations use same FAULT mechanism

## License

Part of the CTMM Simulator project implementing Kenneth James Hamer-Hodges' capability-based architecture.
