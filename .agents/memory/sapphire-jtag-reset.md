---
name: Sapphire SoC jtagCtrl_reset polarity
description: VexRiscv Sapphire SoC jtagCtrl_reset is active-LOW — tying to 0 freezes io_systemReset HIGH
---

# Sapphire SoC `jtagCtrl_reset` polarity

## The rule
`jtagCtrl_reset` must be tied to `1'b1` (not `1'b0`) when JTAG debug is disabled.

In top.v:
```verilog
.jtagCtrl_enable  (1'b0),
.jtagCtrl_reset   (1'b1),   // 1 = TAP not in reset
```

## Why
VexRiscv Sapphire SoC treats `jtagCtrl_reset = 0` as "JTAG TAP held in reset". The debug clock domain reset propagates into `io_systemReset`, keeping it HIGH permanently. The 6-bit system reset counter (`systemCd_logic_holdingLogic_resetCounter`) never completes because the debug domain continually re-asserts reset via `system_cores_0_logic_cpu_debug_resetOut`.

**Symptom**: LED0 (`~system_reset`) stays OFF forever. CM runs fine (LEDs 2/3 active) because church_ti60f225 doesn't share the Sapphire reset domain. UART silent.

## How to apply
Any time `sapphire` is instantiated with JTAG tied off — always use `1'b1` for `jtagCtrl_reset`. This applies even if `jtagCtrl_enable = 0` (disabled debug still needs the TAP out of reset to release the reset chain).
