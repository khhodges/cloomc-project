---
name: Ti60 SoC UART clockDivider
description: Sapphire SoC UART register reset value and required firmware initialization for 115200 baud on Ti60F225
---

## The Rule

The Sapphire SoC UART `clockDivider` register resets to **0x00** on power-up.
Firmware **must** write `UART_CLOCKDIV = 53` before the first `uart_puts()` / `uart_putc()` call.

## Register address

```c
#define UART_CLOCKDIV  (*(volatile uint32_t *)(0xF8010000UL + 0x08))
```

Offset +0x08 from UART_BASE (0xF8010000).

## Baud rate formula

```
baudRate = ClkIn / (8 × (clockDivider + 1))
```

For 50 MHz crystal → 115200 baud:
```
clockDivider = (50_000_000 / (8 × 115_200)) − 1 = 53.25 → 53
actual baud  = 50_000_000 / (8 × 54) = 115_740  (0.47% error — fine)
```

## Why this matters

With `clockDivider = 0`, the UART runs at 50 MHz / 8 = **6.25 Mbaud**.
Every byte is transmitted in ~1.6 µs. Any serial terminal running at ≤ 3 Mbaud sees
complete silence (framing errors on every character, nothing decoded). This bug
caused the entire Ti60 UART commissioning to appear as "no output at any baud rate".

**Why:** The Sapphire SoC IP generator comment in sapphire.v was misread as
"clockDivider resets to 0x35" but the Verilog RTL default for the register is 0x00.
The comment was in the firmware source and was wrong. Always set explicitly.

## How to apply

In `main()`, as the very first statement before any UART output:

```c
UART_CLOCKDIV = 53;   /* 50 MHz → 115200 baud; must be first */
```

After changing firmware, regenerate symbol bins and re-synthesise:
```bash
# On Penguin, from ~/church_project/SoC/
cd firmware && make && cd ..
python3 scripts/gen_sapphire_symbol_bins.py firmware/firmware.bin --out-dir .
python3 scripts/patch_sapphire_init.py sapphire.v \
    EfxSapphireSoc.v_toplevel_system_ramA_logic_ram_symbol{0..3}.bin
# then re-synthesise with 2025.2 efx_map → 2026.1 pnr → pgm → flash
```
