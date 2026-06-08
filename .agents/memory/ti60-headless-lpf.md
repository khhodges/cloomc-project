---
name: Ti60 headless build — IO placement and LPF
description: Complete working headless build flow for Ti60F225 on Chromebook Penguin; IO pin placement pitfalls; Patch4 must call not bypass check_design
---

# Ti60F225 Headless Build — IO Placement and LPF

## IO pins randomly placed root cause
All IO cells show "no assigned placement; will be placed randomly" when:
1. peri.xml is missing the `clk` GPIO entry (`gpio_def="GPIOL_P_18"`) — peri.xml starts with only 7 GPIOs, `clk` must be added before the `cm_uart_tx` entry
2. Python heredoc writes (`python3 - <<'EOF'`) silently fail to write the file on this Penguin setup — use `cat > /tmp/fix.py << 'EOF'` then `python3 /tmp/fix.py` instead
3. Patch 4 bypasses `check_design()` entirely — IO config state never populated, LPF has `comp_gpio` copied from peri.xml but no HSIO instance configurations

## Correct Patch 4 (design.py)
Do NOT use `if True:` — call `check_design()` but ignore its return:
```python
try:
    self.check_design()  # populate IO config state
except Exception:
    print("WARNING: check_design raised (headless patch)")
if True:  # always generate constraint
    try:
        self.__gen_report(outdir)
    except Exception:
        print("WARNING: report generation skipped (headless patch)")
    self.__gen_constraint(enable_bitstream, outdir)
```

## Use efx_run wrappers, not bare efx_pnr/efx_pgm
- `efx_run church_soc_cm --prj --flow pnr --family Titanium --device Ti60F225` (NOT bare efx_pnr)
- `efx_run church_soc_cm --prj --flow pgm --family Titanium --device Ti60F225` (NOT efx_pgm --source)

The bare tools need extra flags and don't automatically apply the LPF for IO placement.

## Confirming IO placement worked
After PnR: `grep -c "random placement" work_pnr/pnr.log` should return 0.
If still random: check peri.xml has all 8 GPIOs (clk included), re-run Interface Designer.

## clk not in LPF grep
`grep -c "GPIOL_P_18" outflow/church_soc_cm.lpf` returns 0 — the LPF uses HSIO instance IDs (GPIOL_PN_18), not the peri.xml resource name. The clk IS constrained; grep for "clk" or line count instead.

## UART device mapping on Penguin (FT4232H)
`/dev/ttyUSB2` does not exist — USB devices enumerated as ttyUSB0, ttyUSB1, ttyUSB3, ttyUSB4.
SoC UART (FT4232H interface B) is likely ttyUSB1 or ttyUSB3 at 57600 baud.

**Why:** Efinix 2026.1 headless IO placement requires check_design() side effects. The `if True:` Patch4 shortcut produces a structurally incomplete LPF.
