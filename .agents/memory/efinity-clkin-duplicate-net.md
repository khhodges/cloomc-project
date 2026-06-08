---
name: Ti60 clk peri.xml config + the "1 unassigned core pin" phantom
description: The authoritative clk-input config for church_soc_cm.peri.xml, and why a route.rpt.xml "Unassigned Core Pins=1" can be a stale-file phantom
---

# Authoritative clk config (BUILD_SOC_CM.md wins)

`hardware/soc_combined/BUILD_SOC_CM.md` is the tested build guide. For the
external clock on `GPIOL_P_18`, the **corrected** `church_soc_cm.peri.xml` is:

- clk `comp_gpio` `input_config`: `conn_type="normal"` (NOT `"clkin"`), `clkmux_buf_name=""` (NOT `"CLKMUX_L"`)
- ALL `clkmux` ROUTE0 pins: `name=""` (no pin named `clk`)
- `mode="input"`, `gpio_def="GPIOL_P_18"`, `io_standard="1.8 V LVCMOS"`
- peri.xml header MUST be `version="2025.2.288.4.15"` `db_version="20252999"` and declare all 12 `<efxpt:iobank>` — wrong version / missing banks → efx_pnr packing aborts with "Failed to read core interface constraints file".

**Why:** `conn_type="normal"` + empty clkmux avoids the PLL-writer assertion and
the "duplicate pin name" error. The repo file is served to the Penguin via the
IDE route `/dl/peri-xml`, so fixing the repo file + restarting the IDE is the
transfer path (no git on the Penguin).

# The "1 unassigned core pin" was a PHANTOM

A previous session burned hours chasing `Unassigned Core Pins severity="error"
value="1"` in `outflow/<proj>.route.rpt.xml` and "fixed" it by setting the clk
to `conn_type="clkin"` + clkmux ROUTE0 `name="clk"`. That report was **STALE** —
left over from an earlier failed run; PnR never actually reached placement.

**How to apply:** before trusting ANY count in `route.rpt.xml`, check its mtime
vs the current run. If PnR died early (e.g. packing "Failed to read core
interface constraints file"), the route report is from a prior run — ignore it.
Do NOT re-introduce `conn_type="clkin"`/`CLKMUX_L`; that is the disproven fix and
it re-breaks the doc-blessed config.
