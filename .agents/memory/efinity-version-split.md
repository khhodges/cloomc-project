---
name: Efinity version split for Ti60 SoC build — CONFLICTED, verify via BRAM
description: In-repo wrappers/doc say efx_map=2025.2, efx_pnr/efx_run=2026.1; a prior memory claimed all-2026.1. Unresolved — verify by checking BRAM INIT_0, not by trusting a version.
---

# Conflicting claims — do not trust blindly

| Source | efx_map (synth) | efx_pnr / efx_run (pnr, interface, pgm) |
|---|---|---|
| `run_efx_map.sh` / `run_efx_pnr.sh` / BUILD_SOC_CM.md (in repo) | **2025.2** ("2026.1 segfaults on efx_map") | **2026.1** ("2025.2 segfaults on efx_pnr") |
| Older memory note (this file, prior rev) | 2026.1 ("2025.2 zeros BRAM") | 2026.1 |

These are opposites and were **not** re-verified in the session that wrote this.
The in-repo wrappers + BUILD_SOC_CM.md are the authoritative, dated sources — prefer
them — but treat the version as unproven until the BRAM check below passes.

# The real test is BRAM embedding, not the version number

Whatever efx_map version is used, the only thing that matters is that firmware
bytes land in the EFX_RAM10 `INIT_` params. After synthesis, verify:

```bash
for sym in 0 1 2 3; do
  L=$(grep -n "EFX_RAM10" outflow/church_soc_cm.map.v | grep "ram_symbol${sym}__D\$g1" | head -1 | cut -d: -f1)
  sed -n "${L},$((L+3))p" outflow/church_soc_cm.map.v | grep INIT_0
done
# all four lanes MUST be non-zero hex
```

**Why:** a silently-zeroed BRAM (wrong efx_map / `$readmemb` ignored / patch_sapphire_init.py
not run) boots the SoC with blank firmware — looks like a build success but UART is silent.
**How to apply:** run the INIT_0 check after every synthesis before spending time on PnR.
Prerequisites that gate BRAM embed regardless of version: `patch_sapphire_init.py` must have
run (grep -c readmemb sapphire.v → 0) and `optimize-zero-init-rom` must be `"0"`.

# Proven non-version facts
- Build dir on Penguin: `~/church_project/SoC/` (no git clone — files arrive via IDE `/dl/*` curl routes).
- `efx_pnr` needs explicit `--family Titanium --device Ti60F225 --operating_conditions C3` or it SIGSEGVs.
- 2026.1 headless needs the 5 one-time PT patches (BUILD_SOC_CM.md "5 one-time patches") or the Interface Designer refuses to emit the LPF.
- `efx_pgm` flow: `efx_run <proj> --prj --flow interface` (makes LPF from peri.xml) THEN `--flow pgm`.
