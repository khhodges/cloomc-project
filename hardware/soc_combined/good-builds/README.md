# Good Builds

Snapshot of known-working Ti60 SoC firmware builds.

## Files

| File | Description |
|---|---|
| `firmware-YYYY-MM-DD.hex` | Compiled C firmware (BRAM initialisation input for Efinity) |
| `church_soc_cm-YYYY-MM-DD.hex` | Final Efinity output hex — flash this to the Ti60 |

## How to use a saved build

Flash the `.hex` directly with Efinity Programmer (no recompile needed):

1. Open Efinity Programmer
2. Select the `.hex` file
3. Program → the FPGA loads the saved firmware immediately

The `church_soc_cm-*.hex` files can also be uploaded to the IDE so users can
flash via the Builder:

```bash
curl -X POST http://localhost:5000/api/bitstream/upload \
     -F "board=ti60-f225" \
     -F "file=@church_soc_cm-YYYY-MM-DD.hex"
```

## Adding a new good build

After a successful test run:

```bash
cp firmware/firmware.hex        good-builds/firmware-$(date +%Y-%m-%d).hex
cp outflow/church_soc_cm.hex    good-builds/church_soc_cm-$(date +%Y-%m-%d).hex
git add good-builds/
git commit -m "good-build: Ti60 $(date +%Y-%m-%d) — all UART tests passing"
git push
```
