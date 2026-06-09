#!/bin/bash
# flash_soc.sh — standalone Sapphire SoC RISC-V example for Ti60F225.
# No Church Machine.  Prints "CHURCH Ti60 v1.0" over UART, LED0 ON.
# Button re-sends greeting without reprogramming.
#
# Run on the Penguin:
#   curl -fsS "$IDE_URL/dl/flash-soc" -o ~/flash_soc.sh && bash ~/flash_soc.sh
#
# Steps: firmware build → synthesis (2025.2 efx_map, ~20 min) →
#        interface + P&R + bitstream (2026.1) → flash → UART test.
# Stops on first failure and prints the log tail.

set -uo pipefail

B="${IDE_URL:-https://31592a69-0a64-402e-9237-89b7ce66a127-00-1hr1bt2ealopt.kirk.replit.dev}"
WORK="$HOME/church_project/SoC_minimal"
MAP_HOME="${EFX_MAP_HOME:-$HOME/efinity/2025.2}"
PNR_HOME="${EFX_PNR_HOME:-$HOME/efinity/2026.1}"
CIRCUIT="church_soc"
TOOLCHAIN="${RISCV_TOOLCHAIN:-$HOME/efinity/efinity-riscv-ide-2025.2/toolchain/bin}"

echo "===> [1/9] Create working directory"
mkdir -p "$WORK/firmware" "$WORK/work_syn" "$WORK/work_pnr" "$WORK/outflow"
cd "$WORK"

echo "===> [2/9] Download source files from IDE"
for pair in \
    "top.v:soc-top-v" \
    "church_soc.xml:soc-xml" \
    "church_soc.peri.xml:soc-peri-xml" \
    "firmware/main.c:soc-fw-main" \
    "firmware/crt0.S:soc-fw-crt0" \
    "firmware/link.ld:soc-fw-link" \
    "firmware/Makefile:soc-fw-make"
do
    DST="${pair%%:*}"
    ROUTE="${pair##*:}"
    curl -fsS "$B/dl/$ROUTE" -o "$WORK/$DST" \
        || { echo "FATAL: failed to download $DST from /dl/$ROUTE"; exit 1; }
done
echo "     Source files downloaded."
grep -o 'db_version="[0-9]*"' "$WORK/church_soc.peri.xml" | head -1

echo "===> [3/9] Copy Sapphire SoC IP (sapphire.v + sapphire_define.vh)"
SAP_BASE="$MAP_HOME/ipm/ip/efx_tsemac/fpga/Ti60F225_devkit/ip/sapphire"
if [ ! -d "$SAP_BASE" ]; then
    SAP_BASE=$(find "$MAP_HOME" -name "sapphire.v" 2>/dev/null | grep Ti60 | head -1 | xargs dirname 2>/dev/null || true)
fi
if [ -z "$SAP_BASE" ] || [ ! -f "$SAP_BASE/sapphire.v" ]; then
    echo "FATAL: sapphire.v not found under $MAP_HOME — is Efinity 2025.2 installed?"
    echo "       Try: find ~/efinity -name 'sapphire.v' 2>/dev/null"
    exit 1
fi
cp "$SAP_BASE/sapphire.v"         "$WORK/"
cp "$SAP_BASE/sapphire_define.vh" "$WORK/"
echo "     sapphire.v copied from $SAP_BASE"

echo "===> [4/9] Build RISC-V firmware"
if [ ! -x "$TOOLCHAIN/riscv-none-embed-gcc" ]; then
    echo "FATAL: riscv-none-embed-gcc not found at $TOOLCHAIN"
    echo "       Is Efinity RISC-V IDE 2025.2 installed?"
    exit 1
fi
make -C "$WORK/firmware" TOOLCHAIN="$TOOLCHAIN" > "$WORK/_fw_build.log" 2>&1
if [ ! -f "$WORK/firmware/firmware.hex" ]; then
    tail -10 "$WORK/_fw_build.log"
    echo "FATAL: firmware.hex not produced"; exit 1
fi
cp "$WORK/firmware/firmware.hex" "$WORK/"
echo "     firmware.hex: $(wc -l < "$WORK/firmware/firmware.hex") lines"

echo "===> [5/9] Synthesis with efx_map 2025.2 (~20 min — grab a coffee)"
echo "     Started at $(date '+%H:%M:%S')"
# efx_map writes its primary log to work_syn/<circuit>.log (not stdout).
# We cd into WORK so all relative paths in church_soc.xml resolve correctly.
(
    cd "$WORK"
    # Deactivate any active venv (OSS CAD Suite / venv-church) before sourcing
    # Efinity's own setup.sh — otherwise Efinity's Python is shadowed and
    # efx_map exits 1 with an empty log.
    # shellcheck disable=SC1091
    deactivate 2>/dev/null || true
    source "$MAP_HOME/bin/setup.sh" 2>/dev/null || true
    "$MAP_HOME/bin/efx_map" --project-xml "$CIRCUIT.xml"
) > "$WORK/work_syn/efx_map_stdout.log" 2>&1
SYN_RC=$?
# The real log is work_syn/<circuit>.log; fall back to our captured stdout
SYNLOG="$WORK/work_syn/${CIRCUIT}.log"
[ -f "$SYNLOG" ] || SYNLOG="$WORK/work_syn/efx_map_stdout.log"
echo "     Synthesis log: $SYNLOG"
tail -10 "$SYNLOG"
if [ $SYN_RC -ne 0 ]; then
    echo ""
    echo "FATAL: Synthesis failed (exit $SYN_RC) — full log: $SYNLOG"
    echo "       All logs in: $WORK/work_syn/"
    ls "$WORK/work_syn/" 2>/dev/null
    exit 1
fi
if grep -qiE '^error' "$SYNLOG" 2>/dev/null; then
    echo "FATAL: Synthesis produced errors — full log: $SYNLOG"; exit 1
fi
echo "     Finished at $(date '+%H:%M:%S')"

echo "===> [6/9] Verify BRAM firmware embedding"
MAPV="$WORK/work_syn/${CIRCUIT}.map.v"
if [ ! -f "$MAPV" ]; then
    echo "FATAL: $MAPV not found — synthesis produced no map.v"; exit 1
fi
NONZERO=$(grep "INIT_0" "$MAPV" 2>/dev/null | grep -vE '"0+"' | wc -l)
if [ "$NONZERO" -eq 0 ]; then
    echo "WARNING: BRAM INIT_0 values look all-zero — firmware may not be embedded."
    echo "         efx_map 2025.2 may have ignored \$readmemh."
    echo "         Continuing anyway; UART will be silent if firmware is missing."
else
    echo "     BRAM check OK: $NONZERO non-zero INIT_0 words found."
fi

echo "===> [7/9] Interface Designer (efx_run 2026.1 — generates LPF from peri.xml)"
export EFINITY_HOME="$PNR_HOME"
# shellcheck disable=SC1091
source "$PNR_HOME/bin/setup.sh" 2>/dev/null || true
"$PNR_HOME/bin/efx_run" "$CIRCUIT" --prj --flow interface \
    --family Titanium -d Ti60F225 > "$WORK/_interface.log" 2>&1
RC=$?
tail -4 "$WORK/_interface.log"
if [ $RC -ne 0 ] || grep -qiE '^error|fail' "$WORK/_interface.log"; then
    echo "FATAL: Interface Designer failed — full log: $WORK/_interface.log"; exit 1
fi

echo "===> [8/9] Place & Route (efx_pnr 2026.1)"
"$PNR_HOME/bin/efx_pnr" \
    --prj            "$WORK/$CIRCUIT.xml" \
    --circuit        "$CIRCUIT" \
    --family         Titanium \
    --device         Ti60F225 \
    --operating_conditions C3 \
    --pack --place --route \
    --vdb_file       "$WORK/work_syn/top.vdb" \
    --work_dir       "$WORK/work_pnr" \
    --output_dir     "$WORK/outflow" \
    > "$WORK/_pnr.log" 2>&1
PNR_RC=$?
tail -10 "$WORK/_pnr.log"
if [ $PNR_RC -ne 0 ] || grep -qiE '^error|fail' "$WORK/_pnr.log"; then
    echo "FATAL: P&R failed — full log: $WORK/_pnr.log"; exit 1
fi

echo "===> [9/9] Bitstream (efx_run 2026.1 pgm)"
"$PNR_HOME/bin/efx_run" "$CIRCUIT" --prj --flow pgm \
    --family Titanium -d Ti60F225 > "$WORK/_pgm.log" 2>&1
RC=$?
tail -4 "$WORK/_pgm.log"
HEX="$WORK/outflow/${CIRCUIT}.hex"
if [ $RC -ne 0 ] || [ ! -f "$HEX" ]; then
    echo "FATAL: bitstream not produced — full log: $WORK/_pgm.log"; exit 1
fi
echo "     hex: $(ls -lh "$HEX" | awk '{print $5}')  $HEX"

echo "===> [flash] Flash via JTAG (sudo — enter password if prompted)"
LOADER="$(command -v openFPGALoader || echo "$HOME/oss-cad-suite/bin/openFPGALoader")"
sudo "$LOADER" -b titanium_ti60_f225_jtag -f "$HEX" \
    || { echo "FATAL: flash failed (loader: $LOADER)"; exit 1; }

echo "===> [test] UART smoke test on ttyUSB2 (5s settle, 115200 baud)"
sleep 5
python3 - <<'PYEOF'
import serial, sys
try:
    s = serial.Serial('/dev/ttyUSB2', 115200, timeout=8)
    s.setRTS(False); s.setDTR(False)
    data = s.read(64).decode('ascii', errors='replace')
    s.close()
    if 'CHURCH' in data:
        print('UART OK:', repr(data))
        sys.exit(0)
    else:
        print('UART: no CHURCH greeting — got:', repr(data))
        sys.exit(1)
except Exception as e:
    print('UART test error:', e)
    sys.exit(1)
PYEOF
UART_RC=$?

echo ""
echo "==============================================================="
if [ $UART_RC -eq 0 ]; then
    echo " SUCCESS! RISC-V SoC is running."
else
    echo " Flash succeeded but UART test failed."
    echo " Try:  python3 -c \\"
    echo "   \"import serial; s=serial.Serial('/dev/ttyUSB2',115200,timeout=5); print(s.read(64))\""
fi
echo " Expect:  LED0 = ON  (SoC out of reset)"
echo "          LED1, LED2 = OFF  (reserved in this build)"
echo " Press the push button to re-send 'CHURCH Ti60 v1.0' on UART."
echo "==============================================================="
