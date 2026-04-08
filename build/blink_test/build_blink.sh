#!/bin/bash
set -e

echo "=== Step 1: Yosys synthesis ==="
yosys -p "read_verilog blink.v; synth_gowin -top top -json blink.json"

echo ""
echo "=== Step 2: nextpnr place-and-route ==="
nextpnr-himbaechel \
  --device GW2AR-LV18QN88C8/I7 \
  --vopt family=GW2A-18C \
  --vopt partname=GW2AR-LV18QN88C8/I7 \
  --vopt cst=blink_pintest.cst \
  --json blink.json \
  --write blink_pnr.json

echo ""
echo "=== Step 3: gowin_pack bitstream ==="
gowin_pack -d GW2A-18C -o blink.fs blink_pnr.json

echo ""
echo "=== Step 4: Program FPGA ==="
echo "Run: openFPGALoader -b tangnano20k blink.fs"
echo "If SRAM fails, try: openFPGALoader -b tangnano20k -f blink.fs"

echo ""
echo "Build complete! blink.fs is ready."
ls -la blink.fs
