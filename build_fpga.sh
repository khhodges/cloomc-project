#!/bin/bash
set -e
cd /home/runner/workspace

echo "=== Generating Verilog ==="
python -m church_machine.gen_verilog --pico-ice

echo "=== Synthesizing ==="
yosys -q -p 'synth_ice40 -top top -json build/church_pico_ice.json' build/church_pico_ice.v

echo "=== Place & Route (this takes 3-5 minutes) ==="
nextpnr-ice40 --up5k --package sg48 \
    --pcf church_machine/pico_ice.pcf --pcf-allow-unconstrained \
    --json build/church_pico_ice.json --asc build/church_pico_ice.asc \
    --freq 12 2>&1 | grep -E 'Max freq|ICESTORM_LC|SPRAM'

echo "=== Packing bitstream ==="
icepack build/church_pico_ice.asc build/church_pico_ice.bin
echo "=== DONE: build/church_pico_ice.bin ($(wc -c < build/church_pico_ice.bin) bytes) ==="
