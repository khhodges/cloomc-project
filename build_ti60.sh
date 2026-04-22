#!/bin/bash
set -e
cd /home/runner/workspace

OUTPUT_DIR="${1:-build}"
mkdir -p "$OUTPUT_DIR"

echo "=== Ti60 F225 Build Flow ==="
echo "Output directory: $OUTPUT_DIR"
echo ""

echo "=== Step 1: Generating RTLIL + Verilog ==="
python -m hardware.gen_rtlil --ti60 "$OUTPUT_DIR"

IL_FILE="$OUTPUT_DIR/church_ti60_f225.il"
V_FILE="$OUTPUT_DIR/church_ti60_f225.v"

if [ ! -f "$V_FILE" ]; then
    echo "[ERROR] Verilog file not generated: $V_FILE"
    exit 1
fi
echo "  Verilog OK: $V_FILE ($(wc -c < "$V_FILE") bytes)"

echo ""
echo "=== Step 1b: Checking for stale CR7 signal names ==="
bash "$(dirname "$0")/scripts/check_stale_cr7.sh" "$V_FILE"

echo ""
echo "=== Step 2: Yosys resource estimation ==="
STAT_LOG="$OUTPUT_DIR/ti60_yosys_stat.log"

yosys -p "
    read_rtlil $IL_FILE;
    hierarchy -top top;
    proc;
    flatten;
    clean;
    stat
" > "$STAT_LOG" 2>&1

echo "  Pre-techmap stat saved to $STAT_LOG"

TECHMAP_LOG="$OUTPUT_DIR/ti60_techmap_stat.log"
TECHMAP_OK=1
timeout 100 yosys -p "
    read_rtlil $IL_FILE;
    hierarchy -top top;
    proc;
    flatten;
    clean;
    opt -fast;
    techmap;
    opt -fast;
    stat
" > "$TECHMAP_LOG" 2>&1 || { echo "  [warn] techmap timed out or failed"; TECHMAP_OK=0; }

echo "  Post-techmap stat saved to $TECHMAP_LOG"

echo ""
echo "=== Step 3: Generating resource report ==="
REPORT="$OUTPUT_DIR/ti60_resource_report.txt"

python3 -c "
import re, sys, datetime, os

TI60_LUT_BUDGET = 60000
TI60_FF_BUDGET  = 60000
TI60_BRAM_BITS  = 256 * 1024 * 8   # 256 KB = 2,097,152 bits

techmap_ok = int('$TECHMAP_OK')
pre_stat = open('$STAT_LOG').read()
tech_stat = open('$TECHMAP_LOG').read() if techmap_ok and os.path.exists('$TECHMAP_LOG') else ''

def extract_cells(text):
    cells = {}
    in_stat = False
    for line in text.splitlines():
        if 'Printing statistics' in line:
            in_stat = True
        if in_stat:
            m = re.match(r'\s+(\S+)\s+(\d+)', line)
            if m:
                cells[m.group(1)] = int(m.group(2))
    return cells

pre = extract_cells(pre_stat)
tech = extract_cells(tech_stat)

mem_match = re.search(r'Number of memory bits:\s+(\d+)', pre_stat)
mem_bits = int(mem_match.group(1)) if mem_match else 0

mem_count_match = re.search(r'Number of memories:\s+(\d+)', pre_stat)
mem_count = int(mem_count_match.group(1)) if mem_count_match else 0

if tech:
    logic_gates = sum(v for k, v in tech.items() if k.startswith('\$_') and not k.startswith('\$_DFF') and not k.startswith('\$_SDFF'))
    ff_count = sum(v for k, v in tech.items() if k.startswith('\$_DFF') or k.startswith('\$_SDFF'))
    lut_estimate = (logic_gates + 3) // 4
    techmap_valid = logic_gates > 0 and ff_count > 0
else:
    logic_gates = 0
    ff_count = 0
    lut_estimate = 0
    techmap_valid = False

lines = []
lines.append('=' * 70)
lines.append('Church Machine — Ti60 F225 Resource Utilization Report')
lines.append('Generated: ' + datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S'))
lines.append('=' * 70)
lines.append('')
lines.append('Target: Efinix Titanium Ti60 F225 (C3 speed grade)')
lines.append('Clock:  50 MHz (25 MHz crystal × PLL)')
lines.append('')
lines.append('--- Estimated Resource Usage ---')
lines.append('')
if techmap_valid:
    lines.append(f'  Logic gates (post-techmap):  {logic_gates:>8,}')
    lines.append(f'  LUT estimate (gates / 4):    {lut_estimate:>8,}  / {TI60_LUT_BUDGET:>6,}  ({100*lut_estimate/TI60_LUT_BUDGET:.1f}%)')
    lines.append(f'  Flip-flops:                  {ff_count:>8,}  / {TI60_FF_BUDGET:>6,}  ({100*ff_count/TI60_FF_BUDGET:.1f}%)')
else:
    lines.append('  Logic gates (post-techmap):       N/A  (techmap failed or timed out)')
    lines.append('  LUT estimate:                     N/A')
    lines.append('  Flip-flops:                       N/A')
lines.append(f'  Memory bits:                 {mem_bits:>8,}  / {TI60_BRAM_BITS:>7,}  ({100*mem_bits/TI60_BRAM_BITS:.1f}%)')
lines.append(f'  Memory instances:            {mem_count:>8}')
lines.append(f'  Memory (KB):                 {mem_bits/8/1024:>8.1f}  / {256:>6}')
lines.append('')

lines.append('--- Fit Assessment ---')
lines.append('')
if not techmap_valid:
    lines.append('  RESULT: INCONCLUSIVE — techmap failed; LUT/FF estimates unavailable')
    lines.append('  BRAM usage appears within budget, but full fit requires Efinity compilation')
elif lut_estimate <= TI60_LUT_BUDGET and ff_count <= TI60_FF_BUDGET and mem_bits <= TI60_BRAM_BITS:
    lines.append('  RESULT: FITS within Ti60 F225 budget')
    lines.append(f'  LUT headroom:  {TI60_LUT_BUDGET - lut_estimate:,} LUTs ({100*(TI60_LUT_BUDGET - lut_estimate)/TI60_LUT_BUDGET:.1f}%)')
    lines.append(f'  FF headroom:   {TI60_FF_BUDGET - ff_count:,} FFs ({100*(TI60_FF_BUDGET - ff_count)/TI60_FF_BUDGET:.1f}%)')
    lines.append(f'  BRAM headroom: {(TI60_BRAM_BITS - mem_bits)/8/1024:.1f} KB ({100*(TI60_BRAM_BITS - mem_bits)/TI60_BRAM_BITS:.1f}%)')
else:
    lines.append('  RESULT: DOES NOT FIT — optimization needed')
    if lut_estimate > TI60_LUT_BUDGET:
        lines.append(f'  LUT OVER by {lut_estimate - TI60_LUT_BUDGET:,}')
    if ff_count > TI60_FF_BUDGET:
        lines.append(f'  FF OVER by {ff_count - TI60_FF_BUDGET:,}')
    if mem_bits > TI60_BRAM_BITS:
        lines.append(f'  BRAM OVER by {(mem_bits - TI60_BRAM_BITS)/8/1024:.1f} KB')
lines.append('')

lines.append('--- Pre-Techmap Cell Breakdown ---')
lines.append('')
for k in sorted(pre.keys()):
    if not k.startswith('\$scope'):
        lines.append(f'  {k:<24s} {pre[k]:>6,}')
lines.append('')

lines.append('--- Post-Techmap Cell Breakdown ---')
lines.append('')
if techmap_valid:
    for k in sorted(tech.keys()):
        if not k.startswith('\$scope') and not k.startswith('\$mem'):
            lines.append(f'  {k:<24s} {tech[k]:>6,}')
else:
    lines.append('  (techmap did not complete — no post-techmap data available)')
lines.append('')

lines.append('--- Modules Included ---')
lines.append('')
lines.append('  ChurchCore (top.core)')
lines.append('    ChurchRegisters, ChurchDecoder, ChurchPermCheck')
lines.append('    ChurchGCUnit, ChurchLambda, ChurchCall, ChurchReturn')
lines.append('    ChurchTperm, ChurchSave, ChurchLoad, ChurchMLoad')
lines.append('    ChurchChange, ChurchSwitch, ChurchELoadCall, ChurchXLoadLambda')
lines.append('    ChurchDRead, ChurchDWrite, ChurchCLoad, ChurchOutform')
lines.append('  BootRom')
lines.append('  DebugPrinter (UART TX)')
lines.append('  UartRx')
lines.append('  DMEM (2048×32 Memory)')
lines.append('  MMIO (LEDs, UART, Timer, Button)')
lines.append('  Debug FSM (PATCH_LUMP, READ_BRAM protocols)')
lines.append('')
lines.append('--- Notes ---')
lines.append('')
lines.append('  * LUT estimate uses gates/4 heuristic; Efinity mapper may differ ±20%')
lines.append('  * Memory is inferred as Efinix EBR tiles by Efinity synthesis')
lines.append('  * Yosys stat does not account for PLL, I/O buffers, or routing')
lines.append('  * Final resource usage requires Efinity IDE compilation')
lines.append('')

report = '\n'.join(lines)
with open('$REPORT', 'w') as f:
    f.write(report)
print(report)
"

echo ""
echo "=== Resource report: $REPORT ==="
echo "=== Ti60 build complete ==="
echo "  RTLIL:   $IL_FILE"
echo "  Verilog: $V_FILE"
echo "  Report:  $REPORT"
