#!/bin/bash
# run_efx_pnr.sh — Place & Route for SoC+CM combined project
# Run from the project root (the folder containing SoC/)
# Usage: bash SoC/run_efx_pnr.sh
#
# NOTE: Some Efinity installations (especially Chromebook Penguin) have a broken
# command-line efx_pnr that crashes with "Unsupported value for family=".  If
# that happens, use the Efinity GUI instead:
#   1. Open Efinity GUI
#   2. File → Open Project → select church_soc_cm.xml
#   3. Project → Place & Route
#   4. Project → Generate Bitstream

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
EFINITY="${EFINITY_HOME:-$HOME/efinity/2025.2}"
EFX_PNR="$EFINITY/bin/efx_pnr"

PROJECT="${1:-$PROJECT_ROOT/SoC/church_soc_cm.xml}"

echo "==> Place & Route $PROJECT with EFX_PNR..."
echo "    EFX_PNR: $EFX_PNR"
echo "    Project: $PROJECT"
echo ""

mkdir -p "$PROJECT_ROOT/SoC/work_pnr"
cd "$PROJECT_ROOT/SoC"
$EFX_PNR church_soc_cm.xml church_soc_cm 2>&1 | tee "$PROJECT_ROOT/SoC/work_pnr/pnr.log"
echo ""
echo "==> Place & Route complete. Output in SoC/work_pnr/"
