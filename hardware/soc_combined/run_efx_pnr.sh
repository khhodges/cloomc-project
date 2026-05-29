#!/bin/bash
# run_efx_pnr.sh — Place & Route for SoC+CM combined project
# Run from the project root (the folder containing SoC/)
# Usage: bash SoC/run_efx_pnr.sh

set -euo pipefail

EFINITY="${EFINITY_HOME:-$HOME/efinity/2025.2}"
EFX_PNR="$EFINITY/bin/efx_pnr"

PROJECT="${1:-SoC/church_soc_cm.xml}"

echo "==> Place & Route $PROJECT with EFX_PNR..."
echo "    EFX_PNR: $EFX_PNR"
echo "    Project: $PROJECT"
echo ""

$EFX_PNR --project-xml "$PROJECT" 2>&1 | tee SoC/work_pnr/pnr.log
echo ""
echo "==> Place & Route complete. Output in SoC/work_pnr/"
