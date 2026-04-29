#!/usr/bin/env bash
# ─── Church Machine — Markdown → PDF exporter ────────────────────────────────
# Usage:
#   ./docs/export/export-pdf.sh docs/golden-tokens.md        # single file
#   ./docs/export/export-pdf.sh docs/golden-tokens.md out/   # custom output dir
#   ./docs/export/export-pdf.sh --all                         # every doc in docs/
#   ./docs/export/export-pdf.sh --all out/                    # all → custom dir
# Requires: pandoc, chromium (both present in this environment)
# ─────────────────────────────────────────────────────────────────────────────

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TEMPLATE="$SCRIPT_DIR/template.html"
CSS_DIR="$SCRIPT_DIR/"          # trailing slash — referenced inside template
DOCS_DIR="$(dirname "$SCRIPT_DIR")"

# ── helpers ──────────────────────────────────────────────────────────────────
die()  { echo "ERROR: $*" >&2; exit 1; }
info() { echo "  →  $*"; }

which pandoc   >/dev/null 2>&1 || die "pandoc not found"
which chromium >/dev/null 2>&1 || die "chromium not found"

convert_one() {
  local src="$1"
  local outdir="$2"

  [[ -f "$src" ]] || die "File not found: $src"

  local base
  base="$(basename "$src" .md)"
  local html_tmp
  html_tmp="$(mktemp /tmp/cm_export_XXXXXX.html)"
  local pdf_out="$outdir/${base}.pdf"

  # ── Step 1: markdown → standalone HTML ────────────────────────────────────
  pandoc \
    --from=markdown \
    --to=html5 \
    --standalone \
    --template="$TEMPLATE" \
    --variable "exportdir:$CSS_DIR" \
    --metadata "title=$(head -1 "$src" | sed 's/^# //')" \
    --output="$html_tmp" \
    "$src"

  # ── Step 2: HTML → PDF via Chromium headless ──────────────────────────────
  chromium \
    --headless=new \
    --no-sandbox \
    --disable-gpu \
    --run-all-compositor-stages-before-draw \
    --print-to-pdf="$pdf_out" \
    --no-pdf-header-footer \
    --print-to-pdf-no-header \
    "file://$html_tmp" \
    2>/dev/null

  rm -f "$html_tmp"
  info "$base.pdf"
}

# ── argument parsing ──────────────────────────────────────────────────────────
ALL=false
INPUT_FILES=()
OUTDIR=""

for arg in "$@"; do
  case "$arg" in
    --all) ALL=true ;;
    --*) die "Unknown option: $arg" ;;
    *)
      # Treat as output dir if: it is an existing dir, ends with /, or has no .md extension
      if [[ -d "$arg" ]] || [[ "$arg" == */ ]] || [[ "$arg" != *.md ]]; then
        OUTDIR="${arg%/}"
      else
        INPUT_FILES+=("$arg")
      fi
      ;;
  esac
done

# Default output directory
if [[ -z "$OUTDIR" ]]; then
  OUTDIR="$DOCS_DIR/export/pdf"
fi
mkdir -p "$OUTDIR"

# ── run ───────────────────────────────────────────────────────────────────────
if $ALL; then
  echo "Exporting all docs → $OUTDIR"
  while IFS= read -r -d '' f; do
    convert_one "$f" "$OUTDIR"
  done < <(find "$DOCS_DIR" -maxdepth 1 -name "*.md" -print0 | sort -z)
elif [[ ${#INPUT_FILES[@]} -gt 0 ]]; then
  echo "Exporting ${#INPUT_FILES[@]} file(s) → $OUTDIR"
  for f in "${INPUT_FILES[@]}"; do
    convert_one "$f" "$OUTDIR"
  done
else
  echo "Church Machine PDF Exporter"
  echo "Usage:"
  echo "  $0 <file.md> [output-dir/]"
  echo "  $0 --all [output-dir/]"
  exit 0
fi

echo "Done. PDFs written to: $OUTDIR"
