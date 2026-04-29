#!/usr/bin/env bash
# ─── Church Machine — new document scaffold ───────────────────────────────────
# Creates a pre-stamped .md file ready for content.
# Usage:
#   ./docs/export/new-doc.sh "Document Title" docs/my-document.md
# ─────────────────────────────────────────────────────────────────────────────
set -euo pipefail

AUTHOR="Kenneth Hamer-Hodges"
TODAY="$(date +%Y-%m-%d)"
MONTH_YEAR="$(date +'%B %Y')"

# ── args ──────────────────────────────────────────────────────────────────────
if [[ $# -lt 2 ]]; then
  echo "Usage: $0 \"Document Title\" path/to/output.md"
  exit 1
fi

TITLE="$1"
OUTFILE="$2"

if [[ -e "$OUTFILE" ]]; then
  echo "ERROR: $OUTFILE already exists. Edit it directly, or remove it first."
  exit 1
fi

mkdir -p "$(dirname "$OUTFILE")"

cat > "$OUTFILE" << TEMPLATE
# ${TITLE}

**v1.0 — ${TODAY}**
**CONFIDENTIAL**

## Overview

<!-- Add content here -->

---
*Confidential — ${AUTHOR} — ${MONTH_YEAR}*
TEMPLATE

echo "Created: $OUTFILE"
