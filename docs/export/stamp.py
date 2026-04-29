#!/usr/bin/env python3
"""
Church Machine — Document stamp utility
Usage:
  python3 docs/export/stamp.py                     # stamp all docs/*.md
  python3 docs/export/stamp.py docs/my-doc.md      # stamp a single file
  python3 docs/export/stamp.py docs/a.md docs/b.md # stamp multiple files

Idempotent — safe to re-run on already-stamped documents.
Adds (if missing):
  - **vX.Y — YYYY-MM-DD**   immediately after the first # heading
  - **CONFIDENTIAL**          immediately after the version line
  - footer separator + author line at end of file
"""

import os, re, sys, glob
from datetime import date

AUTHOR      = "Kenneth Hamer-Hodges"
TODAY       = date.today().strftime("%Y-%m-%d")
MONTH_YEAR  = date.today().strftime("%B %Y")

VERSION_RE  = re.compile(r'^\*\*v\d+\.\d+', re.MULTILINE)
FOOTER_TEXT = f"*Confidential — {AUTHOR} — {MONTH_YEAR}*"
CONF_LINE   = "**CONFIDENTIAL**"

def stamp(path: str) -> bool:
    with open(path, encoding="utf-8") as f:
        content = f.read()

    original = content
    lines = content.splitlines(keepends=True)

    # ── 1. Version line after first H1 ───────────────────────────────────────
    if not VERSION_RE.search(content):
        version_line = f"**v1.0 — {TODAY}**\n"
        h1_idx = next((i for i, l in enumerate(lines) if l.startswith("# ")), None)
        if h1_idx is not None:
            insert = h1_idx + 1
            # skip any existing blank line after H1
            if insert < len(lines) and lines[insert].strip() == "":
                lines.insert(insert + 1, version_line)
                lines.insert(insert + 2, "\n")
            else:
                lines.insert(insert, "\n")
                lines.insert(insert + 1, version_line)
                lines.insert(insert + 2, "\n")
        else:
            lines.insert(0, version_line + "\n")
        content = "".join(lines)

    # ── 2. CONFIDENTIAL line after version line ───────────────────────────────
    if CONF_LINE not in content:
        m = VERSION_RE.search(content)
        if m:
            end = content.index("\n", m.start()) + 1
            content = content[:end] + CONF_LINE + "\n" + content[end:]

    # ── 3. Author footer at end of file ──────────────────────────────────────
    if FOOTER_TEXT not in content:
        content = content.rstrip("\n") + "\n\n---\n" + FOOTER_TEXT + "\n"

    if content != original:
        with open(path, "w", encoding="utf-8") as f:
            f.write(content)
        return True
    return False


def main():
    if len(sys.argv) > 1:
        targets = sys.argv[1:]
    else:
        targets = sorted(glob.glob("docs/*.md"))
        for extra in ["church_machine_final_design.md"]:
            if os.path.exists(extra):
                targets.append(extra)

    updated = 0
    for path in targets:
        if not os.path.isfile(path):
            print(f"  skip (not found): {path}")
            continue
        if stamp(path):
            print(f"  stamped: {path}")
            updated += 1
        else:
            print(f"  ok:      {path}")

    print(f"\n{updated} file(s) updated, {len(targets) - updated} already compliant.")


if __name__ == "__main__":
    main()
