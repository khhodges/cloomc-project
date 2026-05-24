#!/usr/bin/env python3
"""
check_ti60_utilisation.py — Ti60F225 synthesis resource utilisation guard.

Parses Efinity synthesis report files under hardware/soc_combined/work_syn/
and exits non-zero if Logic Element (LE) or BRAM usage exceeds the configured
thresholds (default 90 % each).

Usage:
    python scripts/check_ti60_utilisation.py [options]

Options:
    --report-dir PATH       Directory containing Efinity .rpt files
                            (default: hardware/soc_combined/work_syn)
    --le-threshold N        Fail if LE utilisation exceeds N % (default: 90)
    --bram-threshold N      Fail if BRAM utilisation exceeds N % (default: 90)
    --le-total N            Ti60F225 total Logic Elements (default: 59904)
    --bram-total-kb N       Ti60F225 total BRAM in KB (default: 256)
    --report-file PATH      Parse a single named file instead of scanning dir

Exit codes:
    0   All utilisation figures within thresholds (or no report found and
        --missing-ok is set)
    1   One or more thresholds exceeded
    2   No report files found (and --missing-ok not set)
    3   Report found but required resource lines could not be parsed

Ti60F225 capacity reference:
    Logic Elements : 59 904 LE
    Embedded BRAM  : 256 KB  (260 × 8 Kbit blocks)

Efinity report formats supported
---------------------------------
Format A — pipe-delimited table (Efinity 2024+):
    | Logic Element (LE) |  12345 | 59904 |  20.61 |
    | Block RAM (BRAM)   |    128 |   256 |  50.00 |

Format B — colon + slash (older Efinity):
    Logic Elements: 12345 / 59904 (20.61%)
    Block RAM     : 128 / 256 (50.00%)

Format C — percentage only:
    Logic Elements    20.61 %
    Block RAM         50.00 %

Format D — "Used / Total" without percentage:
    LE Used / LE Total : 12345 / 59904
    BRAM Used (KB)     : 128

All formats are detected by keyword matching on the resource line.
"""

import argparse
import re
import sys
from pathlib import Path

# ---------------------------------------------------------------------------
# Ti60F225 hard capacity constants (authoritative values from BUILD_SOC_CM.md)
# ---------------------------------------------------------------------------
TI60_LE_TOTAL = 59_904
TI60_BRAM_TOTAL_KB = 256

DEFAULT_THRESHOLD_PCT = 90

# ---------------------------------------------------------------------------
# Keyword sets that identify LE and BRAM lines in any report variant
# ---------------------------------------------------------------------------
_LE_KEYWORDS = re.compile(
    r"logic[\s_-]*element|(?<!\w)le(?!\w)|logic[\s_-]*cell",
    re.IGNORECASE,
)
_BRAM_KEYWORDS = re.compile(
    r"block[\s_-]*ram|bram|embedded[\s_-]*mem|block[\s_-]*mem|m\d+k",
    re.IGNORECASE,
)

# ---------------------------------------------------------------------------
# Number extraction helpers
# ---------------------------------------------------------------------------

def _extract_percentage(line: str) -> float | None:
    """Return the first percentage value found on a line, or None."""
    m = re.search(r"(\d+(?:\.\d+)?)\s*%", line)
    if m:
        return float(m.group(1))
    return None


def _extract_used_total(line: str) -> tuple[int, int] | None:
    """
    Return (used, total) from patterns like:
        12345 / 59904
        12345 out of 59904
    Returns None if not found.
    """
    m = re.search(r"(\d[\d,]*)\s*(?:/|out\s+of)\s*(\d[\d,]*)", line)
    if m:
        used = int(m.group(1).replace(",", ""))
        total = int(m.group(2).replace(",", ""))
        if total > 0:
            return used, total
    return None


def _extract_first_integer(line: str) -> int | None:
    """Return the first integer on a line (ignoring leading non-digits)."""
    m = re.search(r"\b(\d[\d,]*)\b", line)
    if m:
        return int(m.group(1).replace(",", ""))
    return None


# ---------------------------------------------------------------------------
# Per-line parser
# ---------------------------------------------------------------------------

def _parse_line(line: str, total_fallback: int) -> float | None:
    """
    Given a resource line and a fallback total, return the utilisation
    percentage, or None if the line cannot be interpreted.
    """
    pct = _extract_percentage(line)
    if pct is not None:
        return pct

    ut = _extract_used_total(line)
    if ut is not None:
        used, total = ut
        return 100.0 * used / total

    used = _extract_first_integer(line)
    if used is not None and total_fallback > 0:
        return 100.0 * used / total_fallback

    return None


# ---------------------------------------------------------------------------
# Report file parser
# ---------------------------------------------------------------------------

def parse_report(text: str, le_total: int, bram_total_kb: int) -> dict:
    """
    Parse the body of an Efinity synthesis report.

    Returns a dict with keys:
        le_pct   : float | None
        bram_pct : float | None
    """
    result = {"le_pct": None, "bram_pct": None}

    for line in text.splitlines():
        stripped = line.strip()
        if not stripped:
            continue

        if _LE_KEYWORDS.search(stripped) and result["le_pct"] is None:
            pct = _parse_line(stripped, le_total)
            if pct is not None:
                result["le_pct"] = pct

        if _BRAM_KEYWORDS.search(stripped) and result["bram_pct"] is None:
            pct = _parse_line(stripped, bram_total_kb)
            if pct is not None:
                result["bram_pct"] = pct

        if result["le_pct"] is not None and result["bram_pct"] is not None:
            break

    return result


# ---------------------------------------------------------------------------
# Report file discovery
# ---------------------------------------------------------------------------
_REPORT_SUFFIXES = {".rpt", ".log", ".txt"}


def find_reports(report_dir: Path) -> list[Path]:
    """Return all candidate report files in report_dir (non-recursive)."""
    if not report_dir.is_dir():
        return []
    files = []
    for p in sorted(report_dir.iterdir()):
        if p.suffix.lower() in _REPORT_SUFFIXES and p.is_file():
            files.append(p)
    return files


# ---------------------------------------------------------------------------
# Main check logic
# ---------------------------------------------------------------------------

def check(
    report_dir: Path,
    report_file: Path | None,
    le_threshold: float,
    bram_threshold: float,
    le_total: int,
    bram_total_kb: int,
    missing_ok: bool,
) -> int:
    """
    Run the utilisation check.  Returns an exit code (0 = pass).
    """
    # Gather report files to inspect
    if report_file is not None:
        if not report_file.is_file():
            print(f"ERROR: specified report file not found: {report_file}", file=sys.stderr)
            return 2
        candidates = [report_file]
    else:
        candidates = find_reports(report_dir)
        if not candidates:
            msg = f"No report files found in {report_dir}"
            if missing_ok:
                print(f"WARNING: {msg} — skipping check (--missing-ok set)")
                return 0
            print(f"ERROR: {msg}", file=sys.stderr)
            print(
                "  Run Efinity synthesis first: Compile → Synthesis in the Efinity IDE,\n"
                f"  then re-run this script.  Expected directory: {report_dir}",
                file=sys.stderr,
            )
            return 2

    # Parse each file; accumulate best (most complete) reading
    best = {"le_pct": None, "bram_pct": None}
    parsed_files = []

    for rpt in candidates:
        try:
            text = rpt.read_text(encoding="utf-8", errors="replace")
        except OSError as exc:
            print(f"WARNING: could not read {rpt}: {exc}", file=sys.stderr)
            continue

        result = parse_report(text, le_total, bram_total_kb)
        if result["le_pct"] is not None or result["bram_pct"] is not None:
            parsed_files.append(rpt.name)
            if result["le_pct"] is not None and best["le_pct"] is None:
                best["le_pct"] = result["le_pct"]
            if result["bram_pct"] is not None and best["bram_pct"] is None:
                best["bram_pct"] = result["bram_pct"]

    if best["le_pct"] is None and best["bram_pct"] is None:
        if missing_ok:
            print(
                "WARNING: resource utilisation data not found in any report file "
                "— skipping check (--missing-ok set)"
            )
            return 0
        print(
            "ERROR: could not extract LE or BRAM utilisation from any report file.",
            file=sys.stderr,
        )
        print(
            "  Checked files: " + (", ".join(str(c.name) for c in candidates) or "(none)"),
            file=sys.stderr,
        )
        print(
            "  Ensure synthesis has completed successfully (not just synthesis start).\n"
            "  See hardware/soc_combined/BUILD_SOC_CM.md for expected report location.",
            file=sys.stderr,
        )
        return 3

    # Print summary
    print("Ti60F225 Resource Utilisation Check")
    print(f"  Report source  : {', '.join(parsed_files)}")
    print(f"  LE total       : {le_total:,}")
    print(f"  BRAM total     : {bram_total_kb} KB")
    print(f"  Threshold      : {le_threshold:.0f}% LE / {bram_threshold:.0f}% BRAM")
    print()

    failures = []

    if best["le_pct"] is not None:
        status = "PASS" if best["le_pct"] <= le_threshold else "FAIL"
        print(f"  Logic Elements : {best['le_pct']:.2f}%  [{status}]")
        if status == "FAIL":
            failures.append(
                f"LE utilisation {best['le_pct']:.2f}% exceeds threshold {le_threshold:.0f}%"
            )
    else:
        print("  Logic Elements : (not found in report)")

    if best["bram_pct"] is not None:
        status = "PASS" if best["bram_pct"] <= bram_threshold else "FAIL"
        print(f"  Block RAM      : {best['bram_pct']:.2f}%  [{status}]")
        if status == "FAIL":
            failures.append(
                f"BRAM utilisation {best['bram_pct']:.2f}% exceeds threshold {bram_threshold:.0f}%"
            )
    else:
        print("  Block RAM      : (not found in report)")

    print()

    if failures:
        print("FAIL — Ti60F225 resource utilisation exceeds limits:")
        for msg in failures:
            print(f"  • {msg}")
        print(
            "\n  The combined SoC+CM bitstream is too large.  Review recent RTL changes\n"
            "  and consider enabling logic_opting in church_soc_cm.xml.  See\n"
            "  hardware/soc_combined/BUILD_SOC_CM.md § Troubleshooting."
        )
        return 1

    print("PASS — all resources within Ti60F225 capacity limits.")
    return 0


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="Check Efinity synthesis utilisation against Ti60F225 limits.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "--report-dir",
        default="hardware/soc_combined/work_syn",
        metavar="PATH",
        help="Directory containing Efinity .rpt files (default: hardware/soc_combined/work_syn)",
    )
    parser.add_argument(
        "--report-file",
        default=None,
        metavar="PATH",
        help="Parse a single named file instead of scanning --report-dir",
    )
    parser.add_argument(
        "--le-threshold",
        type=float,
        default=DEFAULT_THRESHOLD_PCT,
        metavar="N",
        help=f"Fail if LE %% > N (default: {DEFAULT_THRESHOLD_PCT})",
    )
    parser.add_argument(
        "--bram-threshold",
        type=float,
        default=DEFAULT_THRESHOLD_PCT,
        metavar="N",
        help=f"Fail if BRAM %% > N (default: {DEFAULT_THRESHOLD_PCT})",
    )
    parser.add_argument(
        "--le-total",
        type=int,
        default=TI60_LE_TOTAL,
        metavar="N",
        help=f"Total Ti60F225 Logic Elements (default: {TI60_LE_TOTAL})",
    )
    parser.add_argument(
        "--bram-total-kb",
        type=int,
        default=TI60_BRAM_TOTAL_KB,
        metavar="N",
        help=f"Total Ti60F225 BRAM in KB (default: {TI60_BRAM_TOTAL_KB})",
    )
    parser.add_argument(
        "--missing-ok",
        action="store_true",
        help="Exit 0 (with a warning) if no report files are found, instead of failing",
    )

    args = parser.parse_args(argv)

    return check(
        report_dir=Path(args.report_dir),
        report_file=Path(args.report_file) if args.report_file else None,
        le_threshold=args.le_threshold,
        bram_threshold=args.bram_threshold,
        le_total=args.le_total,
        bram_total_kb=args.bram_total_kb,
        missing_ok=args.missing_ok,
    )


if __name__ == "__main__":
    sys.exit(main())
