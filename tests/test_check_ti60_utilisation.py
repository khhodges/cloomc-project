"""
tests/test_check_ti60_utilisation.py

Unit tests for scripts/check_ti60_utilisation.py.

Tests cover:
- Format A: pipe-delimited table (Efinity 2024+ style)
- Format B: colon + slash (older Efinity style)
- Format C: percentage-only lines
- Format D: used/total without percentage
- Pass / fail thresholds for LE and BRAM independently
- Missing report directory behaviour (with and without --missing-ok)
- Unparseable report returns exit code 3
- CLI argument wiring via main()
"""

import sys
import textwrap
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parent.parent / "scripts"))

from check_ti60_utilisation import (  # noqa: E402
    parse_report,
    check,
    main,
    TI60_LE_TOTAL,
    TI60_BRAM_TOTAL_KB,
)

# ---------------------------------------------------------------------------
# parse_report unit tests
# ---------------------------------------------------------------------------

class TestParseReportFormatA:
    """Pipe-delimited table — Efinity 2024+ style."""

    REPORT = textwrap.dedent("""\
        +-----------------------------+-------+-------+----------+
        | Resource                    | Used  | Total | Util (%) |
        +-----------------------------+-------+-------+----------+
        | Logic Element (LE)          | 30000 | 59904 |  50.08   |
        | Block RAM (BRAM) [KB]       |   128 |   256 |  50.00   |
        | Register                    | 18000 | 59904 |  30.05   |
        +-----------------------------+-------+-------+----------+
    """)

    def test_le_pct(self):
        r = parse_report(self.REPORT, TI60_LE_TOTAL, TI60_BRAM_TOTAL_KB)
        assert r["le_pct"] == pytest.approx(50.08, abs=0.1)

    def test_bram_pct(self):
        r = parse_report(self.REPORT, TI60_LE_TOTAL, TI60_BRAM_TOTAL_KB)
        assert r["bram_pct"] == pytest.approx(50.00, abs=0.1)


class TestParseReportFormatB:
    """Colon + slash — older Efinity style."""

    REPORT = textwrap.dedent("""\
        Synthesis Resource Summary
        ==========================
        Logic Elements  : 45000 / 59904 (75.11%)
        Block RAM       : 200 / 256 (78.12%)
        Registers       : 20000 / 59904 (33.38%)
    """)

    def test_le_pct(self):
        r = parse_report(self.REPORT, TI60_LE_TOTAL, TI60_BRAM_TOTAL_KB)
        assert r["le_pct"] == pytest.approx(75.11, abs=0.1)

    def test_bram_pct(self):
        r = parse_report(self.REPORT, TI60_LE_TOTAL, TI60_BRAM_TOTAL_KB)
        assert r["bram_pct"] == pytest.approx(78.12, abs=0.1)


class TestParseReportFormatC:
    """Percentage-only lines."""

    REPORT = textwrap.dedent("""\
        Resource Utilisation
        Logic Elements    20.61 %
        Block RAM         50.00 %
        DSP               5.00 %
    """)

    def test_le_pct(self):
        r = parse_report(self.REPORT, TI60_LE_TOTAL, TI60_BRAM_TOTAL_KB)
        assert r["le_pct"] == pytest.approx(20.61, abs=0.01)

    def test_bram_pct(self):
        r = parse_report(self.REPORT, TI60_LE_TOTAL, TI60_BRAM_TOTAL_KB)
        assert r["bram_pct"] == pytest.approx(50.00, abs=0.01)


class TestParseReportFormatD:
    """Used count without percentage — falls back to total_fallback."""

    REPORT = textwrap.dedent("""\
        Resource Report
        LE Used        : 29952
        BRAM Used (KB) : 128
    """)

    def test_le_pct_uses_fallback(self):
        r = parse_report(self.REPORT, 59904, 256)
        assert r["le_pct"] == pytest.approx(50.0, abs=0.1)

    def test_bram_pct_uses_fallback(self):
        r = parse_report(self.REPORT, 59904, 256)
        assert r["bram_pct"] == pytest.approx(50.0, abs=0.1)


class TestParseReportEmpty:
    def test_empty_returns_none(self):
        r = parse_report("", TI60_LE_TOTAL, TI60_BRAM_TOTAL_KB)
        assert r["le_pct"] is None
        assert r["bram_pct"] is None

    def test_irrelevant_lines_returns_none(self):
        r = parse_report("Timing closure achieved.\nSlack: 0.5 ns\n", TI60_LE_TOTAL, TI60_BRAM_TOTAL_KB)
        assert r["le_pct"] is None
        assert r["bram_pct"] is None


# ---------------------------------------------------------------------------
# check() integration tests (uses tmp_path fixture)
# ---------------------------------------------------------------------------

def _write_report(tmp_path: Path, content: str, filename: str = "church_soc_cm.map.rpt") -> Path:
    rpt = tmp_path / filename
    rpt.write_text(content)
    return rpt


PASSING_REPORT = textwrap.dedent("""\
    Logic Element (LE)   : 30000 / 59904 (50.08%)
    Block RAM (BRAM)     : 128 / 256 (50.00%)
""")

FAILING_LE_REPORT = textwrap.dedent("""\
    Logic Element (LE)   : 55000 / 59904 (91.81%)
    Block RAM (BRAM)     : 128 / 256 (50.00%)
""")

FAILING_BRAM_REPORT = textwrap.dedent("""\
    Logic Element (LE)   : 30000 / 59904 (50.08%)
    Block RAM (BRAM)     : 235 / 256 (91.80%)
""")

BOTH_FAILING_REPORT = textwrap.dedent("""\
    Logic Element (LE)   : 55000 / 59904 (91.81%)
    Block RAM (BRAM)     : 235 / 256 (91.80%)
""")


class TestCheckPassFail:
    def test_pass_both_resources(self, tmp_path):
        _write_report(tmp_path, PASSING_REPORT)
        rc = check(tmp_path, None, 90, 90, TI60_LE_TOTAL, TI60_BRAM_TOTAL_KB, False)
        assert rc == 0

    def test_fail_le_exceeded(self, tmp_path):
        _write_report(tmp_path, FAILING_LE_REPORT)
        rc = check(tmp_path, None, 90, 90, TI60_LE_TOTAL, TI60_BRAM_TOTAL_KB, False)
        assert rc == 1

    def test_fail_bram_exceeded(self, tmp_path):
        _write_report(tmp_path, FAILING_BRAM_REPORT)
        rc = check(tmp_path, None, 90, 90, TI60_LE_TOTAL, TI60_BRAM_TOTAL_KB, False)
        assert rc == 1

    def test_fail_both_exceeded(self, tmp_path):
        _write_report(tmp_path, BOTH_FAILING_REPORT)
        rc = check(tmp_path, None, 90, 90, TI60_LE_TOTAL, TI60_BRAM_TOTAL_KB, False)
        assert rc == 1

    def test_custom_threshold_pass(self, tmp_path):
        _write_report(tmp_path, FAILING_LE_REPORT)
        rc = check(tmp_path, None, 95, 95, TI60_LE_TOTAL, TI60_BRAM_TOTAL_KB, False)
        assert rc == 0

    def test_exactly_at_threshold_passes(self, tmp_path):
        report = "Logic Element (LE) : 53914 / 59904 (90.00%)\nBlock RAM : 230 / 256 (89.84%)\n"
        _write_report(tmp_path, report)
        rc = check(tmp_path, None, 90, 90, TI60_LE_TOTAL, TI60_BRAM_TOTAL_KB, False)
        assert rc == 0


class TestCheckMissingReports:
    def test_missing_dir_returns_2(self, tmp_path):
        missing = tmp_path / "nonexistent"
        rc = check(missing, None, 90, 90, TI60_LE_TOTAL, TI60_BRAM_TOTAL_KB, False)
        assert rc == 2

    def test_missing_dir_missing_ok_returns_0(self, tmp_path):
        missing = tmp_path / "nonexistent"
        rc = check(missing, None, 90, 90, TI60_LE_TOTAL, TI60_BRAM_TOTAL_KB, True)
        assert rc == 0

    def test_empty_dir_returns_2(self, tmp_path):
        rc = check(tmp_path, None, 90, 90, TI60_LE_TOTAL, TI60_BRAM_TOTAL_KB, False)
        assert rc == 2

    def test_empty_dir_missing_ok_returns_0(self, tmp_path):
        rc = check(tmp_path, None, 90, 90, TI60_LE_TOTAL, TI60_BRAM_TOTAL_KB, True)
        assert rc == 0


class TestCheckUnparseable:
    def test_unparseable_report_returns_3(self, tmp_path):
        _write_report(tmp_path, "Timing closure achieved.\nSlack: 0.5 ns\n")
        rc = check(tmp_path, None, 90, 90, TI60_LE_TOTAL, TI60_BRAM_TOTAL_KB, False)
        assert rc == 3

    def test_unparseable_missing_ok_returns_0(self, tmp_path):
        _write_report(tmp_path, "Timing closure achieved.\nSlack: 0.5 ns\n")
        rc = check(tmp_path, None, 90, 90, TI60_LE_TOTAL, TI60_BRAM_TOTAL_KB, True)
        assert rc == 0


class TestCheckReportFile:
    def test_explicit_report_file_pass(self, tmp_path):
        rpt = _write_report(tmp_path, PASSING_REPORT, "my_report.rpt")
        rc = check(tmp_path, rpt, 90, 90, TI60_LE_TOTAL, TI60_BRAM_TOTAL_KB, False)
        assert rc == 0

    def test_explicit_report_file_missing_returns_2(self, tmp_path):
        rpt = tmp_path / "missing.rpt"
        rc = check(tmp_path, rpt, 90, 90, TI60_LE_TOTAL, TI60_BRAM_TOTAL_KB, False)
        assert rc == 2


# ---------------------------------------------------------------------------
# CLI (main) wiring tests
# ---------------------------------------------------------------------------

class TestCLI:
    def test_missing_ok_flag(self, tmp_path):
        rc = main([
            "--report-dir", str(tmp_path / "nonexistent"),
            "--missing-ok",
        ])
        assert rc == 0

    def test_threshold_flags(self, tmp_path):
        _write_report(tmp_path, FAILING_LE_REPORT)
        rc = main([
            "--report-dir", str(tmp_path),
            "--le-threshold", "95",
            "--bram-threshold", "95",
        ])
        assert rc == 0

    def test_report_file_flag(self, tmp_path):
        rpt = _write_report(tmp_path, PASSING_REPORT)
        rc = main(["--report-file", str(rpt)])
        assert rc == 0

    def test_fail_via_cli(self, tmp_path):
        _write_report(tmp_path, FAILING_LE_REPORT)
        rc = main(["--report-dir", str(tmp_path)])
        assert rc == 1
