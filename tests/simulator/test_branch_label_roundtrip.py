"""Round-trip regression tests for BRANCH label emission and re-assembly (Task #472).

Verifies that a BRANCH word with a known signed offset:
  1. Is decompiled to assembly text containing the correct "Ln:" label
     definition and "BRANCHcond  Ln" reference (using the same logic as
     editCRCodeInEditor in simulator/app-cr-display.js).
  2. Re-assembles back to the exact same 32-bit word via ChurchAssembler.

All assertions live in the JS harness (tests/simulator/sim_branch_label_roundtrip.js)
which exits 0 on success.  This Python wrapper runs it under pytest.
"""

import os
import subprocess

import pytest

ROOT    = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
HARNESS = os.path.join(ROOT, "tests", "simulator", "sim_branch_label_roundtrip.js")


def _node_available():
    try:
        subprocess.run(["node", "--version"], capture_output=True, check=True)
        return True
    except (FileNotFoundError, subprocess.CalledProcessError):
        return False


@pytest.mark.skipif(not _node_available(), reason="Node.js not available")
def test_branch_label_roundtrip_harness():
    """Run the JS harness and assert all BRANCH label round-trip checks pass."""
    result = subprocess.run(
        ["node", HARNESS],
        capture_output=True,
        text=True,
        cwd=ROOT,
        timeout=30,
    )
    stdout = result.stdout.strip()
    stderr = result.stderr.strip()

    if result.returncode != 0 or stderr:
        lines = []
        if stdout:
            lines.append("stdout:\n" + stdout)
        if stderr:
            lines.append("stderr:\n" + stderr)
        pytest.fail(
            f"sim_branch_label_roundtrip.js exited with code {result.returncode}\n"
            + "\n".join(lines)
        )

    assert "[PASS]" in stdout, f"No [PASS] markers in harness output:\n{stdout}"

    pass_count = stdout.count("[PASS]")
    assert pass_count >= 7, (
        f"Expected at least 7 [PASS] markers, got {pass_count}:\n{stdout}"
    )
