"""Regression tests for the CR detail tab-correction logic (Task #825).

The tab-correction block in simulator/app-memory.js updateCRDetail() was
previously inlined and silently broke three tab/CR combinations.  The logic
has been extracted into simulator/tab-correction.js as the pure function
correctCRDetailTab(tab, showCode, showCList, showData).

This test drives the JS harness (tests/simulator/sim_tab_correction.js) and
verifies that every invalid (tab, capability) combination produces the correct
reset tab, and that valid tabs are not changed.

Coverage:
  - 'api' / 'code' tab on a non-code CR → 'clist', 'register', or 'lump'
  - 'register' / 'binary' tab on a code-only CR → 'code', 'clist', or 'lump'
  - 'clist' tab on a CR without a C-List → 'code', 'register', or 'lump'
  - Already-valid tabs remain unchanged
"""

import os
import subprocess

import pytest

ROOT    = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
HARNESS = os.path.join(ROOT, "tests", "simulator", "sim_tab_correction.js")


def _node_available():
    try:
        subprocess.run(["node", "--version"], capture_output=True, check=True)
        return True
    except (FileNotFoundError, subprocess.CalledProcessError):
        return False


@pytest.mark.skipif(not _node_available(), reason="Node.js not available")
def test_tab_correction_harness():
    """Run the JS harness and assert all tab-correction checks pass."""
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
            f"sim_tab_correction.js exited with code {result.returncode}\n"
            + "\n".join(lines)
        )

    assert "[PASS]" in stdout, f"No [PASS] markers in harness output:\n{stdout}"

    pass_count = stdout.count("[PASS]")
    assert pass_count == 22, (
        f"Expected exactly 22 [PASS] markers, got {pass_count}:\n{stdout}"
    )
