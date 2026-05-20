"""Simulator regression test for Task #1495 — RUN on LUMP boots wrong slot.

Exercises the JavaScript simulator via Node.js to verify that when
sim.bootEntrySlot is set to a non-3 slot (e.g. 51, simulating a prior
boot-entry badge click), temporarily forcing it to 3 before loadLumpBinary
allows the load to succeed and correctly restores the original slot value.

Without the guard added in Task #1495, calling instantBoot() with
sim.bootEntrySlot=51 would cause NUC_CLIST to do an mLoad on slot 51 (which
has magic=0x0), triggering a LUMP_MAGIC fault before the lump binary is written.
"""

import os
import subprocess

import pytest

ROOT    = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
HARNESS = os.path.join(ROOT, "tests", "simulator", "sim_run_lump_boots_slot3.js")


def _node_available():
    try:
        subprocess.run(["node", "--version"], capture_output=True, check=True)
        return True
    except (FileNotFoundError, subprocess.CalledProcessError):
        return False


@pytest.mark.skipif(not _node_available(), reason="Node.js not available")
def test_run_lump_boots_slot3():
    """Run the JS harness; fail if it exits non-zero or writes to stderr."""
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
            f"sim_run_lump_boots_slot3.js exited with code "
            f"{result.returncode}\n" + "\n".join(lines)
        )

    assert "[PASS]" in stdout, f"No PASS markers in output:\n{stdout}"
