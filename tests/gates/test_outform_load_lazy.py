"""Simulator-level regression test for Mode 2 (Outform) lazy load via LOAD.

Exercises the JavaScript simulator via Node.js to verify:

  1. LOAD CRn where the c-list slot holds an Outform GT (type=0b10) triggers
     the lazy loader (Mode 2), installs the lump, promotes the destination CR's
     GT to Inform (type=0b01), and does not fault.
     (Task #462)

  2. LOAD CRn where the c-list slot holds an ordinary Inform GT does NOT
     trigger the Mode 2 path — the LOAD proceeds normally without calling
     _dispatchLoaderLoad.
     (Task #462)
"""

import os
import subprocess

import pytest

ROOT    = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
HARNESS = os.path.join(ROOT, "tests", "gates", "sim_outform_load_lazy.js")


def _node_available():
    try:
        subprocess.run(["node", "--version"], capture_output=True, check=True)
        return True
    except (FileNotFoundError, subprocess.CalledProcessError):
        return False


@pytest.mark.skipif(not _node_available(), reason="Node.js not available")
def test_outform_load_lazy():
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
            f"sim_outform_load_lazy.js exited with code {result.returncode}\n"
            + "\n".join(lines)
        )

    assert "[PASS]" in stdout, f"No PASS markers in output:\n{stdout}"
