"""Simulator regression test for Task #649 — fault location after lump switch.

Exercises the JavaScript simulator via Node.js to verify that, after a CALL
E-GT instruction switches CR14 from the boot-entry lump to a different lump
(SlideRule, NS slot 16), any fault inside the new lump is correctly attributed
to that lump — not to Boot.NS or the original boot-entry lump.

The test uses the raw crSnapshot[14] path (crSnapshot[14].word0 & 0xFFFF for
the NS index, crSnapshot[14].word1 for the lump base) because that is exactly
the path app-run.js uses to populate _nsSnapshot.label and _nsSnapshot.offset
in the fault log.

Without the fix from Task #649, the fault location would resolve through
_nsOwnerOf(physicalPC), which performs a memory-range search and can return
the boot-entry lump's label even when the actual executing lump has already
been switched — hence the "Boot.NS +NNN" regression.
"""

import os
import subprocess

import pytest

ROOT    = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
HARNESS = os.path.join(ROOT, "tests", "simulator",
                       "sim_fault_location_after_lump_switch.js")


def _node_available():
    try:
        subprocess.run(["node", "--version"], capture_output=True, check=True)
        return True
    except (FileNotFoundError, subprocess.CalledProcessError):
        return False


@pytest.mark.skipif(not _node_available(), reason="Node.js not available")
def test_fault_location_after_lump_switch():
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
            f"sim_fault_location_after_lump_switch.js exited with code "
            f"{result.returncode}\n" + "\n".join(lines)
        )

    assert "[PASS]" in stdout, f"No PASS markers in output:\n{stdout}"
