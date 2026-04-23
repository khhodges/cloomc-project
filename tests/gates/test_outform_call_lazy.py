"""Simulator-level regression tests for Mode 2 (Outform) lazy load.

Exercises the JavaScript simulator via Node.js to verify:

  1. initLazyManifest writes gtType=2 (Outform) into the NS entry word1 for
     every cold-priority slot at manifest init time, while warm entries are
     left with their existing gtType so Mode 1 restore continues to work.
     (Task #459)

  2. lazyLoad promotes the NS entry's gtType from 2→1 (Inform) after a
     successful lump install, so subsequent GT derivations from the NS entry
     carry type=1 and do not re-trigger Mode 2.
     (Task #459)

  3. CALL CRn where CRn holds an Outform GT (type=0b10) triggers the lazy
     loader (Mode 2), installs the lump, promotes the live CR's GT to
     Inform (type=0b01), and falls through to the normal Inform CALL path
     without faulting with "GT type is Outform, must be Inform or Abstract".
     (Task #459)

  4. Warm entries are NOT marked Outform by initLazyManifest — the Outform
     marking is strictly for cold slots.
     (Task #459)
"""

import os
import subprocess

import pytest

ROOT    = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
HARNESS = os.path.join(ROOT, "tests", "gates", "sim_outform_call_lazy.js")


def _node_available():
    try:
        subprocess.run(["node", "--version"], capture_output=True, check=True)
        return True
    except (FileNotFoundError, subprocess.CalledProcessError):
        return False


@pytest.mark.skipif(not _node_available(), reason="Node.js not available")
def test_outform_call_lazy():
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
            f"sim_outform_call_lazy.js exited with code {result.returncode}\n"
            + "\n".join(lines)
        )

    assert "[PASS]" in stdout, f"No PASS markers in output:\n{stdout}"
