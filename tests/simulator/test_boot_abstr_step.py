"""Boot.Abstr 3-instruction step-through test (Task #655).

Verifies that the simulator correctly executes all 3 instructions of the
redesigned Boot.Abstr program (Task #651) after the boot ROM completes:

    PC=0  CHANGE AL, CR12, CR12, #1   -- reload thread; RESTORE_CALL loads CR0-CR11
    PC=1  TPERM  AL, CR0,  #E        -- check/restrict CR0 to E-permission only
    PC=2  CALL   AL, CR0,  CR0       -- enter the configured first abstraction

Without the Task #655 fix, CHANGE CR12 ran in the system-wide path which did
NOT restore CR0-CR11 from the thread caps zone (THREAD_CAPS_OFFSET), leaving
CR0 NULL so TPERM would trivially fail and CALL would fault.

The harness boots the simulator, then calls sim.step() three times and checks
that each step executes without fault and that CR0 is non-NULL after CHANGE.
"""
import json
import os
import subprocess
import sys

ROOT    = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
HARNESS = os.path.join(ROOT, 'tests', 'simulator', 'sim_boot_abstr_step.js')


def _run():
    proc = subprocess.run(
        ['node', HARNESS],
        capture_output=True,
        timeout=30,
        cwd=ROOT,
    )
    if proc.returncode != 0:
        raise RuntimeError(
            f'sim_boot_abstr_step.js exited {proc.returncode}\n'
            f'stderr:\n{proc.stderr.decode("utf-8", errors="replace")}'
        )
    out = proc.stdout.decode('utf-8', errors='replace').strip()
    try:
        return json.loads(out)
    except json.JSONDecodeError as e:
        raise RuntimeError(
            f'sim_boot_abstr_step.js produced non-JSON output: {e}\n'
            f'stdout:\n{out}'
        )


def test_boot_abstr_step_through():
    status = _run()

    assert status['bootComplete'] is True, (
        'boot ROM did not complete before step-through'
    )
    assert status['bootFaultLog'] == [], (
        'unexpected fault(s) during boot: '
        + ', '.join(f"[{f['type']}] {f['message']}" for f in status['bootFaultLog'])
    )

    assert not status['step1']['faulted'], (
        f"CHANGE CR12 (step 1) faulted: {status['step1']}"
    )
    assert status['step1']['desc'] is not None, (
        'CHANGE CR12 (step 1) returned no description (returned null)'
    )

    cr0 = status['cr0AfterChange']
    assert cr0['word0'] != 0, (
        f'CR0 is still NULL after CHANGE CR12 -- RESTORE_CALL fix not applied; '
        f'cr0={cr0}'
    )

    assert not status['step2']['faulted'], (
        f"TPERM CR0,#E (step 2) faulted: {status['step2']}"
    )
    assert status['step2']['desc'] is not None, (
        'TPERM (step 2) returned no description'
    )

    assert not status['step3']['faulted'], (
        f"CALL CR0,CR0 (step 3) faulted: {status['step3']}"
    )

    assert status['allStepsFaultFree'] is True, (
        f'not all 3 Boot.Abstr steps completed fault-free: {status}'
    )


if __name__ == '__main__':
    try:
        test_boot_abstr_step_through()
        print('PASS: test_boot_abstr_step_through')
        sys.exit(0)
    except AssertionError as e:
        print(f'FAIL: {e}')
        sys.exit(1)
