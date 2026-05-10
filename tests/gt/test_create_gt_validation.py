"""Tests for createGT domain-purity and single-Church-perm guards.

Task #952: Enforce domain purity at GT creation.

Three test cases:
1. createGT with domain-impure perms {X:1,E:1} must throw containing "domain-impure".
2. createGT with multi-Church perms {L:1,E:1} must throw containing "single".
3. Normal boot sequence completes without triggering either validation error.
"""
import json
import os
import subprocess
import sys

import pytest

ROOT    = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
HARNESS = os.path.join(ROOT, 'tests', 'gt', 'sim_gt_harness.js')


def _run_harness(tests):
    """Run sim_gt_harness.js with the given test list; return parsed results."""
    payload = json.dumps({'tests': tests})
    proc = subprocess.run(
        ['node', HARNESS],
        input=payload.encode('utf-8'),
        capture_output=True,
        timeout=30,
        cwd=ROOT,
    )
    if proc.returncode != 0:
        raise RuntimeError(
            f'sim_gt_harness.js exited {proc.returncode}\n'
            f'stderr:\n{proc.stderr.decode("utf-8", errors="replace")}'
        )
    out = proc.stdout.decode('utf-8', errors='replace').strip()
    try:
        return json.loads(out)
    except json.JSONDecodeError as e:
        raise RuntimeError(
            f'sim_gt_harness.js produced non-JSON output: {e}\nstdout:\n{out}'
        )


def test_domain_impure_perms_throw():
    """createGT({X:1,E:1}) must throw with a message containing 'domain-impure'."""
    results = _run_harness([{
        'name': 'domain_impure',
        'action': 'createGT',
        'args': [0, 20, {'R': 0, 'W': 0, 'X': 1, 'L': 0, 'S': 0, 'E': 1, 'B': 0}, 1],
    }])
    r = results[0]
    assert r['threw'], (
        'Expected createGT({X:1,E:1}) to throw, but it returned without error'
    )
    assert 'domain-impure' in (r['errorMessage'] or ''), (
        f"Expected error message to contain 'domain-impure', got: {r['errorMessage']}"
    )


def test_multi_church_perms_throw():
    """createGT({L:1,E:1}) must throw with a message containing 'single'."""
    results = _run_harness([{
        'name': 'multi_church',
        'action': 'createGT',
        'args': [0, 37, {'R': 0, 'W': 0, 'X': 0, 'L': 1, 'S': 0, 'E': 1, 'B': 0}, 1],
    }])
    r = results[0]
    assert r['threw'], (
        'Expected createGT({L:1,E:1}) to throw, but it returned without error'
    )
    assert 'single' in (r['errorMessage'] or ''), (
        f"Expected error message to contain 'single', got: {r['errorMessage']}"
    )


def test_boot_completes_without_gt_validation_errors():
    """The boot sequence must complete cleanly with no createGT validation errors."""
    results = _run_harness([{
        'name': 'boot',
        'action': 'boot',
    }])
    r = results[0]
    assert r['bootComplete'], (
        f"Expected boot to complete, but bootComplete={r['bootComplete']}, "
        f"halted={r['halted']}, faultLog={r['faultLog']}"
    )
    assert not r['gtErrors'], (
        f'Boot triggered createGT validation errors: {r["gtErrors"]}'
    )
