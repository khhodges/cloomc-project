"""Python wrapper: runs the Node.js pet name audit unit tests."""
import subprocess
import pytest


def test_pet_name_audit_node():
    result = subprocess.run(
        ['node', 'tests/simulator/save_lump_pet_name_audit_test.js'],
        capture_output=True, text=True
    )
    print(result.stdout)
    if result.stderr:
        print(result.stderr)
    assert result.returncode == 0, (
        'Node.js pet name audit test failed:\n' + result.stdout + result.stderr
    )
