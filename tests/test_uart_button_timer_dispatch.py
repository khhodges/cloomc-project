"""Simulator dispatch tests for UART, Button, and Timer Abstract GTs (Task #431).

These tests exercise the Abstract Manager (_dispatchAbstractDread /
_dispatchAbstractDwrite) in the JavaScript simulator via the headless
harness tests/sim_dispatch_abstract_io.js.

Covered:
  - DREAD on UART Abstract GT reads uartRegs[device_data]
  - DWRITE on UART Abstract GT writes uartRegs[device_data]
  - UART out-of-range device_data faults INVALID_OP
  - DREAD on Button Abstract GT reads buttonState
  - Button DWRITE with W-perm absent faults PERM_W
  - DREAD on Timer Abstract GT reads timerRegs[device_data]
  - DWRITE on Timer Abstract GT writes timerRegs[device_data]
  - Timer out-of-range device_data faults INVALID_OP
"""

import json
import os
import subprocess
import sys

import pytest

ROOT    = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
HARNESS = os.path.join(ROOT, "tests", "sim_dispatch_abstract_io.js")

DEVICE_CLASS_UART   = 0x02
DEVICE_CLASS_BUTTON = 0x03
DEVICE_CLASS_TIMER  = 0x04


def _run(ops):
    """Send ops to the JS harness and return the parsed result list."""
    payload = json.dumps({"ops": ops})
    result  = subprocess.run(
        ["node", HARNESS],
        input=payload, capture_output=True, text=True, timeout=15,
    )
    assert result.returncode == 0, (
        f"Harness exited {result.returncode}\nstderr: {result.stderr}"
    )
    return json.loads(result.stdout)


# ── UART ─────────────────────────────────────────────────────────────────────

def test_dread_uart_tx_register():
    """DREAD on UART GT with device_data=0 reads uartRegs[0] (TX)."""
    results = _run([{
        "op": "DREAD",
        "device_class": DEVICE_CLASS_UART, "device_data": 0,
        "r_perm": 1, "w_perm": 1,
        "seed_uart": [42, 1, 0],
    }])
    assert results[0]["ok"], f"Expected success, got: {results[0]}"
    assert results[0]["dr_value"] == 42


def test_dread_uart_status_register():
    """DREAD on UART GT with device_data=1 reads uartRegs[1] (STATUS)."""
    results = _run([{
        "op": "DREAD",
        "device_class": DEVICE_CLASS_UART, "device_data": 1,
        "r_perm": 1, "w_perm": 1,
        "seed_uart": [0, 99, 0],
    }])
    assert results[0]["ok"], f"Expected success, got: {results[0]}"
    assert results[0]["dr_value"] == 99


def test_dread_uart_rx_register():
    """DREAD on UART GT with device_data=2 reads uartRegs[2] (RX)."""
    results = _run([{
        "op": "DREAD",
        "device_class": DEVICE_CLASS_UART, "device_data": 2,
        "r_perm": 1, "w_perm": 1,
        "seed_uart": [0, 0, 55],
    }])
    assert results[0]["ok"], f"Expected success, got: {results[0]}"
    assert results[0]["dr_value"] == 55


def test_dwrite_uart_tx_register():
    """DWRITE on UART GT with device_data=0 writes uartRegs[0] (TX)."""
    results = _run([{
        "op": "DWRITE",
        "device_class": DEVICE_CLASS_UART, "device_data": 0,
        "r_perm": 1, "w_perm": 1,
        "dr_value": 0xAB,
    }])
    assert results[0]["ok"], f"Expected success, got: {results[0]}"
    assert results[0]["uart_regs"][0] == 0xAB


def test_dread_uart_out_of_range_faults():
    """DREAD with UART device_data=3 (out of range) must fault INVALID_OP."""
    results = _run([{
        "op": "DREAD",
        "device_class": DEVICE_CLASS_UART, "device_data": 3,
        "r_perm": 1, "w_perm": 1,
    }])
    assert not results[0]["ok"], "Expected fault for out-of-range UART reg"
    assert results[0]["fault"] == "INVALID_OP"


def test_dread_uart_no_r_perm_faults():
    """DREAD on UART GT without R permission must fault PERM_R."""
    results = _run([{
        "op": "DREAD",
        "device_class": DEVICE_CLASS_UART, "device_data": 0,
        "r_perm": 0, "w_perm": 1,
    }])
    assert not results[0]["ok"], "Expected PERM_R fault"
    assert results[0]["fault"] == "PERM_R"


# ── Button ────────────────────────────────────────────────────────────────────

def test_dread_button_state():
    """DREAD on Button GT reads buttonState."""
    results = _run([{
        "op": "DREAD",
        "device_class": DEVICE_CLASS_BUTTON, "device_data": 0,
        "r_perm": 1, "w_perm": 0,
        "seed_button": 0b00000101,
    }])
    assert results[0]["ok"], f"Expected success, got: {results[0]}"
    assert results[0]["dr_value"] == 0b00000101


def test_dwrite_button_without_w_perm_faults():
    """DWRITE on Button GT (no W perm) must fault PERM_W."""
    results = _run([{
        "op": "DWRITE",
        "device_class": DEVICE_CLASS_BUTTON, "device_data": 0,
        "r_perm": 1, "w_perm": 0,
        "dr_value": 1,
    }])
    assert not results[0]["ok"], "Expected PERM_W fault"
    assert results[0]["fault"] == "PERM_W"


def test_dread_button_out_of_range_faults():
    """DREAD with Button device_data=1 (out of range) must fault INVALID_OP."""
    results = _run([{
        "op": "DREAD",
        "device_class": DEVICE_CLASS_BUTTON, "device_data": 1,
        "r_perm": 1, "w_perm": 1,
    }])
    assert not results[0]["ok"], "Expected fault for out-of-range Button reg"
    assert results[0]["fault"] == "INVALID_OP"


# ── Timer ─────────────────────────────────────────────────────────────────────

def test_dread_timer_ticks_lo():
    """DREAD on Timer GT with device_data=0 reads timerRegs[0] (TICKS_LO)."""
    results = _run([{
        "op": "DREAD",
        "device_class": DEVICE_CLASS_TIMER, "device_data": 0,
        "r_perm": 1, "w_perm": 1,
        "seed_timer": [123456, 0, 0, 0, 0],
    }])
    assert results[0]["ok"], f"Expected success, got: {results[0]}"
    assert results[0]["dr_value"] == 123456


def test_dread_timer_ctl_register():
    """DREAD on Timer GT with device_data=4 reads timerRegs[4] (CTL)."""
    results = _run([{
        "op": "DREAD",
        "device_class": DEVICE_CLASS_TIMER, "device_data": 4,
        "r_perm": 1, "w_perm": 1,
        "seed_timer": [0, 0, 0, 0, 0xFF],
    }])
    assert results[0]["ok"], f"Expected success, got: {results[0]}"
    assert results[0]["dr_value"] == 0xFF


def test_dwrite_timer_alarm_cmp():
    """DWRITE on Timer GT with device_data=3 writes timerRegs[3] (ALARM_CMP)."""
    results = _run([{
        "op": "DWRITE",
        "device_class": DEVICE_CLASS_TIMER, "device_data": 3,
        "r_perm": 1, "w_perm": 1,
        "dr_value": 0xDEAD,
    }])
    assert results[0]["ok"], f"Expected success, got: {results[0]}"
    assert results[0]["timer_regs"][3] == 0xDEAD


def test_dread_timer_out_of_range_faults():
    """DREAD with Timer device_data=5 (out of range) must fault INVALID_OP."""
    results = _run([{
        "op": "DREAD",
        "device_class": DEVICE_CLASS_TIMER, "device_data": 5,
        "r_perm": 1, "w_perm": 1,
    }])
    assert not results[0]["ok"], "Expected fault for out-of-range Timer reg"
    assert results[0]["fault"] == "INVALID_OP"
