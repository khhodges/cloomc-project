"""End-to-end integration test for the Hello-Mum flow on a real boot image.

Verifies the full causal chain that the unit tests cannot cover in isolation:

    Board boot image
        → ChurchSimulator (real AbstractionRegistry + SystemAbstractions)
        → boot state machine (_bootStep loop)
        → Navana.Init (NS[5].Init)
        → Keystone.Init wires Tunnel E-GT into c-list slot 0
        → Keystone.Connect(identity_word) issues MumGT into c-list slot 1
        → Keystone.Hello() dispatches through Tunnel.Call
        → Tunnel.Call POSTs to the live Flask bridge at /mum/hello   ← bridge call
        → bridge returns { ok, result: 0x48454C4C, tunnel: 'online' }
        → GREET_RESPONSE 0x48454C4C ('HELL') propagated back to Keystone.Hello()

The harness (sim_hello_mum_flow.js) rebinds Tunnel.Call at runtime to make a
real HTTP POST to the bridge server started here in the test process, rather
than using the in-process constant from system_abstractions.js.  This makes the
integration observable:

  - bridgeHit == True  → the HTTP call actually happened
  - bridgeStatus == 200 → the server accepted the request
  - bridgeResult == GREET_RESPONSE → the correct value came back through the wire
  - greetResult == GREET_RESPONSE  → Keystone.Hello() returned that value

Failure here catches wiring problems that pass the isolated unit tests but break
when all layers are combined on a real boot image with a live server in the loop.

Relevant files:
  - tests/boot/sim_hello_mum_flow.js  (JS harness / bridge integration)
  - simulator/system_abstractions.js  (_initKeystone, ~line 2665)
  - server/app.py                     (/mum/hello endpoint, ~line 4375)
"""
import base64
import json
import os
import socket
import subprocess
import sys
import threading

import pytest

ROOT      = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
LUMPS_DIR = os.path.join(ROOT, "server", "lumps")
sys.path.insert(0, ROOT)

from server.boot_image import generate_boot_image  # noqa: E402

_HARNESS = os.path.join(os.path.dirname(__file__), "sim_hello_mum_flow.js")

GREET_RESPONSE = 0x48454C4C  # 'HELL' in big-endian ASCII

_BOOT_CFG = {
    "step1": {
        "totalNamespaceWords": 16384,
        "namespaceLumpWords":     64,
        "threadLumpWords":       256,
    },
}

# Protocol-tag-1 identity word that Keystone.Connect will accept.
# Mirrors the default used inside the harness so the two are always in sync.
_VALID_IDENTITY = (0x1 << 28) | 0x0CEEFFE


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _free_port():
    """Return an unused TCP port on localhost."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(('127.0.0.1', 0))
        return s.getsockname()[1]


def _start_flask_thread(port):
    """Start the Flask app in a background thread on *port*.

    Returns the thread (already started).  The server is ready once it begins
    accepting connections, which the caller is expected to verify with a brief
    retry loop before running the harness.
    """
    from server.app import app  # noqa: E402 — deferred to avoid side-effects

    from werkzeug.serving import make_server

    srv = make_server('127.0.0.1', port, app)

    t = threading.Thread(target=srv.serve_forever, daemon=True)
    t.start()
    return t, srv


def _wait_for_port(port, timeout=5.0):
    """Block until localhost:port accepts a TCP connection or timeout expires."""
    import time
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            with socket.create_connection(('127.0.0.1', port), timeout=0.1):
                return True
        except OSError:
            time.sleep(0.05)
    return False


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture(scope="module")
def boot_image_b64():
    """Generate a canonical boot image and return its base64 encoding."""
    img_bytes = generate_boot_image(_BOOT_CFG, LUMPS_DIR)
    return base64.b64encode(img_bytes).decode("ascii")


@pytest.fixture(scope="module")
def bridge_server():
    """Start the Flask server on a random port and yield the bridge URL.

    The server thread is daemonised so it is cleaned up automatically when the
    test process exits.  We call srv.shutdown() explicitly in the finaliser so
    that the port is released before the next test module starts.
    """
    port = _free_port()
    _t, srv = _start_flask_thread(port)
    ready = _wait_for_port(port)
    assert ready, f"Flask bridge server did not start on port {port} within 5 s"
    url = f"http://127.0.0.1:{port}"
    yield url
    srv.shutdown()


@pytest.fixture(scope="module")
def harness_result(boot_image_b64, bridge_server):
    """Run sim_hello_mum_flow.js with a real boot image and a live bridge URL.

    This is the primary integration fixture: the JS harness boots the simulator,
    runs the full Init→Connect→Hello chain, and makes a real HTTP POST to the
    Flask bridge server for Tunnel.Call.

    Returns the parsed JSON dict from the harness stdout.  The fixture fails
    immediately if the harness crashes or produces unparseable output.
    """
    envelope = json.dumps({
        "imageBase64":  boot_image_b64,
        "config":       _BOOT_CFG,
        "identityWord": _VALID_IDENTITY,
        "bridgeUrl":    bridge_server,
    }).encode("utf-8")

    proc = subprocess.run(
        ["node", _HARNESS],
        input=envelope,
        capture_output=True,
        timeout=30,
        cwd=ROOT,
    )
    stdout = proc.stdout.decode("utf-8", errors="replace").strip()
    stderr = proc.stderr.decode("utf-8", errors="replace").strip()

    assert proc.returncode == 0, (
        f"sim_hello_mum_flow.js exited {proc.returncode}.\n"
        f"stdout: {stdout}\nstderr: {stderr}"
    )

    try:
        return json.loads(stdout)
    except json.JSONDecodeError as exc:
        pytest.fail(
            f"sim_hello_mum_flow.js produced non-JSON output: {exc}\n"
            f"stdout: {stdout}"
        )


# ---------------------------------------------------------------------------
# Boot-side assertions (simulator)
# ---------------------------------------------------------------------------

def test_harness_boot_image_loaded(harness_result):
    """Boot image must load successfully into the live simulator."""
    assert harness_result.get("loaded") is True, (
        "loadBootImage() returned False — boot image was not accepted by the simulator.\n"
        f"faultCount={harness_result.get('faultCount')}"
    )


def test_harness_boot_completes(harness_result):
    """Boot state machine must run to completion before Hello-Mum can proceed."""
    assert harness_result.get("bootComplete") is True, (
        "Boot state machine did not complete.\n"
        f"faultCount={harness_result.get('faultCount')}, "
        f"message={harness_result.get('message')!r}"
    )


def test_harness_navana_init_succeeds(harness_result):
    """Navana.Init (NS[5].Init) must succeed, triggering Keystone.Init."""
    assert harness_result.get("navanaOk") is True, (
        "Navana.Init dispatch failed — Keystone cannot have been wired.\n"
        f"message={harness_result.get('message')!r}"
    )


def test_harness_keystone_connect_succeeds(harness_result):
    """Keystone.Connect(identity_word) must accept the identity and issue a MumGT."""
    assert harness_result.get("connectOk") is True, (
        "Keystone.Connect() did not return ok=1 — MumGT was not placed in c-list slot 1.\n"
        f"connectResult={harness_result.get('connectResult')!r}, "
        f"message={harness_result.get('message')!r}"
    )


def test_harness_tunnel_slot0_is_wired(harness_result):
    """After Navana.Init, Keystone c-list slot 0 must hold the Tunnel E-GT (NS[31])."""
    slot0    = harness_result.get("slot0", 0)
    tunnelNS = harness_result.get("tunnelNS", -1)
    eBitSet  = harness_result.get("eBitSet", False)

    assert slot0 != 0, (
        "Keystone c-list slot 0 is NULL GT after Navana.Init — Tunnel not wired."
    )
    assert tunnelNS == 31, (
        f"c-list slot 0 NS index = {tunnelNS}, expected 31 (Tunnel).\n"
        f"slot0=0x{slot0:08X}"
    )
    assert eBitSet is True, (
        f"Tunnel GT in c-list slot 0 has E-bit clear: 0x{slot0:08X}"
    )


# ---------------------------------------------------------------------------
# Bridge-side assertions (HTTP integration)
# ---------------------------------------------------------------------------

def test_bridge_was_called(harness_result):
    """Tunnel.Call must have made a real HTTP POST to /mum/hello.

    This is the key causal-integration assertion: it fails if Tunnel.Call
    returns the response constant directly without going through the bridge,
    which would mean wiring breakage between the simulator and the IDE server
    remains invisible to the test suite.
    """
    assert harness_result.get("bridgeHit") is True, (
        "bridgeHit is False — Tunnel.Call did not POST to /mum/hello.\n"
        "Either the bridge URL was not wired up or Tunnel.Call short-circuited.\n"
        f"message={harness_result.get('message')!r}"
    )


def test_bridge_returned_http_200(harness_result):
    """The /mum/hello bridge endpoint must have responded with HTTP 200."""
    status = harness_result.get("bridgeStatus")
    assert status == 200, (
        f"/mum/hello returned HTTP {status}, expected 200.\n"
        f"message={harness_result.get('message')!r}"
    )


def test_bridge_returned_greet_response(harness_result):
    """The /mum/hello response body must carry result=0x48454C4C ('HELL')."""
    br = harness_result.get("bridgeResult")
    assert br == GREET_RESPONSE, (
        f"/mum/hello bridgeResult=0x{(br or 0):08X}, "
        f"expected 0x{GREET_RESPONSE:08X} ('HELL').\n"
        f"message={harness_result.get('message')!r}"
    )


# ---------------------------------------------------------------------------
# Primary end-to-end assertion
# ---------------------------------------------------------------------------

def test_hello_mum_greet_response_propagated_through_bridge(harness_result):
    """Keystone.Hello() must return the GREET_RESPONSE received from the bridge.

    This is the primary end-to-end assertion for the Hello-Mum flow.  It
    confirms that:
      1. The simulator's Hello() call triggered a real HTTP POST to /mum/hello.
      2. The response value from the server (0x48454C4C) was propagated back
         through Tunnel.Call into Keystone.Hello(), which returned it as its
         own result.

    A failure here catches any wiring regression between the JS abstraction
    layer and the Flask IDE bridge that the isolated unit tests cannot see.
    """
    assert harness_result.get("bridgeHit") is True, (
        "Bridge was not hit — causal chain is broken before the assertion."
    )
    greet  = harness_result.get("greetResult", 0)
    bridge = harness_result.get("bridgeResult", -1)
    assert greet == GREET_RESPONSE, (
        f"Keystone.Hello() returned 0x{greet:08X}, "
        f"expected 0x{GREET_RESPONSE:08X} ('HELL').\n"
        f"bridgeResult=0x{(bridge or 0):08X}, "
        f"message={harness_result.get('message')!r}"
    )
    assert greet == bridge, (
        f"Keystone.Hello() result 0x{greet:08X} != bridge response 0x{(bridge or 0):08X} — "
        "the value was not propagated correctly from /mum/hello back through Tunnel.Call."
    )
