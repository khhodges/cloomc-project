"""
tests/server/test_bridge_baud.py

Verifies that local_bridge.py /status returns the baud rate that was
actually used when /connect opened the port, not the startup default.

The test spins up the real BridgeHandler on a free port and mocks
serial.Serial so no hardware is required.
"""

import importlib
import json
import socket
import sys
import threading
import unittest
from http.server import HTTPServer
from unittest.mock import MagicMock, patch
import os

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _free_port() -> int:
    """Return an available localhost TCP port."""
    with socket.socket() as s:
        s.bind(('127.0.0.1', 0))
        return s.getsockname()[1]


def _get(port, path):
    import urllib.request
    with urllib.request.urlopen(f'http://127.0.0.1:{port}{path}', timeout=5) as r:
        return json.loads(r.read())


def _post(port, path, body):
    import urllib.request
    data = json.dumps(body).encode()
    req = urllib.request.Request(
        f'http://127.0.0.1:{port}{path}',
        data=data,
        headers={'Content-Type': 'application/json'},
        method='POST',
    )
    with urllib.request.urlopen(req, timeout=5) as r:
        return json.loads(r.read())


# ---------------------------------------------------------------------------
# Fixture: isolated bridge module + live HTTP server
# ---------------------------------------------------------------------------

class BridgeFixture:
    """
    Loads local_bridge as a fresh module (to reset all globals), patches
    serial.Serial, and starts a real HTTPServer on a free port.
    """

    def __init__(self, startup_baud=115200):
        self.port = _free_port()
        self.startup_baud = startup_baud
        self._server = None
        self._thread = None
        self._mod = None
        self._serial_patch = None

    def start(self):
        # Remove any cached copy so we get fresh module-level globals
        for key in list(sys.modules.keys()):
            if 'local_bridge' in key:
                del sys.modules[key]

        # Patch sys.argv so the module parses our chosen baud as its default
        fake_argv = ['local_bridge.py', '/dev/ttyUSB1', str(self.startup_baud), str(self.port)]

        # Mock serial.Serial globally before import so the import-time check passes
        mock_serial_mod = MagicMock()
        mock_serial_instance = MagicMock()
        mock_serial_instance.is_open = True
        mock_serial_instance.port = '/dev/ttyUSB1'
        mock_serial_mod.Serial.return_value = mock_serial_instance

        root = os.path.normpath(os.path.join(os.path.dirname(__file__), '..', '..'))
        if root not in sys.path:
            sys.path.insert(0, root)

        with patch.dict('sys.modules', {'serial': mock_serial_mod}), \
             patch('sys.argv', fake_argv):
            import server.local_bridge as mod
            self._mod = mod

        # Patch serial in the already-imported module namespace too
        self._serial_patch = patch.object(mod, 'serial', mock_serial_mod)
        self._serial_patch.start()
        self._mock_serial_cls = mock_serial_mod.Serial
        self._mock_serial_instance = mock_serial_instance

        # Start the HTTP server using the handler from the loaded module
        self._server = HTTPServer(('127.0.0.1', self.port), mod.Handler)
        self._thread = threading.Thread(target=self._server.serve_forever, daemon=True)
        self._thread.start()

    def stop(self):
        if self._serial_patch:
            self._serial_patch.stop()
        if self._server:
            self._server.shutdown()

    def reset_globals(self):
        """Return the module to its no-connection state."""
        self._mod._ser = None
        self._mod._active_baud = self._mod.BAUD
        self._mod._reader_running = False


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

class TestBridgeBaudReporting(unittest.TestCase):

    def setUp(self):
        self.fx = BridgeFixture(startup_baud=115200)
        self.fx.start()
        self.port = self.fx.port
        self.mod = self.fx._mod

    def tearDown(self):
        self.fx.stop()
        for key in list(sys.modules.keys()):
            if 'local_bridge' in key:
                del sys.modules[key]

    # -- /status before any /connect ----------------------------------------

    def test_status_before_connect_returns_startup_baud(self):
        """Before any /connect call, /status must return the startup baud."""
        self.fx.reset_globals()
        st = _get(self.port, '/status')
        self.assertEqual(st['baud'], 115200)

    # -- /status after /connect with default baud ---------------------------

    def test_status_after_connect_default_baud(self):
        """After /connect with the default baud, /status reports 115200."""
        self.fx.reset_globals()
        r = _post(self.port, '/connect', {'port': '/dev/ttyUSB1', 'baud': 115200})
        self.assertTrue(r.get('ok'), f'connect failed: {r}')
        st = _get(self.port, '/status')
        self.assertEqual(st['baud'], 115200)

    # -- /status after /connect with a non-default baud ---------------------

    def test_status_after_connect_custom_baud(self):
        """
        Core regression: after /connect with baud=9600, /status must report
        9600, not the startup default of 115200.
        """
        self.fx.reset_globals()
        r = _post(self.port, '/connect', {'port': '/dev/ttyUSB1', 'baud': 9600})
        self.assertTrue(r.get('ok'), f'connect failed: {r}')
        st = _get(self.port, '/status')
        self.assertEqual(
            st['baud'], 9600,
            f"Expected baud=9600 in /status after connecting at 9600, got {st['baud']}",
        )

    # -- auto-connect path (port was closed, IDE sends its BAUD constant) ---

    def test_auto_connect_path_baud_matches_connect_request(self):
        """
        Simulates the auto-connect path in app-ti60-connect.js:
          1. /status → open:false
          2. POST /connect {baud: 115200}
          3. /status (re-fetch) → baud must equal what was sent in step 2
        The step-log line uses status.baud from the re-fetched response.
        """
        # Start with closed serial
        self.fx.reset_globals()
        self.mod._ser = None

        # Step 1: first /status call (port is closed)
        st1 = _get(self.port, '/status')
        self.assertFalse(st1['open'])

        # Step 2: IDE auto-opens with its client-side BAUD constant (115200)
        client_baud = 115200
        r = _post(self.port, '/connect', {'port': '/dev/ttyUSB1', 'baud': client_baud})
        self.assertTrue(r.get('ok'))

        # Step 3: re-fetch /status (this is what the JS assigns back to `status`)
        st2 = _get(self.port, '/status')
        self.assertEqual(
            st2['baud'], client_baud,
            f"Re-fetched /status should report baud={client_baud}, got {st2['baud']}",
        )

    # -- baud key is always present -----------------------------------------

    def test_status_always_includes_baud_key(self):
        """/status must always include the 'baud' key so the JS fallback is never needed."""
        self.fx.reset_globals()
        st = _get(self.port, '/status')
        self.assertIn('baud', st, "/status response missing 'baud' key")


if __name__ == '__main__':
    unittest.main()
