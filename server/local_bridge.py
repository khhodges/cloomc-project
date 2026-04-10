#!/usr/bin/env python3
"""
Church Machine FPGA Serial Bridge
===================================
Run this in your Linux terminal when WebSerial is blocked (e.g. ChromeOS).
The browser IDE talks to this script over HTTP; this script talks to the FPGA
over UART.

Usage:
    python3 server/local_bridge.py [port] [baud] [http_port]

Defaults:
    port      = /dev/ttyUSB3
    baud      = 115200
    http_port = 8766

Then in the Church Machine IDE click  "Bridge"  instead of the normal connect
button, and enter the bridge URL when prompted (default already filled in).
"""
import sys, json, time, threading
from http.server import HTTPServer, BaseHTTPRequestHandler

try:
    import serial
except ImportError:
    print("ERROR: pyserial not installed.  Run:  pip3 install pyserial")
    sys.exit(1)

_first_arg = sys.argv[1] if len(sys.argv) > 1 else ''
if _first_arg.startswith('--'):
    SERIAL_PORT = '/dev/ttyUSB1'
    BAUD = 115200
    HTTP_PORT = 8766
else:
    SERIAL_PORT = _first_arg or '/dev/ttyUSB3'
    BAUD        = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
    HTTP_PORT   = int(sys.argv[3]) if len(sys.argv) > 3 else 8766

_ser       = None
_ser_lock  = threading.Lock()
_rx_buf    = bytearray()
_rx_lock   = threading.Lock()
_reader_running = False

# ── background reader ────────────────────────────────────────────────────────

def _reader():
    global _ser, _rx_buf
    while _reader_running:
        try:
            with _ser_lock:
                s = _ser
            if s and s.is_open:
                waiting = s.in_waiting
                if waiting:
                    chunk = s.read(waiting)
                    with _rx_lock:
                        _rx_buf.extend(chunk)
                else:
                    time.sleep(0.005)
            else:
                time.sleep(0.02)
        except Exception:
            time.sleep(0.05)

# ── HTTP handler ─────────────────────────────────────────────────────────────

class Handler(BaseHTTPRequestHandler):

    def log_message(self, fmt, *args):
        print(f'  [{self.path}] {fmt % args}')

    def _cors(self):
        self.send_header('Access-Control-Allow-Origin',  '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')

    def do_OPTIONS(self):
        self.send_response(204)
        self._cors()
        self.end_headers()

    def _json_resp(self, obj, code=200):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header('Content-Type', 'application/json')
        self._cors()
        self.end_headers()
        self.wfile.write(body)

    def _read_body(self):
        n = int(self.headers.get('Content-Length', 0))
        return json.loads(self.rfile.read(n)) if n else {}

    def do_GET(self):
        global _ser, _rx_buf
        if self.path == '/status':
            with _ser_lock:
                open_ = bool(_ser and _ser.is_open)
            self._json_resp({'ok': True, 'open': open_, 'port': SERIAL_PORT, 'baud': BAUD})

        elif self.path == '/drain':
            with _rx_lock:
                data = bytes(_rx_buf)
                _rx_buf.clear()
            self._json_resp({'ok': True, 'bytes': list(data)})

        else:
            self._json_resp({'ok': False, 'error': 'not found'}, 404)

    def do_POST(self):
        global _ser, _rx_buf, _reader_running

        if self.path == '/connect':
            body = self._read_body()
            port = body.get('port', SERIAL_PORT)
            baud = int(body.get('baud', BAUD))
            try:
                with _ser_lock:
                    if _ser and _ser.is_open:
                        _ser.close()
                    _ser = serial.Serial(port, baud, timeout=0)
                with _rx_lock:
                    _rx_buf.clear()
                if not _reader_running:
                    _reader_running = True
                    t = threading.Thread(target=_reader, daemon=True)
                    t.start()
                print(f'  Opened {port} @ {baud}')
                self._json_resp({'ok': True})
            except Exception as e:
                self._json_resp({'ok': False, 'error': str(e)})

        elif self.path == '/disconnect':
            with _ser_lock:
                if _ser and _ser.is_open:
                    _ser.close()
            print('  Port closed.')
            self._json_resp({'ok': True})

        elif self.path == '/transact':
            body = self._read_body()
            tx_bytes  = bytes(body.get('tx', []))
            rx_count  = int(body.get('rx_count', 0))
            timeout_s = body.get('timeout_ms', 3000) / 1000.0
            try:
                with _rx_lock:
                    stale = bytes(_rx_buf)
                    _rx_buf.clear()
                if stale:
                    print(f'  [transact] drained {len(stale)} stale bytes: {stale.hex()}')
                with _ser_lock:
                    if not (_ser and _ser.is_open):
                        raise IOError('Serial port not open')
                    _ser.write(tx_bytes)
                print(f'  [transact] TX {len(tx_bytes)} bytes: {tx_bytes[:16].hex()}{"..." if len(tx_bytes)>16 else ""}')
                rx = bytearray()
                deadline = time.time() + timeout_s
                while len(rx) < rx_count and time.time() < deadline:
                    with _rx_lock:
                        chunk = bytes(_rx_buf)
                        _rx_buf.clear()
                    rx.extend(chunk)
                    if len(rx) < rx_count:
                        time.sleep(0.005)
                print(f'  [transact] RX {len(rx)} bytes: {rx.hex() if rx else "(empty)"}')
                self._json_resp({'ok': True, 'rx': list(rx)})
            except Exception as e:
                print(f'  [transact] ERROR: {e}')
                self._json_resp({'ok': False, 'error': str(e)})

        else:
            self._json_resp({'ok': False, 'error': 'not found'}, 404)


def _generate_self_signed_cert():
    """Generate a self-signed certificate with SAN for HTTPS bridge."""
    import tempfile, subprocess, os
    cert_dir = tempfile.mkdtemp(prefix='church_bridge_')
    key_path  = os.path.join(cert_dir, 'key.pem')
    cert_path = os.path.join(cert_dir, 'cert.pem')
    conf_path = os.path.join(cert_dir, 'openssl.cnf')
    with open(conf_path, 'w') as f:
        f.write(
            "[req]\n"
            "default_bits = 2048\n"
            "prompt = no\n"
            "distinguished_name = dn\n"
            "x509_extensions = v3_ext\n"
            "[dn]\n"
            "CN = penguin.linux.test\n"
            "[v3_ext]\n"
            "subjectAltName = DNS:penguin.linux.test, DNS:localhost, IP:127.0.0.1\n"
            "basicConstraints = CA:FALSE\n"
            "keyUsage = digitalSignature, keyEncipherment\n"
            "extendedKeyUsage = serverAuth\n"
        )
    result = subprocess.run([
        'openssl', 'req', '-x509', '-newkey', 'rsa:2048',
        '-keyout', key_path, '-out', cert_path,
        '-days', '365', '-nodes',
        '-config', conf_path,
    ], capture_output=True)
    if result.returncode != 0:
        print(f"WARNING: openssl failed: {result.stderr.decode()}")
        print("Falling back to plain HTTP (HTTPS not available)")
        return None, None
    return cert_path, key_path


def _scan_ports():
    """List all serial ports and probe each for incoming data."""
    import glob
    ports = sorted(glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyS*'))
    ports = [p for p in ports if not p.startswith('/dev/ttyS') or int(p.replace('/dev/ttyS','')) < 4]
    print(f'Found {len(ports)} serial port(s): {", ".join(ports) if ports else "(none)"}')
    print()
    for port in ports:
        try:
            s = serial.Serial(port, 115200, timeout=0.5)
            time.sleep(0.1)
            w = s.in_waiting
            chunk = s.read(max(w, 64)) if True else b''
            s.close()
            if chunk:
                print(f'  {port}: OPEN OK — got {len(chunk)} bytes: {chunk[:32].hex()} ascii={chunk[:32]}')
            else:
                print(f'  {port}: OPEN OK — 0 bytes waiting')
        except Exception as e:
            print(f'  {port}: FAILED — {e}')
    print()


def _monitor_mode(port, baud):
    """Open port and print everything received for 10 seconds."""
    print(f'MONITOR MODE: listening on {port} @ {baud} for 10 seconds...')
    print(f'  (Flash the FPGA, then run this within 3 seconds to catch the boot banner)')
    print()
    try:
        s = serial.Serial(port, baud, timeout=0.1)
    except Exception as e:
        print(f'  FAILED to open {port}: {e}')
        return
    total = 0
    start = time.time()
    while time.time() - start < 10:
        w = s.in_waiting
        if w:
            chunk = s.read(w)
            total += len(chunk)
            elapsed = time.time() - start
            hex_str = chunk.hex()
            try:
                ascii_str = chunk.decode('ascii', errors='replace')
            except:
                ascii_str = ''
            print(f'  [{elapsed:5.2f}s] +{len(chunk)} bytes (total {total}): hex={hex_str}')
            if ascii_str.strip():
                print(f'         ascii: {repr(ascii_str)}')
        else:
            time.sleep(0.01)
    s.close()
    print()
    if total == 0:
        print(f'  RESULT: 0 bytes received in 10 seconds.')
        print(f'  This means either:')
        print(f'    1. {port} is not bridged to the FPGA UART pins')
        print(f'    2. The BL616 firmware is not in UART bridge mode')
        print(f'    3. The baud rate is wrong (try --monitor {port} 9600)')
    else:
        print(f'  RESULT: {total} bytes received — UART link is working!')
    print()


def _probe_bauds(port):
    """Try common baud rates and listen briefly on each."""
    bauds = [115200, 9600, 19200, 38400, 57600, 230400, 460800, 921600, 1000000, 2000000]
    print(f'BAUD PROBE: trying {len(bauds)} rates on {port}...')
    print()
    for baud in bauds:
        try:
            s = serial.Serial(port, baud, timeout=0.3)
            time.sleep(0.3)
            w = s.in_waiting
            chunk = s.read(max(w, 32))
            s.close()
            if chunk:
                print(f'  {baud:>7d}: GOT {len(chunk)} bytes — {chunk[:16].hex()} ascii={repr(chunk[:16].decode("ascii", errors="replace"))}')
            else:
                print(f'  {baud:>7d}: (silence)')
        except Exception as e:
            print(f'  {baud:>7d}: FAILED — {e}')
    print()


if __name__ == '__main__':
    if _first_arg == '--scan':
        _scan_ports()
        sys.exit(0)

    if _first_arg == '--monitor':
        port = sys.argv[2] if len(sys.argv) > 2 else '/dev/ttyUSB1'
        baud = int(sys.argv[3]) if len(sys.argv) > 3 else 115200
        _monitor_mode(port, baud)
        sys.exit(0)

    if _first_arg == '--probe-bauds':
        port = sys.argv[2] if len(sys.argv) > 2 else '/dev/ttyUSB1'
        _probe_bauds(port)
        sys.exit(0)

    cert_path, key_path = _generate_self_signed_cert()
    use_https = cert_path is not None

    if use_https:
        scheme = 'https'
    else:
        scheme = 'http'

    print(f'Church Machine FPGA Bridge ({scheme.upper()})')
    print(f'  Serial : {SERIAL_PORT} @ {BAUD} baud')
    print(f'  {scheme.upper():7s}: {scheme}://0.0.0.0:{HTTP_PORT}')
    print(f'  ChromeOS bridge URL: {scheme}://penguin.linux.test:{HTTP_PORT}')
    print()
    if use_https:
        print('IMPORTANT — first time setup:')
        print(f'  1. Open https://penguin.linux.test:{HTTP_PORT}/status in Chrome')
        print(f'  2. Click "Advanced" → "Proceed to penguin.linux.test (unsafe)"')
        print(f'  3. You should see {{"ok": true, ...}}')
        print(f'  4. Now go to the IDE and click "Bridge"')
    else:
        print('NOTE: Running in HTTP mode (openssl not available).')
        print('      Mixed-content blocking may prevent connection from HTTPS IDE pages.')
        print(f'      Bridge URL: http://penguin.linux.test:{HTTP_PORT}')
    print()
    print('Press Ctrl+C to stop.')
    print()

    server = HTTPServer(('0.0.0.0', HTTP_PORT), Handler)
    if use_https:
        import ssl as _ssl
        ctx = _ssl.SSLContext(_ssl.PROTOCOL_TLS_SERVER)
        ctx.load_cert_chain(cert_path, key_path)
        server.socket = ctx.wrap_socket(server.socket, server_side=True)
    server.serve_forever()
