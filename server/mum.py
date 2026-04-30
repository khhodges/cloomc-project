"""
server/mum.py — Mum identity module (Stage 3, Keystone Hello Mum).

Generates a real Ed25519 key pair on first run; persists the private key
to server/mum_key.pem (gitignored).

Key-generation strategy — preferred order:
  1. cryptography.hazmat (Ed25519PrivateKey) — used when available.
  2. OpenSSL subprocess fallback — used when cffi / Rust bindings are absent
     (e.g. NixOS dev environment: `No module named '_cffi_backend'`).

Derives the canonical identity string (public key bytes, base64url, no padding)
and the 32-bit identity word used by Keystone.Connect().

Identity-word format (matches keystone.cloomc and system_abstractions.js):
  bits [31:28]  version tag        = 0x1  (Ed25519 / GTKN-1)
  bits [27:16]  fingerprint[28:17] — top 12 bits of SHA-256(pubkey)
  bits [15: 0]  fingerprint[15: 0] — low  16 bits of SHA-256(pubkey)
"""

import os
import hashlib
import base64
import logging
import subprocess

log = logging.getLogger(__name__)

_SERVER_DIR = os.path.dirname(os.path.abspath(__file__))
_KEY_PATH   = os.path.join(_SERVER_DIR, "mum_key.pem")

_private_key_pem  = None
_public_key_bytes = None   # raw 32 bytes
_identity_string  = None   # base64url, no padding
_identity_word    = None   # 32-bit encoded value


# ---------------------------------------------------------------------------
# Strategy 1 — Python cryptography library
# ---------------------------------------------------------------------------

def _try_load_cryptography():
    """
    Attempt to load / generate the key pair using the `cryptography` library.
    Raises ImportError if cffi / Rust bindings are unavailable.
    """
    # This import triggers the cffi/Rust check; it will raise ImportError on
    # NixOS environments where `_cffi_backend` is missing.
    from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PrivateKey
    from cryptography.hazmat.primitives.serialization import (
        Encoding, PrivateFormat, PublicFormat, NoEncryption,
        load_pem_private_key,
    )

    global _private_key_pem, _public_key_bytes

    if os.path.isfile(_KEY_PATH):
        with open(_KEY_PATH, "rb") as fh:
            raw_pem = fh.read()
        private_key = load_pem_private_key(raw_pem, password=None)
        _private_key_pem = raw_pem
        log.info("mum.py: loaded Ed25519 private key via cryptography library from %s", _KEY_PATH)
    else:
        private_key = Ed25519PrivateKey.generate()
        _private_key_pem = private_key.private_bytes(Encoding.PEM, PrivateFormat.PKCS8, NoEncryption())
        with open(_KEY_PATH, "wb") as fh:
            fh.write(_private_key_pem)
        log.info("mum.py: generated new Ed25519 key via cryptography library, stored at %s", _KEY_PATH)

    _public_key_bytes = private_key.public_key().public_bytes(Encoding.Raw, PublicFormat.Raw)


# ---------------------------------------------------------------------------
# Strategy 2 — OpenSSL subprocess fallback
# ---------------------------------------------------------------------------

# Ed25519 SubjectPublicKeyInfo DER prefix (12 bytes):
#   SEQUENCE { SEQUENCE { OID 1.3.101.112 } BIT STRING 0x00 <32 bytes> }
_ED25519_DER_PREFIX_LEN = 12


def _run_openssl(*args, stdin=None):
    """Run an openssl subcommand and return stdout bytes; raise on error."""
    result = subprocess.run(
        ["openssl"] + list(args),
        input=stdin,
        capture_output=True,
    )
    if result.returncode != 0:
        raise RuntimeError(
            f"openssl {' '.join(args)} failed: {result.stderr.decode(errors='replace')}"
        )
    return result.stdout


def _try_load_openssl():
    """
    Load / generate the key pair using `openssl` subprocess.
    Used as a fallback when the `cryptography` library is unavailable.
    """
    global _private_key_pem, _public_key_bytes

    if os.path.isfile(_KEY_PATH):
        with open(_KEY_PATH, "rb") as fh:
            _private_key_pem = fh.read()
        log.info("mum.py: loaded Ed25519 private key via openssl subprocess from %s", _KEY_PATH)
    else:
        _private_key_pem = _run_openssl("genpkey", "-algorithm", "Ed25519")
        with open(_KEY_PATH, "wb") as fh:
            fh.write(_private_key_pem)
        log.info("mum.py: generated new Ed25519 key via openssl subprocess, stored at %s", _KEY_PATH)

    pub_der = _run_openssl("pkey", "-pubout", "-outform", "DER", stdin=_private_key_pem)
    if len(pub_der) < _ED25519_DER_PREFIX_LEN + 32:
        raise RuntimeError(f"Unexpected DER public key length: {len(pub_der)} bytes")
    _public_key_bytes = pub_der[_ED25519_DER_PREFIX_LEN : _ED25519_DER_PREFIX_LEN + 32]


# ---------------------------------------------------------------------------
# Shared initialisation
# ---------------------------------------------------------------------------

def _load_or_generate():
    global _identity_string, _identity_word

    if _private_key_pem is not None:
        return

    try:
        _try_load_cryptography()
    except ImportError:
        log.warning(
            "mum.py: cryptography library unavailable (cffi/Rust bindings missing in "
            "this Nix environment); falling back to openssl subprocess."
        )
        _try_load_openssl()

    _identity_string = (
        base64.urlsafe_b64encode(_public_key_bytes)
        .rstrip(b"=")
        .decode("ascii")
    )

    digest = hashlib.sha256(_public_key_bytes).digest()
    fp_hi  = (digest[0] << 4) | (digest[1] >> 4)
    fp_lo  = ((digest[1] & 0x0F) << 12) | (digest[2] << 4) | (digest[3] >> 4)
    _identity_word = (0x1 << 28) | ((fp_hi & 0xFFF) << 16) | (fp_lo & 0xFFFF)


def get_identity_string() -> str:
    _load_or_generate()
    return _identity_string


def get_identity_word() -> int:
    _load_or_generate()
    return _identity_word


def regenerate_key() -> None:
    """Delete the persisted key file and regenerate a fresh Ed25519 key pair.

    Resets all module-level globals so the next call to any public accessor
    will trigger a fresh key generation via _load_or_generate().
    """
    global _private_key_pem, _public_key_bytes, _identity_string, _identity_word

    if os.path.isfile(_KEY_PATH):
        os.remove(_KEY_PATH)
        log.info("mum.py: deleted %s for key regeneration", _KEY_PATH)

    _private_key_pem  = None
    _public_key_bytes = None
    _identity_string  = None
    _identity_word    = None

    _load_or_generate()
    log.info("mum.py: key regenerated; new identity = %s", _identity_string)


# ---------------------------------------------------------------------------
# QR code renderer (pure-stdlib PNG — no Pillow required)
# ---------------------------------------------------------------------------

def _matrix_to_png(matrix, box_size: int = 10, border: int = 4) -> bytes:
    """
    Convert a QR code matrix (list of lists of bool) to a PNG byte string.
    Uses only stdlib (zlib + struct) — no Pillow required.
    """
    import struct
    import zlib

    n    = len(matrix)
    side = (n + 2 * border) * box_size

    def png_chunk(tag, data):
        crc = zlib.crc32(tag + data) & 0xFFFFFFFF
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", crc)

    sig  = b"\x89PNG\r\n\x1a\n"
    ihdr = png_chunk(b"IHDR", struct.pack(">IIBBBBB", side, side, 8, 0, 0, 0, 0))

    rows       = []
    white_row  = b"\x00" + b"\xff" * side
    for _ in range(border * box_size):
        rows.append(white_row)
    for row_bits in matrix:
        pixels = bytearray()
        pixels += b"\xff" * (border * box_size)
        for bit in row_bits:
            pixels += (b"\x00" if bit else b"\xff") * box_size
        pixels += b"\xff" * (border * box_size)
        row_bytes = b"\x00" + bytes(pixels)
        for _ in range(box_size):
            rows.append(row_bytes)
    for _ in range(border * box_size):
        rows.append(white_row)

    raw        = b"".join(rows)
    compressed = zlib.compress(raw, 6)
    idat       = png_chunk(b"IDAT", compressed)
    iend       = png_chunk(b"IEND", b"")
    return sig + ihdr + idat + iend


def get_qr_png() -> bytes:
    """Return a PNG-encoded QR code of Mum's canonical identity string."""
    import qrcode
    _load_or_generate()
    qr = qrcode.QRCode(
        version=None,
        error_correction=qrcode.constants.ERROR_CORRECT_M,
        box_size=1,
        border=0,
    )
    qr.add_data(_identity_string)
    qr.make(fit=True)
    matrix = qr.get_matrix()
    return _matrix_to_png(matrix, box_size=10, border=4)


# ---------------------------------------------------------------------------
# Identity-word derivation (for submitted identity strings)
# ---------------------------------------------------------------------------

def identity_word_from_string(identity_string: str) -> int:
    """
    Derive the 32-bit identity word from a canonical identity string.
    Validates that the decoded payload is exactly 32 bytes (Ed25519 pubkey).
    Returns 0 if the string is invalid.
    """
    try:
        padded    = identity_string + "=" * (-len(identity_string) % 4)
        pub_bytes = base64.urlsafe_b64decode(padded)
        if len(pub_bytes) != 32:
            return 0
        digest = hashlib.sha256(pub_bytes).digest()
        fp_hi  = (digest[0] << 4) | (digest[1] >> 4)
        fp_lo  = ((digest[1] & 0x0F) << 12) | (digest[2] << 4) | (digest[3] >> 4)
        return (0x1 << 28) | ((fp_hi & 0xFFF) << 16) | (fp_lo & 0xFFFF)
    except Exception:
        return 0
