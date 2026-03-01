"""Upload a memory image to the Church Machine over serial.

Usage:
    python -m church_machine.upload [--port /dev/ttyACM1] [--image file.bin] [--dump file.bin]

Protocol:
    1. Send sync byte: 0xAA
    2. Send 4-byte header: total word count (little-endian u32)
    3. Send N x 4-byte words (little-endian u32):
       - Words 0..255: Instruction memory (boot program)
       - Words 256..447: Namespace (192 words)
       - Words 448..511: C-list (64 words)
    4. Wait for "CHURCH v1.0" banner confirming successful boot
"""

import sys
import struct
import time
import argparse

from .boot_rom import BOOT_PROGRAM, DEMO_NAMESPACE, DEMO_CLIST


IMEM_WORDS = 256
NS_WORDS = 192
CLIST_WORDS = 64
TOTAL_WORDS = IMEM_WORDS + NS_WORDS + CLIST_WORDS
SYNC_BYTE = 0xAA


def build_default_image():
    imem = list(BOOT_PROGRAM[:IMEM_WORDS])
    while len(imem) < IMEM_WORDS:
        imem.append(0)

    ns_flat = []
    for i in range(0, len(DEMO_NAMESPACE), 3):
        if i + 2 < len(DEMO_NAMESPACE):
            ns_flat.extend([DEMO_NAMESPACE[i], DEMO_NAMESPACE[i+1], DEMO_NAMESPACE[i+2]])
    while len(ns_flat) < NS_WORDS:
        ns_flat.append(0)

    clist = list(DEMO_CLIST[:CLIST_WORDS])
    while len(clist) < CLIST_WORDS:
        clist.append(0)

    return imem + ns_flat + clist


def image_to_bytes(image):
    data = struct.pack('<I', len(image))
    for word in image:
        data += struct.pack('<I', word)
    return bytes([SYNC_BYTE]) + data


def dump_image(filename, image):
    data = image_to_bytes(image)
    with open(filename, 'wb') as f:
        f.write(data)
    print(f"Dumped {len(data)} bytes ({len(image)} words) to {filename}")


def upload_image(port, image, timeout_s=10):
    try:
        import serial
    except ImportError:
        print("Error: pyserial not installed. Run: pip install pyserial")
        sys.exit(1)

    data = image_to_bytes(image)

    print(f"Opening {port}...")
    ser = serial.Serial(port, 115200, timeout=1)
    time.sleep(0.1)
    ser.reset_input_buffer()

    print(f"Sending {len(data)} bytes ({len(image)} words)...")
    ser.write(data)
    ser.flush()

    print("Waiting for banner...")
    deadline = time.time() + timeout_s
    banner_lines = []
    while time.time() < deadline:
        line = ser.readline()
        if line:
            text = line.decode('ascii', errors='replace').rstrip()
            print(f"  {text}")
            banner_lines.append(text)
            if "HALT" in text:
                break

    ser.close()

    if any("CHURCH" in l for l in banner_lines):
        print("Upload successful!")
        return True
    else:
        print("Warning: Did not receive expected banner.")
        return False


def main():
    parser = argparse.ArgumentParser(description="Upload memory image to Church Machine")
    parser.add_argument('--port', default='/dev/ttyACM1', help='Serial port (default: /dev/ttyACM1)')
    parser.add_argument('--image', help='Binary image file to upload (default: built-in demo)')
    parser.add_argument('--dump', help='Dump image to file instead of uploading')
    args = parser.parse_args()

    if args.image:
        with open(args.image, 'rb') as f:
            raw = f.read()
        if len(raw) < 4:
            print("Error: Image file too small")
            sys.exit(1)
        if raw[0] == SYNC_BYTE:
            raw = raw[1:]
        word_count = struct.unpack('<I', raw[:4])[0]
        raw = raw[4:]
        image = []
        for i in range(word_count):
            if i * 4 + 4 <= len(raw):
                image.append(struct.unpack('<I', raw[i*4:i*4+4])[0])
            else:
                image.append(0)
    else:
        image = build_default_image()

    print(f"Image: {len(image)} words ({len(image)*4} bytes)")
    print(f"  Instructions: words 0..{IMEM_WORDS-1}")
    print(f"  Namespace:    words {IMEM_WORDS}..{IMEM_WORDS+NS_WORDS-1}")
    print(f"  C-list:       words {IMEM_WORDS+NS_WORDS}..{TOTAL_WORDS-1}")

    if args.dump:
        dump_image(args.dump, image)
    else:
        upload_image(args.port, image)


if __name__ == '__main__':
    main()
