#!/usr/bin/env python3
"""
Church Machine FPGA Patcher — command-line tool
================================================
Reads a .patch file exported from the Church Machine IDE and sends the
pre-compiled binary frames to the FPGA over UART.

Usage:
    python3 patch_fpga.py <serial-port> <patch-file>

Example:
    python3 patch_fpga.py /dev/ttyUSB1 CR14_patch.patch

.patch file format (CHPF v1):
    Bytes 0-3:  Magic "CHPF" (0x43 0x48 0x50 0x46)
    Byte  4:    Version (0x01)
    Byte  5:    Number of PATCH_LUMP blocks (1-255)
    Byte  6:    Flags (bit 0 = file includes RUN sentinel after all blocks)
    Byte  7:    Reserved (0x00)
    Then for each block, a complete UART frame:
        Bytes 0-1:   Tag [0xBE][0xEF]
        Bytes 2-3:   Address (big-endian, BRAM word address)
        Bytes 4-5:   Word count N (big-endian)
        Bytes 6..6+N*4-1:  N words (little-endian, 4 bytes each)
        Last 2 bytes: CRC-16/CCITT over the frame body (tag+addr+count+words)
    If flags bit 0 is set, the last 2 bytes of the file are:
        [0xBE][0xAA]  (RUN sentinel — tells FPGA to start executing)

The script sends each stored frame verbatim over UART, waits for the
4-byte echo (addr+count), verifies it, and then sends the RUN sentinel
if present.

Requires: pyserial (pip3 install pyserial)
"""
import sys, time

def crc16_ccitt(data):
    crc = 0xFFFF
    for byte in data:
        for i in range(8):
            bit = ((byte >> (7 - i)) & 1) ^ ((crc >> 15) & 1)
            crc = ((crc << 1) & 0xFFFF) ^ (0x1021 if bit else 0)
    return crc

def parse_patch_file(path):
    with open(path, 'rb') as f:
        data = f.read()

    if len(data) < 8 or data[:4] != b'CHPF':
        print(f"ERROR: '{path}' is not a valid .patch file (bad magic)")
        sys.exit(1)

    version = data[4]
    if version != 1:
        print(f"ERROR: Unsupported patch version {version} (expected 1)")
        sys.exit(1)

    num_blocks = data[5]
    flags = data[6]
    has_run = bool(flags & 1)

    frames = []
    offset = 8
    for i in range(num_blocks):
        if offset + 6 > len(data):
            print(f"ERROR: Patch file truncated at block {i} header")
            sys.exit(1)
        if data[offset] != 0xBE or data[offset + 1] != 0xEF:
            print(f"ERROR: Block {i} missing PATCH_LUMP tag (expected 0xBEEF, got 0x{data[offset]:02X}{data[offset+1]:02X})")
            sys.exit(1)
        addr = (data[offset + 2] << 8) | data[offset + 3]
        count = (data[offset + 4] << 8) | data[offset + 5]
        body_len = 6 + count * 4
        frame_len = body_len + 2
        if offset + frame_len > len(data):
            print(f"ERROR: Patch file truncated in block {i} (need {frame_len} bytes, have {len(data) - offset})")
            sys.exit(1)
        frame = data[offset:offset + frame_len]
        stored_crc = (frame[-2] << 8) | frame[-1]
        computed_crc = crc16_ccitt(frame[:-2])
        if stored_crc != computed_crc:
            print(f"ERROR: Block {i} CRC mismatch — stored=0x{stored_crc:04X}, computed=0x{computed_crc:04X}")
            sys.exit(1)
        frames.append((addr, count, frame, stored_crc))
        offset += frame_len

    run_sentinel = None
    if has_run:
        if offset + 2 > len(data):
            print(f"ERROR: Patch file missing RUN sentinel")
            sys.exit(1)
        run_sentinel = data[offset:offset + 2]
        if run_sentinel != b'\xBE\xAA':
            print(f"ERROR: Invalid RUN sentinel (expected 0xBEAA, got 0x{run_sentinel[0]:02X}{run_sentinel[1]:02X})")
            sys.exit(1)
        offset += 2

    if offset != len(data):
        print(f"WARNING: {len(data) - offset} unexpected trailing bytes after patch data")

    return frames, run_sentinel

def main():
    if len(sys.argv) < 3:
        print("Usage: python3 patch_fpga.py <serial-port> <patch-file>")
        print("Example: python3 patch_fpga.py /dev/ttyUSB1 CR14_patch.patch")
        sys.exit(1)

    serial_port = sys.argv[1]
    patch_path = sys.argv[2]

    try:
        import serial
    except ImportError:
        print("ERROR: pyserial not installed.  Run:  pip3 install pyserial")
        sys.exit(1)

    frames, run_sentinel = parse_patch_file(patch_path)

    print(f"Church Machine FPGA Patcher")
    print(f"  File   : {patch_path}")
    print(f"  Blocks : {len(frames)}")
    print(f"  RUN    : {'yes' if run_sentinel else 'no'}")
    print()

    for i, (addr, count, frame, crc) in enumerate(frames):
        print(f"  Block {i}: addr=0x{addr:04X}  words={count}  CRC=0x{crc:04X}  frame={len(frame)} bytes")
    print()

    try:
        ser = serial.Serial(serial_port, 115200, timeout=0)
    except Exception as e:
        print(f"ERROR: Cannot open {serial_port}: {e}")
        sys.exit(1)

    print(f"  Serial : {serial_port} @ 115200 baud")
    print()

    ser.reset_input_buffer()
    time.sleep(0.05)

    all_ok = True
    for i, (addr, count, frame, crc) in enumerate(frames):
        print(f"  Block {i}: TX {len(frame)} bytes  addr=0x{addr:04X}  words={count}  CRC=0x{crc:04X}")

        ser.reset_input_buffer()
        ser.write(frame)
        ser.flush()

        rx = bytearray()
        deadline = time.time() + 3.0
        while len(rx) < 4 and time.time() < deadline:
            waiting = ser.in_waiting
            if waiting:
                rx.extend(ser.read(waiting))
            else:
                time.sleep(0.005)

        if len(rx) >= 4:
            echo_addr = (rx[0] << 8) | rx[1]
            echo_count = (rx[2] << 8) | rx[3]
            addr_ok = echo_addr == addr
            count_ok = echo_count == count
            if addr_ok and count_ok:
                print(f"           RX echo OK: addr=0x{echo_addr:04X}  count={echo_count}")
            else:
                print(f"           RX echo MISMATCH: expected addr=0x{addr:04X} count={count}, got addr=0x{echo_addr:04X} count={echo_count}")
                all_ok = False
        else:
            print(f"           RX no echo ({len(rx)} bytes received)")
            all_ok = False

    if run_sentinel:
        print()
        print("  Sending RUN sentinel (0xBE 0xAA)...")
        ser.write(run_sentinel)
        ser.flush()
        print("  RUN sent — core executing from PC=0.")

    ser.close()

    print()
    if all_ok:
        print("SUCCESS — all blocks patched and verified.")
    else:
        print("WARNING — some blocks did not echo correctly. Check UART connection.")

    sys.exit(0 if all_ok else 1)

if __name__ == '__main__':
    main()
