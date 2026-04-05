#!/usr/bin/env python3
"""
decode_sniffer.py — Decode FSDLink sniffer serial output using a DBC file.

Usage:
    python3 decode_sniffer.py --port /dev/ttyACM1 --dbc tesla.dbc
    python3 decode_sniffer.py --port /dev/cu.usbmodem3101 --dbc tesla.dbc

DBC source: https://github.com/joshwardell/model3dbc
    Covers: 1021 (0x3FD UI_autopilotControl), 920 (0x398 DAS_status)
    Raw hex fallback for: 2047 (0x7FF), 968 (0x3C8)

Requirements:
    pip install cantools pyserial
"""

import argparse
import re
import sys
import serial
import cantools

SNIFFER_PATTERN = re.compile(
    r"CAN 0x([0-9A-Fa-f]+)\s+\(\s*\d+\)\s+DLC:(\d+)\s+((?:[0-9A-Fa-f]{2}\s*)+)"
)


def parse_args():
    parser = argparse.ArgumentParser(description="Decode FSDLink sniffer output")
    parser.add_argument("--port", required=True, help="Serial port (e.g. /dev/ttyACM1)")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    parser.add_argument("--dbc", required=True, help="Path to DBC file")
    return parser.parse_args()


def decode_line(db, line):
    m = SNIFFER_PATTERN.search(line)
    if not m:
        return line  # pass through non-sniffer lines as-is

    can_id = int(m.group(1), 16)
    dlc = int(m.group(2))
    raw_bytes = bytes(int(b, 16) for b in m.group(3).split())

    try:
        msg = db.get_message_by_frame_id(can_id)
        signals = msg.decode(raw_bytes, decode_choices=True)
        parts = ", ".join(f"{k}={v}" for k, v in signals.items())
        return f"CAN 0x{can_id:03X} {msg.name}: {parts}"
    except KeyError:
        hex_str = " ".join(f"{b:02X}" for b in raw_bytes)
        return f"CAN 0x{can_id:03X} (unknown) DLC:{dlc}  {hex_str}"
    except Exception as e:
        hex_str = " ".join(f"{b:02X}" for b in raw_bytes)
        return f"CAN 0x{can_id:03X} (decode error: {e}) DLC:{dlc}  {hex_str}"


def main():
    args = parse_args()

    try:
        db = cantools.database.load_file(args.dbc)
        print(f"[decoder] Loaded DBC: {args.dbc} ({len(db.messages)} messages)")
    except Exception as e:
        print(f"[decoder] Failed to load DBC: {e}", file=sys.stderr)
        sys.exit(1)

    try:
        ser = serial.Serial(args.port, args.baud, timeout=1)
        print(f"[decoder] Connected to {args.port} @ {args.baud} baud")
    except Exception as e:
        print(f"[decoder] Failed to open port: {e}", file=sys.stderr)
        sys.exit(1)

    print("[decoder] Listening... Press Ctrl+C to exit\n")
    try:
        while True:
            line = ser.readline().decode("utf-8", errors="replace").strip()
            if line:
                print(decode_line(db, line))
    except KeyboardInterrupt:
        print("\n[decoder] Stopped.")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
