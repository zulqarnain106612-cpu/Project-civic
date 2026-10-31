#!/usr/bin/env python3
"""Capture STM32F4DISCOVERY CAN CSV records from its ST-LINK virtual COM port."""
import argparse
import csv
import sys
import time

import serial

FIELDS = ("timestamp_ms", "format", "id", "dlc", "rtr", "payload_hex")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="serial port, e.g. /dev/ttyACM0")
    parser.add_argument("--output", required=True, help="output CSV path")
    parser.add_argument("--duration", type=float, default=0.0,
                        help="capture seconds; 0 means until Ctrl-C")
    args = parser.parse_args()
    if args.duration < 0:
        parser.error("--duration must be zero or positive")

    try:
        with serial.Serial(args.port, 115200, timeout=0.25) as port, \
             open(args.output, "w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(FIELDS)
            deadline = time.monotonic() + args.duration if args.duration else None
            count = 0
            while deadline is None or time.monotonic() < deadline:
                raw = port.readline()
                if not raw:
                    continue
                try:
                    row = next(csv.reader([raw.decode("ascii").strip()]))
                except (UnicodeDecodeError, csv.Error):
                    continue
                if len(row) != len(FIELDS):
                    continue
                # Keep only syntactically plausible firmware records.
                try:
                    int(row[0], 10)
                    frame_format = row[1]
                    identifier = int(row[2], 16)
                    dlc = int(row[3], 10)
                    remote = int(row[4], 10)
                    payload = row[5]
                except ValueError:
                    continue
                if (frame_format not in ("STD", "EXT") or dlc < 0 or dlc > 8 or
                    remote not in (0, 1) or
                    identifier < 0 or identifier > (0x7ff if frame_format == "STD" else 0x1fffffff) or
                    (remote == 0 and len(payload) != dlc * 2) or
                    (remote == 1 and payload != "")):
                    continue
                writer.writerow(row)
                stream.flush()
                count += 1
    except KeyboardInterrupt:
        pass
    except (OSError, serial.SerialException) as exc:
        print(f"serial capture failed: {exc}", file=sys.stderr)
        return 2
    print(f"capture saved: {args.output} ({count} frames)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
