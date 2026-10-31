#!/usr/bin/env python3
"""Capture Classical CAN frames through an explicitly selected python-can backend.

This is host-side tooling only. It does not configure or replace the STM32 firmware,
and it never chooses an interface, channel, or bitrate implicitly.
"""
from __future__ import annotations
import argparse
import csv
import math
import sys
import time
from pathlib import Path


def frame_to_row(message, elapsed_ms: int) -> list[str]:
    """Convert a python-can Message-like object to the project's six-field CSV row."""
    is_extended = bool(message.is_extended_id)
    is_remote = bool(message.is_remote_frame)
    identifier = int(message.arbitration_id)
    dlc = int(message.dlc)
    max_id = 0x1FFFFFFF if is_extended else 0x7FF
    if not 0 <= identifier <= max_id:
        raise ValueError("arbitration ID outside selected frame-format range")
    if not 0 <= dlc <= 8:
        raise ValueError("Classical CAN DLC must be between 0 and 8")
    data = bytes(message.data)
    if is_remote:
        if data:
            raise ValueError("remote frame must not contain payload bytes")
        payload = ""
    else:
        if len(data) != dlc:
            raise ValueError("payload length does not match DLC")
        payload = data.hex().upper()
    timestamp_ms = int(elapsed_ms) & 0xFFFFFFFF
    return [str(timestamp_ms), "EXT" if is_extended else "STD",
            f"0x{identifier:X}", str(dlc), "1" if is_remote else "0", payload]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--interface", required=True, help="explicit python-can backend, e.g. socketcan or slcan")
    parser.add_argument("--channel", required=True, help="explicit backend channel/device name")
    parser.add_argument("--bitrate", type=int, required=True, help="configured Classical CAN bitrate in bit/s")
    parser.add_argument("--duration", type=float, help="optional capture duration in seconds")
    parser.add_argument("--output", type=Path, required=True, help="output six-field CSV path")
    args = parser.parse_args()
    if args.bitrate <= 0 or args.bitrate > 1_000_000:
        parser.error("--bitrate must be within Classical CAN range 1..1000000")
    if args.duration is not None and (not math.isfinite(args.duration) or args.duration <= 0):
        parser.error("--duration must be a finite, positive number")
    try:
        import can
    except ImportError:
        print("error: python-can is required for host capture", file=sys.stderr)
        return 2

    bus = None
    try:
        bus = can.Bus(interface=args.interface, channel=args.channel, bitrate=args.bitrate)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        start = time.monotonic_ns()
        deadline = start + int(args.duration * 1_000_000_000) if args.duration is not None else None
        with args.output.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle, lineterminator="\n")
            while deadline is None or time.monotonic_ns() < deadline:
                timeout = 0.25
                if deadline is not None:
                    timeout = min(timeout, max(0.0, (deadline-time.monotonic_ns())/1_000_000_000))
                message = bus.recv(timeout=timeout)
                if message is None:
                    continue
                if bool(message.is_error_frame):
                    print("warning: skipped CAN error frame (not representable in six-field data CSV)", file=sys.stderr)
                    continue
                elapsed_ms = (time.monotonic_ns() - start) // 1_000_000
                writer.writerow(frame_to_row(message, elapsed_ms))
                handle.flush()
    except (OSError, ValueError, RuntimeError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    except Exception as exc:
        # Backend-specific python-can exceptions vary; surface them without a traceback by default.
        print(f"error: CAN backend capture failed: {exc}", file=sys.stderr)
        return 2
    finally:
        if bus is not None:
            bus.shutdown()
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
