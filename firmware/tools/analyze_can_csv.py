#!/usr/bin/env python3
"""Evidence-only summary of Project-civic's six-field frame CSV.

Input rows: timestamp_ms,format,id,dlc,rtr,payload_hex.
This reports observed traffic statistics only; it does not decode vehicle signals.
"""
from __future__ import annotations
import argparse, csv, statistics, sys
from collections import defaultdict
from dataclasses import dataclass, field
from pathlib import Path

@dataclass
class MessageStats:
    fmt: str
    identifier: int
    count: int = 0
    dlcs: set[int] = field(default_factory=set)
    timestamps: list[int] = field(default_factory=list)
    byte_values: list[set[int]] = field(default_factory=lambda: [set() for _ in range(8)])
    rtr_count: int = 0

def parse_row(row: list[str], line_number: int):
    if len(row) != 6:
        raise ValueError(f"line {line_number}: expected 6 fields, got {len(row)}")
    ts_s, fmt, id_s, dlc_s, rtr_s, payload_s = (v.strip() for v in row)
    try:
        ts, identifier, dlc, rtr = int(ts_s, 10), int(id_s, 0), int(dlc_s, 10), int(rtr_s, 10)
    except ValueError as exc:
        raise ValueError(f"line {line_number}: invalid numeric field: {exc}") from exc
    if not 0 <= ts <= 0xFFFFFFFF:
        raise ValueError(f"line {line_number}: timestamp_ms outside uint32 range")
    if fmt not in ("STD", "EXT"):
        raise ValueError(f"line {line_number}: format must be STD or EXT")
    if not 0 <= identifier <= (0x7FF if fmt == "STD" else 0x1FFFFFFF):
        raise ValueError(f"line {line_number}: identifier outside {fmt} range")
    if not 0 <= dlc <= 8 or rtr not in (0, 1):
        raise ValueError(f"line {line_number}: invalid DLC or RTR flag")
    if any(ch not in "0123456789abcdefABCDEF" for ch in payload_s):
        raise ValueError(f"line {line_number}: payload is not hexadecimal")
    if rtr:
        if payload_s:
            raise ValueError(f"line {line_number}: RTR frame must have empty payload")
        payload = b""
    else:
        if len(payload_s) != dlc * 2:
            raise ValueError(f"line {line_number}: payload length does not match DLC")
        payload = bytes.fromhex(payload_s)
    return ts, fmt, identifier, dlc, rtr, payload

def analyze(lines):
    stats = {}
    for line_number, row in enumerate(csv.reader(lines, strict=True), start=1):
        if not row or (len(row) == 1 and not row[0].strip()):
            continue
        ts, fmt, identifier, dlc, rtr, payload = parse_row(row, line_number)
        item = stats.setdefault((fmt, identifier), MessageStats(fmt, identifier))
        item.count += 1
        item.dlcs.add(dlc)
        item.timestamps.append(ts)
        item.rtr_count += rtr
        for index, value in enumerate(payload):
            item.byte_values[index].add(value)
    return sorted(stats.values(), key=lambda item: (item.fmt, item.identifier))

def render_report(stats):
    out = [
        "# Project-civic CAN traffic summary", "",
        "Evidence-only report from captured raw frames. Byte positions are zero-based.",
        "No vehicle-specific signal meaning, endianness, signedness, scale, or offset is inferred.",
        "Every candidate remains a hypothesis until corroborated.", "",
        "| Format | ID | Frames | DLC values | Median period (ms) | Variable byte positions | RTR frames |",
        "|---|---:|---:|---|---:|---|---:|",
    ]
    for item in stats:
        periods = [((b - a) & 0xFFFFFFFF) for a, b in zip(item.timestamps, item.timestamps[1:])]
        median_period = f"{statistics.median(periods):g}" if periods else "n/a"
        variable = ",".join(str(i) for i, values in enumerate(item.byte_values) if len(values) > 1) or "none observed"
        dlcs = ",".join(str(value) for value in sorted(item.dlcs))
        width = 3 if item.fmt == "STD" else 8
        out.append(f"| {item.fmt} | 0x{item.identifier:0{width}X} | {item.count} | {dlcs} | {median_period} | {variable} | {item.rtr_count} |")
    out.extend(["", "## Interpretation limits",
        "- Variable bytes are observations, not decoded signals.",
        "- A single observed value does not prove a constant signal.",
        "- Periods use adjacent captured timestamps and the firmware millisecond tick; timer accuracy and capture losses are not independently verified here.",
        "- Correlate with controlled observations and a matching DBC/protocol before assigning signal names or physical units.", ""])
    return "\n".join(out)

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_csv", type=Path, help="six-field CSV produced by can_capture_frame_to_csv")
    parser.add_argument("-o", "--output", type=Path, help="Markdown report path (default: stdout)")
    args = parser.parse_args()
    try:
        with args.input_csv.open("r", newline="", encoding="utf-8") as handle:
            report = render_report(analyze(handle))
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(report, encoding="utf-8")
        else:
            sys.stdout.write(report)
    except (OSError, UnicodeError, csv.Error, ValueError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
