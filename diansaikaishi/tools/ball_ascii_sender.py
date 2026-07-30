#!/usr/bin/env python3
"""Generate or send the simplified wired K230 ball-position protocol."""

from __future__ import annotations

import argparse
import time


def xor_checksum(body: str) -> int:
    checksum = 0
    for value in body.encode("ascii"):
        checksum ^= value
    return checksum


def build_line(
    sequence: int,
    timestamp_ms: int,
    valid: int,
    state: str,
    position_mm: float,
    predicted_mm: float,
    velocity_mm_s: float,
    confidence: float,
    measured: int,
) -> bytes:
    body = (
        f"B,{sequence & 0xFFFF},{timestamp_ms & 0xFFFFFFFF},"
        f"{valid},{state},{position_mm:.1f},{predicted_mm:.1f},"
        f"{velocity_mm_s:.1f},{confidence:.3f},{measured}"
    )
    return f"@{body}*{xor_checksum(body):02X}\n".encode("ascii")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--fps", type=float, default=20.0)
    parser.add_argument("--count", type=int, default=1)
    parser.add_argument("--seq", type=int, default=0)
    parser.add_argument("--valid", type=int, choices=(0, 1), default=1)
    parser.add_argument("--state", default="TRACK")
    parser.add_argument("--pos", type=float, default=0.0)
    parser.add_argument("--pred", type=float)
    parser.add_argument("--velocity", type=float, default=0.0)
    parser.add_argument("--confidence", type=float, default=0.9)
    parser.add_argument("--measured", type=int, choices=(0, 1), default=1)
    args = parser.parse_args()

    if args.fps <= 0.0:
        raise ValueError("--fps must be positive")
    if not args.state or len(args.state) >= 12 or not all(
        value.isalnum() or value in "_-" for value in args.state
    ):
        raise ValueError("--state must use 1..11 letters/digits/_/-")

    serial_port = None
    if args.port:
        import serial

        serial_port = serial.Serial(args.port, args.baud, timeout=0.1)

    start = time.monotonic()
    predicted = args.pos if args.pred is None else args.pred
    try:
        for index in range(args.count):
            line = build_line(
                args.seq + index,
                int((time.monotonic() - start) * 1000.0),
                args.valid,
                args.state,
                args.pos,
                predicted,
                args.velocity,
                args.confidence,
                args.measured,
            )
            print(line.decode("ascii").rstrip())
            if serial_port is not None:
                serial_port.write(line)
                serial_port.flush()
            if index + 1 < args.count:
                time.sleep(1.0 / args.fps)
    finally:
        if serial_port is not None:
            serial_port.close()


if __name__ == "__main__":
    main()
