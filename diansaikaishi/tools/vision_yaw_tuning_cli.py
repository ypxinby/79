#!/usr/bin/env python3
"""PC command-line client for the MSPM0 Vision Yaw Tuning console."""

from __future__ import annotations

import argparse
import sys
import time
from typing import Optional, Sequence


COMMAND_PREFIX = "$VYT "


def build_wire_command(command: str) -> bytes:
    command = command.strip()
    if not command:
        raise ValueError("command cannot be empty")
    if "\r" in command or "\n" in command:
        raise ValueError("command must be a single line")
    if command.upper().startswith(COMMAND_PREFIX):
        wire = COMMAND_PREFIX + command[len(COMMAND_PREFIX) :]
    else:
        wire = COMMAND_PREFIX + command
    try:
        return (wire + "\r\n").encode("ascii")
    except UnicodeEncodeError as error:
        raise ValueError("command must contain ASCII characters only") from error


def run_self_test() -> None:
    vectors = {
        "SET DB 8": b"$VYT SET DB 8\r\n",
        "SET KP 0.080": b"$VYT SET KP 0.080\r\n",
        "SET MAXSPD 24.0": b"$VYT SET MAXSPD 24.0\r\n",
        "GET PARAM": b"$VYT GET PARAM\r\n",
        "DEFAULT": b"$VYT DEFAULT\r\n",
        "$VYT SET TIMEOUT 300": b"$VYT SET TIMEOUT 300\r\n",
        "$vyt get param": b"$VYT get param\r\n",
    }
    for command, expected in vectors.items():
        actual = build_wire_command(command)
        if actual != expected:
            raise AssertionError(f"wire mismatch for {command!r}: {actual!r}")
    print("SELF-TEST PASS")
    print(f"vectors={len(vectors)} prefix={COMMAND_PREFIX!r}")


def send_command(port: str, baud: int, timeout_s: float, command: str) -> int:
    try:
        import serial  # type: ignore
    except ImportError:
        print("error: pyserial is required: python -m pip install pyserial", file=sys.stderr)
        return 2

    wire = build_wire_command(command)
    print(f"TX {wire.decode('ascii').rstrip()}")

    try:
        with serial.Serial(port=port, baudrate=baud, timeout=0.05) as device:
            device.reset_input_buffer()
            device.write(wire)
            device.flush()

            deadline = time.monotonic() + timeout_s
            response = bytearray()
            while time.monotonic() < deadline:
                chunk = device.read(1)
                if not chunk:
                    continue
                response += chunk
                if chunk == b"\n":
                    break
    except serial.SerialException as error:
        print(f"error: serial communication failed: {error}", file=sys.stderr)
        return 2

    if not response:
        print("error: no response before timeout", file=sys.stderr)
        return 1

    try:
        decoded = response.decode("ascii").strip()
    except UnicodeDecodeError:
        print(f"RX_HEX {response.hex(' ').upper()}")
        return 1

    print(f"RX {decoded}")
    return 0 if decoded.startswith(("OK ", "PARAM ")) else 1


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Vision Yaw Tuning serial command client"
    )
    parser.add_argument("command", nargs="*", help='for example: "SET KP 0.080"')
    parser.add_argument("--port", help="serial port, for example COM7")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=1.0)
    parser.add_argument("--self-test", action="store_true")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    if args.self_test:
        run_self_test()
        return 0
    if not args.port:
        parser.error("--port is required unless --self-test is used")
    if args.baud <= 0:
        parser.error("--baud must be greater than zero")
    if args.timeout <= 0.0:
        parser.error("--timeout must be greater than zero")
    if not args.command:
        parser.error("a tuning command is required")

    command = " ".join(args.command)
    try:
        return send_command(args.port, args.baud, args.timeout, command)
    except ValueError as error:
        parser.error(str(error))
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
