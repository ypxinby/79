#!/usr/bin/env python3
"""Interactive HC-06 Bluetooth SPP serial console.

The same script works with a Windows Bluetooth COM port and a Raspberry Pi
RFCOMM device.  It intentionally sends only explicit ``send``, ``line`` or
``hex`` commands so an accidental terminal input cannot become a vehicle
motion command before the MCU action protocol is implemented.
"""

from __future__ import annotations

import argparse
import string
import sys
import threading
from typing import Iterable, Optional

try:
    import serial
    from serial.tools import list_ports
except ImportError as exc:  # pragma: no cover - depends on host installation
    raise SystemExit(
        "pyserial is required. Install it with: pip install pyserial"
    ) from exc


DEFAULT_BAUD = 9600
DEFAULT_DEVICE_NAME = "HC-06"


def normalized_identifier(text: str) -> str:
    return "".join(character for character in text.upper() if character in string.hexdigits.upper())


def available_ports() -> list:
    return list(list_ports.comports())


def print_ports(ports: Iterable) -> None:
    ports = list(ports)
    if not ports:
        print("No serial ports found.")
        return

    for port in ports:
        print(f"{port.device:8}  {port.description}")
        print(f"          {port.hwid}")


def find_hc06_port(name: str, address: Optional[str]) -> Optional[str]:
    target_address = normalized_identifier(address or "")
    target_name = name.strip().upper()
    matches = []

    for port in available_ports():
        metadata = " ".join(
            str(value or "")
            for value in (
                port.description,
                port.hwid,
                port.manufacturer,
                port.product,
                port.interface,
            )
        )
        address_matches = target_address and (
            target_address in normalized_identifier(port.hwid)
        )
        name_matches = target_name and target_name in metadata.upper()
        if address_matches or name_matches:
            matches.append(port.device)

    return matches[0] if len(matches) == 1 else None


def parse_hex_bytes(text: str) -> bytes:
    compact = text.replace("0x", "").replace("0X", "")
    try:
        return bytes.fromhex(compact)
    except ValueError as exc:
        raise ValueError("hex data must look like: 41 or AA 55 01") from exc


def printable_text(data: bytes) -> str:
    return "".join(chr(byte) if 32 <= byte <= 126 else "." for byte in data)


def print_tx(data: bytes) -> None:
    print(
        f"[TX {len(data)} B] HEX={data.hex(' ').upper()} "
        f"TEXT={printable_text(data)!r}"
    )


def reader_loop(port: serial.Serial, stop_event: threading.Event) -> None:
    while not stop_event.is_set():
        try:
            data = port.read(128)
        except serial.SerialException as exc:
            print(f"\n[RX ERROR] {exc}")
            stop_event.set()
            return

        if data:
            print(
                f"\n[RX {len(data)} B] HEX={data.hex(' ').upper()} "
                f"TEXT={printable_text(data)!r}"
            )


def send_bytes(port: serial.Serial, data: bytes) -> None:
    if not data:
        print("Nothing sent: payload is empty.")
        return

    port.write(data)
    port.flush()
    print_tx(data)


def print_help() -> None:
    print(
        "Commands:\n"
        "  test              Send one ASCII 'A' byte for the MCU BT OLED test\n"
        "  send <text>       Send text exactly, without a line ending\n"
        "  line <text>       Send text followed by LF (0x0A)\n"
        "  hex <bytes>       Send hexadecimal bytes, e.g. hex AA 55 01\n"
        "  ports             List currently available serial ports\n"
        "  help              Show this help\n"
        "  quit              Close the SPP connection and exit"
    )


def interactive_console(port: serial.Serial) -> None:
    stop_event = threading.Event()
    reader = threading.Thread(
        target=reader_loop,
        args=(port, stop_event),
        name="bluetooth-reader",
        daemon=True,
    )
    reader.start()
    print_help()

    try:
        while not stop_event.is_set():
            try:
                command_line = input("bt> ").strip()
            except EOFError:
                break

            if not command_line:
                continue

            command, separator, argument = command_line.partition(" ")
            command = command.lower()

            try:
                if command in {"quit", "exit"}:
                    break
                if command == "help":
                    print_help()
                elif command == "ports":
                    print_ports(available_ports())
                elif command == "test" and not separator:
                    send_bytes(port, b"A")
                elif command == "send" and separator:
                    send_bytes(port, argument.encode("utf-8"))
                elif command == "line" and separator:
                    send_bytes(port, argument.encode("utf-8") + b"\n")
                elif command == "hex" and separator:
                    send_bytes(port, parse_hex_bytes(argument))
                else:
                    print("Unknown or incomplete command. Type 'help'.")
            except (ValueError, serial.SerialException) as exc:
                print(f"[COMMAND ERROR] {exc}")
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        stop_event.set()
        reader.join(timeout=0.5)


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="HC-06 Bluetooth SPP serial console"
    )
    parser.add_argument(
        "--port",
        help="Serial port, for example COM15 or /dev/rfcomm0. Auto-detected when omitted.",
    )
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument(
        "--address",
        help="HC-06 Bluetooth address used for Windows COM auto-detection",
    )
    parser.add_argument(
        "--name",
        default=DEFAULT_DEVICE_NAME,
        help="HC-06 device name used for COM auto-detection",
    )
    parser.add_argument(
        "--list",
        action="store_true",
        help="List serial ports and exit",
    )
    parser.add_argument(
        "--send",
        metavar="TEXT",
        help="Send text once and exit",
    )
    parser.add_argument(
        "--hex",
        dest="hex_data",
        metavar="BYTES",
        help='Send hexadecimal bytes once and exit, e.g. --hex "41"',
    )
    return parser


def main() -> int:
    args = build_argument_parser().parse_args()

    if args.list:
        print_ports(available_ports())
        return 0

    if args.send is not None and args.hex_data is not None:
        print("Use only one of --send or --hex.", file=sys.stderr)
        return 2

    port_name = args.port or find_hc06_port(args.name, args.address)
    if not port_name:
        print(
            "A unique HC-06 outgoing serial port was not found. Use --list "
            "and specify --port explicitly; alternatively provide its "
            "Bluetooth address with --address.",
            file=sys.stderr,
        )
        return 1

    print(f"Opening {port_name} at {args.baud} baud...")
    try:
        with serial.Serial(
            port_name,
            args.baud,
            timeout=0.10,
            write_timeout=1.0,
            rtscts=False,
            dsrdtr=False,
        ) as port:
            print(f"Connected: {port.name}")
            if args.send is not None:
                send_bytes(port, args.send.encode("utf-8"))
            elif args.hex_data is not None:
                send_bytes(port, parse_hex_bytes(args.hex_data))
            else:
                interactive_console(port)
    except (ValueError, serial.SerialException) as exc:
        print(f"Connection failed: {exc}", file=sys.stderr)
        return 1

    print("Disconnected.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
