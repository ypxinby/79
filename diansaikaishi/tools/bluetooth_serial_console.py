#!/usr/bin/env python3
"""Interactive HC-06 Bluetooth SPP serial console.

The same script works with a Windows Bluetooth COM port and a Raspberry Pi
RFCOMM device. Vehicle commands are sent explicitly with ``cmd`` so ordinary
terminal input cannot accidentally become a motion command.
"""

from __future__ import annotations

import argparse
import re
import string
import sys
import threading
import time
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
BLUETOOTH_SPP_UUID = "{00001101-0000-1000-8000-00805F9B34FB}"
BLUETOOTH_ADDRESS_PATTERN = re.compile(
    r"&([0-9A-F]{12})_C[0-9A-F]{8}$", re.IGNORECASE
)


def normalized_identifier(text: str) -> str:
    return "".join(character for character in text.upper() if character in string.hexdigits.upper())


def available_ports() -> list:
    return list(list_ports.comports())


def bluetooth_address_from_hwid(hwid: str) -> Optional[str]:
    match = BLUETOOTH_ADDRESS_PATTERN.search(hwid or "")
    if not match:
        return None

    address = match.group(1).upper()
    return None if address == "000000000000" else address


def formatted_bluetooth_address(address: str) -> str:
    return ":".join(address[index : index + 2] for index in range(0, 12, 2))


def decode_windows_bluetooth_name(value: object) -> str:
    if isinstance(value, str):
        return value.rstrip("\x00")
    if not isinstance(value, bytes):
        return ""

    for encoding in ("utf-8", "utf-16-le"):
        try:
            decoded = value.decode(encoding).rstrip("\x00")
        except UnicodeDecodeError:
            continue
        if decoded and decoded.isprintable():
            return decoded
    return ""


def windows_bluetooth_device_names() -> dict[str, str]:
    if sys.platform != "win32":
        return {}

    try:
        import winreg
    except ImportError:  # pragma: no cover - Windows always provides winreg
        return {}

    registry_path = (
        r"SYSTEM\CurrentControlSet\Services\BTHPORT\Parameters\Devices"
    )
    names: dict[str, str] = {}

    try:
        with winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, registry_path) as root:
            index = 0
            while True:
                try:
                    address = winreg.EnumKey(root, index).upper()
                except OSError:
                    break
                index += 1

                try:
                    with winreg.OpenKey(root, address) as device_key:
                        raw_name, _ = winreg.QueryValueEx(device_key, "Name")
                except OSError:
                    continue

                name = decode_windows_bluetooth_name(raw_name)
                if name:
                    names[address] = name
    except OSError:
        return {}

    return names


def bluetooth_spp_outgoing_ports() -> list[tuple[object, str, str]]:
    device_names = windows_bluetooth_device_names()
    candidates = []

    for port in available_ports():
        hwid = str(port.hwid or "")
        if BLUETOOTH_SPP_UUID not in hwid.upper():
            continue

        address = bluetooth_address_from_hwid(hwid)
        if not address:
            continue

        candidates.append(
            (port, address, device_names.get(address, "Unknown device"))
        )

    def sort_key(candidate: tuple[object, str, str]) -> tuple[int, int, str]:
        port, _, name = candidate
        number_match = re.search(r"(\d+)$", str(port.device))
        port_number = int(number_match.group(1)) if number_match else 99999
        hc_priority = 0 if name.upper().startswith("HC-") else 1
        return hc_priority, port_number, str(port.device)

    candidates.sort(key=sort_key)
    return candidates


def print_bluetooth_spp_ports(candidates: Iterable) -> None:
    candidates = list(candidates)
    if not candidates:
        print("No paired Bluetooth SPP outgoing COM ports were found.")
        return

    print("Paired Bluetooth SPP outgoing COM ports:")
    for index, (port, address, name) in enumerate(candidates, start=1):
        print(
            f"  [{index}] {port.device:<7} {name:<24} "
            f"{formatted_bluetooth_address(address)}"
        )


def select_bluetooth_spp_port() -> Optional[str]:
    candidates = bluetooth_spp_outgoing_ports()
    print_bluetooth_spp_ports(candidates)
    if not candidates:
        print(
            "Pair the HC-06 in Windows first so an outgoing SPP COM port "
            "is created."
        )
        return None

    print(
        "Only paired outgoing SPP ports are selectable. Close HC-PC and "
        "other serial tools before connecting."
    )
    while True:
        try:
            answer = input("Select device number (or q to quit): ").strip()
        except EOFError:
            return None

        if answer.lower() in {"q", "quit", "exit"}:
            return None
        if answer.isdigit():
            selection = int(answer)
            if 1 <= selection <= len(candidates):
                return str(candidates[selection - 1][0].device)
        print(f"Enter a number from 1 to {len(candidates)}, or q.")


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
    target_names = {target_name}
    if target_name == DEFAULT_DEVICE_NAME:
        target_names.add("HC-04")
    device_names = windows_bluetooth_device_names()
    matches = []

    for port in available_ports():
        port_address = bluetooth_address_from_hwid(str(port.hwid or ""))
        registered_name = device_names.get(port_address or "", "")
        metadata = " ".join(
            str(value or "")
            for value in (
                port.description,
                port.hwid,
                port.manufacturer,
                port.product,
                port.interface,
                registered_name,
            )
        )
        address_matches = bool(target_address) and (
            target_address == (port_address or "")
        )
        name_matches = any(
            candidate and candidate in metadata.upper()
            for candidate in target_names
        )
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


def read_one_shot_reply(port: serial.Serial, wait_seconds: float = 1.0) -> None:
    deadline = time.monotonic() + wait_seconds
    received = False

    while time.monotonic() < deadline:
        data = port.read(128)
        if not data:
            if received:
                return
            continue
        received = True
        print(
            f"[RX {len(data)} B] HEX={data.hex(' ').upper()} "
            f"TEXT={printable_text(data)!r}"
        )


def print_connection_failure_hint(exc: Exception) -> None:
    message = str(exc).casefold()
    if "拒绝访问" in message or "access is denied" in message:
        print(
            "Hint: this COM port is busy. Fully exit HC-PC and every other "
            "serial/Bluetooth terminal, then retry.",
            file=sys.stderr,
        )
    elif "信号灯超时" in message or "semaphore timeout" in message:
        print(
            "Hint: Windows could not establish the Bluetooth SPP link. "
            "Check module power, pairing, the outgoing-port direction, and "
            "make sure no phone or driver app is connected.",
            file=sys.stderr,
        )


def print_help() -> None:
    print(
        "Commands:\n"
        "  test              Send PING and expect PONG\n"
        "  cmd <command>     Send one MCU command followed by LF\n"
        "                    e.g. cmd MOVE,50,LOW or cmd TURN,-45\n"
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
                    send_bytes(port, b"PING\n")
                elif command == "cmd" and separator:
                    send_bytes(port, argument.encode("ascii") + b"\n")
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
        help="Serial port, for example COM8 or /dev/rfcomm0. Auto-detected when omitted.",
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
        "--scan",
        action="store_true",
        help="Scan paired outgoing Bluetooth SPP COM ports and select one",
    )
    parser.add_argument(
        "--send",
        metavar="TEXT",
        help="Send text once and exit",
    )
    parser.add_argument(
        "--command",
        metavar="COMMAND",
        help="Send one MCU command with LF and exit",
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

    if args.scan and args.port:
        print("Use either --scan or --port, not both.", file=sys.stderr)
        return 2

    one_shot_count = sum(
        value is not None
        for value in (args.send, args.command, args.hex_data)
    )
    if one_shot_count > 1:
        print("Use only one of --send, --command or --hex.", file=sys.stderr)
        return 2

    if args.scan:
        port_name = select_bluetooth_spp_port()
        if not port_name:
            return 0
    else:
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
            elif args.command is not None:
                send_bytes(port, args.command.encode("ascii") + b"\n")
                read_one_shot_reply(port)
            elif args.hex_data is not None:
                send_bytes(port, parse_hex_bytes(args.hex_data))
            else:
                interactive_console(port)
    except (ValueError, serial.SerialException) as exc:
        print(f"Connection failed: {exc}", file=sys.stderr)
        print_connection_failure_hint(exc)
        return 1

    print("Disconnected.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
