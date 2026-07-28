#!/usr/bin/env python3
"""Interactive HC-06 Bluetooth SPP serial console.

The same script works with a Windows Bluetooth COM port and a Raspberry Pi
RFCOMM device. Vehicle commands are sent explicitly with ``cmd`` so ordinary
terminal input cannot accidentally become a motion command.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
from pathlib import Path
from queue import Empty, Queue
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
PLAN_ACK_TIMEOUT_SECONDS = 2.0
PLAN_RECOVERY_TIMEOUT_SECONDS = 1.5
PLAN_MAX_ACTION_TIMEOUT_SECONDS = 600.0
PLAN_ALLOWED_COMMANDS = {
    "PING",
    "STATUS",
    "MOTOR",
    "SPEED",
    "MOVE",
    "TURN",
    "STOP",
    "ESTOP",
    "RESET",
    "MAGNET",
    "WAIT",
}


@dataclass(frozen=True)
class PlanStep:
    line_number: int
    timeout_seconds: float
    command: str


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


class SerialLineReceiver:
    """Own serial RX and expose complete CR/LF-delimited MCU replies."""

    def __init__(self, port: serial.Serial) -> None:
        self.port = port
        self.stop_event = threading.Event()
        self.lines: Queue[str] = Queue()
        self.buffer = bytearray()
        self.thread = threading.Thread(
            target=self._reader_loop,
            name="bluetooth-reader",
            daemon=True,
        )

    def start(self) -> None:
        self.thread.start()

    def stop(self) -> None:
        self.stop_event.set()
        self.thread.join(timeout=0.5)

    def clear_lines(self) -> None:
        while True:
            try:
                self.lines.get_nowait()
            except Empty:
                return

    def wait_line(self, timeout_seconds: float) -> Optional[str]:
        if timeout_seconds <= 0.0:
            return None
        try:
            return self.lines.get(timeout=timeout_seconds)
        except Empty:
            return None

    def _publish_buffered_line(self) -> None:
        if not self.buffer:
            return
        line = self.buffer.decode("ascii", errors="replace").strip()
        self.buffer.clear()
        if line:
            self.lines.put(line)

    def _reader_loop(self) -> None:
        while not self.stop_event.is_set():
            try:
                data = self.port.read(128)
            except serial.SerialException as exc:
                print(f"\n[RX ERROR] {exc}")
                self.stop_event.set()
                return

            if not data:
                continue
            print(
                f"\n[RX {len(data)} B] HEX={data.hex(' ').upper()} "
                f"TEXT={printable_text(data)!r}"
            )
            for byte in data:
                if byte in (0x0A, 0x0D):
                    self._publish_buffered_line()
                elif len(self.buffer) < 512:
                    self.buffer.append(byte)
                else:
                    self.buffer.clear()
                    print("\n[RX ERROR] reply line exceeded 512 bytes")


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


def plan_command_name(command: str) -> str:
    return command.split(",", 1)[0].strip().upper()


def parse_wait_milliseconds(command: str) -> int:
    tokens = [token.strip() for token in command.split(",")]
    if len(tokens) != 2 or tokens[0].upper() != "WAIT":
        raise ValueError("WAIT must look like WAIT,500 (milliseconds)")
    try:
        milliseconds = int(tokens[1], 10)
    except ValueError as exc:
        raise ValueError("WAIT duration must be an integer number of milliseconds") from exc
    if milliseconds < 0:
        raise ValueError("WAIT duration cannot be negative")
    return milliseconds


def load_plan(path_text: str) -> list[PlanStep]:
    path = Path(path_text)
    try:
        source = path.read_text(encoding="utf-8-sig")
    except OSError as exc:
        raise ValueError(f"cannot read plan file {path}: {exc}") from exc

    steps = []
    for line_number, raw_line in enumerate(source.splitlines(), start=1):
        line = raw_line.split("#", 1)[0].strip()
        if not line:
            continue
        fields = line.split(None, 1)
        if len(fields) != 2:
            raise ValueError(
                f"{path}:{line_number}: expected '<timeout_seconds> <command>'"
            )
        timeout_text, command = fields
        try:
            timeout_seconds = float(timeout_text)
        except ValueError as exc:
            raise ValueError(
                f"{path}:{line_number}: invalid timeout {timeout_text!r}"
            ) from exc
        if (
            not math.isfinite(timeout_seconds)
            or timeout_seconds <= 0.0
            or timeout_seconds > PLAN_MAX_ACTION_TIMEOUT_SECONDS
        ):
            raise ValueError(
                f"{path}:{line_number}: timeout must be within "
                f"(0, {PLAN_MAX_ACTION_TIMEOUT_SECONDS:g}] seconds"
            )

        command = command.strip()
        try:
            command.encode("ascii")
        except UnicodeEncodeError as exc:
            raise ValueError(
                f"{path}:{line_number}: MCU command must contain ASCII only"
            ) from exc
        name = plan_command_name(command)
        if name not in PLAN_ALLOWED_COMMANDS:
            raise ValueError(
                f"{path}:{line_number}: unsupported plan command {name!r}"
            )
        if name == "WAIT":
            wait_seconds = parse_wait_milliseconds(command) / 1000.0
            if wait_seconds > timeout_seconds:
                raise ValueError(
                    f"{path}:{line_number}: WAIT duration exceeds its action timeout"
                )
        steps.append(PlanStep(line_number, timeout_seconds, command))

    if not steps:
        raise ValueError(f"plan file {path} contains no actions")
    return steps


def print_plan(steps: list[PlanStep]) -> None:
    print(f"Validated plan with {len(steps)} action(s):")
    for index, step in enumerate(steps, start=1):
        print(
            f"  {index:02d}. timeout={step.timeout_seconds:g}s  "
            f"{step.command}"
        )


def reply_is_failure(line: str) -> bool:
    upper = line.upper()
    return (
        upper in {"BUSY", "CANCELLED"}
        or upper.startswith("ERR,")
    )


def reply_has_prefix(line: str, prefix: str) -> bool:
    upper = line.upper()
    expected = prefix.upper()
    return upper == expected or upper.startswith(expected + ",")


def immediate_reply_prefix(command_name: str) -> str:
    prefixes = {
        "PING": "PONG",
        "STATUS": "STATUS",
        "MOTOR": "MOTOR",
        "SPEED": "ACK,SPEED",
        "STOP": "ACK,STOP",
        "ESTOP": "ACK,ESTOP",
        "RESET": "ACK,RESET",
        "MAGNET": "ACK,MAGNET",
    }
    return prefixes.get(command_name, "ACK," + command_name)


def mcu_timeout_from_ack(line: str) -> Optional[float]:
    match = re.search(r"(?:^|,)T=(\d+)(?:,|$)", line.upper())
    if not match:
        return None
    return int(match.group(1), 10) / 1000.0


def wait_for_plan_line(
    receiver: SerialLineReceiver, deadline: float
) -> Optional[str]:
    remaining = deadline - time.monotonic()
    return receiver.wait_line(remaining) if remaining > 0.0 else None


def request_status_snapshot(
    port: serial.Serial, receiver: SerialLineReceiver
) -> None:
    print("[PLAN] Requesting STATUS snapshot...")
    receiver.clear_lines()
    send_bytes(port, b"STATUS\n")
    deadline = time.monotonic() + PLAN_RECOVERY_TIMEOUT_SECONDS
    while time.monotonic() < deadline:
        line = wait_for_plan_line(receiver, deadline)
        if line is None:
            break
        if reply_has_prefix(line, "STATUS"):
            print(f"[PLAN STATUS] {line}")
            return
        print(f"[PLAN STATUS IGNORE] {line}")
    print("[PLAN STATUS] No STATUS reply received.")


def send_recovery_command(
    port: serial.Serial,
    receiver: SerialLineReceiver,
    command: str,
    expected_prefix: str,
) -> None:
    receiver.clear_lines()
    send_bytes(port, command.encode("ascii") + b"\n")
    deadline = time.monotonic() + PLAN_RECOVERY_TIMEOUT_SECONDS
    while time.monotonic() < deadline:
        line = wait_for_plan_line(receiver, deadline)
        if line is None:
            break
        if reply_has_prefix(line, expected_prefix) or reply_is_failure(line):
            print(f"[PLAN RECOVERY] {line}")
            return
        print(f"[PLAN RECOVERY IGNORE] {line}")
    print(f"[PLAN RECOVERY] No reply to {command}.")


def recover_from_plan_timeout(
    port: serial.Serial,
    receiver: SerialLineReceiver,
    command_name: str,
) -> None:
    if command_name in {"MOVE", "TURN"}:
        print("[PLAN] Action timed out; sending STOP and aborting the queue.")
        send_recovery_command(port, receiver, "STOP", "ACK,STOP")
    elif command_name == "MAGNET":
        print(
            "[PLAN] Magnet command timed out; forcing MAGNET,OFF and "
            "aborting the queue."
        )
        send_recovery_command(
            port, receiver, "MAGNET,OFF", "ACK,MAGNET,OFF"
        )
    request_status_snapshot(port, receiver)


def execute_plan_step(
    port: serial.Serial,
    receiver: SerialLineReceiver,
    step: PlanStep,
) -> tuple[bool, str]:
    command_name = plan_command_name(step.command)
    if command_name == "WAIT":
        wait_seconds = parse_wait_milliseconds(step.command) / 1000.0
        print(f"[PLAN] Local wait for {wait_seconds:g}s")
        started = time.monotonic()
        time.sleep(wait_seconds)
        if (time.monotonic() - started) > step.timeout_seconds:
            return False, "HOST_TIMEOUT"
        return True, "WAIT complete"

    receiver.clear_lines()
    send_bytes(port, step.command.encode("ascii") + b"\n")
    action_deadline = time.monotonic() + step.timeout_seconds

    if command_name not in {"MOVE", "TURN"}:
        expected = immediate_reply_prefix(command_name)
        while time.monotonic() < action_deadline:
            line = wait_for_plan_line(receiver, action_deadline)
            if line is None:
                break
            if reply_is_failure(line):
                return False, line
            if reply_has_prefix(line, expected):
                return True, line
            print(f"[PLAN IGNORE] {line}")
        return False, "HOST_TIMEOUT"

    ack_deadline = min(
        action_deadline, time.monotonic() + PLAN_ACK_TIMEOUT_SECONDS
    )
    ack_prefix = "ACK," + command_name
    done_prefix = "DONE," + command_name
    ack_received = False
    while time.monotonic() < ack_deadline:
        line = wait_for_plan_line(receiver, ack_deadline)
        if line is None:
            break
        if reply_is_failure(line):
            return False, line
        if reply_has_prefix(line, done_prefix):
            return True, line
        if reply_has_prefix(line, ack_prefix):
            mcu_timeout = mcu_timeout_from_ack(line)
            if (
                mcu_timeout is not None
                and step.timeout_seconds <= mcu_timeout
            ):
                print(
                    "[PLAN WARNING] Host timeout is not greater than MCU "
                    f"timeout ({mcu_timeout:g}s)."
                )
            print(f"[PLAN ACK] {line}")
            ack_received = True
            break
        print(f"[PLAN ACK IGNORE] {line}")
    if not ack_received:
        return False, "HOST_ACK_TIMEOUT"

    while time.monotonic() < action_deadline:
        line = wait_for_plan_line(receiver, action_deadline)
        if line is None:
            break
        if reply_is_failure(line):
            return False, line
        if reply_has_prefix(line, done_prefix):
            return True, line
        print(f"[PLAN DONE IGNORE] {line}")
    return False, "HOST_TIMEOUT"


def run_plan(
    port: serial.Serial,
    receiver: SerialLineReceiver,
    steps: list[PlanStep],
) -> bool:
    print(f"[PLAN] Starting {len(steps)} action(s).")
    for index, step in enumerate(steps, start=1):
        print(
            f"[PLAN {index}/{len(steps)}] timeout={step.timeout_seconds:g}s "
            f"command={step.command}"
        )
        succeeded, detail = execute_plan_step(port, receiver, step)
        if succeeded:
            print(f"[PLAN {index}] OK: {detail}")
            continue

        print(f"[PLAN {index}] FAILED: {detail}")
        command_name = plan_command_name(step.command)
        if detail in {"HOST_TIMEOUT", "HOST_ACK_TIMEOUT"}:
            recover_from_plan_timeout(port, receiver, command_name)
        else:
            request_status_snapshot(port, receiver)
        print("[PLAN] Queue aborted; no later action was sent.")
        return False

    print("[PLAN] COMPLETE: all actions succeeded.")
    return True


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
        "  run <plan.txt>    Run timed actions sequentially; abort on timeout/ERR\n"
        "  check <plan.txt>  Validate a timed action plan without running it\n"
        "                    plan line format: <timeout_seconds> <command>\n"
        "                    e.g. 8 MOVE,50,LOW\n"
        "  send <text>       Send text exactly, without a line ending\n"
        "  line <text>       Send text followed by LF (0x0A)\n"
        "  hex <bytes>       Send hexadecimal bytes, e.g. hex AA 55 01\n"
        "  ports             List currently available serial ports\n"
        "  help              Show this help\n"
        "  quit              Close the SPP connection and exit"
    )


def interactive_console(port: serial.Serial) -> None:
    receiver = SerialLineReceiver(port)
    receiver.start()
    print_help()

    try:
        while not receiver.stop_event.is_set():
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
                elif command == "check" and separator:
                    print_plan(load_plan(argument))
                elif command == "run" and separator:
                    steps = load_plan(argument)
                    print_plan(steps)
                    run_plan(port, receiver, steps)
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
        print("\nInterrupted; sending STOP before leaving the console.")
        try:
            send_recovery_command(port, receiver, "STOP", "ACK,STOP")
        except serial.SerialException as exc:
            print(f"[PLAN RECOVERY ERROR] {exc}")
    finally:
        receiver.stop()


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
        "--plan",
        metavar="FILE",
        help="Run a timed action plan and exit",
    )
    parser.add_argument(
        "--check-plan",
        metavar="FILE",
        help="Validate a timed action plan without opening a serial port",
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

    if args.check_plan:
        if any(
            value is not None
            for value in (args.send, args.command, args.hex_data, args.plan)
        ):
            print(
                "--check-plan cannot be combined with a send or plan option.",
                file=sys.stderr,
            )
            return 2
        try:
            print_plan(load_plan(args.check_plan))
        except ValueError as exc:
            print(f"Plan validation failed: {exc}", file=sys.stderr)
            return 2
        return 0

    if args.scan and args.port:
        print("Use either --scan or --port, not both.", file=sys.stderr)
        return 2

    one_shot_count = sum(
        value is not None
        for value in (args.send, args.command, args.hex_data, args.plan)
    )
    if one_shot_count > 1:
        print(
            "Use only one of --send, --command, --hex or --plan.",
            file=sys.stderr,
        )
        return 2

    plan_steps = None
    if args.plan is not None:
        try:
            plan_steps = load_plan(args.plan)
            print_plan(plan_steps)
        except ValueError as exc:
            print(f"Plan validation failed: {exc}", file=sys.stderr)
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
    result_code = 0
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
            elif plan_steps is not None:
                receiver = SerialLineReceiver(port)
                receiver.start()
                try:
                    if not run_plan(port, receiver, plan_steps):
                        result_code = 3
                except KeyboardInterrupt:
                    print("\n[PLAN] Interrupted; sending STOP.")
                    send_recovery_command(
                        port, receiver, "STOP", "ACK,STOP"
                    )
                    request_status_snapshot(port, receiver)
                    result_code = 130
                finally:
                    receiver.stop()
            elif args.hex_data is not None:
                send_bytes(port, parse_hex_bytes(args.hex_data))
            else:
                interactive_console(port)
    except (ValueError, serial.SerialException) as exc:
        print(f"Connection failed: {exc}", file=sys.stderr)
        print_connection_failure_hint(exc)
        return 1

    print("Disconnected.")
    return result_code


if __name__ == "__main__":
    raise SystemExit(main())
