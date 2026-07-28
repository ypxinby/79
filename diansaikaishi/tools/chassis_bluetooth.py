#!/usr/bin/env python3
"""Blocking HC-06 chassis control API for Raspberry Pi and Windows.

The class is intentionally single-command/single-caller.  MOVE, TURN and
MAGNET use the idempotent ``CMD,<sequence>,...`` protocol.  Heartbeats are
sent while a sequenced motion is running, and no exception path silently
continues a mission.
"""

from __future__ import annotations

import argparse
from collections import deque
from dataclasses import dataclass
import re
import time
from typing import Deque, Optional

try:
    import serial
except ImportError as exc:  # pragma: no cover - host dependency
    raise SystemExit(
        "pyserial is required. Install it with: pip install pyserial"
    ) from exc


DEFAULT_BAUD = 9600
LINK_SETTLE_SECONDS = 0.5
FRAME_CLEAR_SECONDS = 0.15
SYNC_ATTEMPTS = 3
SYNC_TIMEOUT_SECONDS = 1.0
ACK_RETRY_SECONDS = 0.75
MAX_COMMAND_SENDS = 3
HEARTBEAT_INTERVAL_SECONDS = 0.4
RECOVERY_TIMEOUT_SECONDS = 1.5
SEQUENCE_MAX = 0xFFFFFFFF


class ChassisError(RuntimeError):
    """Base class for chassis link and command failures."""


class ChassisLinkError(ChassisError):
    """Serial/SPP connection failed or disappeared."""


class ChassisProtocolError(ChassisError):
    """The peer returned malformed or incompatible protocol data."""


class ChassisCommandError(ChassisError):
    """A command was rejected or finished with ERR/CANCELLED."""

    def __init__(
        self, response: str, status: Optional[str] = None
    ) -> None:
        self.response = response
        self.status = status
        suffix = f"; status={status}" if status else ""
        super().__init__(f"chassis command failed: {response}{suffix}")


class ChassisTimeoutError(ChassisCommandError):
    """The host deadline expired and completion could not be confirmed."""


@dataclass(frozen=True)
class CommandResult:
    sequence: int
    command: str
    acknowledgement: Optional[str]
    final_response: str
    elapsed_seconds: float


def _has_prefix(line: str, prefix: str) -> bool:
    upper = line.upper()
    expected = prefix.upper()
    return upper == expected or upper.startswith(expected + ",")


def _keyed_fields(line: str) -> dict[str, str]:
    fields: dict[str, str] = {}
    for token in line.split(","):
        key, separator, value = token.partition("=")
        if separator:
            fields[key.strip().upper()] = value.strip().upper()
    return fields


def _requires_reset(response: str, command_name: str) -> bool:
    upper = response.upper()
    if any(
        marker in upper
        for marker in ("RESET_REQUIRED", "LINK_TIMEOUT", "LINK_LOST")
    ):
        return True
    tokens = [token.strip() for token in upper.split(",")]
    try:
        command_index = tokens.index(command_name.upper())
    except ValueError:
        return False
    return (
        command_index + 1 < len(tokens)
        and tokens[command_index + 1].isdigit()
    )


class ChassisBluetooth:
    """One-at-a-time blocking controller for the 3507 chassis protocol."""

    def __init__(
        self,
        port: str,
        baud: int = DEFAULT_BAUD,
        *,
        verbose: bool = False,
    ) -> None:
        self.port_name = port
        self.baud = baud
        self.verbose = verbose
        self._serial: Optional[serial.Serial] = None
        self._rx_buffer = bytearray()
        self._lines: Deque[str] = deque()
        self._next_sequence = 1

    def __enter__(self) -> "ChassisBluetooth":
        self.connect()
        return self

    def __exit__(self, exc_type, exc, traceback) -> None:
        self.close()

    @property
    def connected(self) -> bool:
        return self._serial is not None and self._serial.is_open

    def _log(self, text: str) -> None:
        if self.verbose:
            print(text)

    def connect(self) -> None:
        if self.connected:
            return
        try:
            self._serial = serial.Serial(
                self.port_name,
                self.baud,
                timeout=0.10,
                write_timeout=1.0,
                rtscts=False,
                dsrdtr=False,
            )
            self._synchronize_link()
            self._prepare_new_session()
        except ChassisError:
            self.close()
            raise
        except (serial.SerialException, OSError) as exc:
            self.close()
            raise ChassisLinkError(str(exc)) from exc

    def close(self) -> None:
        port = self._serial
        self._serial = None
        self._rx_buffer.clear()
        self._lines.clear()
        if port is not None:
            try:
                port.close()
            except (serial.SerialException, OSError):
                pass

    def _port(self) -> serial.Serial:
        if not self.connected or self._serial is None:
            raise ChassisLinkError("chassis serial port is not connected")
        return self._serial

    def _write_line(self, command: str, *, heartbeat: bool = False) -> None:
        data = command.encode("ascii") + b"\n"
        try:
            port = self._port()
            port.write(data)
            port.flush()
        except (serial.SerialException, OSError) as exc:
            raise ChassisLinkError(str(exc)) from exc
        if not heartbeat:
            self._log(f"TX {command}")

    def _publish_data(self, data: bytes) -> None:
        for byte in data:
            if byte in (0x0A, 0x0D):
                if not self._rx_buffer:
                    continue
                line = self._rx_buffer.decode(
                    "ascii", errors="replace"
                ).strip()
                self._rx_buffer.clear()
                if line:
                    self._lines.append(line)
                    self._log(f"RX {line}")
            elif len(self._rx_buffer) < 512:
                self._rx_buffer.append(byte)
            else:
                self._rx_buffer.clear()
                raise ChassisProtocolError("reply line exceeded 512 bytes")

    def _read_line(
        self,
        deadline: float,
        *,
        heartbeat: bool = False,
        next_heartbeat: Optional[float] = None,
    ) -> tuple[Optional[str], Optional[float]]:
        if heartbeat and next_heartbeat is None:
            next_heartbeat = (
                time.monotonic() + HEARTBEAT_INTERVAL_SECONDS
            )

        while time.monotonic() < deadline:
            if self._lines:
                return self._lines.popleft(), next_heartbeat

            now = time.monotonic()
            if (
                heartbeat
                and next_heartbeat is not None
                and now >= next_heartbeat
            ):
                self._write_line("HB", heartbeat=True)
                next_heartbeat = now + HEARTBEAT_INTERVAL_SECONDS
                continue

            read_deadline = deadline
            if heartbeat and next_heartbeat is not None:
                read_deadline = min(read_deadline, next_heartbeat)
            remaining = max(0.0, read_deadline - now)
            port = self._port()
            port.timeout = min(0.10, remaining)
            try:
                data = port.read(128)
            except (serial.SerialException, OSError) as exc:
                raise ChassisLinkError(str(exc)) from exc
            if data:
                self._publish_data(data)

        return None, next_heartbeat

    def _clear_receive(self) -> None:
        self._rx_buffer.clear()
        self._lines.clear()
        try:
            self._port().reset_input_buffer()
        except (serial.SerialException, OSError) as exc:
            raise ChassisLinkError(str(exc)) from exc

    def _wait_prefix(self, prefix: str, timeout: float) -> str:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            line, _ = self._read_line(deadline)
            if line is None:
                break
            if _has_prefix(line, prefix):
                return line
        raise ChassisTimeoutError(f"HOST_TIMEOUT,{prefix}")

    def _synchronize_link(self) -> None:
        self._log("waiting for SPP link to settle")
        time.sleep(LINK_SETTLE_SECONDS)
        self._clear_receive()
        self._write_line("")
        time.sleep(FRAME_CLEAR_SECONDS)
        self._clear_receive()

        for attempt in range(1, SYNC_ATTEMPTS + 1):
            self._write_line("PING")
            try:
                self._wait_prefix("PONG", SYNC_TIMEOUT_SECONDS)
                self._log(f"link synchronized on attempt {attempt}")
                return
            except ChassisTimeoutError:
                if attempt == SYNC_ATTEMPTS:
                    raise ChassisLinkError("3507 did not answer PING")
                self._clear_receive()
                self._write_line("")
                time.sleep(FRAME_CLEAR_SECONDS)
                self._clear_receive()

    def _prepare_new_session(self) -> None:
        status = self.status()
        fields = _keyed_fields(status)

        if fields.get("CR") == "RUN":
            self._log("stale remote action found; enforcing STOP")
            self.stop()
            status = self.status()
            fields = _keyed_fields(status)
        if fields.get("RL") == "1":
            self._log("previous link timeout found; resetting chassis")
            self.reset()
            status = self.status()
            fields = _keyed_fields(status)

        if "CS" not in fields:
            raise ChassisProtocolError(
                "STATUS has no CS field; flash firmware with CMD support"
            )
        try:
            cached_sequence = int(fields["CS"], 10)
        except ValueError as exc:
            raise ChassisProtocolError("invalid STATUS CS field") from exc
        if not 0 <= cached_sequence <= SEQUENCE_MAX:
            raise ChassisProtocolError("STATUS CS is out of range")
        self._next_sequence = (
            1 if cached_sequence == SEQUENCE_MAX else cached_sequence + 1
        )

    def _take_sequence(self) -> int:
        sequence = self._next_sequence
        self._next_sequence = 1 if sequence == SEQUENCE_MAX else sequence + 1
        return sequence

    def _immediate(self, command: str, expected: str, timeout: float) -> str:
        self._clear_receive()
        self._write_line(command)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            line, _ = self._read_line(deadline)
            if line is None:
                break
            if _has_prefix(line, expected):
                return line
            if line.upper().startswith("ERR,") or line.upper().startswith(
                "CANCELLED"
            ):
                raise ChassisCommandError(line)
        raise ChassisTimeoutError(f"HOST_TIMEOUT,{command}")

    def ping(self) -> str:
        return self._immediate("PING", "PONG", 1.0)

    def status(self) -> str:
        return self._immediate("STATUS", "STATUS", 1.5)

    def stop(self) -> str:
        return self._immediate("STOP", "ACK,STOP", 1.5)

    def estop(self) -> str:
        return self._immediate("ESTOP", "ACK,ESTOP", 1.5)

    def reset(self) -> str:
        return self._immediate("RESET", "ACK,RESET", 1.5)

    def speed(self, gear: str) -> str:
        normalized = gear.strip().upper()
        if normalized not in {"LOW", "HIGH"}:
            raise ValueError("gear must be LOW or HIGH")
        return self._immediate(
            f"SPEED,{normalized}", f"ACK,SPEED,{normalized}", 1.5
        )

    def _is_relevant(self, line: str, sequence: int) -> bool:
        return any(
            _has_prefix(line, f"{kind},{sequence}")
            for kind in ("ACK", "DONE", "ERR", "CANCELLED")
        )

    def _wait_relevant(
        self,
        deadline: float,
        sequence: int,
        *,
        heartbeat: bool,
        next_heartbeat: Optional[float],
    ) -> tuple[Optional[str], Optional[float]]:
        while time.monotonic() < deadline:
            line, next_heartbeat = self._read_line(
                deadline,
                heartbeat=heartbeat,
                next_heartbeat=next_heartbeat,
            )
            if line is None:
                return None, next_heartbeat
            if self._is_relevant(line, sequence):
                return line, next_heartbeat
        return None, next_heartbeat

    def _best_effort(self, operation) -> Optional[str]:
        try:
            return operation()
        except ChassisError:
            return None

    def _raise_command_error(
        self, command_name: str, response: str
    ) -> None:
        upper = response.upper()
        protocol_rejection = any(
            marker in upper
            for marker in (
                "BUSY",
                "SEQ_CONFLICT",
                "UNSUPPORTED",
                "PARAM",
                "ANGLE",
                "SPEED_CONFIG",
                "CACHE_EMPTY",
            )
        )
        if command_name in {"MOVE", "TURN"} and not protocol_rejection:
            self._best_effort(self.stop)
        elif command_name == "MAGNET" and "HARDWARE" in upper:
            self._best_effort(lambda: self._immediate(
                "MAGNET,OFF", "ACK,MAGNET,OFF", 1.5
            ))

        snapshot = self._best_effort(self.status)
        if _requires_reset(response, command_name):
            self._best_effort(self.reset)
        raise ChassisCommandError(response, snapshot)

    def _recover_motion_timeout(
        self,
        wire_command: str,
        sequence: int,
        command_name: str,
        acknowledgement: Optional[str],
        started: float,
    ) -> CommandResult:
        self._best_effort(self.stop)
        self._clear_receive()
        self._write_line(wire_command)
        deadline = time.monotonic() + RECOVERY_TIMEOUT_SECONDS
        while time.monotonic() < deadline:
            line, _ = self._wait_relevant(
                deadline,
                sequence,
                heartbeat=False,
                next_heartbeat=None,
            )
            if line is None:
                break
            if _has_prefix(line, f"DONE,{sequence},{command_name}"):
                return CommandResult(
                    sequence,
                    command_name,
                    acknowledgement,
                    line,
                    time.monotonic() - started,
                )
            self._raise_command_error(command_name, line)

        snapshot = self._best_effort(self.status)
        if snapshot and _keyed_fields(snapshot).get("RL") == "1":
            self._best_effort(self.reset)
        raise ChassisTimeoutError(
            f"HOST_TIMEOUT,{sequence},{command_name}", snapshot
        )

    def _execute_reliable(
        self, command: str, timeout: float
    ) -> CommandResult:
        command_name = command.split(",", 1)[0].strip().upper()
        if command_name not in {"MOVE", "TURN", "MAGNET"}:
            raise ValueError("reliable command must be MOVE, TURN or MAGNET")

        sequence = self._take_sequence()
        wire_command = f"CMD,{sequence},{command}"
        ack_prefix = f"ACK,{sequence},{command_name}"
        done_prefix = f"DONE,{sequence},{command_name}"
        is_motion = command_name in {"MOVE", "TURN"}
        started = time.monotonic()
        action_deadline = started + timeout
        next_heartbeat: Optional[float] = (
            started + HEARTBEAT_INTERVAL_SECONDS if is_motion else None
        )
        acknowledgement: Optional[str] = None

        self._clear_receive()
        try:
            for attempt in range(1, MAX_COMMAND_SENDS + 1):
                self._write_line(wire_command)
                reply_deadline = min(
                    action_deadline,
                    time.monotonic() + ACK_RETRY_SECONDS,
                )
                while time.monotonic() < reply_deadline:
                    line, next_heartbeat = self._wait_relevant(
                        reply_deadline,
                        sequence,
                        heartbeat=is_motion,
                        next_heartbeat=next_heartbeat,
                    )
                    if line is None:
                        break
                    if _has_prefix(line, done_prefix):
                        return CommandResult(
                            sequence,
                            command_name,
                            acknowledgement,
                            line,
                            time.monotonic() - started,
                        )
                    if _has_prefix(line, ack_prefix):
                        acknowledgement = line
                        break
                    self._raise_command_error(command_name, line)
                if acknowledgement is not None:
                    break
                self._log(
                    f"no ACK for CMD,{sequence}; retry {attempt}/"
                    f"{MAX_COMMAND_SENDS}"
                )

            if acknowledgement is None:
                if is_motion:
                    return self._recover_motion_timeout(
                        wire_command,
                        sequence,
                        command_name,
                        acknowledgement,
                        started,
                    )
                self._best_effort(lambda: self._immediate(
                    "MAGNET,OFF", "ACK,MAGNET,OFF", 1.5
                ))
                raise ChassisTimeoutError(
                    f"HOST_ACK_TIMEOUT,{sequence},{command_name}"
                )

            if not is_motion:
                return CommandResult(
                    sequence,
                    command_name,
                    acknowledgement,
                    acknowledgement,
                    time.monotonic() - started,
                )

            while time.monotonic() < action_deadline:
                line, next_heartbeat = self._wait_relevant(
                    action_deadline,
                    sequence,
                    heartbeat=True,
                    next_heartbeat=next_heartbeat,
                )
                if line is None:
                    break
                if _has_prefix(line, done_prefix):
                    return CommandResult(
                        sequence,
                        command_name,
                        acknowledgement,
                        line,
                        time.monotonic() - started,
                    )
                if _has_prefix(line, ack_prefix):
                    continue
                self._raise_command_error(command_name, line)

            return self._recover_motion_timeout(
                wire_command,
                sequence,
                command_name,
                acknowledgement,
                started,
            )
        except KeyboardInterrupt:
            if is_motion:
                self._best_effort(self.stop)
            elif command_name == "MAGNET":
                self._best_effort(lambda: self._immediate(
                    "MAGNET,OFF", "ACK,MAGNET,OFF", 1.5
                ))
            raise

    def move(
        self,
        distance_cm: int,
        gear: str = "LOW",
        *,
        timeout: Optional[float] = None,
    ) -> CommandResult:
        normalized_gear = gear.strip().upper()
        if distance_cm == 0 or not -1000 <= distance_cm <= 1000:
            raise ValueError("distance_cm must be -1000..-1 or 1..1000")
        if normalized_gear not in {"LOW", "HIGH"}:
            raise ValueError("gear must be LOW or HIGH")
        speed_cmps = 25.0 if normalized_gear == "LOW" else 45.0
        mcu_timeout = max(
            3.0, abs(distance_cm) / speed_cmps * 2.0 + 2.0
        )
        host_timeout = timeout if timeout is not None else mcu_timeout + 1.5
        return self._execute_reliable(
            f"MOVE,{distance_cm},{normalized_gear}", host_timeout
        )

    def turn(
        self, angle_deg: int, *, timeout: Optional[float] = None
    ) -> CommandResult:
        if angle_deg not in {-90, -45, 45, 90, 180}:
            raise ValueError("angle_deg must be -90, -45, 45, 90 or 180")
        mcu_timeout = 4.0 + abs(angle_deg) * 0.04
        host_timeout = timeout if timeout is not None else mcu_timeout + 2.0
        return self._execute_reliable(f"TURN,{angle_deg}", host_timeout)

    def magnet(
        self, enabled: bool, *, timeout: float = 2.0
    ) -> CommandResult:
        return self._execute_reliable(
            "MAGNET,ON" if enabled else "MAGNET,OFF", timeout
        )


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Connect to the 3507 chassis and print STATUS"
    )
    parser.add_argument(
        "--port", required=True, help="COM9 or /dev/rfcomm0"
    )
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    try:
        with ChassisBluetooth(
            args.port, args.baud, verbose=args.verbose
        ) as chassis:
            print(chassis.status())
    except ChassisError as exc:
        print(f"Chassis connection failed: {exc}")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
