#!/usr/bin/env python3
"""Vision Target Protocol V1 PC test sender and offline frame generator."""

from __future__ import annotations

import argparse
import random
import secrets
import struct
import sys
import time
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Iterable, Optional, Sequence


MAGIC = b"\xA5\x5A"
VERSION = 0x01
MESSAGE_TYPE_TARGET_REPORT = 0x01
PAYLOAD_LENGTH = 30
FRAME_LENGTH = 40
MAX_PAYLOAD_LENGTH = 64

FLAG_TARGET_VALID = 1 << 0
FLAG_HAS_BBOX = 1 << 1
FLAG_HAS_TARGET_ID = 1 << 2
FLAG_HAS_CONFIDENCE = 1 << 3
FLAG_SOURCE_RESTART = 1 << 4
FLAG_RESERVED_MASK = 0xE0

HEADER_STRUCT = struct.Struct("<2sBBBBH")
PAYLOAD_STRUCT = struct.Struct("<IHIHHHHHHHHHH")
CRC_STRUCT = struct.Struct("<H")

FIXED_VALID_HEX = (
    "A5 5A 01 01 1F 00 1E 00 78 56 34 12 00 00 E8 03 00 00 "
    "80 02 E0 01 90 01 C8 00 6B 03 07 00 5E 01 96 00 64 00 "
    "64 00 4D D1"
)
FIXED_NO_TARGET_HEX = (
    "A5 5A 01 01 00 00 1E 00 78 56 34 12 01 00 09 04 00 00 "
    "80 02 E0 01 FF FF FF FF 00 00 FF FF 00 00 00 00 00 00 "
    "00 00 5C C7"
)


@dataclass(frozen=True)
class VisionTargetReport:
    session_id: int
    sequence: int
    timestamp_ms: int
    frame_width: int
    frame_height: int
    target_center_x: int
    target_center_y: int
    confidence: int
    target_id: int
    bbox_x: int
    bbox_y: int
    bbox_width: int
    bbox_height: int
    flags: int


@dataclass(frozen=True)
class FrameRecord:
    label: str
    report: VisionTargetReport
    frame: bytes


@dataclass(frozen=True)
class WriteOperation:
    label: str
    data: bytes
    frames: tuple[FrameRecord, ...]
    delay_after_s: float = 0.0


def parse_int(value: str) -> int:
    return int(value, 0)


def parse_bbox(value: str) -> tuple[int, int, int, int]:
    try:
        parts = tuple(int(part.strip(), 0) for part in value.split(","))
    except ValueError as exc:
        raise argparse.ArgumentTypeError("bbox must contain four integers") from exc
    if len(parts) != 4:
        raise argparse.ArgumentTypeError("bbox format is x,y,width,height")
    return parts  # type: ignore[return-value]


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def bytes_to_hex(data: bytes) -> str:
    return " ".join(f"{byte:02X}" for byte in data)


def generate_session_id() -> int:
    session_id = 0
    while session_id == 0:
        session_id = secrets.randbits(32)
    return session_id


def _require_uint(name: str, value: int, bits: int) -> None:
    maximum = (1 << bits) - 1
    if not 0 <= value <= maximum:
        raise ValueError(f"{name} must be in 0..{maximum}, got {value}")


def validate_report(report: VisionTargetReport) -> None:
    for name, value, bits in (
        ("session_id", report.session_id, 32),
        ("sequence", report.sequence, 16),
        ("timestamp_ms", report.timestamp_ms, 32),
        ("frame_width", report.frame_width, 16),
        ("frame_height", report.frame_height, 16),
        ("target_center_x", report.target_center_x, 16),
        ("target_center_y", report.target_center_y, 16),
        ("confidence", report.confidence, 16),
        ("target_id", report.target_id, 16),
        ("bbox_x", report.bbox_x, 16),
        ("bbox_y", report.bbox_y, 16),
        ("bbox_width", report.bbox_width, 16),
        ("bbox_height", report.bbox_height, 16),
        ("flags", report.flags, 8),
    ):
        _require_uint(name, value, bits)

    if report.session_id == 0:
        raise ValueError("session_id must be non-zero")
    if not 1 <= report.frame_width <= 8192:
        raise ValueError("frame_width must be in 1..8192")
    if not 1 <= report.frame_height <= 8192:
        raise ValueError("frame_height must be in 1..8192")
    if report.flags & FLAG_RESERVED_MASK:
        raise ValueError("flags bit5..bit7 must be zero in V1")

    target_valid = bool(report.flags & FLAG_TARGET_VALID)
    has_bbox = bool(report.flags & FLAG_HAS_BBOX)
    has_target_id = bool(report.flags & FLAG_HAS_TARGET_ID)
    has_confidence = bool(report.flags & FLAG_HAS_CONFIDENCE)

    if not target_valid:
        if report.flags & (FLAG_HAS_BBOX | FLAG_HAS_TARGET_ID | FLAG_HAS_CONFIDENCE):
            raise ValueError("optional target flags must be zero when TARGET_VALID=0")
        if report.target_center_x != 0xFFFF or report.target_center_y != 0xFFFF:
            raise ValueError("no-target center must be 0xFFFF,0xFFFF")
        if report.confidence != 0:
            raise ValueError("no-target confidence must be 0")
        if report.target_id != 0xFFFF:
            raise ValueError("no-target target_id must be 0xFFFF")
        if any((report.bbox_x, report.bbox_y, report.bbox_width, report.bbox_height)):
            raise ValueError("no-target bbox fields must all be zero")
        return

    if report.target_center_x >= report.frame_width:
        raise ValueError("target_center_x must be less than frame_width")
    if report.target_center_y >= report.frame_height:
        raise ValueError("target_center_y must be less than frame_height")

    if has_confidence:
        if report.confidence > 1000:
            raise ValueError("confidence must be in 0..1000")
    elif report.confidence != 0:
        raise ValueError("confidence must be 0 when HAS_CONFIDENCE=0")

    if has_target_id:
        if report.target_id == 0xFFFF:
            raise ValueError("target_id 0xFFFF means no target ID")
    elif report.target_id != 0xFFFF:
        raise ValueError("target_id must be 0xFFFF when HAS_TARGET_ID=0")

    if has_bbox:
        if report.bbox_width == 0 or report.bbox_height == 0:
            raise ValueError("bbox width and height must be non-zero")
        if report.bbox_x + report.bbox_width > report.frame_width:
            raise ValueError("bbox exceeds frame width")
        if report.bbox_y + report.bbox_height > report.frame_height:
            raise ValueError("bbox exceeds frame height")
    elif any((report.bbox_x, report.bbox_y, report.bbox_width, report.bbox_height)):
        raise ValueError("bbox fields must be zero when HAS_BBOX=0")


def build_frame(
    report: VisionTargetReport,
    *,
    payload_length: int = PAYLOAD_LENGTH,
    version: int = VERSION,
    message_type: int = MESSAGE_TYPE_TARGET_REPORT,
    reserved: int = 0,
    validate: bool = True,
) -> bytes:
    if validate:
        validate_report(report)

    header = HEADER_STRUCT.pack(
        MAGIC,
        version,
        message_type,
        report.flags,
        reserved,
        payload_length,
    )
    payload = PAYLOAD_STRUCT.pack(
        report.session_id,
        report.sequence,
        report.timestamp_ms,
        report.frame_width,
        report.frame_height,
        report.target_center_x,
        report.target_center_y,
        report.confidence,
        report.target_id,
        report.bbox_x,
        report.bbox_y,
        report.bbox_width,
        report.bbox_height,
    )
    body = header + payload
    frame = body + CRC_STRUCT.pack(crc16_ccitt_false(body))
    if len(header) != 8 or len(payload) != 30 or len(frame) != FRAME_LENGTH:
        raise AssertionError("Vision Target Protocol V1 frame size mismatch")
    return frame


def stored_crc(frame: bytes) -> int:
    if len(frame) < 2:
        return 0
    return int.from_bytes(frame[-2:], "little")


def computed_crc(frame: bytes) -> int:
    if len(frame) < 2:
        return 0
    return crc16_ccitt_false(frame[:-2])


def make_valid_report(
    args: argparse.Namespace,
    session_id: int,
    sequence: int,
    timestamp_ms: int,
    source_restart: bool,
) -> VisionTargetReport:
    bbox_x, bbox_y, bbox_width, bbox_height = args.bbox
    flags = FLAG_TARGET_VALID | FLAG_HAS_BBOX | FLAG_HAS_TARGET_ID | FLAG_HAS_CONFIDENCE
    if source_restart:
        flags |= FLAG_SOURCE_RESTART
    return VisionTargetReport(
        session_id=session_id,
        sequence=sequence & 0xFFFF,
        timestamp_ms=timestamp_ms & 0xFFFFFFFF,
        frame_width=args.width,
        frame_height=args.height,
        target_center_x=args.x,
        target_center_y=args.y,
        confidence=args.confidence,
        target_id=args.target_id,
        bbox_x=bbox_x,
        bbox_y=bbox_y,
        bbox_width=bbox_width,
        bbox_height=bbox_height,
        flags=flags,
    )


def make_no_target_report(
    args: argparse.Namespace,
    session_id: int,
    sequence: int,
    timestamp_ms: int,
    source_restart: bool,
) -> VisionTargetReport:
    flags = FLAG_SOURCE_RESTART if source_restart else 0
    return VisionTargetReport(
        session_id=session_id,
        sequence=sequence & 0xFFFF,
        timestamp_ms=timestamp_ms & 0xFFFFFFFF,
        frame_width=args.width,
        frame_height=args.height,
        target_center_x=0xFFFF,
        target_center_y=0xFFFF,
        confidence=0,
        target_id=0xFFFF,
        bbox_x=0,
        bbox_y=0,
        bbox_width=0,
        bbox_height=0,
        flags=flags,
    )


def make_record(label: str, report: VisionTargetReport, **build_kwargs: object) -> FrameRecord:
    return FrameRecord(label=label, report=report, frame=build_frame(report, **build_kwargs))


def timestamp_for(args: argparse.Namespace, index: int, start_time: float) -> int:
    period_ms = int(round(1000.0 / args.fps))
    if args.timestamp_ms is not None:
        return (args.timestamp_ms + index * period_ms) & 0xFFFFFFFF
    elapsed_ms = int((time.monotonic() - start_time) * 1000.0)
    return (elapsed_ms + index * period_ms) & 0xFFFFFFFF


def normal_operations(
    args: argparse.Namespace, session_id: int, start_time: float
) -> list[WriteOperation]:
    operations: list[WriteOperation] = []
    period_s = 1.0 / args.fps
    for index in range(args.count):
        sequence = (args.sequence + index) & 0xFFFF
        timestamp_ms = timestamp_for(args, index, start_time)
        restart = index < args.restart_frames
        if args.mode == "valid" or (args.mode == "alternating" and index % 2 == 0):
            report = make_valid_report(args, session_id, sequence, timestamp_ms, restart)
            label = "valid"
        else:
            report = make_no_target_report(args, session_id, sequence, timestamp_ms, restart)
            label = "no-target"
        record = make_record(label, report)
        operations.append(WriteOperation(label, record.frame, (record,), period_s))
    return operations


def abnormal_operations(
    args: argparse.Namespace, session_id: int, start_time: float
) -> list[WriteOperation]:
    period_s = 1.0 / args.fps

    def valid(index: int = 0, seq: Optional[int] = None, restart: bool = False) -> VisionTargetReport:
        sequence = (args.sequence + index) & 0xFFFF if seq is None else seq & 0xFFFF
        return make_valid_report(
            args,
            session_id,
            sequence,
            timestamp_for(args, index, start_time),
            restart,
        )

    if args.mode == "half":
        record = make_record("half-frame", valid())
        if not 1 <= args.split < FRAME_LENGTH:
            raise ValueError("--split must be in 1..39")
        return [
            WriteOperation("half-part-1", record.frame[: args.split], (record,), args.half_delay_ms / 1000.0),
            WriteOperation("half-part-2", record.frame[args.split :], (record,), period_s),
        ]

    if args.mode == "truncated":
        first = make_record("truncated-source", valid())
        recovery_report = valid(1)
        recovery = make_record("recovery-valid", recovery_report)
        if not 1 <= args.split < FRAME_LENGTH:
            raise ValueError("--split must be in 1..39")
        return [
            WriteOperation("truncated-prefix", first.frame[: args.split], (first,), args.half_delay_ms / 1000.0),
            WriteOperation("recovery-valid", recovery.frame, (recovery,), period_s),
        ]

    if args.mode == "sticky":
        first = make_record("sticky-valid", valid())
        second_report = make_no_target_report(
            args,
            session_id,
            (args.sequence + 1) & 0xFFFF,
            timestamp_for(args, 1, start_time),
            False,
        )
        second = make_record("sticky-no-target", second_report)
        return [WriteOperation("sticky-two-frames", first.frame + second.frame, (first, second), period_s)]

    if args.mode == "noise":
        rng = random.Random(args.seed)
        noise = bytes(rng.randrange(0, 256) for _ in range(args.noise_length))
        false_header = b"\x00\xFF\xA5\x13\xA5\x5A\x01"
        record = make_record("noise-recovery-valid", valid())
        data = false_header + noise + record.frame + noise
        return [WriteOperation("noise-plus-valid", data, (record,), period_s)]

    if args.mode == "bad-crc":
        report = valid()
        original = bytearray(build_frame(report))
        original[22] ^= 0x01
        record = FrameRecord("bad-crc", report, bytes(original))
        return [WriteOperation("bad-crc", record.frame, (record,), period_s)]

    if args.mode == "bad-length":
        report = valid()
        record = make_record(
            "bad-length",
            report,
            payload_length=args.bad_length,
        )
        return [WriteOperation("bad-length", record.frame, (record,), period_s)]

    if args.mode == "duplicate":
        record = make_record("duplicate", valid())
        return [
            WriteOperation("duplicate-first", record.frame, (record,), period_s),
            WriteOperation("duplicate-repeat", record.frame, (record,), period_s),
        ]

    if args.mode == "old-sequence":
        current = make_record("current-sequence", valid())
        old_report = valid(1, seq=(args.sequence - 1) & 0xFFFF)
        old = make_record("old-sequence", old_report)
        return [
            WriteOperation("current-sequence", current.frame, (current,), period_s),
            WriteOperation("old-sequence", old.frame, (old,), period_s),
        ]

    if args.mode == "sequence-wrap":
        operations = []
        for index, sequence in enumerate((65534, 65535, 0, 1)):
            record = make_record(f"sequence-{sequence}", valid(index, seq=sequence))
            operations.append(WriteOperation(record.label, record.frame, (record,), period_s))
        return operations

    if args.mode == "source-restart":
        old_record = make_record("old-session", valid(0, seq=10))
        new_session = generate_session_id()
        while new_session == session_id:
            new_session = generate_session_id()
        operations = [WriteOperation("old-session", old_record.frame, (old_record,), period_s)]
        for index in range(3):
            report = make_valid_report(
                args,
                new_session,
                index,
                timestamp_for(args, index + 1, start_time),
                True,
            )
            record = make_record(f"new-session-{index}", report)
            operations.append(WriteOperation(record.label, record.frame, (record,), period_s))
        return operations

    if args.mode == "invalid-fields":
        base = valid()
        cases: list[tuple[str, VisionTargetReport, dict[str, object]]] = [
            ("invalid-width-zero", replace(base, frame_width=0), {"validate": False}),
            ("invalid-center-x", replace(base, target_center_x=base.frame_width), {"validate": False}),
            ("invalid-confidence", replace(base, confidence=1001), {"validate": False}),
            (
                "invalid-no-target-flags",
                replace(
                    make_no_target_report(args, session_id, base.sequence, base.timestamp_ms, False),
                    flags=FLAG_HAS_BBOX,
                ),
                {"validate": False},
            ),
            ("invalid-reserved-flag", replace(base, flags=base.flags | 0x20), {"validate": False}),
            ("invalid-session-zero", replace(base, session_id=0), {"validate": False}),
            ("invalid-header-reserved", base, {"reserved": 1}),
        ]
        operations = []
        for label, report, kwargs in cases:
            record = make_record(label, report, **kwargs)
            operations.append(WriteOperation(label, record.frame, (record,), period_s))
        return operations

    raise ValueError(f"unsupported mode: {args.mode}")


class OutputFiles:
    def __init__(self, hex_file: Optional[Path], bin_file: Optional[Path]) -> None:
        self._hex_handle = None
        self._bin_handle = None
        if hex_file is not None:
            hex_file.parent.mkdir(parents=True, exist_ok=True)
            self._hex_handle = hex_file.open("w", encoding="utf-8", newline="\n")
        if bin_file is not None:
            bin_file.parent.mkdir(parents=True, exist_ok=True)
            self._bin_handle = bin_file.open("wb")

    def write(self, operation: WriteOperation) -> None:
        if self._hex_handle is not None:
            self._hex_handle.write(
                f"[{operation.label}] bytes={len(operation.data)}\n{bytes_to_hex(operation.data)}\n"
            )
            self._hex_handle.flush()
        if self._bin_handle is not None:
            self._bin_handle.write(operation.data)
            self._bin_handle.flush()

    def close(self) -> None:
        if self._hex_handle is not None:
            self._hex_handle.close()
        if self._bin_handle is not None:
            self._bin_handle.close()


class SerialSink:
    def __init__(self, port: Optional[str], baud: int, verify_loopback: bool) -> None:
        self._serial = None
        self._verify_loopback = verify_loopback
        if port is None:
            if verify_loopback:
                raise RuntimeError("--loopback-verify requires --port")
            return
        try:
            import serial  # type: ignore
        except ImportError as exc:
            raise RuntimeError("serial sending requires: python -m pip install pyserial") from exc
        self._serial = serial.serial_for_url(
            port,
            baudrate=baud,
            timeout=1 if verify_loopback else 0,
            write_timeout=1,
        )

    @property
    def connected(self) -> bool:
        return self._serial is not None

    def write(self, data: bytes) -> None:
        if self._serial is None:
            return
        written = self._serial.write(data)
        self._serial.flush()
        if written != len(data):
            raise RuntimeError(f"serial short write: {written}/{len(data)} bytes")
        if self._verify_loopback:
            received = self._serial.read(len(data))
            if received != data:
                raise RuntimeError(
                    "serial loopback mismatch: "
                    f"expected={bytes_to_hex(data)} received={bytes_to_hex(received)}"
                )
            print(f"  loopback_verify=PASS bytes={len(data)}")

    def close(self) -> None:
        if self._serial is not None:
            self._serial.close()


def print_operation(operation: WriteOperation, connected: bool) -> None:
    destination = "serial" if connected else "offline"
    print(f"\nWRITE mode={operation.label} destination={destination} bytes={len(operation.data)}")
    for record in operation.frames:
        appended_crc = stored_crc(record.frame)
        expected_crc = computed_crc(record.frame)
        crc_state = "OK" if appended_crc == expected_crc else "BAD"
        print(
            f"  frame={record.label} "
            f"session_id=0x{record.report.session_id:08X} "
            f"sequence={record.report.sequence} "
            f"flags=0x{record.report.flags:02X} "
            f"timestamp_ms={record.report.timestamp_ms} "
            f"CRC=0x{appended_crc:04X} "
            f"computed=0x{expected_crc:04X} "
            f"crc_state={crc_state}"
        )
        print(f"  frame_hex={bytes_to_hex(record.frame)}")
    if len(operation.frames) != 1 or operation.data != operation.frames[0].frame:
        print(f"  write_hex={bytes_to_hex(operation.data)}")


def execute_operations(args: argparse.Namespace, operations: Iterable[WriteOperation]) -> None:
    sink = SerialSink(args.port, args.baud, args.loopback_verify)
    outputs = OutputFiles(args.hex_file, args.bin_file)
    try:
        for operation in operations:
            print_operation(operation, sink.connected)
            sink.write(operation.data)
            outputs.write(operation)
            if sink.connected and operation.delay_after_s > 0:
                time.sleep(operation.delay_after_s)
    finally:
        outputs.close()
        sink.close()


def run_self_test() -> None:
    if HEADER_STRUCT.size != 8:
        raise AssertionError("header size must be 8")
    if PAYLOAD_STRUCT.size != 30:
        raise AssertionError("payload size must be 30")

    valid_report = VisionTargetReport(
        session_id=0x12345678,
        sequence=0,
        timestamp_ms=1000,
        frame_width=640,
        frame_height=480,
        target_center_x=400,
        target_center_y=200,
        confidence=875,
        target_id=7,
        bbox_x=350,
        bbox_y=150,
        bbox_width=100,
        bbox_height=100,
        flags=0x1F,
    )
    valid_frame = build_frame(valid_report)
    expected_valid = bytes.fromhex(FIXED_VALID_HEX)
    if valid_frame != expected_valid or stored_crc(valid_frame) != 0xD14D:
        raise AssertionError("fixed valid vector mismatch")

    no_target_report = VisionTargetReport(
        session_id=0x12345678,
        sequence=1,
        timestamp_ms=1033,
        frame_width=640,
        frame_height=480,
        target_center_x=0xFFFF,
        target_center_y=0xFFFF,
        confidence=0,
        target_id=0xFFFF,
        bbox_x=0,
        bbox_y=0,
        bbox_width=0,
        bbox_height=0,
        flags=0,
    )
    no_target_frame = build_frame(no_target_report)
    expected_no_target = bytes.fromhex(FIXED_NO_TARGET_HEX)
    if no_target_frame != expected_no_target or stored_crc(no_target_frame) != 0xC75C:
        raise AssertionError("fixed no-target vector mismatch")

    corrupted = bytearray(valid_frame)
    corrupted[22] ^= 0x01
    if stored_crc(corrupted) == computed_crc(corrupted):
        raise AssertionError("bad CRC self-test did not fail")
    if computed_crc(bytes(corrupted)) != 0x7969:
        raise AssertionError("mutated vector CRC mismatch")

    print("SELF-TEST PASS")
    print("header_size=8 payload_size=30 frame_size=40")
    print("valid_crc=0xD14D")
    print(f"valid_hex={bytes_to_hex(valid_frame)}")
    print("no_target_crc=0xC75C")
    print(f"no_target_hex={bytes_to_hex(no_target_frame)}")
    print("mutated_target_center_x_crc=0x7969")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Vision Target Protocol V1 serial sender and offline generator"
    )
    parser.add_argument(
        "--mode",
        required=True,
        choices=(
            "self-test",
            "valid",
            "no-target",
            "alternating",
            "half",
            "truncated",
            "sticky",
            "noise",
            "bad-crc",
            "bad-length",
            "invalid-fields",
            "duplicate",
            "old-sequence",
            "sequence-wrap",
            "source-restart",
        ),
    )
    parser.add_argument("--port", help="serial port, for example COM7; omit for offline mode")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument(
        "--loopback-verify",
        action="store_true",
        help="read back and compare every serial write (use loop:// or TX/RX short)",
    )
    parser.add_argument("--count", type=int, default=1, help="frame count for normal modes")
    parser.add_argument("--fps", type=float, default=20.0)
    parser.add_argument("--session-id", type=parse_int)
    parser.add_argument("--sequence", type=parse_int, default=0)
    parser.add_argument("--timestamp-ms", type=parse_int)
    parser.add_argument("--restart-frames", type=int, default=3)

    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--x", type=int, default=400)
    parser.add_argument("--y", type=int, default=200)
    parser.add_argument("--confidence", type=int, default=875)
    parser.add_argument("--target-id", type=int, default=7)
    parser.add_argument("--bbox", type=parse_bbox, default=(350, 150, 100, 100))

    parser.add_argument("--split", type=int, default=13)
    parser.add_argument("--half-delay-ms", type=int, default=50)
    parser.add_argument("--noise-length", type=int, default=16)
    parser.add_argument("--seed", type=int, default=12345)
    parser.add_argument("--bad-length", type=int, default=29)

    parser.add_argument("--hex-file", type=Path, help="write each serial operation as hex text")
    parser.add_argument("--bin-file", type=Path, help="write the exact serial byte stream")
    return parser


def validate_args(args: argparse.Namespace) -> None:
    if args.count < 1:
        raise ValueError("--count must be at least 1")
    if args.fps <= 0:
        raise ValueError("--fps must be greater than 0")
    if not 0 <= args.sequence <= 0xFFFF:
        raise ValueError("--sequence must be in 0..65535")
    if args.timestamp_ms is not None and not 0 <= args.timestamp_ms <= 0xFFFFFFFF:
        raise ValueError("--timestamp-ms must be in 0..0xFFFFFFFF")
    if args.session_id is not None and not 1 <= args.session_id <= 0xFFFFFFFF:
        raise ValueError("--session-id must be non-zero uint32")
    if args.restart_frames < 0:
        raise ValueError("--restart-frames cannot be negative")
    if not 0 <= args.bad_length <= 0xFFFF:
        raise ValueError("--bad-length must fit uint16")
    if args.noise_length < 0:
        raise ValueError("--noise-length cannot be negative")


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        validate_args(args)
        if args.mode == "self-test":
            run_self_test()
            return 0

        session_id = args.session_id if args.session_id is not None else generate_session_id()
        start_time = time.monotonic()
        if args.mode in ("valid", "no-target", "alternating"):
            operations = normal_operations(args, session_id, start_time)
        else:
            operations = abnormal_operations(args, session_id, start_time)
        execute_operations(args, operations)
        return 0
    except KeyboardInterrupt:
        print("\nInterrupted", file=sys.stderr)
        return 130
    except (OSError, RuntimeError, ValueError, struct.error) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
