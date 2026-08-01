#!/usr/bin/env python3
"""MSPM0G3507 balance-ball serial tuning and JustFloat monitor.

The tuning UART is independent from the K230 UART:
    PA21 UART2_TX -> USB-TTL RX
    PA22 UART2_RX <- USB-TTL TX
    GND           <-> USB-TTL GND

Install:
    python -m pip install pyserial matplotlib numpy

Run:
    python balance_pid_monitor.py --list
    python balance_pid_monitor.py --port COM9
"""

from __future__ import annotations

import argparse
import csv
import math
import struct
import threading
import time
from collections import deque
from datetime import datetime
from pathlib import Path

try:
    import matplotlib.pyplot as plt
    import numpy as np
    import serial
    import serial.tools.list_ports
    from matplotlib.animation import FuncAnimation
    from matplotlib.widgets import Button, TextBox
except ImportError as exc:
    raise SystemExit(
        "缺少依赖，请执行：python -m pip install pyserial matplotlib numpy\n"
        f"原始错误：{exc}"
    ) from exc


TAIL = b"\x00\x00\x80\x7f"
FLOAT_COUNT = 16
PAYLOAD_SIZE = FLOAT_COUNT * 4
FRAME_SIZE = PAYLOAD_SIZE + len(TAIL)
CHANNEL_NAMES = (
    "position_mm",
    "predicted_mm",
    "velocity_used_mm_s",
    "error_mm",
    "p_count",
    "d_count",
    "pd_output_count",
    "commanded_offset_count",
    "encoder_count",
    "signed_step_rate_hz",
    "vision_valid",
    "vision_age_ms",
    "kpos_mm_s_per_mm",
    "kvel_count_per_mm_s",
    "tracking_deadband_count",
    "controller_state",
)

PARAMETER_STEPS = {
    "KP": 0.05,
    "KD": 0.01,
    "VMAX": 10.0,
    "BIAS": 2.0,
    "AP": 50.0,
    "AN": 50.0,
    "TD": 10.0,
    "BM": 1.0,
    "VAPP": 5.0,
    "MAX": 8.0,
    "SLEW": 1.0,
    "DB": 1.0,
    "RG": 1.0,
    "T": 5.0,
}


class BalanceSerial:
    def __init__(self, port: str, baud: int) -> None:
        self.port_name = port
        self.baud = baud
        self.port: serial.Serial | None = None
        self.port_lock = threading.Lock()
        self.data_lock = threading.Lock()
        self.stop_event = threading.Event()
        self.thread: threading.Thread | None = None
        self.buffer = bytearray()
        self.pending_frames: deque[
            tuple[int, float, tuple[float, ...]]
        ] = deque(maxlen=5000)
        self.messages: deque[str] = deque(maxlen=100)
        self.sequence = 0
        self.good = 0
        self.bad = 0
        self.rx_bytes = 0
        self.status = f"等待连接 {port}"

    def start(self) -> None:
        self.thread = threading.Thread(target=self._worker, daemon=True)
        self.thread.start()

    def stop(self) -> None:
        self.stop_event.set()
        self._close()
        if self.thread:
            self.thread.join(timeout=1.5)

    def _open(self) -> bool:
        try:
            port = serial.Serial(
                self.port_name,
                self.baud,
                timeout=0.1,
                write_timeout=0.3,
            )
        except (serial.SerialException, OSError) as exc:
            with self.data_lock:
                self.status = f"等待 {self.port_name}: {exc}"
            return False
        try:
            # Opening a USB-UART can leave a noise byte or an unfinished line
            # in the MCU command parser.  A blank line terminates and clears
            # that startup fragment before the operator's first real command.
            port.reset_input_buffer()
            port.write(b"\n")
        except (serial.SerialException, OSError) as exc:
            try:
                port.close()
            except (serial.SerialException, OSError):
                pass
            with self.data_lock:
                self.status = f"初始化 {self.port_name} 失败: {exc}"
            return False
        with self.port_lock:
            self.port = port
        self.buffer.clear()
        with self.data_lock:
            self.status = f"已连接 {self.port_name} @ {self.baud}"
        return True

    def _close(self) -> None:
        with self.port_lock:
            port, self.port = self.port, None
        if port:
            try:
                port.close()
            except (serial.SerialException, OSError):
                pass

    @staticmethod
    def _plausible(values: tuple[float, ...]) -> bool:
        if len(values) != FLOAT_COUNT:
            return False
        if not all(math.isfinite(value) for value in values):
            return False
        if any(abs(value) > 1.0e7 for value in values):
            return False
        if not (-0.25 <= values[10] <= 1.25):
            return False
        if not (0.0 <= values[12] <= 10.0):
            return False
        if not (0.0 <= values[13] <= 1.0):
            return False
        if not (0.0 <= values[14] <= 32.0):
            return False
        return True

    def _save_ascii(self, prefix: bytes) -> None:
        for raw_line in prefix.splitlines():
            try:
                line = raw_line.decode("ascii").strip()
            except UnicodeDecodeError:
                continue
            if line.startswith(("ACK,", "ERR,", "CFG,", "DBG,", "CAL,")):
                with self.data_lock:
                    self.messages.append(line)

    def _extract(self) -> None:
        while True:
            tail_index = self.buffer.find(TAIL)
            if tail_index < 0:
                if len(self.buffer) > 4096:
                    del self.buffer[: -(FRAME_SIZE - 1)]
                    with self.data_lock:
                        self.bad += 1
                return
            if tail_index < PAYLOAD_SIZE:
                self._save_ascii(bytes(self.buffer[:tail_index]))
                del self.buffer[: tail_index + len(TAIL)]
                with self.data_lock:
                    self.bad += 1
                continue

            payload_start = tail_index - PAYLOAD_SIZE
            self._save_ascii(bytes(self.buffer[:payload_start]))
            payload = bytes(self.buffer[payload_start:tail_index])
            del self.buffer[: tail_index + len(TAIL)]
            try:
                values = struct.unpack("<16f", payload)
            except struct.error:
                with self.data_lock:
                    self.bad += 1
                continue
            if not self._plausible(values):
                with self.data_lock:
                    self.bad += 1
                continue
            with self.data_lock:
                self.sequence += 1
                self.good += 1
                self.pending_frames.append(
                    (self.sequence, time.monotonic(), values)
                )

    def _worker(self) -> None:
        while not self.stop_event.is_set():
            with self.port_lock:
                port = self.port
            if port is None or not port.is_open:
                if not self._open():
                    self.stop_event.wait(1.0)
                    continue
                with self.port_lock:
                    port = self.port
            try:
                assert port is not None
                count = port.in_waiting
                chunk = port.read(count if count else 1)
                if chunk:
                    self.buffer.extend(chunk)
                    with self.data_lock:
                        self.rx_bytes += len(chunk)
                    self._extract()
            except (serial.SerialException, OSError) as exc:
                with self.data_lock:
                    self.status = f"串口断开，自动重连: {exc}"
                self._close()
                self.stop_event.wait(0.5)

    def send(self, command: str) -> tuple[bool, str]:
        command = command.strip().upper()
        if not command or len(command) > 40:
            return False, "命令为空或过长"
        try:
            # Prefix a blank line so an unfinished/noisy MCU command is closed
            # before this real command.  The firmware ignores an empty line;
            # if it was discarding a damaged line it may report ERR,LENGTH,
            # but the following command is still parsed normally.
            payload = ("\n" + command + "\n").encode("ascii")
        except UnicodeEncodeError:
            return False, "命令只能包含ASCII字符"
        with self.port_lock:
            port = self.port
            if port is None or not port.is_open:
                return False, "串口未连接"
            try:
                port.write(payload)
                return True, f"TX {command}"
            except (serial.SerialException, OSError) as exc:
                return False, f"发送失败: {exc}"

    def take_frames(self):
        with self.data_lock:
            frames = list(self.pending_frames)
            self.pending_frames.clear()
            return frames

    def snapshot(self):
        with self.data_lock:
            messages = list(self.messages)
            self.messages.clear()
            return self.status, self.good, self.bad, self.rx_bytes, messages


class Monitor:
    def __init__(self, link: BalanceSerial, window: float) -> None:
        self.link = link
        self.window = max(5.0, window)
        self.last_sequence = 0
        self.origin: float | None = None
        self.times: deque[float] = deque(maxlen=30000)
        self.channels = [deque(maxlen=30000) for _ in range(FLOAT_COUNT)]
        self.last_message = ""
        self.last_rate_time = time.monotonic()
        self.last_rate_frames = 0
        self.receive_fps = 0.0
        self.last_autoscale_time = 0.0
        self.log_dir = Path(__file__).resolve().parent / "balance_logs"
        self.debug_log_path = self.log_dir / (
            f"balance_debug_{datetime.now():%Y%m%d_%H%M%S}.log"
        )
        self.debug_lines: deque[str] = deque(maxlen=5)
        self.pending_parameter_save = False
        self.last_config: dict[str, str] = {}
        self.active_parameter = "KP"
        self.calibration_mode = False

        plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
        plt.rcParams["axes.unicode_minus"] = False
        self.figure, self.axes = plt.subplots(3, 1, figsize=(15, 9), sharex=True)
        self.figure.subplots_adjust(left=0.07, right=0.74, bottom=0.20, hspace=0.24)
        self.figure.canvas.manager.set_window_title("平衡滚球PD实时调参")
        self._make_lines()
        self._make_controls()
        self.status_text = self.figure.text(0.76, 0.93, "等待串口", va="top", family="monospace")
        self.value_text = self.figure.text(0.76, 0.82, "等待数据", va="top", family="monospace")
        self.message_text = self.figure.text(
            0.76, 0.49, "", va="top", family="monospace", fontsize=9
        )
        self.debug_text = self.figure.text(
            0.76, 0.43, "保护链日志：等待事件", va="top",
            family="monospace", fontsize=7.5,
        )
        self.shortcut_text = self.figure.text(
            0.76, 0.30,
            "参数框：↑/↓微调，Shift+↑/↓快调\n修改后仍需点击应用按钮",
            va="top", fontsize=9,
        )
        self.figure.canvas.mpl_connect("close_event", lambda _event: self.link.stop())
        self.figure.canvas.mpl_connect("button_press_event", self._select_parameter)
        self.figure.canvas.mpl_connect("key_press_event", self._adjust_parameter_by_key)
        self.animation = FuncAnimation(
            self.figure, self._update, interval=40,
            blit=False, cache_frame_data=False,
        )

    def _make_lines(self) -> None:
        ax_pos, ax_pd, ax_axis = self.axes
        ax_pos.set_title("小球位置与误差")
        ax_pos.set_ylabel("mm")
        self.pos_line, = ax_pos.plot([], [], label="pos")
        self.pred_line, = ax_pos.plot([], [], label="pred", linestyle="--")
        self.err_line, = ax_pos.plot([], [], label="error")
        self.target_line = ax_pos.axhline(
            0, color="purple", linestyle="-.", label="target"
        )
        ax_pos.axhline(10, color="gray", linestyle=":")
        ax_pos.axhline(-10, color="gray", linestyle=":")
        ax_pos.legend(loc="upper left", ncol=4)

        ax_pd.set_title("PD输出")
        ax_pd.set_ylabel("count")
        self.p_line, = ax_pd.plot([], [], label="P")
        self.d_line, = ax_pd.plot([], [], label="D")
        self.pd_line, = ax_pd.plot([], [], label="PD")
        self.cmd_line, = ax_pd.plot([], [], label="cmd", linewidth=2)
        ax_pd.legend(loc="upper left", ncol=4)

        ax_axis.set_title("平衡轴位置")
        ax_axis.set_ylabel("count")
        ax_axis.set_xlabel("time / s")
        self.enc_line, = ax_axis.plot([], [], label="encoder")
        self.axis_target_line, = ax_axis.plot([], [], label="axis target")
        ax_axis.legend(loc="upper left", ncol=2)
        for axis in self.axes:
            axis.grid(True, alpha=0.3)
            axis.axhline(0, color="black", linewidth=0.7)

    def _make_controls(self) -> None:
        fields = (
            ("KP", "1.50"), ("KD", "0.60"), ("DIR", "-1"),
            ("MAX", "80"), ("SLEW", "5"), ("DB", "2"), ("RG", "3"),
            ("T", "0"),
        )
        self.boxes: dict[str, TextBox] = {}
        for index, (name, initial) in enumerate(fields):
            x = 0.07 + (index % 4) * 0.16
            y = 0.105 if index < 4 else 0.045
            self.boxes[name] = TextBox(
                self.figure.add_axes([x, y, 0.11, 0.04]),
                f"{name} ", initial=initial,
            )
        self.buttons = [
            Button(self.figure.add_axes([0.76, 0.22, 0.10, 0.045]), "应用PD/目标"),
            Button(self.figure.add_axes([0.87, 0.22, 0.11, 0.045]), "应用轴参数"),
            Button(self.figure.add_axes([0.76, 0.16, 0.065, 0.045]), "RUN"),
            Button(self.figure.add_axes([0.835, 0.16, 0.065, 0.045]), "STOP"),
            Button(self.figure.add_axes([0.91, 0.16, 0.065, 0.045]), "GET"),
            Button(self.figure.add_axes([0.76, 0.10, 0.08, 0.045]), "默认"),
            Button(self.figure.add_axes([0.85, 0.10, 0.08, 0.045]), "CSV"),
            Button(self.figure.add_axes([0.94, 0.10, 0.05, 0.045]), "PARAM"),
            Button(self.figure.add_axes([0.76, 0.04, 0.10, 0.045]), "诊断"),
            Button(self.figure.add_axes([0.87, 0.04, 0.11, 0.045]), "校准"),
        ]
        self.buttons[0].on_clicked(self._apply_pd)
        self.buttons[1].on_clicked(self._apply_axis)
        self.buttons[2].on_clicked(self._run)
        self.buttons[3].on_clicked(lambda _e: self._send("STOP"))
        self.buttons[4].on_clicked(lambda _e: self._send("GET"))
        self.buttons[5].on_clicked(lambda _e: self._send("DEF"))
        self.buttons[6].on_clicked(lambda _e: self.save_csv())
        self.buttons[7].on_clicked(self._request_parameter_save)
        self.buttons[8].on_clicked(lambda _e: self._send("LOG"))
        self.buttons[9].on_clicked(self._start_calibration)

    def _select_parameter(self, event) -> None:
        for name, box in self.boxes.items():
            if event.inaxes is box.ax:
                self.active_parameter = name
                return

    def _parameter_int(self, name: str, fallback: int) -> int:
        try:
            return int(float(self.boxes[name].text.strip()))
        except (ValueError, KeyError):
            return fallback

    def _adjust_parameter_by_key(self, event) -> None:
        key = (event.key or "").lower()
        direction = 0
        multiplier = 1.0

        if self.calibration_mode:
            if key == "up":
                self._send("J+")
            elif key == "shift+up":
                self._send("J++")
            elif key == "down":
                self._send("J-")
            elif key == "shift+down":
                self._send("J--")
            elif key in ("enter", "return"):
                self._send("SET")
            elif key in ("escape", "esc"):
                self._send("CALX")
            return

        if key in ("up", "shift+up"):
            direction = 1
        elif key in ("down", "shift+down"):
            direction = -1
        else:
            return
        if key.startswith("shift+"):
            multiplier = 5.0

        name = self.active_parameter
        box = self.boxes.get(name)
        if box is None:
            return
        if name == "DIR":
            box.set_val("1" if direction > 0 else "-1")
            self.last_message = f"DIR={box.text}，尚未应用"
            return
        try:
            current = float(box.text.strip())
        except ValueError:
            self.last_message = f"{name}不是有效数值"
            return

        value = current + direction * PARAMETER_STEPS[name] * multiplier
        if name == "KP":
            value = min(20.0, max(0.0, value))
            text = f"{value:.2f}"
        elif name == "KD":
            value = min(5.0, max(0.0, value))
            text = f"{value:.2f}"
        elif name == "MAX":
            minimum = max(8, self._parameter_int("RG", 3))
            value = min(1024, max(minimum, round(value)))
            text = str(int(value))
        elif name == "SLEW":
            value = min(128, max(1, round(value)))
            text = str(int(value))
        elif name == "DB":
            maximum = max(0, min(32, self._parameter_int("RG", 3) - 1))
            value = min(maximum, max(0, round(value)))
            text = str(int(value))
        elif name == "RG":
            minimum = max(1, self._parameter_int("DB", 2) + 1)
            maximum = max(minimum,
                min(64, self._parameter_int("MAX", 128)))
            value = min(maximum, max(minimum, round(value)))
            text = str(int(value))
        else:  # T, physical millimetres relative to O
            value = min(125, max(-125, round(value)))
            text = str(int(value))
        box.set_val(text)
        self.last_message = f"{name}={text}，尚未应用"

    def _send(self, command: str) -> None:
        _ok, self.last_message = self.link.send(command)

    def _apply_names(self, names: tuple[str, ...]) -> None:
        for name in names:
            ok, message = self.link.send(f"{name}={self.boxes[name].text.strip()}")
            self.last_message = message
            if not ok:
                break

    def _apply_pd(self, _event) -> None:
        self._apply_names(("KP", "KD", "T"))

    def _apply_axis(self, _event) -> None:
        self._apply_names(("DIR", "MAX", "SLEW", "DB", "RG"))

    def _load_cfg(self, message: str) -> None:
        try:
            values = dict(item.split("=", 1) for item in message.split(",")[1:])
            self.last_config = values
            if "KP" in values:
                self.boxes["KP"].set_val(f"{int(values['KP']) / 100:.2f}")
            if "KD" in values:
                self.boxes["KD"].set_val(f"{int(values['KD']) / 100:.2f}")
            for name in ("DIR", "MAX", "SLEW", "DB", "RG"):
                if name in values:
                    self.boxes[name].set_val(values[name])
            if "T" in values:
                self.boxes["T"].set_val(values["T"])
            if self.pending_parameter_save:
                self.pending_parameter_save = False
                self._write_parameter_snapshot(values)
        except (ValueError, KeyError):
            self.last_message = f"CFG解析失败: {message}"

    def _run(self, _event) -> None:
        ok, self.last_message = self.link.send(
            f"T={self.boxes['T'].text.strip()}"
        )
        if ok:
            _ok, self.last_message = self.link.send("RUN")

    def _start_calibration(self, _event) -> None:
        ok, self.last_message = self.link.send("CAL")
        if ok:
            self.calibration_mode = True
            self.shortcut_text.set_text(
                "校准：↑/↓点动，Shift+↑/↓快速点动\n"
                "回车确认当前点，Esc取消"
            )

    def _handle_calibration_status(self, message: str) -> None:
        try:
            values = dict(item.split("=", 1) for item in message.split(",")[1:])
            stage = values.get("STAGE", "?")
            raw = values.get("RAW", "?")
            pos = values.get("POS", "?")
            low = values.get("L", "?")
            high = values.get("H", "?")
            if stage == "ZERO":
                instruction = "调至机械水平，回车确认ZERO"
                self.calibration_mode = True
            elif stage == "LOW":
                instruction = "调至第一端点，回车确认"
                self.calibration_mode = True
            elif stage == "HIGH":
                instruction = "调至另一端点，回车保存"
                self.calibration_mode = True
            else:
                instruction = "校准结束/取消，等待电机回水平"
                self.calibration_mode = False
                self.shortcut_text.set_text(
                    "参数框：↑/↓微调，Shift+↑/↓快调\n"
                    "修改后仍需点击应用按钮"
                )
            self.last_message = (
                f"CAL {stage}: {instruction}\n"
                f"RAW={raw} POS={pos} L={low} H={high}"
            )
        except (ValueError, KeyError):
            self.last_message = f"CAL状态解析失败: {message}"

    def _request_parameter_save(self, _event) -> None:
        self.pending_parameter_save = True
        ok, self.last_message = self.link.send("GET")
        if not ok:
            self.pending_parameter_save = False

    def _write_parameter_snapshot(self, values: dict[str, str]) -> None:
        required = ("KP", "KD", "DIR", "MAX", "SLEW", "DB", "RG", "T")
        if any(name not in values for name in required):
            self.last_message = "PARAM CFG incomplete"
            return
        path = self.log_dir / f"balance_params_{datetime.now():%Y%m%d_%H%M%S}.txt"
        lines = [
            "# Runtime commands",
            f"KP={int(values['KP']) / 100:.2f}",
            f"KD={int(values['KD']) / 100:.2f}",
            f"DIR={values['DIR']}",
            f"MAX={values['MAX']}",
            f"SLEW={values['SLEW']}",
            f"DB={values['DB']}",
            f"RG={values['RG']}",
            f"T={values['T']}",
            "",
            "# app_features.h defaults",
            f"BALANCE_BALL_PD_KP_COUNTS_PER_MM_X100={values['KP']}",
            f"BALANCE_BALL_PD_KD_COUNTS_PER_MM_S_X100={values['KD']}",
            f"BALANCE_BALL_PD_TILT_SIGN={values['DIR']}",
            f"BALANCE_BALL_PD_MAX_OFFSET_COUNTS={values['MAX']}",
            f"BALANCE_BALL_PD_TARGET_SLEW_COUNTS_PER_20MS={values['SLEW']}",
            f"BALANCE_POSITION_TRACKING_DEADBAND_COUNTS={values['DB']}",
            f"BALANCE_POSITION_TRACKING_REENGAGE_COUNTS={values['RG']}",
        ]
        try:
            self.log_dir.mkdir(parents=True, exist_ok=True)
            path.write_text("\n".join(lines) + "\n", encoding="utf-8")
            self.last_message = f"PARAM {path.name}"
        except OSError as exc:
            self.last_message = f"PARAM save failed: {exc}"

    def _append_frames(self) -> None:
        for sequence, timestamp, values in self.link.take_frames():
            self.last_sequence = sequence
            if self.origin is None:
                self.origin = timestamp
            self.times.append(timestamp - self.origin)
            for channel, value in zip(self.channels, values):
                channel.append(float(value))
        if self.times:
            cutoff = self.times[-1] - self.window
            while self.times and self.times[0] < cutoff:
                self.times.popleft()
                for channel in self.channels:
                    channel.popleft()

    def save_csv(self) -> None:
        if not self.times:
            self.last_message = "没有数据可保存"
            return
        self.log_dir.mkdir(parents=True, exist_ok=True)
        path = self.log_dir / f"balance_{datetime.now():%Y%m%d_%H%M%S}.csv"
        with path.open("w", newline="", encoding="utf-8-sig") as handle:
            writer = csv.writer(handle)
            writer.writerow(("time_s", *CHANNEL_NAMES))
            writer.writerows(zip(self.times, *self.channels))
        self.last_message = f"CSV {path.name}"

    @staticmethod
    def _compact_debug(message: str) -> str:
        parts = message.split(",")
        if len(parts) < 2:
            return message
        layer = parts[1]
        fields = {}
        for item in parts[2:]:
            if "=" in item:
                key, value = item.split("=", 1)
                fields[key] = value
        timestamp = fields.get("t", "?")
        if layer == "CTRL":
            text = (
                f"{timestamp} CTRL {fields.get('old', '?')}→"
                f"{fields.get('new', '?')}"
            )
            if "cause" in fields:
                text += f" {fields['cause']}"
            return (
                f"{text} age={fields.get('mage', '?')} "
                f"rx={fields.get('rxage', '?')} seq={fields.get('seq', '?')}"
            )
        if layer == "FILTER":
            return (
                f"{timestamp} FILTER {fields.get('result', '?')} "
                f"seq={fields.get('seq', '?')} pos={fields.get('pos', '?')} "
                f"v/m={fields.get('valid', '?')}/{fields.get('meas', '?')} "
                f"jump/cand/rebase={fields.get('jump', '?')}/"
                f"{fields.get('cand', '?')}/{fields.get('rebase', '?')}"
            )
        if layer == "RX":
            return (
                f"{timestamp} RX {fields.get('event', '?')} "
                f"len/crc/fld={fields.get('len', '?')}/"
                f"{fields.get('crc', '?')}/{fields.get('field', '?')} "
                f"dup/old/ovf={fields.get('dup', '?')}/"
                f"{fields.get('old', '?')}/{fields.get('ovf', '?')} "
                f"hw={fields.get('hw', '?')} sem={fields.get('sem', '?')}"
            )
        if layer == "AXIS":
            return (
                f"{timestamp} AXIS z/l={fields.get('zero', '?')}/"
                f"{fields.get('limits', '?')} cal/test/osc="
                f"{fields.get('cal', '?')}/{fields.get('test', '?')}/"
                f"{fields.get('osc', '?')} fault={fields.get('pfault', '?')}"
            )
        if layer == "PCTRL":
            return (
                f"{timestamp} PCTRL {fields.get('fault', '?')} "
                f"err={fields.get('err', '?')} follow={fields.get('follow', '?')} "
                f"noenc={fields.get('noenc', '?')}"
            )
        if layer == "LIMIT":
            return (
                f"{timestamp} LIMIT count={fields.get('count', '?')} "
                f"dir={fields.get('dir', '?')} cur={fields.get('current', '?')} "
                f"[{fields.get('min', '?')},{fields.get('max', '?')}] "
                f"err={fields.get('err', '?')}"
            )
        return message

    def _record_debug(self, message: str) -> None:
        compact = self._compact_debug(message)
        wall_time = f"{datetime.now():%H:%M:%S.%f}"[:-3]
        self.debug_lines.append(compact)
        print(f"[{wall_time}] {compact}", flush=True)
        print(f"           RAW {message}", flush=True)
        try:
            self.log_dir.mkdir(parents=True, exist_ok=True)
            with self.debug_log_path.open("a", encoding="utf-8") as handle:
                handle.write(
                    f"{datetime.now():%Y-%m-%d %H:%M:%S.%f} {message}\n"
                )
        except OSError as exc:
            self.last_message = f"调试日志写入失败: {exc}"

    def _update(self, _frame):
        self._append_frames()
        status, good, bad, rx_bytes, messages = self.link.snapshot()
        now = time.monotonic()
        rate_elapsed = now - self.last_rate_time
        if rate_elapsed >= 0.5:
            self.receive_fps = (good - self.last_rate_frames) / rate_elapsed
            self.last_rate_frames = good
            self.last_rate_time = now
        if messages:
            for message in messages:
                if message.startswith("DBG,"):
                    self._record_debug(message)
                    continue
                if message.startswith("CAL,"):
                    self._handle_calibration_status(message)
                    print(f"[{datetime.now():%H:%M:%S.%f}] {message}", flush=True)
                    continue
                if message.startswith("CFG,"):
                    self._load_cfg(message)
                if message.startswith("ERR,CAL,BUSY_OR_FAULT"):
                    self.calibration_mode = False
                    self.shortcut_text.set_text(
                        "参数框：↑/↓微调，Shift+↑/↓快调\n"
                        "修改后仍需点击应用按钮"
                    )
                self.last_message = message
        self.status_text.set_text(
            f"{status}\nFPS={self.receive_fps:5.1f} "
            f"frames={good} bad={bad}\nRX={rx_bytes}B"
        )
        self.message_text.set_text(f"最后消息\n{self.last_message}")
        if self.debug_lines:
            self.debug_text.set_text(
                "保护链日志（自动保存）\n" + "\n".join(self.debug_lines)
            )
        if not self.times:
            return ()
        t = np.asarray(self.times)
        c = [np.asarray(channel) for channel in self.channels]
        for line, data in (
            (self.pos_line, c[0]), (self.pred_line, c[1]), (self.err_line, c[3]),
            (self.p_line, c[4]), (self.d_line, c[5]), (self.pd_line, c[6]),
            (self.cmd_line, c[7]), (self.enc_line, c[8]),
            (self.axis_target_line, c[7]),
        ):
            line.set_data(t, data)
        if (now - self.last_autoscale_time) >= 0.25:
            for axis in self.axes:
                axis.relim()
                axis.autoscale_view()
            self.last_autoscale_time = now
        self.axes[-1].set_xlim(max(0.0, t[-1] - self.window), max(self.window, t[-1]))
        try:
            target_mm = float(self.boxes["T"].text.strip())
        except ValueError:
            target_mm = 0.0
        self.target_line.set_ydata([target_mm, target_mm])
        self.value_text.set_text(
            "当前值\n"
            f"target{target_mm:+8.1f} mm\n"
            f"pos   {c[0][-1]:+8.1f} mm\n"
            f"error {c[3][-1]:+8.1f} mm\n"
            f"vel   {c[2][-1]:+8.1f} mm/s\n"
            f"P/D   {c[4][-1]:+6.1f}/{c[5][-1]:+6.1f}\n"
            f"PD    {c[6][-1]:+8.1f} count\n"
            f"cmd   {c[7][-1]:+8.1f} count\n"
            f"enc   {c[8][-1]:+8.1f} count\n"
            f"step  {c[9][-1]:+8.1f} Hz\n"
            f"valid {int(c[10][-1])} age={c[11][-1]:.0f}ms\n"
            f"KP/KD {c[12][-1]:.2f}/{c[13][-1]:.2f}\n"
            f"DB/state {c[14][-1]:.0f}/{c[15][-1]:.0f}"
        )
        return ()

    def show(self) -> None:
        self.link.start()
        try:
            plt.show()
        finally:
            self.link.stop()


class TerminalMonitor:
    """Low-overhead terminal console without Matplotlib redraws."""

    STATE_NAMES = {
        0: "OFF",
        1: "AXIS",
        2: "VIS",
        3: "ACT",
        4: "ZERO",
        5: "LOST",
        6: "FAULT",
        7: "HOLD",
    }

    def __init__(self, link: BalanceSerial, watch_hz: float = 2.0) -> None:
        self.link = link
        self.stop_event = threading.Event()
        self.print_thread: threading.Thread | None = None
        self.watch_hz = max(0.0, min(20.0, watch_hz))
        self.last_watch_time = 0.0
        self.last_values: tuple[float, ...] | None = None
        self.last_good = 0
        self.show_rx_errors = False
        self.force_rx_debug_once = False
        self.rx_error_suppressed = 0
        self.last_rx_error_print = 0.0

    @staticmethod
    def _help() -> str:
        return """可用命令：
  cal              进入完整标定
  u / d            正向/反向点动20 STEP
  uu / dd          正向/反向快速点动100 STEP
  set              确认当前ZERO/端点
  cal?             查询标定状态
  abort            取消标定
  t 0              设置小球目标mm；例如t 50、t -50
  kp 1.50          设置位置环KPOS；KD为速度环KVEL
  vmax/bias/...    可设置VMAX、BIAS、AP、AN、TD、BM、VAPP
  run / stop       启动或停止平衡
  get / def        读取参数或恢复编译默认值
  status           获取参数、标定和保护链快照
  cfg 1.5 .6 -1 80 5 2 3 0 120 0 500 500 80 5 20
                   一键设置基础参数和串级/制动参数，并自动重启
  watch 2          每秒显示2次遥测；watch off关闭
  errors on        显示RX错误摘要；errors off关闭
  help             显示帮助
  quit             退出终端
也可以直接输入原始命令，例如 KP=1.50、VMAX=120、T=0。"""

    def _translate(self, line: str) -> str | None:
        stripped = line.strip()
        if not stripped:
            return None
        parts = stripped.split()
        key = parts[0].lower()
        aliases = {
            "u": "J+", "up": "J+",
            "d": "J-", "down": "J-",
            "uu": "J++", "fastup": "J++",
            "dd": "J--", "fastdown": "J--",
            "set": "SET", "enter": "SET",
            "abort": "CALX", "cancel": "CALX",
            "cal": "CAL", "cal?": "CAL?",
            "run": "RUN", "stop": "STOP",
            "get": "GET", "def": "DEF", "log": "LOG",
        }
        if key in aliases and len(parts) == 1:
            return aliases[key]
        parameter_names = {
            "kp", "kd", "vmax", "bias", "ap", "an", "td", "bm",
            "vapp", "dir", "max", "slew", "db", "rg", "t",
        }
        if key in parameter_names and len(parts) == 2:
            return f"{key.upper()}={parts[1]}"
        return stripped.upper()

    def _print_telemetry(self, values: tuple[float, ...]) -> None:
        state_value = int(round(values[15]))
        state = self.STATE_NAMES.get(state_value, str(state_value))
        print(
            "TEL "
            f"state={state:<5} pos={values[0]:+6.1f}mm "
            f"err={values[3]:+6.1f} vel={values[2]:+7.1f} "
            f"PD={values[6]:+7.1f} cmd={values[7]:+6.1f} "
            f"enc={values[8]:+7.1f} step={values[9]:+6.1f}Hz "
            f"valid={int(values[10])} age={values[11]:.0f}ms",
            flush=True,
        )

    def _apply_full_config(self, parts: list[str]) -> None:
        if len(parts) not in (9, 16):
            print(
                "用法：cfg KP KD DIR MAX SLEW DB RG T\n"
                "完整：cfg KP KD DIR MAX SLEW DB RG T "
                "VMAX BIAS AP AN TD BM VAPP\n"
                "示例：cfg 1.50 0.60 -1 80 5 2 3 0 "
                "120 0 500 500 80 5 20",
                flush=True,
            )
            return
        try:
            kp = float(parts[1])
            kd = float(parts[2])
            direction = int(parts[3])
            maximum = int(parts[4])
            slew = int(parts[5])
            deadband = int(parts[6])
            reengage = int(parts[7])
            target = int(parts[8])
            if len(parts) == 16:
                vmax = int(parts[9])
                bias = int(parts[10])
                accel_pos = int(parts[11])
                accel_neg = int(parts[12])
                delay_ms = int(parts[13])
                brake_margin = int(parts[14])
                approach_velocity = int(parts[15])
        except ValueError:
            print("cfg参数格式错误", flush=True)
            return
        if not (
            0.0 <= kp <= 20.0
            and 0.0 <= kd <= 5.0
            and direction in (-1, 1)
            and 8 <= maximum <= 1024
            and 1 <= slew <= 128
            and 0 <= deadband <= 32
            and deadband < reengage <= 64
            and reengage <= maximum
            and -125 <= target <= 125
            and (
                len(parts) == 9
                or (
                    10 <= vmax <= 1000
                    and -256 <= bias <= 256
                    and 50 <= accel_pos <= 5000
                    and 50 <= accel_neg <= 5000
                    and 0 <= delay_ms <= 500
                    and 0 <= brake_margin <= 50
                    and 1 <= approach_velocity <= vmax
                )
            )
        ):
            print("cfg参数超出允许范围", flush=True)
            return

        ok, message = self.link.send("STOP")
        print(message, flush=True)
        if not ok:
            return
        print("等待平衡轴回零并进入OFF...", flush=True)
        time.sleep(0.2)
        deadline = time.monotonic() + 6.0
        while time.monotonic() < deadline:
            values = self.last_values
            if values is not None and int(round(values[15])) == 0:
                break
            time.sleep(0.05)
        else:
            print("等待OFF超时，未修改轴参数", flush=True)
            return

        commands = (
            f"DIR={direction}",
            f"MAX={maximum}",
            f"SLEW={slew}",
            f"DB={deadband}",
            f"RG={reengage}",
            f"KP={kp:.2f}",
            f"KD={kd:.2f}",
            f"T={target}",
        )
        if len(parts) == 16:
            commands += (
                f"VMAX={vmax}",
                f"BIAS={bias}",
                f"AP={accel_pos}",
                f"AN={accel_neg}",
                f"TD={delay_ms}",
                f"BM={brake_margin}",
                f"VAPP={approach_velocity}",
            )
        for command in commands:
            ok, message = self.link.send(command)
            print(message, flush=True)
            if not ok:
                print("配置中止", flush=True)
                return
            time.sleep(0.10)
        self.link.send("GET")
        time.sleep(0.15)
        ok, message = self.link.send("RUN")
        print(message, flush=True)
        if ok:
            print("一键配置已发送，等待ACK,RUN", flush=True)

    def _printer(self) -> None:
        while not self.stop_event.is_set():
            frames = self.link.take_frames()
            if frames:
                self.last_values = frames[-1][2]
            status, good, bad, rx_bytes, messages = self.link.snapshot()
            now = time.monotonic()
            for message in messages:
                wall_time = f"{datetime.now():%H:%M:%S.%f}"[:-3]
                if message.startswith("DBG,"):
                    if message.startswith("DBG,RX,"):
                        self.rx_error_suppressed += 1
                        should_print_rx = self.force_rx_debug_once or (
                            self.show_rx_errors
                            and now - self.last_rx_error_print >= 1.0
                        )
                        if not should_print_rx:
                            continue
                        compact = Monitor._compact_debug(message)
                        print(
                            f"[{wall_time}] {compact} "
                            f"(过去1秒合并{self.rx_error_suppressed}条RX事件)",
                            flush=True,
                        )
                        self.rx_error_suppressed = 0
                        self.last_rx_error_print = now
                        self.force_rx_debug_once = False
                        continue
                    print(
                        f"[{wall_time}] {Monitor._compact_debug(message)}",
                        flush=True,
                    )
                else:
                    print(f"[{wall_time}] {message}", flush=True)
            if (
                self.watch_hz > 0.0
                and self.last_values is not None
                and now - self.last_watch_time >= 1.0 / self.watch_hz
            ):
                self._print_telemetry(self.last_values)
                self.last_watch_time = now
            if (
                self.watch_hz > 0.0
                and good != self.last_good
                and good % 500 == 0
            ):
                print(
                    f"LINK {status} frames={good} bad={bad} RX={rx_bytes}B",
                    flush=True,
                )
            self.last_good = good
            self.stop_event.wait(0.03)

    def _handle_local_command(self, line: str) -> bool:
        parts = line.strip().split()
        if not parts:
            return True
        key = parts[0].lower()
        if key in ("quit", "exit", "q"):
            return False
        if key in ("help", "h", "?"):
            print(self._help(), flush=True)
            return True
        if key in ("cfg", "tune"):
            self._apply_full_config(parts)
            return True
        if key == "watch":
            if len(parts) != 2:
                print(f"watch={self.watch_hz:g} Hz", flush=True)
                return True
            if parts[1].lower() in ("off", "0"):
                self.watch_hz = 0.0
            else:
                try:
                    self.watch_hz = max(0.1, min(20.0, float(parts[1])))
                except ValueError:
                    print("用法：watch 2 或 watch off", flush=True)
                    return True
            print(f"watch={self.watch_hz:g} Hz", flush=True)
            return True
        if key == "errors":
            if len(parts) != 2 or parts[1].lower() not in ("on", "off"):
                state = "on" if self.show_rx_errors else "off"
                print(f"用法：errors on/off（当前{state}）", flush=True)
                return True
            self.show_rx_errors = parts[1].lower() == "on"
            self.rx_error_suppressed = 0
            print(
                f"RX错误输出={'on' if self.show_rx_errors else 'off'}",
                flush=True,
            )
            return True
        if key == "status":
            self.force_rx_debug_once = True
            for command in ("GET", "CAL?", "LOG"):
                ok, message = self.link.send(command)
                if not ok:
                    print(message, flush=True)
                    break
            return True
        command = self._translate(line)
        if command is not None:
            ok, message = self.link.send(command)
            print(message, flush=True)
            if not ok:
                return True
        return True

    def show(self) -> None:
        self.link.start()
        self.print_thread = threading.Thread(target=self._printer, daemon=True)
        self.print_thread.start()
        print(self._help(), flush=True)
        try:
            while True:
                try:
                    line = input("bal> ")
                except EOFError:
                    break
                if not self._handle_local_command(line):
                    break
        except KeyboardInterrupt:
            print("\n退出", flush=True)
        finally:
            self.stop_event.set()
            if self.print_thread:
                self.print_thread.join(timeout=1.0)
            self.link.stop()


def list_ports() -> None:
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("没有发现串口")
        return
    for port in ports:
        print(f"{port.device:<10} {port.description}")


def main() -> None:
    parser = argparse.ArgumentParser(description="平衡滚球PD实时串口调参")
    parser.add_argument("--port", default="COM9")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--window", type=float, default=20.0)
    parser.add_argument("--cli", action="store_true", help="使用轻量终端模式，不启动绘图")
    parser.add_argument(
        "--watch", default="2",
        help="CLI遥测刷新率Hz；使用off或0关闭",
    )
    parser.add_argument("--list", action="store_true")
    args = parser.parse_args()
    if args.list:
        list_ports()
        return
    if str(args.watch).lower() in ("off", "none"):
        watch_hz = 0.0
    else:
        try:
            watch_hz = float(args.watch)
        except ValueError:
            parser.error("--watch 必须是数字、0或off")
    link = BalanceSerial(args.port, args.baud)
    if args.cli:
        TerminalMonitor(link, watch_hz).show()
    else:
        Monitor(link, args.window).show()


if __name__ == "__main__":
    main()
