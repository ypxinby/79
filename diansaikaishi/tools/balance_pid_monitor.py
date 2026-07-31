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
    "kp_count_per_mm",
    "kd_count_per_mm_s",
    "tracking_deadband_count",
    "controller_state",
)

PARAMETER_STEPS = {
    "KP": 0.05,
    "KD": 0.01,
    "MAX": 8.0,
    "SLEW": 1.0,
    "DB": 1.0,
    "RG": 1.0,
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
            if line.startswith(("ACK,", "ERR,", "CFG,", "DBG,")):
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
            payload = (command + "\n").encode("ascii")
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
        ax_pos.axhline(10, color="gray", linestyle=":")
        ax_pos.axhline(-10, color="gray", linestyle=":")
        ax_pos.legend(loc="upper left", ncol=3)

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
            ("KP", "6.00"), ("KD", "0.10"), ("DIR", "1"),
            ("MAX", "200"), ("SLEW", "5"), ("DB", "2"), ("RG", "3"),
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
            Button(self.figure.add_axes([0.76, 0.22, 0.10, 0.045]), "应用PD"),
            Button(self.figure.add_axes([0.87, 0.22, 0.11, 0.045]), "应用轴参数"),
            Button(self.figure.add_axes([0.76, 0.16, 0.065, 0.045]), "RUN"),
            Button(self.figure.add_axes([0.835, 0.16, 0.065, 0.045]), "STOP"),
            Button(self.figure.add_axes([0.91, 0.16, 0.065, 0.045]), "GET"),
            Button(self.figure.add_axes([0.76, 0.10, 0.08, 0.045]), "默认"),
            Button(self.figure.add_axes([0.85, 0.10, 0.08, 0.045]), "CSV"),
            Button(self.figure.add_axes([0.94, 0.10, 0.05, 0.045]), "PARAM"),
            Button(self.figure.add_axes([0.76, 0.04, 0.10, 0.045]), "诊断"),
        ]
        self.buttons[0].on_clicked(self._apply_pd)
        self.buttons[1].on_clicked(self._apply_axis)
        self.buttons[2].on_clicked(lambda _e: self._send("RUN"))
        self.buttons[3].on_clicked(lambda _e: self._send("STOP"))
        self.buttons[4].on_clicked(lambda _e: self._send("GET"))
        self.buttons[5].on_clicked(lambda _e: self._send("DEF"))
        self.buttons[6].on_clicked(lambda _e: self.save_csv())
        self.buttons[7].on_clicked(self._request_parameter_save)
        self.buttons[8].on_clicked(lambda _e: self._send("LOG"))

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
            value = min(10.0, max(0.0, value))
            text = f"{value:.2f}"
        elif name == "KD":
            value = min(1.0, max(0.0, value))
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
        else:  # RG
            minimum = max(1, self._parameter_int("DB", 2) + 1)
            maximum = max(minimum,
                min(64, self._parameter_int("MAX", 128)))
            value = min(maximum, max(minimum, round(value)))
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
        self._apply_names(("KP", "KD"))

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
            if self.pending_parameter_save:
                self.pending_parameter_save = False
                self._write_parameter_snapshot(values)
        except (ValueError, KeyError):
            self.last_message = f"CFG解析失败: {message}"

    def _request_parameter_save(self, _event) -> None:
        self.pending_parameter_save = True
        ok, self.last_message = self.link.send("GET")
        if not ok:
            self.pending_parameter_save = False

    def _write_parameter_snapshot(self, values: dict[str, str]) -> None:
        required = ("KP", "KD", "DIR", "MAX", "SLEW", "DB", "RG")
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
                f"v/m={fields.get('valid', '?')}/{fields.get('meas', '?')}"
            )
        if layer == "RX":
            return (
                f"{timestamp} RX {fields.get('event', '?')} "
                f"len/crc/fld={fields.get('len', '?')}/"
                f"{fields.get('crc', '?')}/{fields.get('field', '?')} "
                f"dup/old/ovf={fields.get('dup', '?')}/"
                f"{fields.get('old', '?')}/{fields.get('ovf', '?')}"
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
        self.debug_lines.append(self._compact_debug(message))
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
                if message.startswith("CFG,"):
                    self._load_cfg(message)
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
        self.value_text.set_text(
            "当前值\n"
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
    parser.add_argument("--list", action="store_true")
    args = parser.parse_args()
    if args.list:
        list_ports()
        return
    Monitor(BalanceSerial(args.port, args.baud), args.window).show()


if __name__ == "__main__":
    main()
