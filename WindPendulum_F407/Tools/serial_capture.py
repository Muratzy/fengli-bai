#!/usr/bin/env python3
"""Capture the STM32 motor telemetry without sending data to the target."""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import json
import math
import os
import re
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple


PROTOCOL_FIELDS = [
    "time_ms",
    "state",
    "reason",
    "duty_permille",
    "en_command",
    "en_pin",
    "ccr1",
    "ccr2",
    "ccr3",
    "ccr4",
    "current_adc",
    "current_valid",
    "voltage_adc",
    "vrefint_adc",
    "vdda_mv",
    "u1_pin_mv",
    "u1_input_mv",
    "u1_filtered_mv",
    "voltage_valid",
    "keys",
    "run_ms",
    "step_gap_ms",
    "max_step_gap_ms",
    "current_ma",
    "current_filtered_ma",
    "current_calc_valid",
    "current_overrange",
    "reference_calibrated",
]
EXPECTED_HEADER = ",".join(PROTOCOL_FIELDS)
STATES = {"RELEASE", "READY", "FIXED", "RAMP"}
REASONS = {"BOOT", "STOP_KEY", "LOOP_DELAY", "EN_LOW"}
UINT32_MAX = 0xFFFFFFFF

HOST_FIELDS = [
    "pc_sequence",
    "pc_receive_time",
    "pc_elapsed_ms",
    "connection_id",
    "board_epoch",
    "board_timestamp_delta_ms",
    "timing_gap_suspected",
    "board_event",
]
DERIVED_FIELDS = [
    "duty_percent",
    "current_a",
    "voltage_v",
    "key1_pressed",
    "key2_pressed",
    "key3_pressed",
]
CSV_FIELDS = HOST_FIELDS + PROTOCOL_FIELDS + DERIVED_FIELDS

FIELD_DESCRIPTIONS = {
    "time_ms": ("ms", "板端 uint32 启动计时，约 49.7 天回绕"),
    "state": ("enum", "电机测试状态 RELEASE / READY / FIXED / RAMP"),
    "reason": ("enum", "最近一次停止原因；BOOT 不代表每行都在复位"),
    "duty_permille": ("permille", "PWM 指令千分比 0~800；不是实际转速"),
    "en_command": ("bool", "EN 软件命令，1 表示释放开漏输出"),
    "en_pin": ("bool", "PD7 实际输入电平，不证明驱动输出正常"),
    "ccr1": ("count", "TIM1_CH1 比较值，当前 ARR=8399"),
    "ccr2": ("count", "TIM1_CH2 比较值"),
    "ccr3": ("count", "TIM1_CH3 比较值"),
    "ccr4": ("count", "TIM1_CH4 比较值"),
    "current_adc": ("count", "PB0 电流 ADC 原始码 0~4095"),
    "current_valid": ("bool", "仅表示 PB0 原始 ADC 转换成功"),
    "voltage_adc": ("count", "PB1 电压 ADC 原始码 0~4095"),
    "vrefint_adc": ("count", "内部 VREFINT 原始码"),
    "vdda_mv": ("mV", "根据内部参考估算的 VDDA"),
    "u1_pin_mv": ("mV", "PB1 引脚电压"),
    "u1_input_mv": ("mV", "按配置比例折算的 U1 电压；当前未验证为母线电压"),
    "u1_filtered_mv": ("mV", "U1 滤波电压，仍需结合 voltage_valid"),
    "voltage_valid": ("bool", "电压采样及换算有效标志"),
    "keys": ("bitmask", "bit0=KEY1/PC13，bit1=KEY2/PC0，bit2=KEY3/PC1；1 为按下"),
    "run_ms": ("ms", "电机测试运行时间"),
    "step_gap_ms": ("ms", "电机任务相邻步进间隔"),
    "max_step_gap_ms": ("ms", "本次 MCU 启动以来最大步进间隔"),
    "current_ma": ("mA", "未滤波电流换算值；不是已实测标定值"),
    "current_filtered_ma": ("mA", "滤波电流换算值；原值保留，即使无效"),
    "current_calc_valid": ("bool", "电流换算有效标志"),
    "current_overrange": ("bool", "电流采样或换算超量程"),
    "reference_calibrated": ("bool", "是否配置用户参考校准；不是完整电流精度认证"),
    "pc_sequence": ("count", "从 1 开始的电脑保存行号，不是 MCU 样本序号"),
    "pc_receive_time": ("ISO 8601", "完整行所在字节块的电脑接收时间，含时区；不是采样时刻"),
    "pc_elapsed_ms": ("ms", "采集启动后的电脑单调计时，不受系统校时影响"),
    "connection_id": ("count", "同一会话内的连接编号，从 1 开始"),
    "board_epoch": ("count", "观察到启动横幅、疑似复位或回绕时递增，不是可靠启动次数"),
    "board_timestamp_delta_ms": ("ms", "同一连接内相邻有效行的板端时间差；首行/复位留空"),
    "timing_gap_suspected": ("bool", "板端时间差超过阈值，默认 250 ms；不等于精确丢帧数"),
    "board_event": ("enum", "initial / boot_banner / timestamp_reset / uptime_wrap / 空"),
    "duty_percent": ("%", "duty_permille/10；数值 30.0 表示 30%"),
    "current_a": ("A", "current_filtered_ma/1000；采样/换算无效或超量程时留空"),
    "voltage_v": ("V", "u1_filtered_mv/1000；电压无效时留空"),
    "key1_pressed": ("bool", "KEY1/PC13 按下状态"),
    "key2_pressed": ("bool", "KEY2/PC0 按下状态"),
    "key3_pressed": ("bool", "KEY3/PC1 按下状态"),
}


def iso_now() -> str:
    return dt.datetime.now(dt.timezone.utc).astimezone().isoformat(timespec="milliseconds")


def sanitize_label(value: str) -> str:
    clean = re.sub(r"[^\w.-]+", "_", value.strip())
    return clean.strip("._-")[:48]


def parse_int(value: str, name: str, minimum: int = 0, maximum: int = UINT32_MAX) -> int:
    if not re.fullmatch(r"[0-9]+", value):
        raise ValueError("%s is not an unsigned integer" % name)
    number = int(value)
    if number < minimum or (maximum is not None and number > maximum):
        raise ValueError("%s is outside [%s, %s]" % (name, minimum, maximum))
    return number


def validate_protocol_row(values: Sequence[str]) -> Dict[str, object]:
    if len(values) != len(PROTOCOL_FIELDS):
        raise ValueError("expected %d fields, got %d" % (len(PROTOCOL_FIELDS), len(values)))
    row: Dict[str, object] = {}
    for name, value in zip(PROTOCOL_FIELDS, values):
        row[name] = value

    row["time_ms"] = parse_int(values[0], "time_ms", 0, UINT32_MAX)
    if values[1] not in STATES:
        raise ValueError("unknown state %r" % values[1])
    if values[2] not in REASONS:
        raise ValueError("unknown reason %r" % values[2])
    row["duty_permille"] = parse_int(values[3], "duty_permille", 0, 800)
    for name in ("en_command", "en_pin"):
        row[name] = parse_int(str(row[name]), name, 0, 1)
    for name in ("ccr1", "ccr2", "ccr3", "ccr4"):
        row[name] = parse_int(str(row[name]), name, 0, 8399)
    for name in ("current_adc", "voltage_adc", "vrefint_adc"):
        row[name] = parse_int(str(row[name]), name, 0, 4095)
    for name in (
        "current_valid",
        "voltage_valid",
        "current_calc_valid",
        "current_overrange",
        "reference_calibrated",
    ):
        row[name] = parse_int(str(row[name]), name, 0, 1)
    for name in (
        "vdda_mv",
        "u1_pin_mv",
        "u1_input_mv",
        "u1_filtered_mv",
        "run_ms",
        "step_gap_ms",
        "max_step_gap_ms",
        "current_ma",
        "current_filtered_ma",
    ):
        row[name] = parse_int(str(row[name]), name)
    row["keys"] = parse_int(values[19], "keys", 0, 7)
    return row


class TelemetryParser:
    """Parse the fixed firmware protocol and infer host-side timing context."""

    def __init__(self, expected_period_ms: int = 100, gap_factor: float = 2.5) -> None:
        self.expected_period_ms = expected_period_ms
        self.gap_threshold_ms = max(expected_period_ms + 1, int(expected_period_ms * gap_factor))
        self.last_board_time: Optional[int] = None
        self.board_epoch = 0
        self.connection_id = 0
        self.schema_matched = False
        self.schema_assumed = True
        self.boot_pending = False

    def start_connection(self) -> None:
        self.connection_id += 1
        self.last_board_time = None
        self.schema_matched = False
        self.schema_assumed = True
        self.boot_pending = False

    def process_line(self, raw_line: bytes) -> Tuple[str, Optional[Dict[str, object]]]:
        try:
            text = raw_line.decode("ascii").rstrip("\r\n")
        except UnicodeDecodeError:
            return "invalid_row", None
        if not text:
            return "empty", None
        if text == EXPECTED_HEADER:
            self.schema_matched = True
            self.schema_assumed = False
            return "header", None
        if text.startswith("time_ms,") or ("," in text and len(set(text.split(",")) & set(PROTOCOL_FIELDS)) >= 2):
            self.schema_matched = False
            self.schema_assumed = False
            return "schema_mismatch", None
        if text.startswith("MOTOR1_BENCH,") or text.startswith("KEY1="):
            if text.startswith("MOTOR1_BENCH,"):
                self.boot_pending = True
            return "banner", None
        if not self.schema_matched and self.schema_assumed is False:
            return "schema_blocked", None
        if "," not in text:
            return "noise", None
        try:
            values = next(csv.reader([text]))
            row = validate_protocol_row(values)
        except (StopIteration, ValueError, csv.Error):
            return "invalid_row", None

        board_time = int(row["time_ms"])
        event = ""
        delta: Optional[int] = None
        gap = 0
        if self.boot_pending:
            self.board_epoch += 1
            event = "boot_banner"
            self.boot_pending = False
        elif self.last_board_time is None:
            event = "initial"
        elif board_time >= self.last_board_time:
            delta = board_time - self.last_board_time
            gap = int(delta > self.gap_threshold_ms)
        elif self.last_board_time >= 0xF0000000 and board_time <= 0x0FFFFFFF:
            delta = (board_time - self.last_board_time) & UINT32_MAX
            self.board_epoch += 1
            event = "uptime_wrap"
            gap = int(delta > self.gap_threshold_ms)
        else:
            self.board_epoch += 1
            event = "timestamp_reset"
        self.last_board_time = board_time
        return "row", {
            "protocol": row,
            "board_delta": "" if delta is None else delta,
            "board_epoch": self.board_epoch,
            "timing_gap": gap,
            "board_event": event,
        }


def enrich_row(parsed: Dict[str, object], sequence: int, receive_time: str,
               elapsed_ms: int, connection_id: int) -> Dict[str, object]:
    protocol = parsed["protocol"]
    assert isinstance(protocol, dict)
    keys = int(protocol["keys"])
    values: Dict[str, object] = {
        "pc_sequence": sequence,
        "pc_receive_time": receive_time,
        "pc_elapsed_ms": elapsed_ms,
        "connection_id": connection_id,
        "board_epoch": parsed["board_epoch"],
        "board_timestamp_delta_ms": parsed["board_delta"],
        "timing_gap_suspected": parsed["timing_gap"],
        "board_event": parsed["board_event"],
    }
    values.update(protocol)
    values.update(
        {
            "duty_percent": "%.1f" % (int(protocol["duty_permille"]) / 10.0),
            "current_a": ("%.3f" % (int(protocol["current_filtered_ma"]) / 1000.0)
                          if protocol["current_valid"] and protocol["current_calc_valid"]
                          and not protocol["current_overrange"] else ""),
            "voltage_v": ("%.3f" % (int(protocol["u1_filtered_mv"]) / 1000.0)
                          if protocol["voltage_valid"] else ""),
            "key1_pressed": (keys >> 0) & 1,
            "key2_pressed": (keys >> 1) & 1,
            "key3_pressed": (keys >> 2) & 1,
        }
    )
    return values


def firmware_manifest(project_root: Path) -> Optional[Dict[str, object]]:
    manifest_path = project_root / "build" / "workflow" / "last-run.json"
    try:
        with manifest_path.open("r", encoding="utf-8") as handle:
            data = json.load(handle)
    except (OSError, ValueError):
        return None
    keys = ("sha256", "builtAtUtc", "flashedAtUtc", "flashVerified", "probeSerial", "speedKHz")
    return {key: data[key] for key in keys if key in data}


def find_port_description(port: str) -> str:
    try:
        from serial.tools import list_ports
        for item in list_ports.comports():
            if item.device.upper() == port.upper():
                return item.description or ""
    except ImportError:
        pass
    return ""


def event_row(event_type: str, severity: str, connection_id: int, detail: str) -> List[object]:
    # Prevent raw event text from becoming an Excel formula; raw.bin is unchanged.
    if detail.lstrip()[:1] in {"=", "+", "-", "@"}:
        detail = "'" + detail
    return [iso_now(), event_type, severity, connection_id, detail]


def write_event(handle, event_type: str, severity: str, connection_id: int, detail: str) -> None:
    csv.writer(handle).writerow(event_row(event_type, severity, connection_id, detail))


def make_session_dir(root: Path, port: str, label: str) -> Path:
    stamp = dt.datetime.now().astimezone().strftime("%Y%m%d_%H%M%S")
    name = "%s_%s" % (stamp, sanitize_label(port))
    clean_label = sanitize_label(label)
    if clean_label:
        name += "_" + clean_label
    candidate = root / name
    suffix = 1
    while candidate.exists():
        candidate = root / (name + "_%d" % suffix)
        suffix += 1
    candidate.mkdir(parents=True, exist_ok=False)
    return candidate


def write_fields_file(path: Path) -> None:
    with path.open("w", encoding="utf-8-sig", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["order", "field", "unit", "description"])
        for index, field in enumerate(CSV_FIELDS, 1):
            unit, description = FIELD_DESCRIPTIONS[field]
            writer.writerow([index, field, unit, description])


class LineFramer:
    """Frame LF lines across reads and discard oversize input until LF."""

    def __init__(self, limit: int) -> None:
        self.limit = limit
        self.buffer = bytearray()
        self.discarding = False

    def reset(self) -> None:
        self.buffer.clear()
        self.discarding = False

    def feed(self, chunk: bytes):
        parts = chunk.split(b"\n")
        for index, part in enumerate(parts):
            complete = index < len(parts) - 1
            if not self.discarding:
                if len(self.buffer) + len(part) + int(complete) > self.limit:
                    self.buffer.clear()
                    self.discarding = True
                    yield "line_oversize", b""
                else:
                    self.buffer.extend(part)
            if complete:
                if not self.discarding:
                    yield "line", bytes(self.buffer) + b"\n"
                self.reset()


def write_metadata(path: Path, value: Dict[str, object]) -> None:
    temporary = path.with_suffix(".json.tmp")
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding="utf-8")
    os.replace(str(temporary), str(path))


def capture(args: argparse.Namespace) -> int:
    try:
        import serial
    except ImportError:
        print("Install dependency: python -m pip install -r Tools/requirements-pc.txt", file=sys.stderr)
        return 3

    project_root = Path(__file__).resolve().parents[1]
    root = Path(args.output).expanduser() if args.output else project_root / "数据记录" / "串口"
    directory = make_session_dir(root.resolve(), args.port, args.label)
    metadata_path = directory / "session.json"
    parser = TelemetryParser(args.expected_period_ms, args.gap_factor)
    framer = LineFramer(args.max_line_bytes)
    stats = dict.fromkeys((
        "valid_rows", "invalid_rows", "bytes", "lines", "connections",
        "connect_errors", "disconnects", "schema_mismatches",
        "rows_with_assumed_schema", "oversize_lines", "timing_gaps",
    ), 0)
    start = time.monotonic()
    deadline = start + args.duration if args.duration is not None else float("inf")
    session = {
        "capture_version": 1, "status": "running", "started_at": iso_now(),
        "label": args.label, "port": args.port, "baudrate": args.baud,
        "format": "8N1", "flow_control": "none", "write_to_target": False,
        "control_lines_requested": {"dtr": False, "rts": False},
        "protocol": "motor1_bench_28", "protocol_header": EXPECTED_HEADER,
        "protocol_source": "Core/Src/diagnostics/motor1_test_stm32.c",
        "expected_period_ms": args.expected_period_ms,
        "gap_threshold_ms": parser.gap_threshold_ms,
        "session_directory": str(directory),
        "port_description": find_port_description(args.port),
        "python_version": sys.version.split()[0], "pyserial_version": serial.VERSION,
        "local_firmware_manifest": firmware_manifest(project_root),
        "firmware_provenance_note": "Local file only; firmware identity was not queried from the board.",
        "schema_matched": False, "schema_assumed": True, "stats": stats,
        "output_files": {name: name for name in ("telemetry.csv", "raw.bin", "events.csv", "fields.csv")},
    }
    write_fields_file(directory / "fields.csv")
    write_metadata(metadata_path, session)
    print("Session: %s\nReceiving %s at %d baud. Ctrl+C stops capture." %
          (directory, args.port, args.baud), flush=True)
    device = None
    stop_reason = "duration"
    fatal_error = None

    def close_device():
        nonlocal device
        if device is not None:
            try:
                device.close()
            except OSError:
                pass
            device = None

    def retry_delay():
        time.sleep(max(0.0, min(args.reconnect_delay, deadline - time.monotonic())))

    try:
        with (directory / "raw.bin").open("wb") as raw, \
                (directory / "telemetry.csv").open("w", encoding="utf-8-sig", newline="") as telemetry, \
                (directory / "events.csv").open("w", encoding="utf-8-sig", newline="") as events:
            writer = csv.DictWriter(telemetry, fieldnames=CSV_FIELDS)
            writer.writeheader()
            csv.writer(events).writerow(["pc_event_time", "event", "severity", "connection_id", "detail"])

            def event(kind, detail, severity="info"):
                write_event(events, kind, severity, parser.connection_id, detail)

            def checkpoint():
                for handle in (raw, telemetry, events):
                    handle.flush()
                session.update(schema_matched=parser.schema_matched, schema_assumed=parser.schema_assumed,
                               updated_at=iso_now(), duration_s=round(time.monotonic() - start, 3))
                write_metadata(metadata_path, session)

            def discard_partial(reason):
                if framer.buffer or framer.discarding:
                    event(reason, "buffer_bytes=%d; oversize=%s; raw_offset=%d" %
                          (len(framer.buffer), framer.discarding, stats["bytes"]), "warning")
                framer.reset()

            checkpoint()
            last_flush = last_progress = time.monotonic()
            try:
                while time.monotonic() < deadline:
                    if device is None:
                        try:
                            device = serial.Serial(
                                port=None, baudrate=args.baud, bytesize=serial.EIGHTBITS,
                                parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                                timeout=args.read_timeout, xonxoff=False, rtscts=False, dsrdtr=False,
                            )
                            device.dtr = False
                            device.rts = False
                            device.port = args.port
                            device.open()
                        except (OSError, serial.SerialException) as error:
                            close_device()
                            stats["connect_errors"] += 1
                            event("connect_error", str(error), "error")
                            if stats["connect_errors"] == 1 or stats["connect_errors"] % 5 == 0:
                                print("Cannot open %s: %s; retrying." % (args.port, error), flush=True)
                            checkpoint()
                            retry_delay()
                            continue
                        parser.start_connection()
                        stats["connections"] += 1
                        event("connect", "port=%s; raw_offset=%d; schema=assumed" % (args.port, stats["bytes"]))
                        checkpoint()
                        print("Connected: %s (connection %d)" % (args.port, parser.connection_id), flush=True)
                    # Only transport operations are retried. Disk errors stop capture.
                    try:
                        # Keep the timeout fixed: pyserial on Windows reapplies the
                        # UART configuration even when assigning the same timeout.
                        # A timed capture can overrun by at most one read timeout.
                        chunk = device.read(max(1, min(device.in_waiting, args.chunk_size)))
                    except (OSError, serial.SerialException) as error:
                        stats["disconnects"] += 1
                        event("disconnect", str(error), "warning")
                        print("Disconnected; retrying: %s" % error, flush=True)
                        close_device()
                        discard_partial("partial_line_at_disconnect")
                        checkpoint()
                        retry_delay()
                        continue
                    received = iso_now()
                    elapsed_ms = int((time.monotonic() - start) * 1000)
                    if chunk:
                        raw.write(chunk)
                        stats["bytes"] += len(chunk)
                        stats["lines"] += chunk.count(b"\n")
                        for kind, line in framer.feed(chunk):
                            if kind == "line_oversize":
                                stats["oversize_lines"] += 1
                                event(kind, "line exceeded %d bytes; retained in raw.bin" % args.max_line_bytes, "warning")
                                continue
                            result, parsed = parser.process_line(line)
                            if result == "row":
                                sequence = stats["valid_rows"] + 1
                                writer.writerow(enrich_row(parsed, sequence, received, elapsed_ms, parser.connection_id))
                                stats["valid_rows"] += 1
                                stats["rows_with_assumed_schema"] += int(parser.schema_assumed)
                                if parsed["board_event"]:
                                    event(parsed["board_event"], "pc_sequence=%d; time_ms=%d" %
                                          (stats["valid_rows"], parsed["protocol"]["time_ms"]))
                                if parsed["timing_gap"]:
                                    stats["timing_gaps"] += 1
                                    event("timing_gap_suspected", "pc_sequence=%d; delta_ms=%s" %
                                          (stats["valid_rows"], parsed["board_delta"]), "warning")
                            elif result in ("invalid_row", "schema_blocked"):
                                stats["invalid_rows"] += 1
                                event(result, line.decode("utf-8", errors="replace").rstrip(), "warning")
                            elif result == "schema_mismatch":
                                stats["schema_mismatches"] += 1
                                event(result, line.decode("ascii").rstrip(), "error")
                                print("Protocol changed: structured parsing is blocked until a matching header.", flush=True)
                            elif result == "header":
                                event("schema_header", "matched bundled 28-column header")
                            elif result in ("noise", "banner"):
                                event(result, line.decode("utf-8", errors="replace").rstrip())
                    now = time.monotonic()
                    if now - last_flush >= 1.0:
                        checkpoint()
                        last_flush = now
                    if now - last_progress >= 2.0:
                        print("Rows=%d, invalid=%d, bytes=%d%s" %
                              (stats["valid_rows"], stats["invalid_rows"], stats["bytes"],
                               "; waiting for valid telemetry" if not stats["valid_rows"] else ""), flush=True)
                        last_progress = now
            except KeyboardInterrupt:
                stop_reason = "keyboard_interrupt"
            except (OSError, ValueError):
                stop_reason = "capture_error"
                raise
            finally:
                close_device()
                discard_partial("partial_line_at_stop")
                event("stop", stop_reason)
                checkpoint()
    except (OSError, ValueError) as error:
        fatal_error = str(error)
        stop_reason = "capture_error"
        print("Capture failed: %s" % error, file=sys.stderr)
    finally:
        close_device()
        exit_code = 5 if fatal_error else (4 if stats["schema_mismatches"] else (0 if stats["valid_rows"] else 2))
        session.update(
            status="failed" if fatal_error else ("schema_mismatch" if exit_code == 4 else
                   ("completed" if stats["valid_rows"] else "no_valid_data")),
            ended_at=iso_now(), stop_reason=stop_reason, exit_code=exit_code,
            duration_s=round(time.monotonic() - start, 3), error=fatal_error,
            schema_matched=parser.schema_matched, schema_assumed=parser.schema_assumed,
        )
        try:
            write_metadata(metadata_path, session)
        except OSError as error:
            print("Cannot save final session metadata: %s" % error, file=sys.stderr)
            exit_code = 5
    print("Saved: %s\nValid rows: %d; invalid rows: %d; bytes: %d; exit: %d" %
          (directory, stats["valid_rows"], stats["invalid_rows"], stats["bytes"], exit_code), flush=True)
    return exit_code


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Read-only STM32 telemetry capture")
    parser.add_argument("--list-ports", action="store_true", help="list detected serial ports and exit")
    parser.add_argument("--port", default="COM9", help="serial port (default: COM9)")
    parser.add_argument("--baud", type=int, default=115200, help="baud rate (default: 115200)")
    parser.add_argument("--duration", type=float, default=None, help="stop after this many seconds")
    parser.add_argument("--label", default="", help="optional session label")
    parser.add_argument("--output", default=None, help="session root directory")
    parser.add_argument("--read-timeout", type=float, default=0.2, help="serial read timeout")
    parser.add_argument("--reconnect-delay", type=float, default=2.0, help="seconds between reconnect attempts")
    parser.add_argument("--chunk-size", type=int, default=4096, help="maximum bytes per serial read")
    parser.add_argument("--max-line-bytes", type=int, default=4096, help="bounded line size")
    parser.add_argument("--expected-period-ms", type=int, default=100, help="expected board sample period")
    parser.add_argument("--gap-factor", type=float, default=2.5, help="timing gap threshold multiplier")
    return parser


def list_ports() -> int:
    try:
        from serial.tools import list_ports
    except ImportError:
        print("pyserial is required; run: python -m pip install -r Tools/requirements-pc.txt", file=sys.stderr)
        return 3
    ports = list(list_ports.comports())
    if not ports:
        print("No serial ports detected.")
        return 0
    for item in ports:
        print("%s\t%s\t%s" % (item.device, item.description or "", item.hwid or ""))
    return 0


def main(argv=None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.list_ports:
        return list_ports()
    for name in ("baud", "read_timeout", "reconnect_delay", "chunk_size",
                 "max_line_bytes", "expected_period_ms", "gap_factor", "duration"):
        value = getattr(args, name)
        if value is not None and (not math.isfinite(value) or value <= 0):
            parser.error("--%s must be finite and positive" % name.replace("_", "-"))
    if not args.port.strip() or args.gap_factor <= 1 or args.read_timeout > 1:
        parser.error("port must be nonempty, gap-factor > 1, read-timeout <= 1 second")
    if args.chunk_size > 1048576 or not 512 <= args.max_line_bytes <= 1048576:
        parser.error("chunk-size <= 1048576 and 512 <= max-line-bytes <= 1048576 required")
    try:
        return capture(args)
    except OSError as error:
        print("Unable to create/save capture files: %s" % error, file=sys.stderr)
        return 5


if __name__ == "__main__":
    raise SystemExit(main())
