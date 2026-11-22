import contextlib
import csv
import io
import json
import pathlib
import tempfile
import unittest
from unittest import mock

import serial
from test_serial_capture import serial_capture, ROW, HEADER


class FakeSerial:
    def __init__(self, chunks, open_error=None):
        self.chunks = list(chunks)
        self.open_error = open_error
        self.closed = False
        self.opened = False
        self._timeout = 0.2
        self.dtr = self.rts = True

    def open(self):
        if self.dtr or self.rts:
            raise AssertionError("DTR and RTS must be false before opening")
        if self.open_error:
            raise self.open_error
        self.opened = True

    @property
    def timeout(self):
        return self._timeout

    @timeout.setter
    def timeout(self, value):
        if self.opened:
            raise AssertionError("Do not reconfigure an open UART during capture")
        self._timeout = value

    @property
    def in_waiting(self):
        return len(self.chunks[0]) if self.chunks and isinstance(self.chunks[0], bytes) else 0

    def read(self, size):
        value = self.chunks.pop(0) if self.chunks else KeyboardInterrupt()
        if isinstance(value, BaseException):
            raise value
        return value

    def close(self):
        self.closed = True

    def write(self, value):
        raise AssertionError("Receiver must never transmit")


class ReceiverIOTests(unittest.TestCase):
    def capture_devices(self, devices):
        with tempfile.TemporaryDirectory() as root:
            args = serial_capture.build_parser().parse_args([
                "--output", root, "--port", "TEST_ONLY", "--duration", "2",
                "--reconnect-delay", "0.001",
            ])
            with mock.patch.object(serial, "Serial", side_effect=devices), \
                    mock.patch.object(serial_capture, "find_port_description", return_value="fake"), \
                    contextlib.redirect_stdout(io.StringIO()):
                result = serial_capture.capture(args)
            directory, = pathlib.Path(root).iterdir()
            with (directory / "telemetry.csv").open(encoding="utf-8-sig", newline="") as handle:
                rows = list(csv.DictReader(handle))
            with (directory / "events.csv").open(encoding="utf-8-sig", newline="") as handle:
                events = list(csv.DictReader(handle))
            metadata = json.loads((directory / "session.json").read_text(encoding="utf-8"))
            return result, rows, events, metadata, (directory / "raw.bin").read_bytes()

    def test_every_chunk_boundary_and_multiple_lines(self):
        data = (ROW + "\r\n" + ROW + "\n").encode("ascii")
        for split in range(len(data) + 1):
            framer = serial_capture.LineFramer(4096)
            lines = list(framer.feed(data[:split])) + list(framer.feed(b"")) + list(framer.feed(data[split:]))
            self.assertEqual([line for kind, line in lines if kind == "line"],
                             [(ROW + "\r\n").encode(), (ROW + "\n").encode()])
            self.assertEqual(framer.buffer, b"")

    def test_oversize_noise_resynchronizes_at_newline(self):
        framer = serial_capture.LineFramer(100)
        output = list(framer.feed(b"x" * 101))
        self.assertEqual(len(framer.buffer), 0)
        output += list(framer.feed(b"x" * 10000))
        output += list(framer.feed(b"tail\nokay\n"))
        self.assertEqual(output, [("line_oversize", b""), ("line", b"okay\n")])

    def test_invalid_sensor_values_leave_derived_cells_blank(self):
        for name, value in [("current_valid", 0), ("current_calc_valid", 0), ("current_overrange", 1)]:
            _, parsed = serial_capture.TelemetryParser().process_line(ROW.encode())
            parsed["protocol"][name] = value
            row = serial_capture.enrich_row(parsed, 1, "test", 0, 1)
            self.assertEqual(row["current_a"], "")
            self.assertEqual(row["current_filtered_ma"], 39)
        parsed["protocol"]["voltage_valid"] = 0
        self.assertEqual(serial_capture.enrich_row(parsed, 1, "test", 0, 1)["voltage_v"], "")

    def test_ranges_and_enums_are_checked(self):
        for index, bad in [(0, "4294967296"), (1, "NEWSTATE"), (3, "1001"),
                           (4, "2"), (6, "8401"), (10, "4096"), (19, "8"), (23, "-1")]:
            values = ROW.split(",")
            values[index] = bad
            self.assertEqual(serial_capture.TelemetryParser().process_line(",".join(values).encode())[0], "invalid_row")

    def test_auto_full_duty_and_completion_are_accepted(self):
        values = ROW.split(",")
        values[1] = "AUTO"
        values[3] = "1000"
        values[6] = "8400"
        kind, parsed = serial_capture.TelemetryParser().process_line(
            ",".join(values).encode())
        self.assertEqual(kind, "row")
        self.assertEqual(parsed["protocol"]["ccr1"], 8400)
        self.assertEqual(serial_capture.enrich_row(parsed, 1, "test", 0, 1)["duty_percent"], "100.0")
        values[1], values[2], values[3], values[6] = "RELEASE", "AUTO_DONE", "0", "0"
        kind, parsed = serial_capture.TelemetryParser().process_line(
            ",".join(values).encode())
        self.assertEqual(kind, "row")
        self.assertEqual(parsed["protocol"]["reason"], "AUTO_DONE")
        # Old captures with the original fixed-duty state remain readable.
        values[1], values[2] = "FIXED", "BOOT"
        self.assertEqual(serial_capture.TelemetryParser().process_line(
            ",".join(values).encode())[0], "row")

    def test_timestamp_gap_and_boot_banner(self):
        parser = serial_capture.TelemetryParser()
        parser.process_line(ROW.encode())
        _, parsed = parser.process_line(ROW.replace("34701", "35101").encode())
        self.assertEqual(parsed["timing_gap"], 1)
        parser.process_line(b"MOTOR1_BENCH,EN=HIGH")
        _, parsed = parser.process_line(ROW.replace("34701", "51").encode())
        self.assertEqual(parsed["board_event"], "boot_banner")
        self.assertEqual(parsed["board_delta"], "")

    def test_reconnect_keeps_raw_without_joining_partial_lines(self):
        first = FakeSerial([(ROW + "\n").encode(), b"partial_old", serial.SerialException("disconnected")])
        second_data = (HEADER + "\n" + ROW + "\n").encode()
        second = FakeSerial([second_data, b"partial_final", KeyboardInterrupt()])
        result, rows, events, metadata, raw = self.capture_devices([first, second])
        self.assertEqual(result, 0)
        self.assertEqual([r["connection_id"] for r in rows], ["1", "2"])
        self.assertEqual([r["pc_sequence"] for r in rows], ["1", "2"])
        self.assertEqual(raw, (ROW + "\n").encode() + b"partial_old" + second_data + b"partial_final")
        self.assertEqual(metadata["stats"]["rows_with_assumed_schema"], 1)
        self.assertEqual(metadata["stop_reason"], "keyboard_interrupt")
        self.assertTrue(metadata["schema_matched"])
        self.assertTrue(first.closed and second.closed)
        self.assertTrue({"disconnect", "partial_line_at_disconnect", "partial_line_at_stop", "stop"}
                        <= {e["event"] for e in events})

    def test_wrong_schema_is_nonzero_and_raw_is_preserved(self):
        data = (ROW + "\n" + "time_ms,state,changed\n" + ROW + "\n").encode()
        result, rows, _, metadata, raw = self.capture_devices([FakeSerial([data])])
        self.assertEqual(result, 4)
        self.assertEqual(len(rows), 1)
        self.assertEqual(raw, data)
        self.assertEqual(metadata["status"], "schema_mismatch")
        self.assertEqual(metadata["stats"]["invalid_rows"], 1)

    def test_open_failure_retries_and_closes_failed_handle(self):
        failed = FakeSerial([], serial.SerialException("port busy"))
        success = FakeSerial([(ROW + "\n").encode()])
        result, rows, _, metadata, _ = self.capture_devices([failed, success])
        self.assertEqual(result, 0)
        self.assertTrue(failed.closed and success.closed)
        self.assertEqual(metadata["stats"]["connect_errors"], 1)
        self.assertEqual(len(rows), 1)

    def test_empty_connection_returns_no_data(self):
        result, rows, _, metadata, _ = self.capture_devices([FakeSerial([b""])])
        self.assertEqual(result, 2)
        self.assertFalse(rows)
        self.assertEqual(metadata["status"], "no_valid_data")

    def test_storage_failure_does_not_trigger_reconnect(self):
        device = FakeSerial([(ROW + "\n").encode()])
        original = serial_capture.csv.DictWriter
        def failing_writer(*args, **kwargs):
            writer = original(*args, **kwargs)
            write = writer.writerow
            def fail_on_data(row):
                if row.get("pc_sequence") != "pc_sequence":
                    raise OSError("simulated disk full")
                return write(row)
            writer.writerow = fail_on_data
            return writer
        with mock.patch.object(serial_capture.csv, "DictWriter", side_effect=failing_writer), \
                contextlib.redirect_stderr(io.StringIO()):
            result, rows, events, metadata, _ = self.capture_devices([device])
        self.assertEqual(result, 5)
        self.assertEqual(metadata["stats"]["valid_rows"], 0)
        self.assertEqual(metadata["stats"]["rows_with_assumed_schema"], 0)
        self.assertFalse(rows)
        self.assertEqual([e["detail"] for e in events if e["event"] == "stop"], ["capture_error"])
        self.assertEqual(metadata["stats"]["disconnects"], 0)
        self.assertEqual(metadata["status"], "failed")
        self.assertTrue(device.closed)

    def test_invalid_cli_values_fail_before_accessing_serial(self):
        for option, value in [("--duration", "-1"), ("--duration", "nan"),
                              ("--chunk-size", "0"), ("--read-timeout", "2")]:
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as raised:
                serial_capture.main([option, value])
            self.assertEqual(raised.exception.code, 2)


if __name__ == "__main__":
    unittest.main()
