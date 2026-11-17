import importlib.util
import pathlib
import unittest


MODULE_PATH = pathlib.Path(__file__).resolve().parents[1] / "Tools" / "serial_capture.py"
SPEC = importlib.util.spec_from_file_location("serial_capture", MODULE_PATH)
serial_capture = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(serial_capture)


HEADER = serial_capture.EXPECTED_HEADER
ROW = "34701,READY,BOOT,0,1,1,0,0,0,0,9,1,1160,1499,3307,937,937,921,1,0,0,10,10,29,39,1,0,0"


class SerialCaptureTests(unittest.TestCase):
    def test_valid_row_and_derived_values(self):
        parser = serial_capture.TelemetryParser()
        parser.start_connection()
        result, parsed = parser.process_line((ROW + "\r\n").encode("ascii"))
        self.assertEqual(result, "row")
        enriched = serial_capture.enrich_row(parsed, 1, "2026-11-16T21:00:00+08:00", 12, 1)
        self.assertEqual(enriched["pc_sequence"], 1)
        self.assertEqual(enriched["duty_percent"], "0.0")
        self.assertEqual(enriched["current_a"], "0.039")
        self.assertEqual(enriched["key1_pressed"], 0)
        self.assertEqual(enriched["voltage_v"], "0.921")

    def test_header_banner_and_partial_line_independent(self):
        parser = serial_capture.TelemetryParser()
        parser.start_connection()
        self.assertEqual(parser.process_line(b"MOTOR1_BENCH,EN=HIGH\r\n")[0], "banner")
        self.assertEqual(parser.process_line((HEADER + "\r\n").encode("ascii"))[0], "header")
        self.assertEqual(parser.process_line((ROW + "\r").encode("ascii"))[0], "row")

    def test_bad_field_count_is_rejected(self):
        parser = serial_capture.TelemetryParser()
        parser.start_connection()
        self.assertEqual(parser.process_line(b"1,READY,BOOT\r\n")[0], "invalid_row")

    def test_reset_and_uint32_wrap_are_distinguished(self):
        parser = serial_capture.TelemetryParser()
        parser.start_connection()
        first = ROW.replace("34701", "4294967290")
        second = ROW.replace("34701", "5")
        self.assertEqual(parser.process_line((first + "\n").encode("ascii"))[0], "row")
        result, parsed = parser.process_line((second + "\n").encode("ascii"))
        self.assertEqual(result, "row")
        self.assertEqual(parsed["board_event"], "uptime_wrap")
        self.assertEqual(parsed["board_delta"], 11)

        parser.start_connection()
        self.assertEqual(parser.process_line((ROW.replace("34701", "5000") + "\n").encode("ascii"))[0], "row")
        result, parsed = parser.process_line((ROW.replace("34701", "100") + "\n").encode("ascii"))
        self.assertEqual(result, "row")
        self.assertEqual(parsed["board_event"], "timestamp_reset")
        self.assertEqual(parsed["board_delta"], "")

    def test_incompatible_header_blocks_structured_rows(self):
        parser = serial_capture.TelemetryParser()
        parser.start_connection()
        result, _ = parser.process_line(b"time_ms,state,changed\n")
        self.assertEqual(result, "schema_mismatch")
        result, _ = parser.process_line((ROW + "\n").encode("ascii"))
        self.assertEqual(result, "schema_blocked")


if __name__ == "__main__":
    unittest.main()
