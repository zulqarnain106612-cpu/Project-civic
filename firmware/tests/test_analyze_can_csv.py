import contextlib
import io
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
from analyze_can_csv import analyze, main, parse_row, render_report

class AnalyzeCanCsvTests(unittest.TestCase):
    def test_groups_messages_and_reports_observed_variation(self):
        rows = io.StringIO("10,STD,0x123,2,0,0102\n20,STD,0x123,2,0,0103\n35,EXT,0x18DAF110,8,0,0001020304050607\n")
        stats = analyze(rows)
        self.assertEqual(len(stats), 2)
        standard = next(item for item in stats if item.fmt == "STD")
        self.assertEqual(standard.count, 2)
        self.assertEqual(standard.byte_values[0], {1})
        self.assertEqual(standard.byte_values[1], {2, 3})
        self.assertIn("| STD | 0x123 | 2 | 2 | 10 | 1 | 0 |", render_report(stats))
        self.assertIn("remains a hypothesis", render_report(stats).lower())

    def test_rtr_dlc_does_not_require_payload(self):
        parsed = parse_row(["100", "STD", "0x7DF", "8", "1", ""], 1)
        self.assertEqual(parsed[-1], b"")

    def test_rejects_wrong_field_count_and_payload_length(self):
        with self.assertRaises(ValueError):
            parse_row(["1", "STD"], 1)
        with self.assertRaisesRegex(ValueError, "payload length"):
            parse_row(["1", "STD", "0x123", "2", "0", "AA"], 1)

    def test_rejects_out_of_range_standard_id(self):
        with self.assertRaisesRegex(ValueError, "identifier outside STD range"):
            parse_row(["1", "STD", "0x800", "0", "0", ""], 1)

    def test_timestamp_wrap_is_reported_modulo_uint32(self):
        rows = io.StringIO("4294967280,STD,0x100,0,0,\n10,STD,0x100,0,0,\n")
        stats = analyze(rows)
        self.assertIn("| STD | 0x100 | 2 | 0 | 26 | none observed | 0 |", render_report(stats))

    def test_rejects_invalid_rtr_payload_and_timestamp(self):
        with self.assertRaisesRegex(ValueError, "RTR frame must have empty payload"):
            parse_row(["1", "STD", "0x123", "1", "1", "AA"], 7)
        with self.assertRaisesRegex(ValueError, "timestamp_ms outside uint32 range"):
            parse_row(["4294967296", "STD", "0x123", "0", "0", ""], 8)

    def test_empty_capture_has_header_and_no_message_rows(self):
        report = render_report(analyze(io.StringIO("\n")))
        self.assertIn("# Project-civic CAN traffic summary", report)
        self.assertIn("| Format | ID | Frames |", report)
        self.assertNotIn("| STD |", report)

    def test_cli_writes_report_to_requested_path(self):
        with tempfile.TemporaryDirectory() as directory:
            input_path = Path(directory) / "capture.csv"
            output_path = Path(directory) / "nested" / "report.md"
            input_path.write_text("1,STD,0x123,1,0,AA\n", encoding="utf-8")
            with patch("sys.argv", ["analyze_can_csv.py", str(input_path), "--output", str(output_path)]):
                self.assertEqual(main(), 0)
            report = output_path.read_text(encoding="utf-8")
            self.assertIn("| STD | 0x123 | 1 | 1 | n/a | none observed | 0 |", report)

    def test_cli_reports_malformed_input_without_traceback(self):
        with tempfile.TemporaryDirectory() as directory:
            input_path = Path(directory) / "bad.csv"
            input_path.write_text("1,STD,0x123,2,0,AA\n", encoding="utf-8")
            stderr = io.StringIO()
            with patch("sys.argv", ["analyze_can_csv.py", str(input_path)]), contextlib.redirect_stderr(stderr):
                self.assertEqual(main(), 2)
            self.assertIn("payload length does not match DLC", stderr.getvalue())
            self.assertNotIn("Traceback", stderr.getvalue())

if __name__ == "__main__":
    unittest.main()
