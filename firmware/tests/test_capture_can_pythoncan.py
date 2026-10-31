import pathlib
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "tools"))
import capture_can_pythoncan as CAPTURE

class Message:
    def __init__(self, arbitration_id=0x123, is_extended_id=False, is_remote_frame=False, dlc=2, data=b"\x12\x34"):
        self.arbitration_id = arbitration_id
        self.is_extended_id = is_extended_id
        self.is_remote_frame = is_remote_frame
        self.dlc = dlc
        self.data = data

class CaptureFormatTests(unittest.TestCase):
    def test_standard_data_frame(self):
        self.assertEqual(CAPTURE.frame_to_row(Message(), 42), ["42", "STD", "0x123", "2", "0", "1234"])

    def test_extended_identifier(self):
        msg = Message(arbitration_id=0x1ABCDEFF, is_extended_id=True, dlc=1, data=b"\xab")
        self.assertEqual(CAPTURE.frame_to_row(msg, 7), ["7", "EXT", "0x1ABCDEFF", "1", "0", "AB"])

    def test_remote_frame_has_no_payload(self):
        msg = Message(is_remote_frame=True, dlc=8, data=b"")
        self.assertEqual(CAPTURE.frame_to_row(msg, 0), ["0", "STD", "0x123", "8", "1", ""])

    def test_timestamp_wraps_uint32(self):
        self.assertEqual(CAPTURE.frame_to_row(Message(), 0x100000001)[0], "1")

    def test_reject_invalid_identifier_dlc_and_payload(self):
        with self.assertRaises(ValueError):
            CAPTURE.frame_to_row(Message(arbitration_id=0x800), 0)
        with self.assertRaises(ValueError):
            CAPTURE.frame_to_row(Message(dlc=9), 0)
        with self.assertRaises(ValueError):
            CAPTURE.frame_to_row(Message(dlc=1, data=b"\x01\x02"), 0)
        with self.assertRaises(ValueError):
            CAPTURE.frame_to_row(Message(is_remote_frame=True, dlc=1, data=b"\x01"), 0)

    def test_cli_rejects_non_finite_duration_before_opening_backend(self):
        for duration in ("nan", "inf", "-inf"):
            with self.subTest(duration=duration):
                argv = ["capture_can_pythoncan.py", "--interface", "test", "--channel", "test0",
                        "--bitrate", "500000", f"--duration={duration}", "--output", "capture.csv"]
                with patch("sys.argv", argv), self.assertRaises(SystemExit) as raised:
                    CAPTURE.main()
                self.assertEqual(raised.exception.code, 2)

if __name__ == "__main__":
    unittest.main()
