"""Tests for the standalone Backpack diagnostic's wire framing."""

import struct
import unittest

from diagnose_backpack import gcs_heartbeat, mavlink_frames
from station.mavlink_telemetry import _crc_x25, decode_datagram


class BackpackDiagnosticTests(unittest.TestCase):
    def test_gcs_probe_is_valid_non_command_heartbeat(self):
        packet = gcs_heartbeat()
        frame = mavlink_frames(packet)[0]
        self.assertEqual(frame["message_id"], 0)
        self.assertEqual(frame["system_id"], 255)
        self.assertTrue(frame["crc_valid"])
        self.assertEqual(decode_datagram(packet)[0]["kind"], "heartbeat")

    def test_unknown_frame_is_logged_without_claiming_crc_validation(self):
        payload = b"\x01\x02"
        header = bytes((0xFE, len(payload), 4, 42, 1, 200))
        frame = header + payload + struct.pack("<H", _crc_x25(header[1:] + payload, 0))
        frames = mavlink_frames(b"junk" + frame)
        self.assertEqual(frames[0]["offset"], 4)
        self.assertEqual(frames[0]["message_id"], 200)
        self.assertIsNone(frames[0]["crc_valid"])
        self.assertEqual(frames[0]["payload_hex"], "0102")

    def test_bad_known_crc_is_reported(self):
        packet = bytearray(gcs_heartbeat())
        packet[-1] ^= 0xFF
        self.assertFalse(mavlink_frames(packet)[0]["crc_valid"])
        self.assertEqual(decode_datagram(packet), [])

    def test_radio_status_keeps_rssi_in_device_units(self):
        payload = struct.pack("<HHBBBBB", 2, 3, 91, 255, 75, 12, 255)
        header = bytes((0xFE, len(payload), 9, 1, 1, 109))
        packet = header + payload + struct.pack("<H", _crc_x25(header[1:] + payload, 185))
        frame = mavlink_frames(packet)[0]
        self.assertTrue(frame["crc_valid"])
        self.assertEqual(frame["radio_status"]["rssi_raw"], 91)
        self.assertIsNone(frame["radio_status"]["remote_rssi_raw"])
        self.assertNotIn("lq", frame["radio_status"])


if __name__ == "__main__":
    unittest.main()
