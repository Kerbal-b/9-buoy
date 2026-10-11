import unittest

from station.protocol import format_firmware_version, parse_status_update


class FirmwareVersionTests(unittest.TestCase):
    def test_numeric_version_status(self):
        status = parse_status_update("TEL STATUS FIRMWARE 1 2 0 20261010")
        self.assertEqual(status, ("FIRMWARE", ("1", "2", "0", "20261010")))
        self.assertEqual(format_firmware_version(status[1]), "v1.2.0 (2026-10-10)")

    def test_legacy_banner_remains_visible(self):
        self.assertEqual(
            format_firmware_version(("ESP32_FW_2026_10_10_ELRS2",)),
            "ESP32_FW_2026_10_10_ELRS2",
        )


if __name__ == "__main__":
    unittest.main()
