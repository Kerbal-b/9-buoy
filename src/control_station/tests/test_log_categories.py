from __future__ import annotations

import unittest

from station.log_categories import (
    NAVIGATION_LOG,
    SCIENTIFIC_LOG,
    SYSTEM_LOG,
    classify_log_entry,
    display_timestamp,
    interpret_log_entry,
)


class LogCategoryTests(unittest.TestCase):
    def test_scientific_telemetry_is_classified(self) -> None:
        self.assertEqual(classify_log_entry("RX TEL SCI IMU_TEMP 42.0"), SCIENTIFIC_LOG)
        self.assertEqual(classify_log_entry("RX TEL SCI WATER_TEMP 18.5"), SCIENTIFIC_LOG)

    def test_navigation_commands_and_state_are_classified(self) -> None:
        self.assertEqual(classify_log_entry("TX CTRL MOTION +0 +35 -35"), NAVIGATION_LOG)
        self.assertEqual(classify_log_entry("RX TEL STATUS POS 37.0 -122.0"), NAVIGATION_LOG)
        self.assertEqual(classify_log_entry("NAV ASSIST ON"), NAVIGATION_LOG)

    def test_system_and_motor_entries_are_classified(self) -> None:
        self.assertEqual(classify_log_entry("CONNECT success target=WIFI"), SYSTEM_LOG)
        self.assertEqual(classify_log_entry("RX TEL STATUS BATTERY 11.8 1.2 73"), SYSTEM_LOG)
        self.assertEqual(classify_log_entry("TX CTRL MOTOR rear +100"), SYSTEM_LOG)

    def test_display_timestamp_is_compact_with_milliseconds(self) -> None:
        rendered = display_timestamp(1_789_333_525.227)
        self.assertRegex(rendered, r"^\d{2}:\d{2}:\d{2}\.\d{3}$")

    def test_interprets_motion_and_motor_messages(self) -> None:
        self.assertEqual(
            interpret_log_entry("TX CTRL MOTION +10 -35 +20"),
            "Movement requested — lateral +10%, thrust -35%, yaw +20%",
        )
        self.assertEqual(
            interpret_log_entry("TX CTRL MOTOR front_left -128"),
            "Motor test — Front Left: reverse, 128/255 (50%)",
        )
        self.assertEqual(
            interpret_log_entry("TX CTRL MOTOR PERCENT front_left -30"),
            "Motor test â€” Front Left: reverse, 30%",
        )

    def test_interprets_power_and_science_telemetry(self) -> None:
        self.assertEqual(
            interpret_log_entry("RX TEL STATUS BATTERY 11.47 0.56 62"),
            "Battery — 11.47 V, 0.56 A, 62% remaining",
        )
        self.assertEqual(
            interpret_log_entry("RX TEL SCI IMU_ACCEL 0.012 -0.101 1.138"),
            "Acceleration — X 0.012 g, Y -0.101 g, Z 1.138 g",
        )

    def test_unknown_messages_remain_raw_only(self) -> None:
        self.assertEqual(interpret_log_entry("RX SOMETHING NEW"), "")


if __name__ == "__main__":
    unittest.main()
