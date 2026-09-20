from __future__ import annotations

import unittest

from station.qt_app import _calibrated_motor_pwm, _motor_percent_command, _parse_motor_output_telemetry


class MotorPowerMappingTests(unittest.TestCase):
    def test_percent_is_sent_for_firmware_side_calibration(self) -> None:
        self.assertEqual(_motor_percent_command("rear", 0), "CTRL MOTOR PERCENT rear +0")
        self.assertEqual(_motor_percent_command("front-left", 30), "CTRL MOTOR PERCENT front_left +30")
        self.assertEqual(_motor_percent_command("front_right", -30), "CTRL MOTOR PERCENT front_right -30")

    def test_percent_is_clamped_before_transmission(self) -> None:
        self.assertEqual(_motor_percent_command("rear", 150), "CTRL MOTOR PERCENT rear +100")
        self.assertEqual(_motor_percent_command("rear", -150), "CTRL MOTOR PERCENT rear -100")

    def test_pwm_uses_configured_minimum_maximum_and_curve(self) -> None:
        self.assertEqual(_calibrated_motor_pwm(0, 130, 255, 2.0), 0)
        self.assertEqual(_calibrated_motor_pwm(30, 130, 255, 2.0), 141)
        self.assertEqual(_calibrated_motor_pwm(-30, 130, 255, 2.0), 141)
        self.assertEqual(_calibrated_motor_pwm(100, 130, 255, 2.0), 255)

    def test_actual_motor_output_telemetry_is_parsed(self) -> None:
        self.assertEqual(
            _parse_motor_output_telemetry("TEL MOTOR OUT 40 150 -100 255 100 255"),
            (40, 150, -100, 255, 100, 255),
        )

    def test_invalid_motor_output_telemetry_is_ignored(self) -> None:
        self.assertIsNone(_parse_motor_output_telemetry("TEL MOTOR OUT 40 nope"))
        self.assertIsNone(_parse_motor_output_telemetry("TEL STATUS MODE MANUAL"))


if __name__ == "__main__":
    unittest.main()
