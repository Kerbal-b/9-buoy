from __future__ import annotations

import unittest
import time
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from station.qt_app import (
    ControlStationBackend,
    _calibrated_motor_pwm,
    _motor_percent_command,
    _parse_motor_output_telemetry,
    parse_args,
)


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

    def test_motor_test_keepalive_repeats_until_stop(self) -> None:
        with patch("sys.argv", ["test"]):
            args = parse_args()
        with patch("station.qt_app.list_wifi_interfaces", return_value=[]):
            backend = ControlStationBackend(args, Path("commands.log"), Path("prefs.json"))
        backend._transport = SimpleNamespace(is_open=True)

        with patch("station.qt_app.send_text", return_value="sent") as send:
            backend.sendMotorTestPower("rear", 15)
            backend._send_motor_test_keepalive()
            self.assertEqual(send.call_count, 1)

            backend._last_motor_test_send_at = time.monotonic() - 0.25
            backend._send_motor_test_keepalive()
            self.assertEqual(send.call_count, 2)
            self.assertTrue(any("keepalive" in line for line in backend._command_trace_pending))

            backend.stopMotorTest("released")
            backend._send_motor_test_keepalive()
            self.assertEqual(send.call_count, 3)
            self.assertTrue(any("HOLD ENDED reason=released" in line for line in backend._command_trace_pending))


if __name__ == "__main__":
    unittest.main()
