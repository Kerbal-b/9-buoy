from __future__ import annotations

import unittest

from station.geometry import LEGACY_RADIAL_MOTOR_MIX, build_manual_command


class MotionGeometryTests(unittest.TestCase):
    def test_translation_keeps_legacy_vector_protocol(self) -> None:
        command = build_manual_command(0.0, 0.5)

        self.assertEqual(command.yaw, 0)
        self.assertEqual(command.to_line(), "CTRL VECTOR +0 +50\n")

    def test_yaw_uses_three_axis_motion_protocol(self) -> None:
        command = build_manual_command(0.0, 0.0, 0.35)

        self.assertEqual(command.to_line(), "CTRL MOTION +0 +0 +35\n")
        self.assertEqual(command.rear_motor, 35)
        self.assertEqual(command.front_left_motor, -35)
        self.assertEqual(command.front_right_motor, 35)

    def test_full_right_translation_uses_horizontal_rear_motor(self) -> None:
        command = build_manual_command(1.0, 0.0)

        self.assertEqual(command.rear_motor, 100)
        self.assertEqual(command.front_left_motor, 67)
        self.assertEqual(command.front_right_motor, -67)

    def test_full_left_translation_reverses_right_translation_mix(self) -> None:
        command = build_manual_command(-1.0, 0.0)

        self.assertEqual(command.rear_motor, -100)
        self.assertEqual(command.front_left_motor, -67)
        self.assertEqual(command.front_right_motor, 67)

    def test_combined_translation_and_yaw_stays_bounded(self) -> None:
        command = build_manual_command(1.0, 1.0, 1.0)

        self.assertLessEqual(max(abs(command.rear_motor), abs(command.front_left_motor), abs(command.front_right_motor)), 100)

    def test_forward_drives_both_front_motors_forward(self) -> None:
        command = build_manual_command(0.0, 1.0)

        self.assertEqual(command.rear_motor, 0)
        self.assertEqual(command.front_left_motor, 100)
        self.assertEqual(command.front_right_motor, 100)

    def test_legacy_layout_remains_available(self) -> None:
        command = build_manual_command(0.0, 0.0, 0.35, LEGACY_RADIAL_MOTOR_MIX)

        self.assertEqual(command.rear_motor, 35)
        self.assertEqual(command.front_left_motor, 35)
        self.assertEqual(command.front_right_motor, 35)


if __name__ == "__main__":
    unittest.main()
