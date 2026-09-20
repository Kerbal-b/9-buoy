from __future__ import annotations

import unittest

from station.geometry import build_manual_command


class MotionGeometryTests(unittest.TestCase):
    def test_translation_keeps_legacy_vector_protocol(self) -> None:
        command = build_manual_command(0.0, 0.5)

        self.assertEqual(command.yaw, 0)
        self.assertEqual(command.to_line(), "CTRL VECTOR +0 +50\n")

    def test_yaw_uses_three_axis_motion_protocol(self) -> None:
        command = build_manual_command(0.0, 0.0, 0.35)

        self.assertEqual(command.to_line(), "CTRL MOTION +0 +0 +35\n")
        self.assertEqual(command.rear_motor, 35)
        self.assertEqual(command.front_left_motor, 35)
        self.assertEqual(command.front_right_motor, 35)

    def test_full_lateral_translation_reaches_full_front_motor_output(self) -> None:
        command = build_manual_command(1.0, 0.0)

        self.assertEqual(command.rear_motor, 0)
        self.assertEqual(command.front_left_motor, 100)
        self.assertEqual(command.front_right_motor, -100)

    def test_left_translation_only_reverses_the_two_angled_motors(self) -> None:
        command = build_manual_command(-1.0, 0.0)

        self.assertEqual(command.rear_motor, 0)
        self.assertEqual(command.front_left_motor, -100)
        self.assertEqual(command.front_right_motor, 100)

    def test_combined_translation_and_yaw_stays_bounded(self) -> None:
        command = build_manual_command(1.0, 1.0, 1.0)

        self.assertLessEqual(max(abs(command.rear_motor), abs(command.front_left_motor), abs(command.front_right_motor)), 100)


if __name__ == "__main__":
    unittest.main()
