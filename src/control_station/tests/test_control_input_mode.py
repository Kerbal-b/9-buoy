from __future__ import annotations

import unittest

from station.geometry import build_manual_command
from station.qt_app import _limit_motion_to_motor_output, _select_manual_input


class ControlInputModeTests(unittest.TestCase):
    def test_controller_mode_ignores_keyboard_input(self) -> None:
        self.assertEqual(
            _select_manual_input("controller", 0.4, -0.3, -1.0, 1.0, 1.0, -0.6),
            (0.4, -0.3, -0.6),
        )

    def test_full_translation_is_limited_by_shared_mixer_output(self) -> None:
        limited = _limit_motion_to_motor_output(0.0, 1.0, 0.0)
        command = build_manual_command(*limited)

        self.assertEqual(command.rear_motor, 50)
        self.assertLessEqual(max(abs(command.front_left_motor), abs(command.front_right_motor)), 50)

    def test_combined_motion_cannot_saturate_any_motor(self) -> None:
        limited = _limit_motion_to_motor_output(1.0, 1.0, 1.0)
        command = build_manual_command(*limited)

        self.assertLessEqual(
            max(abs(command.rear_motor), abs(command.front_left_motor), abs(command.front_right_motor)),
            50,
        )

    def test_adjustable_mixer_limit_preserves_translation_and_yaw_ratio(self) -> None:
        limited = _limit_motion_to_motor_output(0.6, 0.8, 0.4, maximum=0.35)
        command = build_manual_command(*limited)

        scale = limited[0] / 0.6
        self.assertAlmostEqual(limited[1], 0.8 * scale)
        self.assertAlmostEqual(limited[2], 0.4 * scale)
        self.assertLessEqual(
            max(abs(command.rear_motor), abs(command.front_left_motor), abs(command.front_right_motor)),
            35,
        )

    def test_keyboard_mode_ignores_controller_input(self) -> None:
        self.assertEqual(
            _select_manual_input("keyboard", 1.0, -1.0, -0.35, 0.35, 0.35),
            (-0.35, 0.35, 0.35),
        )

    def test_auto_mode_is_neutral_until_programs_are_implemented(self) -> None:
        self.assertEqual(
            _select_manual_input("auto", 1.0, 1.0, 1.0, 1.0, 1.0),
            (0.0, 0.0, 0.0),
        )


if __name__ == "__main__":
    unittest.main()
