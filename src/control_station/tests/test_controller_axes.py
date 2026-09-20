from __future__ import annotations

import unittest

from station.controller import _map_controller_motion


class ControllerAxisMappingTests(unittest.TestCase):
    def test_left_stick_controls_translation_and_right_x_controls_yaw(self) -> None:
        self.assertEqual(
            _map_controller_motion(0.5, -0.75, -0.4, 0.12),
            (0.5, 0.75, -0.4),
        )

    def test_deadzone_applies_to_all_three_motion_axes(self) -> None:
        self.assertEqual(
            _map_controller_motion(0.05, -0.08, 0.1, 0.12),
            (0.0, 0.0, 0.0),
        )


if __name__ == "__main__":
    unittest.main()
