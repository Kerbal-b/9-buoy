from __future__ import annotations

import unittest

from station.navigation import GpsSpeedTracker, NavigationAssist


class NavigationAssistTests(unittest.TestCase):
    def test_requires_zero_and_fresh_telemetry_before_enable(self) -> None:
        assist = NavigationAssist()
        self.assertFalse(assist.set_enabled(True, 1.0))

        assist.update_measurement(0.02, -0.01, 1.0)
        self.assertTrue(assist.zero())
        self.assertTrue(assist.set_enabled(True, 1.1))

    def test_corrects_opposite_a_rightward_acceleration_error(self) -> None:
        assist = NavigationAssist(gain=1.0, max_correction_deg=45.0, filter_alpha=1.0)
        assist.update_measurement(0.0, 0.0, 1.0)
        assist.zero()
        assist.set_enabled(True, 1.0)
        assist.update_measurement(0.10, 0.10, 1.1)

        solution = assist.solve(0.0, 1.0, 1.1)

        self.assertTrue(solution.active)
        self.assertLess(solution.direction_error_deg, 0.0)
        self.assertLess(solution.corrected_x, 0.0)
        self.assertAlmostEqual(
            (solution.corrected_x**2 + solution.corrected_y**2) ** 0.5,
            1.0,
            places=6,
        )

    def test_correction_is_bounded(self) -> None:
        assist = NavigationAssist(gain=1.0, max_correction_deg=10.0, filter_alpha=1.0)
        assist.update_measurement(0.0, 0.0, 1.0)
        assist.zero()
        assist.set_enabled(True, 1.0)
        assist.update_measurement(0.20, 0.0, 1.1)

        solution = assist.solve(0.0, 1.0, 1.1)

        # The first filtered correction moves 25% toward the bounded target.
        self.assertAlmostEqual(solution.correction_deg, -2.5)

    def test_stale_telemetry_disables_assist(self) -> None:
        assist = NavigationAssist(telemetry_timeout_s=0.25)
        assist.update_measurement(0.0, 0.0, 1.0)
        assist.zero()
        assist.set_enabled(True, 1.0)

        solution = assist.solve(0.0, 1.0, 1.5)

        self.assertFalse(assist.enabled)
        self.assertFalse(solution.active)
        self.assertIn("stale", solution.status.lower())

    def test_reports_tilt_from_body_acceleration(self) -> None:
        assist = NavigationAssist()
        assist.update_measurement(0.0, 0.0, 1.0, accel_z_g=1.0)
        self.assertAlmostEqual(assist.roll_deg, 0.0)
        self.assertAlmostEqual(assist.pitch_deg, 0.0)

        assist.update_measurement(0.25, 0.0, 1.1, accel_z_g=0.968)
        self.assertGreater(assist.roll_deg, 10.0)

    def test_acceleration_display_responds_before_explicit_zero(self) -> None:
        assist = NavigationAssist(filter_alpha=1.0, gravity_time_constant_s=1e9)
        assist.update_measurement(0.0, 0.0, 1.0, accel_z_g=1.0)
        assist.update_measurement(0.08, 0.0, 1.1, accel_z_g=1.0)

        self.assertFalse(assist.zeroed)
        self.assertGreater(assist.measured_x_g, 0.05)
        self.assertEqual(assist.estimated_speed_mps, 0.0)

    def test_gyro_confirmed_tilt_is_rejected_from_linear_acceleration(self) -> None:
        uncompensated = NavigationAssist(filter_alpha=1.0, gravity_time_constant_s=1e9)
        compensated = NavigationAssist(filter_alpha=1.0, gravity_time_constant_s=1e9)
        for assist in (uncompensated, compensated):
            assist.update_measurement(0.0, 0.0, 1.0, accel_z_g=1.0)
            assist.zero()

        uncompensated.update_measurement(0.25, 0.0, 1.1, accel_z_g=0.968)
        compensated.update_measurement(
            0.25,
            0.0,
            1.1,
            accel_z_g=0.968,
            gyro_y_dps=30.0,
        )

        self.assertTrue(compensated.tilt_compensating)
        self.assertLess(compensated.measured_magnitude_g, uncompensated.measured_magnitude_g * 0.5)

    def test_short_term_speed_integrates_linear_acceleration(self) -> None:
        assist = NavigationAssist(filter_alpha=1.0, gravity_time_constant_s=1e9)
        assist.update_measurement(0.0, 0.0, 1.0, accel_z_g=1.0)
        assist.zero()
        assist.update_measurement(0.10, 0.0, 1.1, accel_z_g=1.0)

        self.assertGreater(assist.estimated_speed_mps, 0.08)
        self.assertGreater(assist.velocity_confidence_percent(1.1), 0)

    def test_clockwise_mount_maps_sensor_axes_into_buoy_axes(self) -> None:
        assist = NavigationAssist(filter_alpha=1.0, gravity_time_constant_s=1e9)
        assist.set_axis_signs(True, False, True)
        assist.update_measurement(0.0, 0.0, 1.0)
        assist.zero()

        # Sensor +X is buoy-forward; sensor -Y is buoy-right.
        assist.update_measurement(0.10, -0.05, 1.1)

        self.assertAlmostEqual(assist.measured_x_g, 0.05)
        self.assertAlmostEqual(assist.measured_y_g, 0.10)


class GpsSpeedTrackerTests(unittest.TestCase):
    def test_speed_is_unavailable_until_two_fixes_arrive(self) -> None:
        tracker = GpsSpeedTracker()
        tracker.update(37.0, -122.0, 1.0)
        self.assertIsNone(tracker.speed_mps(1.0))

        tracker.update(37.000027, -122.0, 3.0)
        self.assertIsNotNone(tracker.speed_mps(3.0))

    def test_missing_or_stale_gps_falls_back_cleanly(self) -> None:
        tracker = GpsSpeedTracker(stale_after_s=1.0)
        tracker.update(37.0, -122.0, 1.0)
        tracker.update(37.000027, -122.0, 3.0)
        self.assertIsNotNone(tracker.speed_mps(3.0))
        self.assertIsNone(tracker.speed_mps(4.1))

        tracker.invalidate()
        self.assertIsNone(tracker.speed_mps(3.1))


if __name__ == "__main__":
    unittest.main()
