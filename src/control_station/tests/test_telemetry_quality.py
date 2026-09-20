from __future__ import annotations

import unittest

from station.telemetry_quality import ImuStreamQuality


class ImuStreamQualityTests(unittest.TestCase):
    def test_regular_20_hz_stream_has_no_loss(self) -> None:
        quality = ImuStreamQuality()
        self.assertTrue(quality.observe(1, 0))
        self.assertTrue(quality.observe(7, 50))
        self.assertTrue(quality.observe(12, 100))
        self.assertEqual(quality.received_count, 3)
        self.assertEqual(quality.missing_count, 0)
        self.assertEqual(quality.loss_percent, 0.0)
        self.assertAlmostEqual(quality.effective_hz, 20.0)

    def test_firmware_timestamp_gap_counts_missing_imu_periods(self) -> None:
        quality = ImuStreamQuality()
        quality.observe(1, 0)
        quality.observe(20, 50)
        quality.observe(40, 200)
        self.assertEqual(quality.received_count, 3)
        self.assertEqual(quality.missing_count, 2)
        self.assertEqual(quality.expected_count, 5)
        self.assertAlmostEqual(quality.loss_percent, 40.0)

    def test_shared_global_sequence_jumps_do_not_look_like_imu_loss(self) -> None:
        quality = ImuStreamQuality()
        quality.observe(1, 1000)
        quality.observe(30, 1050)
        self.assertEqual(quality.missing_count, 0)

    def test_duplicate_and_old_packet_are_rejected(self) -> None:
        quality = ImuStreamQuality()
        quality.observe(1, 10000)
        self.assertFalse(quality.observe(1, 10000))
        self.assertFalse(quality.observe(2, 9950))
        self.assertEqual(quality.duplicate_count, 1)
        self.assertEqual(quality.out_of_order_count, 1)

    def test_timestamp_wrap_and_firmware_restart_are_supported(self) -> None:
        wrap = ImuStreamQuality()
        wrap.observe(1, 0xFFFFFFF0)
        self.assertTrue(wrap.observe(2, 34))
        self.assertEqual(wrap.last_interval_ms, 50)

        restart = ImuStreamQuality()
        restart.observe(1, 10000)
        self.assertTrue(restart.observe(2, 100))
        self.assertEqual(restart.restart_count, 1)
        self.assertEqual(restart.missing_count, 0)


if __name__ == "__main__":
    unittest.main()
