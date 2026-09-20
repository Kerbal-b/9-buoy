from __future__ import annotations

import json
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest

from station.qt_app import ControlStationBackend
from station.test_session import PersistentTestSession


class PersistentTestSessionTests(unittest.TestCase):
    def test_session_flushes_metadata_sample_and_end_records(self) -> None:
        with TemporaryDirectory() as temp_dir:
            session = PersistentTestSession(
                Path(temp_dir),
                "water",
                {"controller_name": "Xbox", "notes": "test"},
            )
            session.write_sample(
                {
                    "imu": {"sequence": 42, "accel_g": {"x": 0.1, "y": 0.2, "z": 1.0}},
                    "motors": {"rear": {"applied_pwm": 171}},
                },
                timestamp=session.started_at + 0.05,
            )

            # The active file is flushed after each line, so it is readable
            # even if a test is still running.
            active_records = [json.loads(line) for line in session.path.read_text(encoding="utf-8").splitlines()]
            self.assertEqual([record["record_type"] for record in active_records], ["metadata", "sample"])
            self.assertEqual(active_records[1]["imu"]["sequence"], 42)
            self.assertEqual(active_records[1]["motors"]["rear"]["applied_pwm"], 171)

            session.close()
            records = [json.loads(line) for line in session.path.read_text(encoding="utf-8").splitlines()]
            self.assertEqual([record["record_type"] for record in records], ["metadata", "sample", "end"])
            self.assertEqual(records[-1]["sample_count"], 1)

    def test_environment_is_safe_for_filename(self) -> None:
        with TemporaryDirectory() as temp_dir:
            session = PersistentTestSession(Path(temp_dir), "Bath Tub / Water", {})
            try:
                self.assertTrue(session.path.name.startswith("bath-tub-water-"))
            finally:
                session.close()

    def test_backend_records_synchronized_water_calibration_sample(self) -> None:
        with TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            backend = ControlStationBackend(
                SimpleNamespace(wifi_local_ip="auto", send_rate=10.0, deadzone=0.12),
                root / "comm.log",
                root / "prefs.json",
            )
            backend._transport = object()
            backend._set_state(
                navigationTestEnvironment="water",
                rearMotor=30,
                rearMotorActualDrive=77,
                rearMotorPwm=141,
            )
            backend.startTestSession()
            session_path = Path(str(backend._state["testSessionFile"]))
            backend._test_imu_quality.observe(7, 123456)
            backend._record_test_sample(7, 123456, 0.1, -0.2, 1.02, 1.0, 2.0, 3.0, 35.5)
            backend.stopTestSession()

            records = [json.loads(line) for line in session_path.read_text(encoding="utf-8").splitlines()]
            self.assertEqual(records[0]["test_kind"], "water_imu_calibration")
            self.assertEqual(records[1]["imu"]["sequence"], 7)
            self.assertEqual(records[1]["imu"]["firmware_timestamp_ms"], 123456)
            self.assertEqual(records[1]["motors"]["rear"]["actual_drive"], 77)
            self.assertEqual(records[1]["motors"]["rear"]["applied_pwm"], 141)
            self.assertEqual(records[-1]["sample_count"], 1)
            self.assertEqual(records[-1]["summary"]["imu_stream"]["received"], 1)


if __name__ == "__main__":
    unittest.main()
