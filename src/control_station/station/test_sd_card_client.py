from __future__ import annotations

import unittest
from unittest.mock import patch

from . import sd_card_client


class ListSdLogFilesTests(unittest.TestCase):
    def test_recursively_collects_files_under_logs(self) -> None:
        listing = {
            "/logs": [
                {"path": "/logs/telemetry.csv", "name": "telemetry.csv", "size": 4, "isDirectory": False},
                {"path": "/logs/archive", "name": "archive", "size": 0, "isDirectory": True},
            ],
            "/logs/archive": [
                {"path": "/logs/archive/system.csv", "name": "system.csv", "size": 3, "isDirectory": False},
            ],
        }
        with patch.object(
            sd_card_client, "list_sd_directory", side_effect=lambda _host, path, _ip: {"files": listing[path]}
        ):
            files = sd_card_client.list_sd_log_files("buoy")

        self.assertEqual([entry["path"] for entry in files], ["/logs/telemetry.csv", "/logs/archive/system.csv"])

    def test_rejects_listing_paths_outside_logs_root(self) -> None:
        listing = {"files": [{"path": "/missions/current.csv", "name": "current.csv", "size": 0, "isDirectory": False}]}
        with patch.object(sd_card_client, "list_sd_directory", return_value=listing):
            with self.assertRaisesRegex(RuntimeError, "escaped /logs"):
                sd_card_client.list_sd_log_files("buoy")


if __name__ == "__main__":
    unittest.main()
