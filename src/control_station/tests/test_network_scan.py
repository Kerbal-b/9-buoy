from __future__ import annotations

import ipaddress
import time
import unittest
from pathlib import Path
from unittest.mock import patch

from station.qt_app import ControlStationBackend, parse_args


class NetworkScanTests(unittest.TestCase):
    def test_pending_control_target_is_never_probed(self) -> None:
        target = "192.168.8.140"
        attempted: list[str] = []

        def reject_probe(address: tuple[str, int], **_kwargs: object) -> None:
            attempted.append(address[0])
            raise OSError("unreachable")

        with patch("sys.argv", ["test"]):
            args = parse_args()
        args.wifi_local_ip = "192.168.8.141"
        args.tcp_port = 5000
        with (
            patch("station.qt_app.list_wifi_interfaces", return_value=[]),
            patch("station.qt_app.BUOY_ROUTER_NETWORK", ipaddress.IPv4Network("192.168.8.136/29")),
            patch("station.qt_app.socket.create_connection", side_effect=reject_probe),
        ):
            backend = ControlStationBackend(args, Path("commands.log"), Path("prefs.json"))
            backend._state["serialTarget"] = f"WIFI:{target}:5000/udp:5001/audio:5002"
            # Reproduce the window after TCP accepts the control client but
            # before the main thread installs the pending transport.
            backend._connection_in_flight = False
            backend._transport = None
            backend.scanBuoyNetwork()
            deadline = time.monotonic() + 2.0
            while backend._network_scan_pending is None and time.monotonic() < deadline:
                time.sleep(0.01)

        self.assertIsNotNone(backend._network_scan_pending)
        self.assertNotIn(target, attempted)


if __name__ == "__main__":
    unittest.main()
