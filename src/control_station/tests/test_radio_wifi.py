import gzip
import socket
import unittest
from unittest.mock import patch

from station.radio_wifi import _is_tx_backpack, _mavlink_status, discover_tx_backpack


class FakeSocket:
    def __init__(self, response):
        self.response = response
        self.bound = None

    def __enter__(self):
        return self

    def __exit__(self, *args):
        return False

    def settimeout(self, timeout):
        pass

    def bind(self, address):
        self.bound = address

    def connect(self, address):
        pass

    def sendall(self, request):
        pass

    def recv(self, size):
        response, self.response = self.response, b""
        return response


class KeepOpenSocket(FakeSocket):
    def recv(self, size):
        if not self.response:
            raise socket.timeout
        return super().recv(size)


class RadioWifiTests(unittest.TestCase):
    def test_http_probe_requires_tx_backpack_identity(self):
        good_page = b"HTTP/1.0 200 OK\r\n\r\nExpressLRS TX Backpack"
        other_page = b"HTTP/1.0 200 OK\r\n\r\nExpressLRS TX Module"
        with patch("station.radio_wifi.socket.socket", side_effect=lambda *args: FakeSocket(good_page)):
            self.assertTrue(_is_tx_backpack("192.168.1.20"))
        with patch("station.radio_wifi.socket.socket", side_effect=lambda *args: FakeSocket(other_page)):
            self.assertFalse(_is_tx_backpack("192.168.1.20"))

    def test_mdns_discovery(self):
        with patch("station.radio_wifi.socket.gethostbyname", return_value="192.168.1.20"), patch(
            "station.radio_wifi._is_tx_backpack", return_value=True
        ) as probe, patch(
            "station.radio_wifi._mavlink_status", return_value="MAVLink forwarding active"
        ):
            address, status = discover_tx_backpack(["192.168.1.10"], "192.168.1.10")
        self.assertEqual(address, "192.168.1.20")
        self.assertIn("elrs_txbp.local", status)
        self.assertIn("MAVLink forwarding active", status)
        probe.assert_called_once_with("192.168.1.20", "192.168.1.10")

    def test_http_probe_follows_index_redirect_and_gzip(self):
        redirect = b"HTTP/1.0 302 Found\r\nLocation: /index.html\r\n\r\n"
        page = b"HTTP/1.0 200 OK\r\nContent-Encoding: gzip\r\n\r\n" + gzip.compress(
            b"<h1>ExpressLRS</h1><span>TX Backpack</span>"
        )
        with patch("station.radio_wifi._http_get", side_effect=[redirect, page]) as get:
            self.assertTrue(_is_tx_backpack("192.168.1.20"))
        self.assertEqual([call.args[1] for call in get.call_args_list], ["/", "/index.html"])

    def test_http_probe_keeps_complete_response_when_server_stays_open(self):
        page = b"HTTP/1.0 200 OK\r\n\r\nExpressLRS TX Backpack"
        with patch("station.radio_wifi.socket.socket", side_effect=lambda *args: KeepOpenSocket(page)):
            self.assertTrue(_is_tx_backpack("192.168.8.134"))

    def test_subnet_fallback(self):
        with patch("station.radio_wifi.socket.gethostbyname", side_effect=OSError), patch(
            "station.radio_wifi._is_tx_backpack", side_effect=lambda host, source: host == "192.168.1.42"
        ), patch(
            "station.radio_wifi._mavlink_status", return_value="MAVLink forwarding disabled"
        ):
            address, status = discover_tx_backpack(["192.168.1.10"], "192.168.1.10")
        self.assertEqual(address, "192.168.1.42")
        self.assertIn("MAVLink forwarding disabled", status)

    def test_mavlink_status_reports_disabled(self):
        payload = b'HTTP/1.0 200 OK\r\n\r\n{"enabled":false,"ports":{"listen":14555}}'
        with patch("station.radio_wifi._http_get", return_value=payload):
            self.assertEqual(_mavlink_status("192.168.8.134", "192.168.8.141"), "MAVLink forwarding disabled")


if __name__ == "__main__":
    unittest.main()
