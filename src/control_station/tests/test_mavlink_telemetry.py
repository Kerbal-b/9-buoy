import struct
from pathlib import Path
from types import SimpleNamespace
import unittest

from station.mavlink_telemetry import decode_datagram, gcs_heartbeat
from station.qt_app import ControlStationBackend


def backend_args():
    return SimpleNamespace(
        wifi_local_ip="auto", send_rate=10.0, deadzone=0.12,
        tcp_port=4210, udp_port=4211, audio_port=4212,
    )


def make_frame(message_id, payload, *, version=2, system_id=7, component_id=1):
    # Deliberately use a bit-by-bit CRC implementation independent of the decoder.
    extras = {0: 50, 1: 124, 33: 104, 35: 244, 65: 118, 109: 185}
    if version == 2:
        header = bytes((0xFD, len(payload), 0, 0, 3, system_id, component_id)) + message_id.to_bytes(3, "little")
    else:
        header = bytes((0xFE, len(payload), 3, system_id, component_id, message_id))
    crc = 0xFFFF
    for byte in header[1:] + payload + bytes((extras[message_id],)):
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0x8408 if crc & 1 else 0)
    return header + payload + struct.pack("<H", crc)


class MavlinkTelemetryTests(unittest.TestCase):
    def test_station_registers_with_active_backpack_once_per_interval(self):
        class DatagramSocket:
            sent = []

            def sendto(self, packet, target):
                self.sent.append((packet, target))

            def recvfrom(self, size):
                raise BlockingIOError

        backend = ControlStationBackend(
            backend_args(),
            Path("unused-comm.log"), Path("unused-prefs.json"),
        )
        receiver = DatagramSocket()
        backend._radio_telemetry_socket = receiver
        backend._set_state(radioWifiIp="192.168.8.134", radioWifiStatus="MAVLink forwarding active")
        backend._poll_radio_telemetry()
        backend._poll_radio_telemetry()
        self.assertEqual(receiver.sent, [(gcs_heartbeat(), ("192.168.8.134", 14555))])

    def test_decodes_v2_heartbeat_and_position(self):
        heartbeat = make_frame(0, struct.pack("<IBBBBB", 0, 11, 0, 0, 4, 3))
        position = make_frame(33, struct.pack("<Iiiii3hH", 10, 470001234, -1220005678, 0, 0, 0, 0, 0, 0))
        messages = decode_datagram(heartbeat + position)
        self.assertEqual([item["kind"] for item in messages], ["heartbeat", "position"])
        self.assertEqual(messages[0]["system_id"], 7)
        self.assertAlmostEqual(messages[1]["latitude"], 47.0001234)
        self.assertAlmostEqual(messages[1]["longitude"], -122.0005678)

    def test_decodes_v1_power_and_rc_channels(self):
        power = bytearray(31)
        struct.pack_into("<Hh", power, 14, 12000, 125)
        struct.pack_into("<b", power, 30, 68)
        channels = struct.pack("<I18HBB", 20, *range(1000, 1018), 16, 255)
        messages = decode_datagram(make_frame(1, power, version=1) + make_frame(65, channels, version=1))
        self.assertEqual(messages[0]["voltage_v"], 12.0)
        self.assertEqual(messages[0]["current_a"], 1.25)
        self.assertEqual(messages[0]["remaining_pct"], 68)
        self.assertEqual(messages[1]["channels"], list(range(1000, 1016)))

    def test_rejects_corrupt_checksum(self):
        frame = bytearray(make_frame(0, struct.pack("<IBBBBB", 0, 11, 0, 0, 4, 3)))
        frame[12] ^= 0xFF
        self.assertEqual(decode_datagram(bytes(frame)), [])

    def test_invalid_battery_fields_are_not_displayed(self):
        power = bytearray(31)
        struct.pack_into("<Hh", power, 14, 0xFFFF, -1)
        struct.pack_into("<b", power, 30, -1)
        message = decode_datagram(make_frame(1, power))[0]
        self.assertIsNone(message["voltage_v"])
        self.assertIsNone(message["current_a"])
        self.assertIsNone(message["remaining_pct"])

    def test_radio_status_is_validated_and_keeps_raw_units(self):
        payload = struct.pack("<HHBBBBB", 2, 3, 91, 255, 75, 12, 255)
        message = decode_datagram(make_frame(109, payload))[0]
        self.assertEqual(message["kind"], "radio_status")
        self.assertEqual(message["rssi_raw"], 91)
        self.assertIsNone(message["remote_rssi_raw"])
        self.assertEqual(message["tx_buffer_free_pct"], 75)

        corrupt = bytearray(make_frame(109, payload))
        corrupt[-1] ^= 0xFF
        self.assertEqual(decode_datagram(corrupt), [])

    def test_relayed_elrs_receiver_status_updates_lq_and_rssi(self):
        class DatagramSocket:
            def __init__(self, frame):
                self.frame = frame

            def recvfrom(self, size):
                if self.frame is None:
                    raise BlockingIOError
                frame, self.frame = self.frame, None
                return frame, ("192.168.8.134", 14550)

        payload = struct.pack("<HHBBBBB", 0, 0, 254, 11, 255, 13, 255)
        frame = make_frame(109, payload, version=1, system_id=1, component_id=68)
        backend = ControlStationBackend(
            backend_args(),
            Path("unused-comm.log"), Path("unused-prefs.json"),
        )
        backend._set_state(radioWifiIp="192.168.8.134")
        backend._radio_telemetry_socket = DatagramSocket(frame)
        backend._poll_radio_telemetry()
        self.assertEqual(backend._state["radioLinkQuality"], "100% (RX uplink)")
        self.assertEqual(backend._state["radioRssiStatus"], "-11 dBm (RX uplink)")
        self.assertEqual(backend._state["radioSnrStatus"], "13 dB (RX uplink)")
        backend._radio_link_last_at -= 6.0
        backend._poll_radio_telemetry()
        self.assertEqual(backend._state["radioLinkQuality"], "Unknown")

    def test_channel_banks_merge_without_erasing_other_bank(self):
        class DatagramSocket:
            def __init__(self, frames):
                self.frames = list(frames)

            def recvfrom(self, size):
                if not self.frames:
                    raise BlockingIOError
                return self.frames.pop(0), ("192.168.8.134", 14550)

        def bank_frame(port, values):
            payload = struct.pack("<I8HBB", 100, *values, port, 255)
            return make_frame(35, payload, version=1, system_id=1)

        backend = ControlStationBackend(
            backend_args(),
            Path("unused-comm.log"), Path("unused-prefs.json"),
        )
        backend._set_state(radioWifiIp="192.168.8.134")
        backend._radio_telemetry_socket = DatagramSocket([
            bank_frame(0, [1000, 1500, 2000, 1500, 1000, 1000, 1000, 1000]),
            bank_frame(1, [1500] * 8),
        ])
        backend._poll_radio_telemetry()
        self.assertEqual(backend._state["radioChannels"],
                         [1000, 1500, 2000, 1500, 1000, 1000, 1000, 1000] + [1500] * 8)
        backend._radio_telemetry_socket = DatagramSocket([bank_frame(1, [2000] * 8)])
        backend._poll_radio_telemetry()
        self.assertEqual(backend._state["radioChannels"][:8],
                         [1000, 1500, 2000, 1500, 1000, 1000, 1000, 1000])
        self.assertEqual(backend._state["radioChannels"][8:], [2000] * 8)

    def test_backend_uses_radio_telemetry_when_buoy_wifi_is_down(self):
        class DatagramSocket:
            def __init__(self, frames):
                self.frames = list(frames)

            def recvfrom(self, size):
                if self.frames:
                    return self.frames.pop(0)
                raise BlockingIOError

        backend = ControlStationBackend(
            backend_args(),
            Path("unused-comm.log"),
            Path("unused-prefs.json"),
        )
        backend._set_state(radioWifiIp="192.168.8.134")
        radio = struct.pack("<HHBBBBB", 2, 3, 91, 255, 75, 12, 255)
        power = bytearray(31)
        struct.pack_into("<Hh", power, 14, 12000, 125)
        struct.pack_into("<b", power, 30, 68)
        position = struct.pack("<Iiiii3hH", 10, 470001234, -1220005678, 0, 0, 0, 0, 0, 0)
        backend._radio_telemetry_socket = DatagramSocket([
            (make_frame(1, power), ("192.168.8.99", 14555)),
            (make_frame(109, radio, system_id=51), ("192.168.8.134", 14555)),
            (make_frame(1, power) + make_frame(33, position), ("192.168.8.134", 14555)),
        ])
        backend._poll_radio_telemetry()
        self.assertEqual(backend._state["radioRssiStatus"], "91 (device units)")
        self.assertEqual(backend._state["radioSystemId"], "7")
        self.assertEqual(backend._state["radioTelemetryPackets"], 2)
        self.assertEqual(backend._state["telemetrySource"], "ELRS")
        self.assertIn("68%", backend._state["batteryStatus"])
        self.assertEqual(backend._state["currentLocation"], "47.000123, -122.000568")

        backend._radio_field_last_at["power"] -= 16.0
        backend._poll_radio_telemetry()
        self.assertEqual(backend._state["batteryStatus"], "N/A")
        self.assertEqual(backend._state["currentLocation"], "47.000123, -122.000568")

        backend._transport = SimpleNamespace(is_open=True)
        backend._set_state(batteryStatus="Direct Wi-Fi battery", currentLocation="Direct Wi-Fi position", telemetrySource="Buoy Wi-Fi")
        channels = struct.pack("<I18HBB", 20, *range(1000, 1018), 16, 255)
        backend._radio_telemetry_socket = DatagramSocket([
            (make_frame(1, power) + make_frame(65, channels), ("192.168.8.134", 14555)),
        ])
        backend._poll_radio_telemetry()
        self.assertEqual(backend._state["batteryStatus"], "Direct Wi-Fi battery")
        self.assertEqual(backend._state["currentLocation"], "Direct Wi-Fi position")
        self.assertEqual(backend._state["radioChannels"][0], 1000)

        backend._transport = None
        backend._set_state(telemetrySource="ELRS")
        backend._radio_telemetry_last_at -= 6.0
        backend._poll_radio_telemetry()
        self.assertEqual(backend._state["telemetrySource"], "No live telemetry")
        self.assertEqual(backend._state["batteryStatus"], "N/A")
        self.assertEqual(backend._state["radioBatteryStatus"], "N/A")
        self.assertEqual(backend._state["radioChannels"], [])


if __name__ == "__main__":
    unittest.main()
