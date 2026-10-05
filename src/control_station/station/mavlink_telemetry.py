"""Small receive-only decoder for MAVLink telemetry forwarded by an ELRS backpack.

Only the common messages used by the control panel are decoded. Checksums are
validated before a packet can change displayed state.
"""

from __future__ import annotations

import struct
from typing import Any


# Common.xml CRC_EXTRA values from the MAVLink generated message headers.
CRC_EXTRAS = {0: 50, 1: 124, 33: 104, 35: 244, 65: 118, 109: 185}


def _crc_x25(data: bytes, extra: int) -> int:
    crc = 0xFFFF
    for value in (*data, extra):
        tmp = value ^ (crc & 0xFF)
        tmp ^= (tmp << 4) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


def gcs_heartbeat() -> bytes:
    """MAVLink v1 GCS HEARTBEAT with no control command."""
    payload = struct.pack("<IBBBBB", 0, 6, 8, 0, 0, 3)
    header = bytes((0xFE, len(payload), 0, 255, 190, 0))
    return header + payload + struct.pack("<H", _crc_x25(header[1:] + payload, CRC_EXTRAS[0]))


def _decode_payload(message_id: int, payload: bytes) -> dict[str, Any] | None:
    if message_id == 0 and len(payload) >= 9:  # HEARTBEAT
        return {"kind": "heartbeat", "system_status": payload[7]}
    if message_id == 1 and len(payload) >= 31:  # SYS_STATUS
        voltage_mv = struct.unpack_from("<H", payload, 14)[0]
        current_ca = struct.unpack_from("<h", payload, 16)[0]
        remaining = struct.unpack_from("<b", payload, 30)[0]
        return {
            "kind": "power",
            "voltage_v": None if voltage_mv == 0xFFFF else voltage_mv / 1000.0,
            "current_a": None if current_ca == -1 else current_ca / 100.0,
            "remaining_pct": None if remaining == -1 else remaining,
        }
    if message_id == 33 and len(payload) >= 28:  # GLOBAL_POSITION_INT
        latitude, longitude = struct.unpack_from("<ii", payload, 4)
        if not (-900000000 <= latitude <= 900000000 and -1800000000 <= longitude <= 1800000000):
            return None
        return {"kind": "position", "latitude": latitude / 1e7, "longitude": longitude / 1e7}
    if message_id == 35 and len(payload) >= 22:  # RC_CHANNELS_RAW, 8 channels per port
        port = payload[20]
        if port not in (0, 1):
            return None
        channels = struct.unpack_from("<8H", payload, 4)
        return {
            "kind": "channels_raw",
            "port": port,
            "channels": [value if value not in (0, 0xFFFF) else None for value in channels],
        }
    if message_id == 65 and len(payload) >= 42:  # RC_CHANNELS
        channels = struct.unpack_from("<18H", payload, 4)
        count = min(payload[40], 18)
        return {
            "kind": "channels",
            "channels": [value if value not in (0, 0xFFFF) else None for value in channels[:count]],
        }
    if message_id == 109 and len(payload) >= 9:  # RADIO_STATUS
        rxerrors, fixed, rssi, remote_rssi, txbuf, noise, remote_noise = struct.unpack_from(
            "<HHBBBBB", payload
        )
        return {
            "kind": "radio_status",
            "rssi_raw": None if rssi == 255 else rssi,
            "remote_rssi_raw": None if remote_rssi == 255 else remote_rssi,
            "tx_buffer_free_pct": txbuf,
            "noise_raw": None if noise == 255 else noise,
            "remote_noise_raw": None if remote_noise == 255 else remote_noise,
            "receive_errors": rxerrors,
            "corrected_packets": fixed,
        }
    return None


def decode_datagram(data: bytes) -> list[dict[str, Any]]:
    """Return validated supported messages from one UDP datagram."""
    messages: list[dict[str, Any]] = []
    offset = 0
    while offset < len(data):
        magic = data[offset]
        if magic not in (0xFD, 0xFE):
            offset += 1
            continue
        v2 = magic == 0xFD
        header_length = 10 if v2 else 6
        if len(data) - offset < header_length:
            break
        length = data[offset + 1]
        signature_length = 13 if v2 and data[offset + 2] & 0x01 else 0
        frame_length = header_length + length + 2 + signature_length
        if len(data) - offset < frame_length:
            break
        frame = data[offset:offset + frame_length]
        message_id = int.from_bytes(frame[7:10], "little") if v2 else frame[5]
        extra = CRC_EXTRAS.get(message_id)
        if extra is not None:
            checksum_at = header_length + length
            received_crc = struct.unpack_from("<H", frame, checksum_at)[0]
            calculated_crc = _crc_x25(frame[1:checksum_at], extra)
            if received_crc == calculated_crc:
                decoded = _decode_payload(message_id, frame[header_length:checksum_at])
                if decoded is not None:
                    decoded["system_id"] = frame[5] if v2 else frame[3]
                    decoded["component_id"] = frame[6] if v2 else frame[4]
                    messages.append(decoded)
        offset += frame_length
    return messages
