"""Small receive-only decoder for MAVLink telemetry forwarded by an ELRS backpack.

Only the common messages used by the control panel are decoded. Checksums are
validated before a packet can change displayed state.
"""

from __future__ import annotations

import struct
import math
from typing import Any


# Common.xml CRC_EXTRA values from the MAVLink generated message headers.
CRC_EXTRAS = {0: 50, 1: 124, 24: 24, 33: 104, 35: 244, 65: 118, 109: 185, 242: 104, 251: 170, 252: 44, 253: 83}


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
    if message_id == 253 and len(payload) >= 2:  # STATUSTEXT, versioned buoy status
        text = payload[1:51].split(b"\0", 1)[0].decode("ascii", errors="replace")
        if text.startswith("B1,"):
            try:
                fields = [int(value) for value in text[3:].split(",")]
            except ValueError:
                return None
            if len(fields) != 13 or fields[0] not in (0, 1) or fields[6] not in (0, 1) or fields[7] not in (0, 1):
                return None
            return {"kind": "compact_status", "armed": bool(fields[0]), "assist": fields[1],
                    "mission_state": fields[2], "percent": fields[3], "waypoint": fields[4],
                    "waypoint_count": fields[5], "recording": bool(fields[6]),
                    "audio_recording": bool(fields[7]), "heading_deg": fields[8],
                    "home_distance_m": fields[9], "gps_fix": bool(fields[10]), "home_kind": fields[11],
                    "mission_block": fields[12]}
        if text.startswith("BC1,"):
            try:
                fields = [int(value) for value in text[4:].split(",")]
            except ValueError:
                return None
            if len(fields) != 5:
                return None
            return {"kind": "calibration_status", "state": fields[0], "step": fields[1],
                    "power": fields[2], "max_power": fields[3], "duration_s": fields[4]}
        return None
    if message_id == 0 and len(payload) >= 9:  # HEARTBEAT
        return {
            "kind": "heartbeat",
            "system_status": payload[7],
            "armed": bool(payload[6] & 0x80),
        }
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
        north_cm_s, east_cm_s = struct.unpack_from("<hh", payload, 20)
        track_cdeg = struct.unpack_from("<H", payload, 26)[0]
        return {
            "kind": "position", "latitude": latitude / 1e7, "longitude": longitude / 1e7,
            "ground_speed_mps": None if track_cdeg == 0xFFFF else round(math.hypot(north_cm_s, east_cm_s) / 100.0, 2),
            "ground_track_deg": None if track_cdeg == 0xFFFF else track_cdeg / 100.0,
        }
    if message_id == 242 and len(payload) >= 52:  # HOME_POSITION
        latitude, longitude = struct.unpack_from("<ii", payload, 0)
        if not (-900000000 <= latitude <= 900000000 and -1800000000 <= longitude <= 1800000000):
            return None
        return {"kind": "home_position", "latitude": latitude / 1e7, "longitude": longitude / 1e7}
    if message_id == 24 and len(payload) >= 30:  # GPS_RAW_INT
        eph = struct.unpack_from("<H", payload, 20)[0]
        fix_type = payload[28]
        satellites = payload[29]
        return {
            "kind": "gps_quality",
            "satellites": None if satellites == 255 else satellites,
            "hdop": None if eph == 0xFFFF else eph / 100.0,
            "fix_type": fix_type,
            "fix_valid": fix_type >= 2,
        }
    if message_id == 252 and len(payload) >= 18:  # NAMED_VALUE_INT
        if payload[8:18] == b"BUOY_AUDIO":
            return {
                "kind": "buoy_audio_state",
                "audio_recording": bool(struct.unpack_from("<i", payload, 4)[0]),
            }
        if payload[8:18].startswith(b"HOME_KIND"):
            return {"kind": "home_source", "source": int(struct.unpack_from("<i", payload, 4)[0])}
        if payload[8:18] != b"BUOY_STATE":
            return None
        rear, left, right, flags = struct.unpack_from("<bbbB", payload, 4)
        mode = flags & 0x07
        source = (flags >> 3) & 0x03
        if mode > 3 or source > 2:
            return None
        return {
            "kind": "buoy_state", "mode": ("IDLE", "MANUAL", "HOLD", "GOTO")[mode],
            "control_source": ("NONE", "WIFI", "TX12")[source],
            "armed": bool(flags & 0x20), "recording": bool(flags & 0x40),
            "recovery_stop": bool(flags & 0x80),
            "motor_pwm": [round(value * 255 / 127) for value in (rear, left, right)],
        }
    if message_id == 251 and len(payload) >= 18:  # NAMED_VALUE_FLOAT
        raw_name = payload[8:18].split(b"\0", 1)[0]
        try:
            name = raw_name.decode("ascii")
        except UnicodeDecodeError:
            return None
        if name not in {"WATER_TEMP", "DEPTH_M", "TDS_V", "TURBIDITY", "HOME_DIST", "DIST_HOME",
                        "MIS_STATE", "MIS_BLOCK", "MIS_PCT", "MIS_COUNT", "DIST_WP", "MIS_WP",
                        "MAG_HEAD", "TILT_HEAD", "SEA_ACC", "SEA_GYR", "MAN_AST",
                        "CAL_STAT", "CAL_STEP", "CAL_PWR", "CAL_MAX", "CAL_DUR"}:
            return None
        value = struct.unpack_from("<f", payload, 4)[0]
        if not math.isfinite(value):
            return None
        return {"kind": "science", "name": name, "value": value}
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
                    if decoded["kind"] == "radio_status" and decoded["system_id"] == 1 and decoded["component_id"] == 68:
                        # The buoy relay uses 255 for full ELRS LQ and for
                        # signed SNR -1, while generic MAVLink uses 255 as unknown.
                        decoded["rssi_raw"] = frame[header_length + 4]
                        decoded["noise_raw"] = frame[header_length + 7]
                    messages.append(decoded)
        offset += frame_length
    return messages
