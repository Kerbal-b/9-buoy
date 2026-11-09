#!/usr/bin/env python3
"""Receive-only MAVLink UDP to a small LAN dashboard (Python 3.6+)."""

from __future__ import print_function

import json
import math
import socket
import struct
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn


UDP_HOST = "0.0.0.0"
UDP_PORT = 14550
HTTP_HOST = "0.0.0.0"
HTTP_PORT = 8080
STALE_SECONDS = 8.0

# MAVLink common.xml CRC_EXTRA values for messages emitted by the buoy and
# standard messages commonly forwarded by the Backpack.
CRC_EXTRA = {
    0: 50, 1: 124, 24: 24, 30: 39, 33: 104, 35: 244, 65: 118,
    74: 20, 109: 185, 147: 154, 242: 104, 251: 170, 252: 44, 253: 83,
}
MESSAGE_NAMES = {
    0: "HEARTBEAT", 1: "SYS_STATUS", 24: "GPS_RAW_INT", 30: "ATTITUDE",
    33: "GLOBAL_POSITION_INT", 35: "RC_CHANNELS_RAW", 65: "RC_CHANNELS",
    74: "VFR_HUD", 109: "RADIO_STATUS", 147: "BATTERY_STATUS",
    242: "HOME_POSITION", 251: "NAMED_VALUE_FLOAT",
    252: "NAMED_VALUE_INT", 253: "STATUSTEXT",
}
MISSION_STATES = {
    0: "IDLE", 1: "STARTING", 2: "MOVING", 3: "COLLECTING",
    4: "RETURNING_HOME", 5: "STOPPED", 6: "COMPLETE",
    7: "RETURNED_HOME", 8: "BLOCKED",
}
MISSION_BLOCKS = {
    0: "", 1: "Cycle the arm switch low, then high",
    2: "Set power to minimum and center the sticks",
    3: "Wi-Fi control currently owns the buoy",
    4: "Mission setup is not ready",
    5: "GPS fix must be fresh, with 5 satellites and HDOP 3.0 or lower",
    6: "Buoy GPS position is outside the mission boundary",
    7: "Move within 5 m of mission Home",
    8: "Magnetometer calibration or heading is unavailable",
    9: "Cycle SE low before retrying Mission",
    10: "Load a V4 mission with Home and a boundary",
    11: "Buoy SD card is unavailable",
    12: "Stop or save the existing science session first",
    13: "Mission route point or speed is invalid",
    14: "Move the arm switch fully high",
    15: "Buoy service mode or SD transfer is active",
    16: "Could not create the mission's SD session; check the card and retry",
}
ASSIST_MODES = {0: "DIRECT", 1: "COMPASS_ASSIST", 2: "ASSIST_NO_HEADING", 3: "MISSION"}
HOME_SOURCES = {0: "NONE", 1: "MANUAL", 2: "MISSION"}
BUOY_MODES = {0: "IDLE", 1: "MANUAL", 2: "HOLD", 3: "GOTO"}
CONTROL_SOURCES = {0: "NONE", 1: "WIFI", 2: "TX12"}


def x25(data, extra):
    crc = 0xffff
    for value in bytearray(data) + bytearray((extra,)):
        tmp = value ^ (crc & 0xff)
        tmp ^= (tmp << 4) & 0xff
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xffff
    return crc


def json_safe(value):
    """Replace non-finite sensor floats with JSON null (Python 3.6 compatible)."""
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return dict((key, json_safe(item)) for key, item in value.items())
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    return value


def frames(data):
    """Yield metadata for complete MAVLink v1/v2 frames, including unknown IDs."""
    offset = 0
    size = len(data)
    while offset < size:
        magic = data[offset]
        if magic not in (0xfe, 0xfd):
            offset += 1
            continue
        v2 = magic == 0xfd
        header_len = 10 if v2 else 6
        if size - offset < header_len:
            return
        payload_len = data[offset + 1]
        signed = v2 and bool(data[offset + 2] & 1)
        frame_end = offset + header_len + payload_len + 2 + (13 if signed else 0)
        if frame_end > size:
            return
        frame = data[offset:frame_end]
        message_id = ((frame[7] | (frame[8] << 8) | (frame[9] << 16))
                      if v2 else frame[5])
        extra = CRC_EXTRA.get(message_id)
        crc_at = header_len + payload_len
        received = struct.unpack_from("<H", frame, crc_at)[0]
        valid = None if extra is None else received == x25(frame[1:crc_at], extra)
        yield {
            "message_id": message_id,
            "payload": frame[header_len:header_len + payload_len],
            "frame": frame,
            "version": 2 if v2 else 1,
            "sequence": frame[4] if v2 else frame[2],
            "system_id": frame[5] if v2 else frame[3],
            "component_id": frame[6] if v2 else frame[4],
            "signed": bool(signed),
            "checksum_valid": valid,
        }
        offset = frame_end


class State(object):
    def __init__(self):
        self.lock = threading.Lock()
        self.values = {
            "position": None, "battery_percent": None, "voltage_v": None,
            "speed_mps": None, "heading_deg": None, "armed": None,
            "system_status": None, "last_update": None, "source": None,
        }
        self.times = {}
        self.trail = []
        self.packets = 0
        self.datagrams = 0
        self.bytes_received = 0
        self.bad_checksums = 0
        self.raw_by_id = {}
        self.message_counts = {}
        self.standard = {}
        self.buoy = {
            "state": {}, "mission": {}, "home": {}, "assist": {},
            "audio": {}, "calibration": {}, "science": {},
        }
        self.last_rx = None
        self.udp_source = None

    @staticmethod
    def _name(payload):
        return payload[8:18].split(b"\0", 1)[0].decode("ascii", errors="replace").strip()

    def note_datagram(self, data, source):
        with self.lock:
            self.datagrams += 1
            self.bytes_received += len(data)
            self.udp_source = {"ip": source[0], "port": source[1]}

    def _decode(self, packet):
        message_id = packet["message_id"]
        payload = packet["payload"]
        decoded = {}
        custom = {}
        try:
            if message_id == 0 and len(payload) >= 9:
                custom_mode = struct.unpack_from("<I", payload, 0)[0]
                base_mode = payload[6]
                decoded["heartbeat"] = {
                    "custom_mode": custom_mode, "vehicle_type": payload[4],
                    "autopilot": payload[5], "base_mode": base_mode,
                    "system_status": payload[7], "mavlink_version": payload[8],
                    "armed": bool(base_mode & 0x80),
                }
            elif message_id == 1 and len(payload) >= 31:
                voltage = struct.unpack_from("<H", payload, 14)[0]
                current = struct.unpack_from("<h", payload, 16)[0]
                remaining = struct.unpack_from("<b", payload, 30)[0]
                decoded["sys_status"] = {
                    "battery_voltage_v": None if voltage == 0xffff else voltage / 1000.0,
                    "battery_current_a": None if current == -1 else current / 100.0,
                    "battery_remaining_pct": None if remaining < 0 else remaining,
                }
            elif message_id == 147 and len(payload) >= 36:
                current_consumed, energy_consumed = struct.unpack_from("<ii", payload, 0)
                temperature = struct.unpack_from("<h", payload, 8)[0]
                cells = list(struct.unpack_from("<10H", payload, 10))
                current = struct.unpack_from("<h", payload, 30)[0]
                remaining = struct.unpack_from("<b", payload, 35)[0]
                decoded["battery_status"] = {
                    "current_consumed_mah": None if current_consumed < 0 else current_consumed,
                    "energy_consumed_hj": None if energy_consumed < 0 else energy_consumed,
                    "temperature_c": None if temperature == 32767 else temperature / 100.0,
                    "cell_voltages_v": [None if cell == 0xffff else cell / 1000.0 for cell in cells],
                    "current_a": None if current == -1 else current / 100.0,
                    "battery_id": payload[32], "battery_function": payload[33],
                    "battery_type": payload[34],
                    "battery_remaining_pct": None if remaining < 0 else remaining,
                }
            elif message_id == 33 and len(payload) >= 28:
                lat, lon, alt, relative_alt = struct.unpack_from("<iiii", payload, 4)
                vx, vy, vz = struct.unpack_from("<hhh", payload, 20)
                heading = struct.unpack_from("<H", payload, 26)[0]
                decoded["global_position"] = {
                    "time_boot_ms": struct.unpack_from("<I", payload, 0)[0],
                    "latitude_deg": None if lat == 0 else lat / 1e7,
                    "longitude_deg": None if lon == 0 else lon / 1e7,
                    "altitude_m": alt / 1000.0, "relative_altitude_m": relative_alt / 1000.0,
                    "velocity_north_mps": vx / 100.0, "velocity_east_mps": vy / 100.0,
                    "velocity_down_mps": vz / 100.0,
                    "ground_speed_mps": round(math.hypot(vx, vy) / 100.0, 2),
                    "heading_deg": None if heading == 0xffff else heading / 100.0,
                }
            elif message_id == 24 and len(payload) >= 30:
                time_usec = struct.unpack_from("<Q", payload, 0)[0]
                lat, lon, alt = struct.unpack_from("<iii", payload, 8)
                eph, epv, velocity, course = struct.unpack_from("<HHHH", payload, 20)
                fix_type, satellites = payload[28], payload[29]
                hdop = None if eph == 0xffff else eph / 100.0
                vdop = None if epv == 0xffff else epv / 100.0
                decoded["gps_raw"] = {
                    "time_usec": time_usec,
                    "latitude_deg": None if lat == 0 else lat / 1e7,
                    "longitude_deg": None if lon == 0 else lon / 1e7,
                    "altitude_m": alt / 1000.0,
                    "hdop": hdop, "vdop": vdop,
                    # Backward-compatible aliases; MAVLink eph/epv are DOP * 100, not metres.
                    "eph_m": hdop, "epv_m": vdop,
                    "ground_speed_mps": None if velocity == 0xffff else velocity / 100.0,
                    "course_deg": None if course == 0xffff else course / 100.0,
                    "fix_type": fix_type, "fix_valid": fix_type >= 2,
                    "satellites_visible": None if satellites == 255 else satellites,
                }
            elif message_id == 30 and len(payload) >= 28:
                time_boot_ms = struct.unpack_from("<I", payload, 0)[0]
                roll, pitch, yaw, rollspeed, pitchspeed, yawspeed = struct.unpack_from("<ffffff", payload, 4)
                decoded["attitude"] = {
                    "time_boot_ms": time_boot_ms, "roll_rad": roll, "pitch_rad": pitch,
                    "yaw_rad": yaw, "roll_deg": roll * 180.0 / math.pi,
                    "pitch_deg": pitch * 180.0 / math.pi,
                    "yaw_deg": yaw * 180.0 / math.pi % 360.0,
                    "rollspeed_rad_s": rollspeed, "pitchspeed_rad_s": pitchspeed,
                    "yawspeed_rad_s": yawspeed,
                }
            elif message_id == 74 and len(payload) >= 20:
                airspeed, groundspeed = struct.unpack_from("<ff", payload, 0)
                heading, throttle = struct.unpack_from("<hH", payload, 8)
                altitude, climb = struct.unpack_from("<ff", payload, 12)
                decoded["vfr_hud"] = {
                    "airspeed_mps": airspeed, "ground_speed_mps": groundspeed,
                    "heading_deg": None if heading < 0 else heading,
                    "throttle_pct": throttle, "altitude_m": altitude, "climb_mps": climb,
                }
            elif message_id == 242 and len(payload) >= 52:
                lat, lon, alt = struct.unpack_from("<iii", payload, 0)
                local = struct.unpack_from("<fff", payload, 12)
                quaternion = struct.unpack_from("<ffff", payload, 24)
                approach = struct.unpack_from("<fff", payload, 40)
                decoded["home_position"] = {
                    "latitude_deg": None if lat == 0 else lat / 1e7,
                    "longitude_deg": None if lon == 0 else lon / 1e7,
                    "altitude_m": alt / 1000.0,
                    "local_position_m": list(local),
                    "orientation_quaternion": list(quaternion),
                    "approach_vector_m": list(approach),
                    "time_usec": struct.unpack_from("<Q", payload, 52)[0] if len(payload) >= 60 else None,
                }
            elif message_id == 109 and len(payload) >= 9:
                rxerrors, fixed = struct.unpack_from("<HH", payload, 0)
                rssi, remote_rssi, txbuf = payload[4], payload[5], payload[6]
                noise, remote_noise = payload[7], payload[8]
                noise = noise if noise < 128 else noise - 256
                remote_noise = remote_noise if remote_noise < 128 else remote_noise - 256
                decoded["radio_status"] = {
                    "rx_errors": rxerrors, "corrected_packets": fixed,
                    "rssi_raw": None if rssi == 255 else rssi,
                    "remote_rssi_dbm_magnitude": None if remote_rssi == 255 else remote_rssi,
                    "tx_buffer_free_pct": txbuf,
                    "noise_db": None if payload[7] == 255 else noise,
                    "remote_noise_db": None if payload[8] == 255 else remote_noise,
                    "uplink_link_quality_pct": None if rssi == 255 else round(rssi * 100.0 / 255.0, 1),
                }
            elif message_id == 35 and len(payload) >= 22:
                bank = payload[20]
                decoded["rc_channels_raw"] = {
                    "time_boot_ms": struct.unpack_from("<I", payload, 0)[0],
                    "bank": bank,
                    "first_channel": bank * 8 + 1,
                    "channels_us": list(struct.unpack_from("<8H", payload, 4)),
                    "rssi": payload[21],
                }
            elif message_id == 65 and len(payload) >= 42:
                count = min(payload[40], 18)
                decoded["rc_channels"] = {
                    "time_boot_ms": struct.unpack_from("<I", payload, 0)[0],
                    "count": count, "channels_us": list(struct.unpack_from("<18H", payload, 4)[:count]),
                    "rssi": payload[41],
                }
            elif message_id == 253 and len(payload) >= 2:
                text = payload[1:51].split(b"\0", 1)[0].decode("utf-8", errors="replace")
                decoded["status_text"] = {
                    "severity": payload[0], "text": text,
                    "chunk_seq": payload[51] if len(payload) > 51 else None,
                    "id": struct.unpack_from("<H", payload, 52)[0] if len(payload) >= 54 else None,
                }
                if text.startswith("B1,"):
                    try:
                        fields = [int(item) for item in text[3:].split(",")]
                        if len(fields) == 13:
                            custom["compact_status"] = {
                                "armed": bool(fields[0]), "assist_code": fields[1],
                                "mission_state_code": fields[2], "mission_percent": fields[3],
                                "current_waypoint": fields[4], "waypoint_count": fields[5],
                                "science_recording": bool(fields[6]), "audio_recording": bool(fields[7]),
                                "heading_deg": fields[8], "home_distance_m": None if fields[9] < 0 else fields[9],
                                "gps_fix": bool(fields[10]), "home_source_code": fields[11],
                                "mission_block_code": fields[12],
                            }
                    except ValueError:
                        pass
                elif text.startswith("BC1,"):
                    try:
                        fields = [int(item) for item in text[4:].split(",")]
                        if len(fields) == 5:
                            custom["calibration_status"] = {
                                "state_code": fields[0], "step": fields[1], "power_pct": fields[2],
                                "max_power_pct": fields[3], "duration_s": fields[4],
                            }
                    except ValueError:
                        pass
            elif message_id == 252 and len(payload) >= 18:
                name = self._name(payload)
                if name == "BUOY_STATE":
                    rear, left, right, flags = struct.unpack_from("<bbbB", payload, 4)
                    mode_code = flags & 7
                    source_code = (flags >> 3) & 3
                    custom["state"] = {
                        "time_boot_ms": struct.unpack_from("<I", payload, 0)[0],
                        "mode_code": mode_code, "mode": BUOY_MODES.get(mode_code, "UNKNOWN"),
                        "control_source_code": source_code,
                        "control_source": CONTROL_SOURCES.get(source_code, "UNKNOWN"),
                        "armed": bool(flags & 0x20), "science_recording": bool(flags & 0x40),
                        "recovery_stop": bool(flags & 0x80),
                        "motor_pwm_signed_127": [rear, left, right],
                        "motor_pwm_pct": [round(value * 100.0 / 127.0, 1) for value in (rear, left, right)],
                    }
                elif name == "BUOY_AUDIO":
                    custom["audio"] = {"recording": bool(struct.unpack_from("<i", payload, 4)[0])}
                elif name == "HOME_KIND":
                    code = struct.unpack_from("<i", payload, 4)[0]
                    custom["home"] = {"source_code": code, "source": HOME_SOURCES.get(code, "UNKNOWN")}
                else:
                    value = struct.unpack_from("<i", payload, 4)[0]
                    custom["named_values"] = {name: value}
            elif message_id == 251 and len(payload) >= 18:
                name = self._name(payload)
                value = struct.unpack_from("<f", payload, 4)[0]
                if math.isfinite(value):
                    custom["science"] = {name: value}
        except (IndexError, struct.error, ValueError, OverflowError):
            return decoded, custom
        return decoded, custom

    def update(self, packet, source):
        now = time.time()
        message_id = packet["message_id"]
        payload = packet["payload"]
        decoded, custom = self._decode(packet) if packet["checksum_valid"] is not False else ({}, {})
        raw = {
            "message_id": message_id,
            "message_name": MESSAGE_NAMES.get(message_id, "UNKNOWN_%d" % message_id),
            "version": packet["version"], "sequence": packet["sequence"],
            "system_id": packet["system_id"], "component_id": packet["component_id"],
            "signed": packet["signed"], "checksum_valid": packet["checksum_valid"],
            "received_at": now, "payload_length": len(payload),
            "payload_hex": payload.hex(), "frame_hex": packet["frame"].hex(),
            "decoded": decoded,
            "buoy_decoded": custom,
        }
        with self.lock:
            if packet["checksum_valid"] is False:
                self.bad_checksums += 1
            else:
                self.packets += 1
            self.last_rx = now
            self.udp_source = {"ip": source[0], "port": source[1]}
            key = str(message_id)
            self.message_counts[key] = self.message_counts.get(key, 0) + 1
            if key not in self.raw_by_id and len(self.raw_by_id) >= 64:
                oldest = min(self.raw_by_id, key=lambda item: self.raw_by_id[item]["received_at"])
                del self.raw_by_id[oldest]
            self.raw_by_id[key] = raw
            for section, updates in ((self.standard, decoded), (self.buoy, custom)):
                for name, value in updates.items():
                    if name == "science":
                        bucket = section.setdefault("science", {})
                        for science_name, science_value in value.items():
                            bucket[science_name] = {"value": science_value, "received_at": now}
                    elif name == "named_values":
                        bucket = section.setdefault("named_values", {})
                        for value_name, int_value in value.items():
                            bucket[value_name] = {"value": int_value, "received_at": now}
                    elif name == "status_text":
                        value["received_at"] = now
                        section[name] = value
                    else:
                        value["received_at"] = now
                        section[name] = value
            compact = custom.get("compact_status")
            if compact:
                mission = self.buoy.setdefault("mission", {})
                for name in ("mission_state_code", "mission_percent", "current_waypoint", "waypoint_count", "home_distance_m", "gps_fix", "mission_block_code"):
                    mission[name] = compact[name]
                mission["state"] = MISSION_STATES.get(compact["mission_state_code"], "UNKNOWN")
                mission["start_block_reason"] = MISSION_BLOCKS.get(compact["mission_block_code"], "Unknown start interlock")
                mission["current_waypoint_index"] = max(0, compact["current_waypoint"] - 1)
                mission["received_at"] = now
                self.buoy["assist"] = {
                    "mode_code": compact["assist_code"],
                    "mode": ASSIST_MODES.get(compact["assist_code"], "UNKNOWN"),
                    "received_at": now,
                }
                self.buoy["home"].update({
                    "source_code": compact["home_source_code"],
                    "source": HOME_SOURCES.get(compact["home_source_code"], "UNKNOWN"),
                    "distance_m": compact["home_distance_m"], "gps_fix": compact["gps_fix"],
                    "received_at": now,
                })
                state = self.buoy.setdefault("state", {})
                state.update({"armed": compact["armed"], "science_recording": compact["science_recording"],
                              "received_at": now})
                self.buoy["audio"] = {"recording": compact["audio_recording"], "received_at": now}
            calibration = custom.get("calibration_status")
            if calibration:
                calibration["received_at"] = now
                self.buoy["calibration"] = calibration
            elif "calibration_status" in decoded:
                pass
            if "state" in custom:
                self.buoy["state"].update(custom["state"])
                self.buoy["state"]["received_at"] = now
            if "audio" in custom:
                self.buoy["audio"].update(custom["audio"])
                self.buoy["audio"]["received_at"] = now
            if "home" in custom:
                self.buoy["home"].update(custom["home"])
                self.buoy["home"]["received_at"] = now
            science = custom.get("science", {})
            named = custom.get("named_values", {})
            mission = self.buoy.setdefault("mission", {})
            for field_name, field_value in dict(science, **named).items():
                if field_name == "MIS_STATE":
                    code = int(field_value)
                    mission.update({"mission_state_code": code, "state": MISSION_STATES.get(code, "UNKNOWN"), "received_at": now})
                elif field_name == "MIS_BLOCK":
                    code = int(field_value)
                    mission.update({"mission_block_code": code, "start_block_reason": MISSION_BLOCKS.get(code, "Unknown start interlock"), "received_at": now})
                elif field_name == "MIS_PCT":
                    mission.update({"mission_percent": field_value, "received_at": now})
                elif field_name == "MIS_WP":
                    mission.update({"current_waypoint": int(field_value), "current_waypoint_index": max(0, int(field_value) - 1), "received_at": now})
                elif field_name == "MIS_COUNT":
                    mission.update({"waypoint_count": int(field_value), "received_at": now})
                elif field_name in ("DIST_WP", "HOME_DIST", "DIST_HOME"):
                    key_name = "distance_to_waypoint_m" if field_name == "DIST_WP" else "home_distance_m"
                    self.buoy["home"][key_name] = field_value
                    self.buoy["home"]["received_at"] = now

            changed = {}
            if "heartbeat" in decoded:
                changed["armed"] = decoded["heartbeat"]["armed"]
                changed["system_status"] = decoded["heartbeat"]["system_status"]
            if "sys_status" in decoded:
                changed["voltage_v"] = decoded["sys_status"]["battery_voltage_v"]
                changed["battery_percent"] = decoded["sys_status"]["battery_remaining_pct"]
            if "battery_status" in decoded:
                if decoded["battery_status"]["battery_remaining_pct"] is not None:
                    changed["battery_percent"] = decoded["battery_status"]["battery_remaining_pct"]
                cells = [v for v in decoded["battery_status"]["cell_voltages_v"] if v is not None]
                if cells:
                    changed["voltage_v"] = sum(cells)
            position = decoded.get("global_position") or decoded.get("gps_raw")
            if position and position.get("latitude_deg") is not None and position.get("longitude_deg") is not None:
                if "gps_raw" not in decoded or decoded["gps_raw"]["fix_valid"]:
                    changed["position"] = {"lat": position["latitude_deg"], "lon": position["longitude_deg"]}
            if "global_position" in decoded:
                changed["speed_mps"] = decoded["global_position"]["ground_speed_mps"]
                if decoded["global_position"]["heading_deg"] is not None:
                    changed["heading_deg"] = decoded["global_position"]["heading_deg"]
            if "gps_raw" in decoded and decoded["gps_raw"]["ground_speed_mps"] is not None:
                changed["speed_mps"] = decoded["gps_raw"]["ground_speed_mps"]
                if decoded["gps_raw"]["course_deg"] is not None:
                    changed["heading_deg"] = decoded["gps_raw"]["course_deg"]
            if "attitude" in decoded:
                changed["heading_deg"] = decoded["attitude"]["yaw_deg"]
            if "vfr_hud" in decoded:
                changed["speed_mps"] = decoded["vfr_hud"]["ground_speed_mps"]
                if decoded["vfr_hud"]["heading_deg"] is not None:
                    changed["heading_deg"] = decoded["vfr_hud"]["heading_deg"]
            if "state" in custom:
                changed["armed"] = custom["state"]["armed"]
            if "compact_status" in custom:
                changed["armed"] = custom["compact_status"]["armed"]
            self.values.update(changed)
            for name in changed:
                self.times[name] = now
            self.values["last_update"] = now
            self.values["source"] = source[0]
            if "position" in changed:
                self.trail.append(changed["position"])
                self.trail = self.trail[-120:]

    def snapshot(self):
        now = time.time()
        with self.lock:
            result = dict(self.values)
            result["trail"] = list(self.trail)
            result["packets"] = self.packets
            result["link_live"] = bool(self.values["last_update"] and now - self.values["last_update"] <= STALE_SECONDS)
            result["age_seconds"] = None if self.values["last_update"] is None else round(now - self.values["last_update"], 1)
            result["field_age"] = dict((key, round(now - stamp, 1)) for key, stamp in self.times.items())
            return result

    def raw_snapshot(self):
        now = time.time()
        with self.lock:
            messages = {}
            for message_id, entry in self.raw_by_id.items():
                value = dict(entry)
                value["age_seconds"] = round(now - entry["received_at"], 1)
                messages[message_id] = value
            return {
                "schema": "buoy.telemetry.raw.v1",
                "generated_at": now,
                "link": {
                    "live": bool(self.last_rx and now - self.last_rx <= STALE_SECONDS),
                    "last_packet_at": self.last_rx,
                    "age_seconds": None if self.last_rx is None else round(now - self.last_rx, 1),
                    "udp_source": self.udp_source,
                    "datagrams_received": self.datagrams,
                    "bytes_received": self.bytes_received,
                    "frames_received": sum(self.message_counts.values()),
                    "checksum_failures": self.bad_checksums,
                    "valid_or_unknown_checksum_frames": self.packets,
                },
                "standard": self.standard,
                "buoy": self.buoy,
                "raw": {
                    "message_counts": self.message_counts,
                    "latest_by_message_id": messages,
                },
            }


PAGE = r'''<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<meta name="theme-color" content="#071923"><title>Buoy</title>
<style>
:root{color-scheme:dark;--bg:#071923;--panel:#102733;--muted:#91a8b2;--ink:#f2f7f8;--green:#59dfb1;--line:#28505c}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--ink);font:16px system-ui,-apple-system,sans-serif;min-height:100vh}
main{max-width:760px;margin:auto;padding:env(safe-area-inset-top) 14px env(safe-area-inset-bottom)}header{height:62px;display:flex;align-items:center;justify-content:space-between}
h1{font-size:20px;margin:0;letter-spacing:.02em}.status{font-size:12px;color:var(--muted);display:flex;align-items:center;gap:8px}.dot{width:9px;height:9px;background:#71838a;border-radius:50%}.live .dot{background:var(--green);box-shadow:0 0 12px #59dfb188}
.map{height:min(49vh,390px);min-height:280px;border:1px solid var(--line);border-radius:18px;position:relative;overflow:hidden;background-color:#0b202a;background-image:linear-gradient(#25465177 1px,transparent 1px),linear-gradient(90deg,#25465177 1px,transparent 1px);background-size:48px 48px}
.map:after{content:"";position:absolute;inset:0;background:radial-gradient(ellipse at center,transparent 5%,#07192344 90%);pointer-events:none}.map-title{position:absolute;z-index:2;top:13px;left:14px;color:#a8bec7;font-size:11px;letter-spacing:.1em;text-transform:uppercase}
svg{position:absolute;inset:0;width:100%;height:100%}.trail{fill:none;stroke:#59dfb188;stroke-width:2;vector-effect:non-scaling-stroke}.marker{fill:var(--green);stroke:#e8fff7;stroke-width:2;vector-effect:non-scaling-stroke;filter:drop-shadow(0 0 8px #59dfb1)}
.coords{position:absolute;z-index:2;bottom:12px;left:14px;font:12px ui-monospace,monospace;color:#d1e1e6}.mapnote{position:absolute;z-index:2;bottom:12px;right:14px;color:var(--muted);font-size:10px}
.cards{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:10px;margin-top:12px}.card{background:var(--panel);border:1px solid #1c3b47;border-radius:15px;padding:15px;min-height:92px}.label{font-size:11px;color:var(--muted);text-transform:uppercase;letter-spacing:.1em}.value{font-size:27px;font-weight:650;margin-top:5px;font-variant-numeric:tabular-nums}.unit{font-size:13px;color:var(--muted);font-weight:450}.foot{display:flex;justify-content:space-between;color:var(--muted);font-size:11px;padding:14px 2px}.empty{position:absolute;inset:0;display:grid;place-items:center;color:#9bb1ba;text-align:center;font-size:13px;z-index:1;padding:30px}.hidden{display:none}
</style></head><body><main><header><h1>BUOY <span style="color:var(--muted);font-weight:400">· FIELD</span></h1><div id="status" class="status"><span class="dot"></span><span id="link">WAITING</span></div></header>
<section class="map" aria-label="Buoy position plot"><div class="map-title">Position · local track</div><div id="empty" class="empty">Waiting for GPS position</div><svg viewBox="0 0 100 100" preserveAspectRatio="none"><polyline id="trail" class="trail" points=""></polyline><circle id="marker" class="marker hidden" cx="50" cy="50" r="2.1"></circle></svg><div class="coords" id="coords">--</div><div class="mapnote">Grid plot · no map tiles</div></section>
<section class="cards"><div class="card"><div class="label">Battery</div><div class="value"><span id="battery">--</span><span class="unit" id="batteryUnit"> %</span></div></div><div class="card"><div class="label">Speed</div><div class="value"><span id="speed">--</span><span class="unit"> m/s</span></div></div><div class="card"><div class="label">Heading</div><div class="value"><span id="heading">--</span><span class="unit"> °</span></div></div><div class="card"><div class="label">Vehicle</div><div class="value" style="font-size:20px" id="armed">--</div></div></section>
<div class="foot"><span id="updated">No data received</span><span id="voltage"></span></div></main>
<script>
const $=id=>document.getElementById(id), finite=x=>typeof x==='number'&&Number.isFinite(x), fmt=(x,n=1)=>finite(x)?x.toFixed(n):'--';
function plot(d){if(!d.position){$('empty').classList.remove('hidden');return}$('empty').classList.add('hidden');$('marker').classList.remove('hidden');$('coords').textContent=d.position.lat.toFixed(6)+', '+d.position.lon.toFixed(6);
let pts=d.trail||[], last=pts[pts.length-1]||d.position, spanLat=Math.max(0.00035,...pts.map(p=>Math.abs(p.lat-last.lat)*2.5)), spanLon=Math.max(0.00045,...pts.map(p=>Math.abs(p.lon-last.lon)*2.5));let cx=p=>50+(p.lon-last.lon)/spanLon*42,cy=p=>50-(p.lat-last.lat)/spanLat*42;$('trail').setAttribute('points',pts.map(p=>cx(p)+','+cy(p)).join(' '));$('marker').setAttribute('cx',cx(last));$('marker').setAttribute('cy',cy(last))}
function render(d){$('status').classList.toggle('live',d.link_live);$('link').textContent=d.link_live?'LIVE':'STALE';$('battery').textContent=finite(d.battery_percent)?Math.round(d.battery_percent):'--';$('batteryUnit').textContent=finite(d.battery_percent)?' %':(finite(d.voltage_v)?' V':'');if(!finite(d.battery_percent)&&finite(d.voltage_v))$('battery').textContent=d.voltage_v.toFixed(1);$('speed').textContent=fmt(d.speed_mps,1);$('heading').textContent=finite(d.heading_deg)?Math.round(d.heading_deg%360).toString().padStart(3,'0'):'--';$('armed').textContent=d.armed===null?'--':d.armed?'ARMED':'DISARMED';$('updated').textContent=d.age_seconds===null?'No data received':'Last update '+d.age_seconds.toFixed(1)+' s ago';$('voltage').textContent=finite(d.voltage_v)?d.voltage_v.toFixed(1)+' V':'';plot(d)}
async function poll(){try{let r=await fetch('/api/state',{cache:'no-store'});if(!r.ok)throw Error();render(await r.json())}catch(e){$('link').textContent='BRIDGE OFFLINE';$('status').classList.remove('live')}}poll();setInterval(poll,1000);
</script></body></html>'''


class Handler(BaseHTTPRequestHandler):
    state = None

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            body = PAGE.encode("utf-8")
            content_type = "text/html; charset=utf-8"
        elif self.path == "/api/state":
            body = json.dumps(json_safe(self.state.snapshot()), separators=(",", ":"), allow_nan=False).encode("utf-8")
            content_type = "application/json; charset=utf-8"
        elif self.path in ("/api/raw", "/api/telemetry"):
            body = json.dumps(json_safe(self.state.raw_snapshot()), separators=(",", ":"), allow_nan=False).encode("utf-8")
            content_type = "application/json; charset=utf-8"
        else:
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Connection", "close")
        self.end_headers()
        self.wfile.write(body)

    def do_OPTIONS(self):
        self.send_response(204)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.send_header("Access-Control-Max-Age", "86400")
        self.end_headers()

    def log_message(self, fmt, *args):
        return


class ThreadingHTTPServer36(ThreadingMixIn, HTTPServer):
    daemon_threads = True


def receive(state, sock):
    while True:
        data, source = sock.recvfrom(65535)
        state.note_datagram(data, source)
        for packet in frames(data):
            state.update(packet, source)


def main():
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--udp-host", default=UDP_HOST)
    parser.add_argument("--udp-port", type=int, default=UDP_PORT)
    parser.add_argument("--http-host", default=HTTP_HOST)
    parser.add_argument("--http-port", type=int, default=HTTP_PORT)
    args = parser.parse_args()
    state = State()
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    udp_socket.bind((args.udp_host, args.udp_port))
    print("MAVLink UDP listening on %s:%d" % (args.udp_host, args.udp_port), flush=True)
    thread = threading.Thread(target=receive, args=(state, udp_socket))
    thread.daemon = True
    thread.start()
    Handler.state = state
    server = ThreadingHTTPServer36((args.http_host, args.http_port), Handler)
    print("Dashboard HTTP listening on %s:%d" % (args.http_host, args.http_port), flush=True)
    server.serve_forever()


if __name__ == "__main__":
    main()
