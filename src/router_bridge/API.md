# Buoy telemetry API reference

This document describes the JSON served by the receive-only router bridge. The implementation is in [`mavlink_bridge.py`](mavlink_bridge.py); buoy-specific wire fields are defined by the ESP32 firmware and may change independently of MAVLink.

## Endpoints and behavior

| Method and path | Response |
| --- | --- |
| `GET /api/raw` | Full snapshot, schema `buoy.telemetry.raw.v1` |
| `GET /api/telemetry` | Alias for `/api/raw` |
| `GET /api/state` | Compact snapshot for the built-in dashboard |
| `OPTIONS` on these endpoints | CORS preflight response |

The JSON endpoints return `application/json`, disable caching, and allow cross-origin browser reads (`Access-Control-Allow-Origin: *`). Poll them as often as needed; the bridge does not push SSE or WebSocket updates. One-second polling is reasonable for this low-rate feed. All receive times are Unix epoch seconds in UTC. `age_seconds` values are floating point seconds.

The bridge keeps the latest decoded value for each known data group and latest frame for each observed message ID. It does not keep a time series. Values are omitted until first received; unavailable numeric values are generally `null`. Non-finite sensor numbers are serialized as `null`. Clients should ignore fields they do not use and tolerate unknown fields, missing fields, unknown enum codes and unknown message IDs.

Example request:

```sh
curl http://192.168.8.1:8080/api/raw
```

## `/api/raw` top-level fields

| Field | Type | Meaning / possible values |
| --- | --- | --- |
| `schema` | string | Versioned shape identifier; currently `buoy.telemetry.raw.v1`. |
| `generated_at` | number | Unix epoch seconds when this response snapshot was assembled. |
| `link` | object | UDP receive and frame counters described below. `live` becomes false after more than 8 seconds without a complete MAVLink frame. |
| `standard` | object | Latest decoded standard MAVLink message groups. Groups appear only after a frame for that message has been decoded. |
| `buoy` | object | Latest buoy-specific status and custom named values. Fields vary with the buoy's telemetry profile and operating state. |
| `raw` | object | Per-message counters and one latest raw MAVLink frame per observed message ID. |

### `link`

| Field | Type | Meaning / possible values |
| --- | --- | --- |
| `live` | boolean | A complete MAVLink frame arrived within the last 8 seconds. This indicates packets reach the router, not that GPS has a fix or the radio link is healthy. |
| `last_packet_at` | number or null | Epoch seconds for the most recent complete frame, including frames with invalid checksums. |
| `age_seconds` | number or null | Age of `last_packet_at`; null before any frame. |
| `udp_source` | object or null | Source of the most recent datagram: `ip` string and `port` integer. This is the Backpack/network sender, not a MAVLink sysid. |
| `datagrams_received` | integer | UDP datagrams received, whether or not they contain a complete MAVLink frame. |
| `bytes_received` | integer | Total UDP datagram bytes received since process start. |
| `frames_received` | integer | Complete frames parsed, including frames with known bad checksums. |
| `checksum_failures` | integer | Frames with a known CRC_EXTRA whose checksum did not match. |
| `valid_or_unknown_checksum_frames` | integer | Frames not identified as checksum failures. Includes valid known messages and unknown IDs whose CRC_EXTRA is not known. |

### `standard`

Every decoded group below has `received_at`, the epoch seconds when that message was last received. The bridge decodes the listed subset of each message; raw payload/frame bytes remain available for additional fields. Units are SI unless the field name says otherwise.

#### `standard.heartbeat` — MAVLink `HEARTBEAT` (ID 0)

| Field | Type | Meaning / possible values |
| --- | --- | --- |
| `custom_mode` | integer | Autopilot-specific mode value. This buoy currently reports 0; no buoy mode interpretation is applied here. |
| `vehicle_type` | integer | MAV_TYPE; buoy firmware uses 11 (`SURFACE_BOAT`). |
| `autopilot` | integer | MAV_AUTOPILOT; firmware uses 8 (`INVALID` / no standard autopilot stack). |
| `base_mode` | integer | MAV_MODE_FLAG bitmask: bit 0 custom mode enabled (`0x01`); bit 1 test (`0x02`); bit 2 auto (`0x04`); bit 3 guided (`0x08`); bit 4 stabilize (`0x10`); bit 5 hardware-in-loop (`0x20`); bit 6 manual input enabled (`0x40`); bit 7 safety armed (`0x80`). |
| `system_status` | integer | MAV_STATE: 0 uninitialized, 1 booting, 2 calibrating, 3 standby, 4 active, 5 critical, 6 emergency, 7 poweroff, 8 flight termination; other values are reserved/unknown. |
| `mavlink_version` | integer | MAVLink protocol marker; normally 3. |
| `armed` | boolean | Derived from `base_mode & 0x80`. |

#### `standard.sys_status` — MAVLink `SYS_STATUS` (ID 1)

| Field | Type | Meaning / possible values |
| --- | --- | --- |
| `battery_voltage_v` | number or null | Battery voltage in volts; null for MAVLink's `0xffff` unknown sentinel. |
| `battery_current_a` | number or null | Battery current in amperes; source is centiamps; null for `-1` unknown. |
| `battery_remaining_pct` | integer or null | Estimated remaining percentage, normally 0–100; null when firmware reports a negative unknown sentinel. |
| `received_at` | number | Last update time. |

#### `standard.battery_status` — MAVLink `BATTERY_STATUS` (ID 147)

| Field | Type | Meaning / possible values |
| --- | --- | --- |
| `current_consumed_mah` | integer or null | Consumed charge in mAh; null if source is negative/unknown. |
| `energy_consumed_hj` | integer or null | Consumed energy in hectojoules; null if negative/unknown. |
| `temperature_c` | number or null | Battery temperature in °C; null for MAVLink sentinel 32767. |
| `cell_voltages_v` | array of 10 numbers/nulls | Cell voltages in volts; an individual `null` means its source value was `0xffff` (unknown/unpopulated). |
| `current_a` | number or null | Battery current in amperes; null for `-1` unknown. |
| `battery_id` | integer | MAVLink battery instance ID. |
| `battery_function` | integer | MAV_BATTERY_FUNCTION: 0 unknown, 1 all, 2 propulsion, 3 avionics, 4 payload, 5 camera, 6 gimbal, 7 rover drive, 8 winch; unknown/reserved values are preserved. |
| `battery_type` | integer | MAV_BATTERY_TYPE: 0 unknown, 1 LiPo, 2 Li-ion, 3 LiFePO4, 4 NiMH, 5 LiPo high-voltage; unknown/reserved values are preserved. |
| `battery_remaining_pct` | integer or null | Estimated remaining percentage; null when source is negative/unknown. |
| `received_at` | number | Last update time. |

#### `standard.global_position` — MAVLink `GLOBAL_POSITION_INT` (ID 33)

| Field | Type | Meaning / possible values |
| --- | --- | --- |
| `time_boot_ms` | integer | Milliseconds since vehicle boot (wraps per MAVLink field width). |
| `latitude_deg`, `longitude_deg` | number or null | WGS84 coordinates in degrees; null when source coordinate is zero. |
| `altitude_m`, `relative_altitude_m` | number | Altitude above mean sea level and relative altitude, in metres. |
| `velocity_north_mps`, `velocity_east_mps`, `velocity_down_mps` | number | N/E/D velocity in metres per second. Positive down is downward. |
| `ground_speed_mps` | number | Horizontal speed calculated as `sqrt(vx² + vy²)`, in m/s. |
| `heading_deg` | number or null | Heading in degrees, 0–359.99; null for `0xffff` unknown. |
| `received_at` | number | Last update time. |

#### `standard.gps_raw` — MAVLink `GPS_RAW_INT` (ID 24)

| Field | Type | Meaning / possible values |
| --- | --- | --- |
| `time_usec` | integer | GPS time in microseconds as supplied by firmware. |
| `latitude_deg`, `longitude_deg` | number or null | WGS84 coordinates in degrees; null when source coordinate is zero. |
| `altitude_m` | number | GPS altitude in metres. |
| `hdop`, `vdop` | number or null | Horizontal/vertical dilution of precision, decoded from MAVLink `eph`/`epv` divided by 100; null when the wire value is `0xffff`. These are dimensionless DOP values, not metres. |
| `eph_m`, `epv_m` | number or null | Deprecated compatibility aliases for `hdop`/`vdop`. Despite the legacy `_m` names, the values are dimensionless DOP values, not metres. |
| `ground_speed_mps` | number or null | GPS ground speed in m/s; null for `0xffff`. |
| `course_deg` | number or null | Course over ground in degrees; null for `0xffff`. |
| `fix_type` | integer | MAV GPS_FIX_TYPE: 0 no GPS, 1 no fix, 2 2D, 3 3D, 4 DGPS, 5 RTK float, 6 RTK fixed, 7 static, 8 PPP; values above are preserved if received. |
| `fix_valid` | boolean | Derived as `fix_type >= 2`. |
| `satellites_visible` | integer or null | Satellite count; null for 255 (unknown). |
| `received_at` | number | Last update time. |

#### `standard.attitude` — MAVLink `ATTITUDE` (ID 30)

| Field | Type | Meaning / possible values |
| --- | --- | --- |
| `time_boot_ms` | integer | Milliseconds since boot. |
| `roll_rad`, `pitch_rad`, `yaw_rad` | number | Euler angles in radians. |
| `roll_deg`, `pitch_deg`, `yaw_deg` | number | Same angles in degrees; yaw is normalized to 0–360. |
| `rollspeed_rad_s`, `pitchspeed_rad_s`, `yawspeed_rad_s` | number | Body angular rates in radians per second. |
| `received_at` | number | Last update time. |

#### `standard.vfr_hud` — MAVLink `VFR_HUD` (ID 74)

| Field | Type | Meaning / possible values |
| --- | --- | --- |
| `airspeed_mps`, `ground_speed_mps` | number | Airspeed and ground speed in m/s. Non-finite source values are returned as null. |
| `heading_deg` | integer or null | Heading in degrees; null for negative source value. |
| `throttle_pct` | integer | Throttle field as sent; nominally 0–100. |
| `altitude_m` | number | Altitude in metres. |
| `climb_mps` | number | Climb rate in m/s. |
| `received_at` | number | Last update time. |

#### `standard.home_position` — MAVLink `HOME_POSITION` (ID 242)

| Field | Type | Meaning / possible values |
| --- | --- | --- |
| `latitude_deg`, `longitude_deg` | number or null | Home WGS84 coordinates; null when source coordinate is zero. This is **home**, not the current vehicle position. |
| `altitude_m` | number | Home altitude in metres. |
| `local_position_m` | array of 3 numbers | Local X/Y/Z home position in metres. The buoy currently transmits zeros because this vector is unavailable. |
| `orientation_quaternion` | array of 4 numbers | Home orientation quaternion `[w, x, y, z]` as carried by MAVLink. |
| `approach_vector_m` | array of 3 numbers | Approach vector in metres. The buoy currently transmits zeros because this vector is unavailable. |
| `time_usec` | integer or null | MAVLink 2 extension timestamp; null if the frame does not include the extension. |
| `received_at` | number | Last update time. |

#### `standard.radio_status` — MAVLink `RADIO_STATUS` (ID 109)

| Field | Type | Meaning / possible values |
| --- | --- | --- |
| `rx_errors` | integer | Cumulative receive error count. |
| `corrected_packets` | integer | Corrected packet count. |
| `rssi_raw` | integer or null | Raw RSSI byte; null for 255 unknown. Scale is radio-specific. |
| `remote_rssi_dbm_magnitude` | integer or null | Remote RSSI byte as magnitude; null for 255. The bridge does not convert it to signed dBm. |
| `tx_buffer_free_pct` | integer | Transmit buffer free percentage as sent. |
| `noise_db`, `remote_noise_db` | integer or null | Signed noise bytes; null for 255 unknown. |
| `uplink_link_quality_pct` | number or null | Approximation `rssi_raw * 100 / 255`; null when RSSI is unknown. It is not an ELRS link-quality measurement. |
| `received_at` | number | Last update time. |

#### `standard.rc_channels_raw` — MAVLink `RC_CHANNELS_RAW` (ID 35)

| Field | Type | Meaning / possible values |
| --- | --- | --- |
| `time_boot_ms` | integer | Milliseconds since boot. |
| `bank` | integer | Firmware bank index. |
| `first_channel` | integer | First channel number in this bank, calculated as `bank * 8 + 1`. |
| `channels_us` | array of 8 integers | Raw RC channel values in microseconds; MAVLink sentinel values such as 0 or 65535 may occur. |
| `rssi` | integer | Raw RSSI byte; 0–254 nominal, 255 unknown. |
| `received_at` | number | Last update time. |

#### `standard.rc_channels` — MAVLink `RC_CHANNELS` (ID 65)

| Field | Type | Meaning / possible values |
| --- | --- | --- |
| `time_boot_ms` | integer | Milliseconds since boot. |
| `count` | integer | Number of channel entries included, capped by this decoder at 18. |
| `channels_us` | array of 0–18 integers | Raw channel values in microseconds; sentinel values may occur. |
| `rssi` | integer | Raw RSSI byte; 255 means unknown. |
| `received_at` | number | Last update time. |

#### `standard.status_text` — MAVLink `STATUSTEXT` (ID 253)

| Field | Type | Meaning / possible values |
| --- | --- | --- |
| `severity` | integer | MAV_SEVERITY: 0 emergency, 1 alert, 2 critical, 3 error, 4 warning, 5 notice, 6 info, 7 debug. |
| `text` | string | UTF-8 text, truncated at the MAVLink field boundary and NUL terminator. `B1,...` and `BC1,...` formats are additionally decoded into `buoy`. |
| `chunk_seq` | integer or null | MAVLink 2 chunk sequence extension; null if absent. |
| `id` | integer or null | MAVLink 2 text ID extension; null if absent. |
| `received_at` | number | Last update time. |

### `buoy`

All decoded buoy groups are optional and may be empty before their first message. Group field timestamps identify the latest source update. The firmware sends the core buoy state/audio and compact `B1` status in both radio telemetry profiles; rotating science/calibration values and additional RC/radio detail depend on the selected profile and activity.

#### `buoy.state` — custom `BUOY_STATE` named integer

| Field | Type | Meaning / possible values |
| --- | --- | --- |
| `time_boot_ms` | integer | ESP32 uptime in milliseconds. |
| `mode_code`, `mode` | integer, string | 0 `IDLE`, 1 `MANUAL`, 2 `HOLD`, 3 `GOTO`; other codes map to `UNKNOWN`. |
| `control_source_code`, `control_source` | integer, string | 0 `NONE`, 1 `WIFI`, 2 `TX12`; other codes map to `UNKNOWN`. |
| `armed` | boolean | Buoy arming flag. |
| `science_recording` | boolean | Science session recording flag. |
| `recovery_stop` | boolean | Link-loss recovery/stop status flag. |
| `motor_pwm_signed_127` | array of 3 integers | Applied motor output scaled to signed -127…127, order `[rear, left, right]`. Zero is neutral; sign is direction. |
| `motor_pwm_pct` | array of 3 numbers | Same outputs scaled to percentage -100…100, same order. |
| `received_at` | number | Last update time. |

`BUOY_STATE` is a firmware-specific payload placed in standard `NAMED_VALUE_INT`. The flags byte stores mode in bits 0–2, source in bits 3–4, armed in bit 5, science recording in bit 6, and recovery stop in bit 7.

#### `buoy.mission`

Mission progress is sourced primarily from the `B1` compact status text and supplemented when rotating named fields arrive. It does not contain the mission route/configuration.

| Field | Type | Meaning / possible values |
| --- | --- | --- |
| `mission_state_code`, `state` | integer, string | 0 `IDLE`, 1 `STARTING`, 2 `MOVING`, 3 `COLLECTING`, 4 `RETURNING_HOME`, 5 `STOPPED`, 6 `COMPLETE`, 7 `RETURNED_HOME`, 8 `BLOCKED`; other codes map to `UNKNOWN`. |
| `mission_percent` | number | Reported mission progress percentage, normally 0–100. |
| `current_waypoint` | integer | Firmware waypoint ordinal; 0 means no active waypoint. Active waypoint values are one-based. |
| `current_waypoint_index` | integer | Zero-based convenience value: `max(0, current_waypoint - 1)`. Since zero also maps to zero, check `current_waypoint`/mission state before treating this as active. |
| `waypoint_count` | integer | Number of route waypoints reported by firmware. It does not provide the waypoint coordinates. |
| `home_distance_m` | number or null | Distance to Home in metres; null when firmware reports unavailable (compact status uses a negative sentinel). |
| `gps_fix` | boolean | Compact firmware readiness/fix flag; not a substitute for standard `gps_raw.fix_type` or satellite/HDOP details. |
| `mission_block_code` | integer | Most recent mission start-block code. 0 means no block reason. |
| `start_block_reason` | string | Human-readable text for codes 0–16 listed below; unknown codes map to `Unknown start interlock`. |
| `received_at` | number | Last mission/compact-status update time. |

Mission block codes: 0 none; 1 cycle arm switch low then high; 2 power minimum and sticks centered; 3 Wi-Fi control owns buoy; 4 mission setup not ready; 5 GPS fix requires at least 5 satellites and HDOP ≤3.0; 6 outside boundary; 7 within 5 m of Home required; 8 compass calibration/heading unavailable; 9 select Manual then Mission; 10 V4 mission with Home and boundary required; 11 SD unavailable; 12 stop/save prior session; 13 route point/speed invalid; 14 arm switch fully high required; 15 service mode/SD transfer active; 16 mission SD session creation failed.

#### `buoy.home`

| Field | Type | Meaning / possible values |
| --- | --- | --- |
| `source_code`, `source` | integer, string | 0 `NONE`, 1 `MANUAL`, 2 `MISSION`; other codes map to `UNKNOWN`. |
| `distance_m` | number or null | Distance from buoy to active Home, metres. |
| `distance_to_waypoint_m` | number | Distance to active waypoint, metres, when `DIST_WP` is received. |
| `gps_fix` | boolean | Compact GPS readiness flag. |
| `received_at` | number | Last home-related update time. |

#### `buoy.assist`

| Field | Type | Meaning / possible values |
| --- | --- | --- |
| `mode_code`, `mode` | integer, string | 0 `DIRECT`, 1 `COMPASS_ASSIST`, 2 `ASSIST_NO_HEADING`, 3 `MISSION`; unknown codes map to `UNKNOWN`. |
| `received_at` | number | Last `B1` update time. |

#### `buoy.audio`

| Field | Type | Meaning / possible values |
| --- | --- | --- |
| `recording` | boolean | Microphone/audio recording state from `BUOY_AUDIO` or `B1`. |
| `received_at` | number | Last update time. |

#### `buoy.calibration`

Received from `BC1,<state>,<step>,<power>,<max_power>,<duration>` status text. Fields:

| Field | Type | Meaning / possible values |
| --- | --- | --- |
| `state_code` | integer | 0 not reported; 1 active; 2 stopped/incomplete; 3 completed; 4 start interlock blocked; 5 SD/log creation failed. |
| `step` | integer | Current/last calibration test step, one-based when meaningful. |
| `power_pct` | integer | Current requested/applied calibration power percentage. |
| `max_power_pct` | integer | Configured maximum power percentage for the calibration run. |
| `duration_s` | integer | Calibration pulse duration in seconds. |
| `received_at` | number | Last `BC1` update time. |

The group stays empty until `BC1` has been received; zero values in a present group are literal decoded values, not necessarily a missing-data marker.

#### `buoy.science` and `buoy.named_values`

These are dynamic maps keyed by the exact MAVLink `name` field. A sample has `{ "value": number, "received_at": epoch_seconds }`. Float samples appear under `science`; integer named values not otherwise specially decoded appear under `named_values`. An application must not assume every name is always present. Common names include:

| Name | Typical meaning / unit |
| --- | --- |
| `WATER_TEMP` | Water temperature, °C. |
| `DEPTH_M` | Water depth, metres. |
| `TDS_V` | TDS sensor output voltage, volts; not a calibrated TDS concentration. |
| `TURBIDITY` / `TURBIDITY_V` | Turbidity sensor output/voltage as emitted; voltage is not a calibrated turbidity unit. |
| `HOME_DIST`, `DIST_HOME` | Distance to Home, metres. |
| `DIST_WP` | Distance to current waypoint, metres. |
| `MIS_STATE` | Mission-state numeric code; same values as `buoy.mission.mission_state_code`. |
| `MIS_BLOCK` | Mission start-block numeric code; same values as `buoy.mission.mission_block_code`. |
| `MIS_PCT` | Mission progress, percent. |
| `MIS_COUNT` | Number of mission route waypoints. |
| `MIS_WP` | Current one-based waypoint ordinal. |
| `MAG_HEAD`, `TILT_HEAD` | Calibrated magnetic / tilt-corrected heading, degrees. |
| `SEA_ACC` | Dynamic acceleration RMS, g; relative hull motion, not wave height. |
| `SEA_GYR` | Angular-rate RMS, degrees/second; relative hull motion. |
| `MAN_AST` | Manual assist telemetry value; firmware-specific. |
| `CAL_STAT` | Calibration status code, see `buoy.calibration.state_code`. |
| `CAL_STEP` | Calibration step, one-based. |
| `CAL_PWR`, `CAL_MAX` | Current and maximum calibration power, percent. |
| `CAL_DUR` | Calibration pulse duration, seconds. |

Names not in this table are still returned. Verify a name's unit against the matching firmware version before interpreting it.

#### `buoy.compact_status`

Latest parsed `B1` status record, retained as the raw compact values. Fields: `armed` boolean; `assist_code` (see assist codes); `mission_state_code` (see mission state codes); `mission_percent` (normally 0–100); `current_waypoint` (zero means inactive, otherwise one-based); `waypoint_count`; `science_recording`, `audio_recording` booleans; `heading_deg` integer (-1 means unavailable); `home_distance_m` (null if firmware sends negative sentinel); `gps_fix` boolean; `home_source_code` (see home codes); `mission_block_code`; and `received_at`.

### `raw`

#### `raw.message_counts`

Object keyed by decimal MAVLink message ID strings (for example, `"0"` for HEARTBEAT). Each value is the number of complete frames parsed for that ID since process start. Counts include known bad-checksum frames.

#### `raw.latest_by_message_id`

Object keyed by decimal message ID string. The bridge stores at most 64 IDs; when a new 65th ID appears, the least recently received ID is evicted. Each value is:

| Field | Type | Meaning / possible values |
| --- | --- | --- |
| `message_id` | integer | MAVLink common/custom numeric message ID. |
| `message_name` | string | Known name from this bridge's mapping, otherwise `UNKNOWN_<id>`. |
| `version` | integer | MAVLink version, 1 or 2. |
| `sequence` | integer | MAVLink packet sequence byte (0–255). |
| `system_id`, `component_id` | integer | MAVLink source IDs (0–255). |
| `signed` | boolean | True when MAVLink 2 signing flag is set. Signature bytes are part of `frame_hex`; the signature is not cryptographically verified by this bridge. |
| `checksum_valid` | boolean or null | True/false when this bridge knows the message CRC_EXTRA; null for unknown IDs whose checksum cannot be checked. |
| `received_at` | number | Epoch seconds this frame was received. |
| `age_seconds` | number | Age of this latest frame at response time. |
| `payload_length` | integer | MAVLink payload byte count. |
| `payload_hex` | string | Payload bytes as lowercase hexadecimal, no separators. |
| `frame_hex` | string | Entire MAVLink frame as lowercase hexadecimal, including header, checksum, and optional signature. |
| `decoded` | object | Decoded standard fields for this message, or `{}` when not decoded/invalid. |
| `buoy_decoded` | object | Buoy-specific interpretation when this frame contains a recognized custom value/status, otherwise `{}`. |

Unknown IDs and messages the bridge does not decode are still available as raw frames. Only the latest frame for each ID is retained; use an external logger if message history is needed.

## `/api/state` compact dashboard fields

The state endpoint remains available for the built-in page. It is a flattened, convenience view and can combine fields arriving at different times.

| Field | Type | Meaning / possible values |
| --- | --- | --- |
| `position` | object or null | `{ "lat": degrees, "lon": degrees }` from `GLOBAL_POSITION_INT`, or valid-fix `GPS_RAW_INT`. |
| `battery_percent` | number or null | Latest remaining percentage from SYS_STATUS/BATTERY_STATUS. |
| `voltage_v` | number or null | Latest battery voltage from SYS_STATUS or sum of available cell voltages. |
| `speed_mps` | number or null | Latest decoded speed; can come from VFR_HUD, GLOBAL_POSITION_INT, or GPS_RAW_INT. |
| `heading_deg` | number or null | Latest decoded heading from VFR_HUD, position/GPS course, or ATTITUDE yaw. |
| `armed` | boolean or null | Latest armed state. |
| `system_status` | integer or null | MAV_STATE value from HEARTBEAT. |
| `last_update` | number or null | Epoch seconds of the latest parsed frame. |
| `source` | string or null | UDP source IP of the latest parsed frame. |
| `trail` | array | Up to 120 `{lat, lon}` positions, appended when valid coordinates arrive. |
| `packets` | integer | Frames not known to have checksum errors since process start. |
| `link_live` | boolean | A frame was received within the last 8 seconds. |
| `age_seconds` | number or null | Age of `last_update`. |
| `field_age` | object | Age in seconds of each flattened field's last update. |

## Data not currently present in this feed

The telemetry stream reports mission state/progress/count and some distances, but does not carry the mission identifier, complete waypoint coordinates/route, boundary polygon, or full mission settings. Those are uploaded/stored through the buoy's Wi-Fi/SD workflow. The API also does not provide command/control endpoints, parameter download, historical storage, or guaranteed map position: the active Backpack profile must actually forward `GLOBAL_POSITION_INT` or valid `GPS_RAW_INT` for current position to appear.
