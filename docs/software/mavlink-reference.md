# MAVLink and Buoy Telemetry Reference

This page describes MAVLink messages currently decoded by the router bridge, their standard payload field layouts, and the buoy-specific fields carried inside `STATUSTEXT`. The bridge implementation is [`../../src/router_bridge/mavlink_bridge.py`](../../src/router_bridge/mavlink_bridge.py). Exact JSON names, units, sentinels, and enum meanings are maintained in [`../../src/router_bridge/API.md`](../../src/router_bridge/API.md), which is the API contract for consumers.

## Transport and parser

- ESP32 production telemetry reaches the TX Backpack over the ELRS MAVLink receiver link; the Backpack forwards UDP to the configured GCS destination. The Opal listens on UDP `14550`; it does not originate MAVLink.
- The bridge accepts MAVLink v1 and v2 frames, decodes a selected message subset, retains the latest decoded value and latest raw frame per message ID, and counts frames. It does not store a time-series history.
- A full frame passing the framing parser is not necessarily CRC-verified. The bridge validates checksums for message IDs for which it has CRC_EXTRA values; unknown/unsupported message checksums are not authenticated. Refer to `link.checksum_failures` and raw frame metadata.
- The browser API is served at HTTP `8080`. `/api/raw` and `/api/telemetry` provide the raw snapshot; `/api/state` is a compact dashboard view. Polling and all field definitions are in the [router API contract](../../src/router_bridge/API.md).

## Decoded standard message inventory

The payload layouts below follow MAVLink standard field order (little-endian wire representation). They list the standard message payload, not the MAVLink frame header/signature. Extension fields may be absent in shorter frames. The bridge currently extracts the fields shown in the final column; unsupported fields remain available only in `raw` bytes.

| ID | Message | Standard payload field layout | Bridge extraction / use |
| ---: | --- | --- | --- |
| 0 | `HEARTBEAT` | `uint32 custom_mode`; `uint8 type`; `uint8 autopilot`; `uint8 base_mode`; `uint8 system_status`; `uint8 mavlink_version` | Type, autopilot, custom/base mode, state, version; armed derives from base-mode safety-armed bit. |
| 1 | `SYS_STATUS` | `uint32 onboard_control_sensors_present`; `uint32 ..._enabled`; `uint32 ..._health`; `uint16 load`; `uint16 voltage_battery` (mV); `int16 current_battery` (cA); `uint8 battery_remaining` (%); `uint16 drop_rate_comm`; `uint16 errors_comm`; `uint16 errors_count1..4` | Battery voltage, current, remaining percent. |
| 24 | `GPS_RAW_INT` | `uint64 time_usec`; `int32 lat`, `int32 lon` (deg × 1e7), `int32 alt` (mm); `uint16 eph`, `epv` (cm); `uint16 vel` (cm/s); `uint16 cog` (cdeg); `uint8 fix_type`; `uint8 satellites_visible`; MAVLink 2 extensions: `uint8 alt_ellipsoid` (mm), `uint32 h_acc`, `v_acc`, `vel_acc` (mm), `uint32 hdg_acc` (mdeg), `uint16 yaw` (cdeg) | Position, altitude, accuracy estimates, speed, course, fix type, satellite count. GPS quality details are available here when transmitted. |
| 30 | `ATTITUDE` | `uint32 time_boot_ms`; `float roll`, `pitch`, `yaw` (rad); `float rollspeed`, `pitchspeed`, `yawspeed` (rad/s) | Euler attitude and rates; yaw is attitude orientation, not necessarily calibrated compass heading. |
| 33 | `GLOBAL_POSITION_INT` | `uint32 time_boot_ms`; `int32 lat`, `int32 lon` (deg × 1e7); `int32 alt`, `relative_alt` (mm); `int16 vx`, `vy`, `vz` (cm/s, NED); `uint16 hdg` (cdeg; `UINT16_MAX` unknown) | Position, altitude, N/E/D velocity, derived horizontal speed, heading field. |
| 35 | `RC_CHANNELS_RAW` | `uint32 time_boot_ms`; `uint16 chan1_raw..chan8_raw`; `uint8 port`; `uint8 rssi` | Eight raw channel values, bank/port, RSSI byte. |
| 65 | `RC_CHANNELS` | `uint32 time_boot_ms`; `uint16 chan1_raw..chan18_raw`; `uint8 chancount`; `uint8 rssi` | Up to 18 raw channel values and count. |
| 74 | `VFR_HUD` | `float airspeed`; `float groundspeed`; `int16 heading` (deg); `uint16 throttle` (%); `float alt` (m); `float climb` (m/s) | Air/ground speed, heading, throttle, altitude, climb. |
| 109 | `RADIO_STATUS` | `uint16 rxerrors`; `uint16 fixed`; `uint8 rssi`; `uint8 remrssi`; `uint8 txbuf`; `uint8 noise`; `uint8 remnoise` | Radio counters and raw RSSI/noise fields. Bridge's derived percentage is only a byte-scale approximation, not ELRS link quality. |
| 147 | `BATTERY_STATUS` | `int32 current_consumed` (mAh); `int32 energy_consumed` (hJ); `int16 temperature` (cdeg C); `uint16 voltages[10]` (mV); `int16 current_battery` (cA); `uint8 id`; `uint8 battery_function`; `uint8 type`; `int8 battery_remaining`; extensions: `int32 time_remaining`; `uint8 charge_state`; `uint16 voltages_ext[4]`; `uint8 mode`; `uint32 fault_bitmask` | Consumption, energy, temperature, per-cell voltage, current, battery ID/type/function, remaining percent. Unknown sentinels become null. |
| 242 | `HOME_POSITION` | `int32 latitude`, `longitude` (deg × 1e7); `int32 altitude` (mm); `float x`, `y`, `z` (m); `float q[4]`; `float approach_x`, `approach_y`, `approach_z` (m); MAVLink 2 extension `uint64 time_usec` | Home coordinates and remaining standard fields. Home is distinct from live position. |
| 251 | `NAMED_VALUE_FLOAT` | `uint32 time_boot_ms`; `float value`; `char name[10]` | Custom buoy values such as named science/navigation/mission fields; see named-field section in `API.md`. |
| 253 | `STATUSTEXT` | `uint8 severity`; `char text[50]`; MAVLink 2 extensions `uint16 id`; `uint8 chunk_seq` | Diagnostic text plus buoy compact records `B1,...` and calibration `BC1,...`. |

The current source decoder requires a minimum payload length for each message; messages not listed above remain in the raw message inventory but are not converted to standard groups. The authoritative supported subset and JSON output field-by-field are in `API.md`.

## GPS position and heading interpretation

1. `GLOBAL_POSITION_INT` is the bridge's preferred current position when present. Its integer latitude/longitude values are divided by `1e7`; zero coordinates map to null in the bridge.
2. If no global-position message is in the same packet update, the bridge can use `GPS_RAW_INT` only when `fix_type >= 2`; it uses latitude/longitude from that message, and its groundspeed/course when available.
3. GPS satellite count is `GPS_RAW_INT.satellites_visible`; 255 means unknown. `fix_type` values are MAVLink GPS fix enums (0 no GPS through 6 RTK fixed, plus 7 static and 8 PPP). `eph_m`/`epv_m` are horizontal/vertical estimated errors if supplied, and `ground_speed_mps` and `course_deg` are GPS velocity/course values.
4. GPS course over ground is not compass heading. It becomes meaningful while moving and can differ from hull orientation due to current, drift, or reverse motion. `ATTITUDE.yaw` is orientation from the source's attitude estimate. `VFR_HUD.heading` and `GLOBAL_POSITION_INT.hdg` are separate transmitted fields. The bridge retains each source in `standard`; its compact dashboard `heading_deg` chooses the latest supported decoded heading source. Do not present a single heading value as “GPS” or “magnetometer” without labeling its source.
5. Mission readiness has stricter firmware-side GPS requirements than “fix valid”: fresh location, ≥5 satellites, HDOP ≤3.0, location inside the mission boundary, and within 30 m of planned Home. The live compact `B1` GPS flag alone does not expose all those details.

## Buoy-specific `STATUSTEXT` fields

The firmware uses standard `STATUSTEXT` text for compact telemetry transported through the Backpack. The bridge recognizes these versioned comma-separated records:

### `B1` compact state

Current payload text layout:

```text
B1,<armed>,<assist_code>,<mission_state>,<mission_percent>,<current_waypoint>,<waypoint_count>,<science_recording>,<audio_recording>,<heading_deg>,<home_distance_m>,<gps_fix>,<home_source>,<mission_block_code>
```

The 13 integer values after `B1,` are mapped in order to the `buoy.compact_status`, `buoy.mission`, `buoy.home`, and `buoy.assist` fields in `API.md`. Negative home distance and heading `-1` are unavailable sentinels. Mission/home/block enum values and codes are documented in that API reference. This is status only; the message does not include the complete mission route.

### `BC1` calibration state

```text
BC1,<state_code>,<step>,<power_pct>,<max_power_pct>,<duration_s>
```

This reports calibration progress; it does not itself contain calibration samples. The detailed sample files are stored on SD. Code mappings and field names are in `API.md`.

### Named custom values

`NAMED_VALUE_FLOAT` carries a 10-character name and a float. The firmware emits named science values, state/mission values, and sensor diagnostics according to telemetry mode and operating state. The API reference lists recognized names and their units/meanings. Values not recognized by the current bridge decoder may still appear as raw MAVLink frames. The full mission plan is not broadcast in telemetry.

## API and change control

Use [`API.md`](../../src/router_bridge/API.md) for exact JSON field names, possible values, source precedence, null/sentinel treatment, and endpoint limitations. Update this page when the firmware changes a MAVLink payload, custom status string, message frequency/routing, GPS parsing/source precedence, or the bridge's decoder. Update `API.md` at the same time when a JSON field changes. Firmware MAVLink pack/unpack helpers and `mavlink_bridge.py` are the implementation sources.
