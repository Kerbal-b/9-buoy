# Router MAVLink dashboard bridge

This is a receive-only, Python 3.6-compatible LAN bridge for the GL.iNet Opal / OpenWrt. It uses only the Python standard library. The dashboard polls `/api/state`; a second application can consume the detailed JSON stream at `/api/raw` (alias `/api/telemetry`). The raw API is read-only and CORS-enabled for browser clients. It has no tile dependency; the map panel is an offline position/trail grid with GPS coordinates.

## Raw feed contract

See the complete [API field reference](API.md) for field types, units, sentinels, enums, custom value names, and compact dashboard fields.

`GET http://192.168.8.1:8080/api/raw` returns `schema: "buoy.telemetry.raw.v1"` with:

- `link`: receive age/live state, UDP sender IP/port, datagram/frame/byte counts and checksum failures.
- `standard`: latest decoded values grouped by MAVLink message name. Each value includes its receive time. This object is an allow-list of decoded fields, not a claim that every MAVLink common.xml message is decoded.
- `buoy`: buoy-specific state, mission progress, home/assist/audio/calibration data, named values, and science samples. Unknown named values are retained by name.
- `raw.latest_by_message_id`: one latest complete frame per observed message ID, including MAVLink version, source IDs, sequence, checksum status, payload/frame hex and decoded standard/custom fields. `raw.message_counts` gives counts by ID. Unknown message IDs remain available here even when the bridge has no decoder for them.

Clients should ignore fields they do not use and tolerate absent/null values. The bridge does not store history; raw latest-frame entries are replaced as new frames arrive. Unknown-message checksums are `null` because their CRC_EXTRA is not known. A known message with a bad checksum is retained for diagnosis but not decoded.

Example:

```sh
curl http://192.168.8.1:8080/api/raw
```

## What the wire currently provides

Standard MAVLink messages decoded in this bridge:

| Message | Data mapped |
| --- | --- |
| `HEARTBEAT` | vehicle/autopilot type, base/custom mode, system status, armed flag |
| `SYS_STATUS` | battery voltage/current/remaining percentage |
| `BATTERY_STATUS` | consumed charge/energy, temperature, cell voltages, current, remaining percentage |
| `GLOBAL_POSITION_INT` | latitude/longitude/altitude, N/E/D velocity, speed, heading |
| `GPS_RAW_INT` | coordinates, altitude, GPS accuracy, speed/course, fix and satellites |
| `ATTITUDE` | roll/pitch/yaw and rates |
| `VFR_HUD` | air/ground speed, heading, throttle, altitude, climb |
| `HOME_POSITION` | home coordinates, altitude, local position, approach vector |
| `RADIO_STATUS` | radio errors, RSSI/noise, transmit buffer and derived link quality |
| `RC_CHANNELS_RAW`, `RC_CHANNELS` | channel values and RSSI |
| `STATUSTEXT` | severity and text; buoy compact status records are additionally interpreted |
| `NAMED_VALUE_INT`, `NAMED_VALUE_FLOAT` | name/value pairs, including unrecognized custom names |

The firmware's standard MAVLink messages describe the vehicle and its telemetry. Buoy-only information is carried inside standard `NAMED_VALUE_*` messages and structured `STATUSTEXT`, so a generic MAVLink UI may display the packet types but will not know the meanings without the buoy mapping. Current custom mappings include:

- `BUOY_STATE`: buoy mode (`IDLE`, `MANUAL`, `HOLD`, `GOTO`), control source (`NONE`, `WIFI`, `TX12`), armed/recording/recovery flags, and three signed motor PWM outputs.
- `BUOY_AUDIO`, `HOME_KIND`: audio recording state and home source.
- `B1,...` status text: armed/assist state, mission state and percentage, current waypoint/count, science/audio recording, heading, home distance, GPS readiness, home source and mission start-block reason.
- `BC1,...` status text: calibration state, step, power, maximum power and duration.
- Named float samples may include `WATER_TEMP`, `DEPTH_M`, `TDS_V`, `TURBIDITY`, `HOME_DIST`, `DIST_HOME`, `MIS_STATE`, `MIS_BLOCK`, `MIS_PCT`, `MIS_COUNT`, `DIST_WP`, `MIS_WP`, `MAG_HEAD`, `TILT_HEAD`, `SEA_ACC`, `SEA_GYR`, `MAN_AST`, `CAL_STAT`, `CAL_STEP`, `CAL_PWR`, `CAL_MAX`, and `CAL_DUR`. Which samples arrive depends on the Backpack telemetry profile/rate; the raw feed keeps names dynamically rather than requiring UI changes.

Mission telemetry is not the same as mission configuration. The current Backpack stream includes limited mission progress/status and distances, but does not transmit the mission ID, complete waypoint route, boundary polygon, or all mission settings. Those are stored/configured on the buoy via its Wi-Fi/SD workflow; a receiving app cannot reconstruct them from this feed. Supporting full mission transfer needs a separate wire protocol implementation.

The custom codes are defined by this buoy firmware and can evolve independently of MAVLink. Consumers should use the numeric `*_code` fields alongside the human-readable labels, and preserve unrecognized named values.

## Ports and Backpack forwarding

The control station in this repository binds UDP `14550` for ELRS Backpack telemetry (`ELRS_MAVLINK_UDP_PORT` in `src/control_station/station/qt_app.py`). The bridge defaults to the same port. It does not send a heartbeat, parameter request, or other MAVLink packet, so it is not a second ground station and does not add ELRS RF traffic.

The Backpack's UDP `/config` endpoint is queried by `src/control_station/diagnose_backpack.py`. Its `ports.listen` value (the diagnostic fallback is `14555`) is the Backpack's inbound UDP port, not proof of the outgoing telemetry destination. The repository does not establish whether this Backpack currently sends to a unicast GCS address or broadcasts. On the Opal, inspect `http://192.168.8.134/config` and `/mavlink`; then capture packets on UDP 14550 while the PC QGC is connected. A unicast stream addressed only to the PC will not reach this router service. Configure Backpack forwarding to the router's LAN broadcast address or a separate telemetry destination if the firmware supports it. Another option is to enable QGC's MAVLink forwarding from the PC to the Opal; that path carries packets over Wi-Fi and does not make the bridge emit MAVLink. Do not run another QGC instance on the phone.

Do not leave the QGC PC and bridge competing for a single learned-unicast destination. For simultaneous QGC and dashboard use, packet delivery must reach both destinations (broadcast or QGC forwarding). The bridge only listens and does not change the radio link's message request rate.

## Install on the router

From a computer connected to the Opal, with SSH enabled, copy the two files:

```sh
ssh root@192.168.8.1 'mkdir -p /root/mavlink-bridge'
scp mavlink_bridge.py root@192.168.8.1:/root/mavlink-bridge/
scp mavlink-bridge.init root@192.168.8.1:/etc/init.d/mavlink-bridge
ssh root@192.168.8.1 'chmod 755 /etc/init.d/mavlink-bridge && /etc/init.d/mavlink-bridge enable && /etc/init.d/mavlink-bridge start'
```

The Opal has Python 3.6 at `/usr/bin/python3`. The service binds UDP `0.0.0.0:14550` and HTTP `0.0.0.0:8080`. The phone should open `http://192.168.8.1:8080` while joined to the buoy network. The Opal also exposes a root ttyd terminal at `http://192.168.8.1:7681/`; this was used to deploy the files because SSH key authentication was rejected. For a foreground diagnostic run, stop the service and execute:

```sh
/etc/init.d/mavlink-bridge stop
/usr/bin/python3 /root/mavlink-bridge/mavlink_bridge.py
```

## Dashboard fields and decoding

- Position: `GLOBAL_POSITION_INT`, falling back to `GPS_RAW_INT` when its fix is valid.
- Speed: `VFR_HUD`, then `GLOBAL_POSITION_INT`, then `GPS_RAW_INT`.
- Heading: `VFR_HUD`, then GPS course from `GLOBAL_POSITION_INT`, then `ATTITUDE` yaw.
- Battery percentage and voltage: `SYS_STATUS`.
- Armed state and vehicle heartbeat status: `HEARTBEAT`.

Known messages with valid MAVLink v1/v2 checksums update decoded state. Stale status is shown after eight seconds without a received telemetry frame. This version does not issue commands and does not implement parameter or mission configuration.

## Deployment and verification (November 8, 2026)

The bridge and service are installed on the Opal at `/root/mavlink-bridge/mavlink_bridge.py` and `/etc/init.d/mavlink-bridge`. The updated service passed the router's Python 3.6 compile check and restarted. From the PC, `/api/raw` returned HTTP 200 with schema `buoy.telemetry.raw.v1`, live packets from `192.168.8.134`, standard and buoy fields, and raw MAVLink frames; `/api/state` also returned HTTP 200. The observed stream included buoy state/audio, mission progress (idle, waypoint count 5), science samples, attitude, radio and RC channel telemetry. No `GPS_RAW_INT` or `GLOBAL_POSITION_INT` was present in that sample, so the feed did not have a live position then. Active mission, live position, phone rendering, and simultaneous QGC operation still need field verification. No offline map tiles are bundled.
