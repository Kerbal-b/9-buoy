# Buoy Software Behavior Guide

This is the maintained operator and integration view of the software behavior. It describes the current source and deployed components, and calls out the difference between implemented behavior and behavior demonstrated in the water. Update this page when control flow, mission gates, logging, or system data paths change.

## End-to-end data paths

### Radio telemetry to the phone dashboard

```text
GPS / sensors / ESP32 firmware
        │ MAVLink telemetry over the production ELRS receiver UART
        ▼
SkyGuy Nano ELRS receiver ⇄ TX12 radio link ⇄ TX Backpack
                                              │ Wi-Fi MAVLink UDP forwarding
                                              ▼
                                  GL.iNet Opal UDP :14550
                                  mavlink_bridge.py (read-only)
                                              │ HTTP :8080, JSON polling
                                              ▼
                                  Phone browser dashboard
```

The firmware emits MAVLink through the receiver's UART2 link. The TX Backpack forwards the MAVLink stream over Wi-Fi. The Opal bridge listens on UDP `14550`, decodes standard and buoy-specific telemetry, and serves the page and JSON API over HTTP `8080`. The bridge does not send MAVLink heartbeats, parameter requests, or control commands. The browser polls JSON and cannot issue buoy controls through this service. The dashboard is intended to work on the local `Bouy` network without internet access; its map is an offline grid, not cached geographic tiles.

The bridge, API, and iPhone page were deployed/viewed on November 8, 2026. A packet sample from the Backpack at `192.168.8.134` contained state, science, attitude, radio, and RC data, but not GPS position. GPS display and PC QGroundControl (QGC) operating simultaneously with the bridge have not yet been verified. Backpack destination behavior (unicast versus broadcast) remains a configuration item; see [`src/router_bridge/README.md`](../../src/router_bridge/README.md). Use [`API.md`](../../src/router_bridge/API.md) for the authoritative JSON fields and meanings.

### Laptop control station and buoy

The laptop application connects to the buoy over its Wi-Fi network. TCP `5000` carries line-oriented commands, acknowledgements, status, and science text. UDP `5001` carries the faster binary telemetry stream; UDP `5002` carries audio. TCP `5003` is used by the station's SD-card file client. The buoy accepts manual Wi-Fi movement only when Wi-Fi owns control. This desktop path is separate from the phone's receive-only router dashboard.

### TX12 control path

The TX12 sends ELRS control over RF to the receiver. The receiver presents MAVLink RC overrides and selected telemetry to the ESP32 on UART2 (`460800`, 8N1; receiver TX → GPIO34, ESP32 TX GPIO25 → receiver RX). The production protocol is ELRS MAVLink; the separate CRSF sketch is a bench test. Channel assignments and switch meanings are in [`../hardware/components/firmware-interface-map.md`](../hardware/components/firmware-interface-map.md) and [`../../src/control_station/config/TX12-CONTROLS.md`](../../src/control_station/config/TX12-CONTROLS.md).

## Control modes

| Mode / path | How it is selected | Behavior and ownership | Current status |
| --- | --- | --- | --- |
| TX12 manual direct | SB/CH7 Low; arm using SE/CH5 | CH1 lateral, CH2 thrust, CH3 power limit, CH4 yaw. ELRS owns propulsion. CH5 low disarms. | Production control path implemented; validate radio model/channel setup before use. |
| TX12 compass assist | SB/CH7 Middle; arm using SE/CH5 | Manual stick movement with heading-based yaw assistance when calibrated heading is available. | Implemented, but heading availability/reliability under motor load is not established. |
| Wi-Fi keyboard/manual | Select Wi-Fi control in the desktop station | Sends `CTRL VECTOR` or `CTRL MOTION`; Wi-Fi takes propulsion ownership and TX12 motor control is interlocked. Fresh commands are required while moving; loss/watchdog stops motors. | Implemented; manual control and sensor acquisition have been tested in stages. |
| Mission request | SB/CH7 High, then arm CH5 through its low-to-high interlock | Firmware evaluates the mission start gates below, records a mission session, then enters the mission execution path. Mission mode is selected and armed on the buoy; desktop Mission Control is primarily planning/upload/readback and telemetry. | Mission start attempts in the Nov 8, 2026 water test were blocked by unavailable heading. A successful mission run has not been demonstrated. |
| Phone dashboard | Join Opal/Bouy Wi-Fi and open `http://192.168.8.1:8080` | Display-only telemetry; no control endpoints. | Deployed and viewed on iPhone; GPS and simultaneous QGC remain unverified. |

Do not treat the dashboard as a second GCS. The router bridge's receive-only design avoids its own radio-link requests, but QGC coexistence depends on Backpack forwarding and has not been field-validated.

### Mission start gate sequence (TX12)

1. Load a valid V4 mission with a Home point, at least three boundary points, and valid route points/speeds (1–100%). Mission upload/readback is performed by the desktop control station over Wi-Fi.
2. Stop and save any existing science session. The SD card must be mounted and SD transfer/service mode must be inactive.
3. Place the buoy inside the mission boundary and within **30 m** of the planned Home. GPS must be fresh (≤1.5 s), have at least 5 satellites, and HDOP ≤3.0.
4. Ensure the mission route segments pass the boundary validation. The captured start Home option (hold SA/CH9 while changing into Mission) also requires the segment from the captured location to the first waypoint to remain inside the boundary. This records a run-specific Home in the session; it does not rewrite the planned mission file.
5. Make sure Wi-Fi does not own propulsion. Put SB/CH7 to Manual first, then select Mission. This reset is required after the start interlock locks.
6. Cycle SE/CH5 low, then high. At the arm transition, CH3 must be at minimum (≤1100 μs) and lateral, thrust, and yaw sticks must be centered. CH5 must be fully high (≥1700 μs).
7. The magnetometer must be available, calibration loaded, and a valid mission heading readable. If the checks pass, firmware creates a mission-origin science session on the SD card, then marks the mission active/armed.

The reported start-block codes and their current meanings are documented in [`API.md`](../../src/router_bridge/API.md#buoymission). In the November 8, 2026 live test, nine starts were attempted and the SD log recorded `HEADING_UNAVAILABLE` blocks. The mission did not start. The mission control/execution logic exists in source, but the gate passing and a successful water route remain unverified; do not represent it as validated autonomy.

Manual Home capture is a separate action: hold SA/CH9 while arming in Manual to save the current valid GPS position as manual Home. See the TX12 map for the distinct mission-run Home capture gesture above.

## Sessions, mission runs, and continuous logs

The firmware has separate session recording and per-start operational logging. A session is not required for the continuous operational telemetry log.

### Standalone per-start operational logs

On SD, each firmware startup gets `/logs/run-<boot_id>/` with `run.txt`, `telemetry.csv`, and `system.csv`. `boot_id` associates files to one startup. The operational telemetry records GPS, RC channels and freshness, control source/mode, arm/link state, motor targets and outputs, electrical readings, sensors, compass/calibration/mission diagnostics, and logging health. System events (including mission-block reasons) are timestamped with GPS date/time when available plus uptime and boot ID. This logging runs independently of science-session recording when the SD card is mounted. GPS may be unavailable at startup, so timestamps can be empty; uptime/boot ID still identify order and run.

### Manual science/audio session

Manual collection is started/stopped from the desktop science controls or TX12 science switch, depending on the active control path. `CTRL SCIENCE START [UTC]` creates `/sessionNNN-<timestamp>/`; the recorder opens `science.csv`, `telemetry.csv`, `audio.wav`, `audio.idx`, and `manifest.txt`. The session tracks sensor values, GPS, operating state, motor outputs, power, and audio indices. A no-UTC start uses a boot-relative marker. Stop changes the state to stopped/incomplete; save closes files and marks the manifest complete. Do not remove the SD card until recording has stopped and the session has saved.

### Mission-origin session

After all start gates pass, mission start automatically opens one session for the entire run. The manifest captures boot/firmware identity, mission ID, run ID, run Home/End, boundary, waypoint definitions, measurement masks, collection/hold/speed settings, and timestamps. Science and telemetry rows retain GPS and the current mission waypoint, collection state, and measurement mask. The session remains open through transit, collection checkpoints, and return home, and closes when the mission stops or completes. Mission audio is captured during stationary collection holds according to the configured policy; transit audio is excluded. A failed mission gate should be visible in `system.csv` but should not create a mission session.

The live mission stream reports state, waypoint/progress, distances, Home source and start-block code, but does not carry the full plan. Retrieve the mission file/session data over the desktop SD/mission workflow for complete route/configuration and historical observations. The router API is latest-value only, not a historical archive.

## Documentation change rule

When changing a control mapping, mode transition, start interlock, mission execution behavior, MAVLink or custom field, telemetry route, SD filename/schema, session lifecycle, or API field, update the corresponding section here and the focused reference (`mavlink-reference.md`, `API.md`, mission/configuration docs, or file-format docs) in the same change. Keep test evidence and known limitations current. Source code is authoritative if a doc and implementation disagree; resolve the discrepancy in the same work item.
