# Laptop Control Station

This program runs on the laptop to monitor the buoy and manage science sessions. Manual propulsion normally comes directly from the TX12 through the ELRS receiver. Wi-Fi keyboard drive is an explicit bench or recovery option.

## Control policy

- TX12/ELRS is the default propulsion source. SE high on CH5 requests arm after SE was observed low, CH3 is fully down, and movement sticks are neutral.
- CH3 sets the TX12 motor power ceiling up to full PWM 255. The buoy view draws observed motor output for both TX12 and Wi-Fi keyboard control; its movement vector is estimated from motor output when telemetry is available.
- In TX12 mode, opening the station's Wi-Fi telemetry connection and switching dashboard tabs do not send stop or movement commands. A deliberate **Stop all motors** action remains available.
- Wi-Fi keyboard drive must be selected explicitly. Entering it stops/disarms TX12 control before Wi-Fi movement commands can take authority.
- During manual operation, lost RC frames or a fresh zero-link-quality report stop propulsion, stop and save an active microSD science/audio session, and require another SE low-to-high arm cycle. Missing `RADIO_STATUS`, Wi-Fi, Backpack, or control-station packets do not affect TX12 control while receiver control frames continue. Mission-mode return-to-home is future work; no recovery switch channel is reserved yet.

## Current Functional Reference

- Primary propulsion source: TX12 over ELRS MAVLink through the buoy receiver
- Optional bench drive: keyboard over the Wi-Fi control link
- Buoy layout: external tangential motors with the rear water jet pointing
  left, both front motors producing forward buoy force, and pure-yaw motor
  signs `+/-/+`
- Output command formats: `CTRL VECTOR <lateral> <thrust>` and `CTRL MOTION <lateral> <thrust> <yaw>`
- Hello ping mode: optional repeated `hello world` messages for link testing
- Default transport path: Wi-Fi TCP for reliable commands plus UDP for fast telemetry
- Alternate paths: HM-10 BLE or serial when explicitly selected
- Main interface goal: show only the movement vector and buoy layout
- Current ESP32 Wi-Fi target: the buoy joins SSID `Bouy` with password `SuperMonkey`, then serves TCP `5000`, UDP telemetry `5001`, and UDP audio `5002`
- The Radio / ELRS panel can find an ExpressLRS TX Backpack on the same network. It checks `elrs_txbp.local`, then probes local IPv4 /24 addresses for the TX Backpack web page and shows its IP and MAVLink forwarding status. The station listens for validated MAVLink HEARTBEAT, BUOY_STATE NAMED_VALUE_INT, science NAMED_VALUE_FLOAT, SYS_STATUS, GLOBAL_POSITION_INT, RADIO_STATUS, RC_CHANNELS, and RC_CHANNELS_RAW messages on UDP `14550`. The production ESP32 firmware reports buoy mode, source, arm, recovery, recording, signed applied motor PWM, receiver uplink LQ/RSSI/SNR, TX12 channels, battery, GPS motion, water temperature, depth, and TDS/turbidity probe voltages when available. These low-rate reports continue when buoy Wi-Fi is unavailable. Set `Backpack > Telemetry` to `WiFi` in the ExpressLRS Lua script to enable forwarding; `Enable Backpack WiFi` is firmware update mode. Discovery and telemetry reception do not send missions over MAVLink.

### TX12 MKII control map

The Radio / ELRS panel puts the TX12 MKII image beside buoy telemetry and shows a compact control map underneath. **Show unmapped** reveals controls with no channel or defined function. The map assumes **Mode 2 / AETR**: right stick horizontal CH1, right stick vertical CH2, left stick vertical CH3, and left stick horizontal CH4. Confirm outputs in the active EdgeTX model's channel monitor. The JSON file labels the station display; actual motor control is decoded from receiver channels by the ESP32 firmware.

SE high on CH5 requests motor arm; low disarms. SF/SB/SC and SA/SD remain unassigned for future features. Those channels must also be mixed in the active EdgeTX model; S1 and S2 remain unassigned. Physical labels follow the [RadioMaster TX12 MKII manual](https://cdn.shopify.com/s/files/1/0701/8066/7584/files/TX12MKII_A2.0.pdf?v=1770617495).

The current channel plan, stick functions, switch assignments, and display thresholds are in [config/TX12-CONTROLS.md](config/TX12-CONTROLS.md). The station shows live bars for all four stick axes and discrete channel states for switches; position command meanings remain open in the JSON map.

### TX Backpack bench diagnostic (no buoy receiver required)

From `src/control_station`, run `python diagnose_backpack.py --host 192.168.8.134 --seconds 20 --log logs/backpack_diagnostic.jsonl` (replace the IP if DHCP changes it). This standard-library tool checks the Backpack web page and firmware revision, reads the `/mavlink` forwarding state and counters, listens on UDP 14550, and logs every raw datagram with any MAVLink frame metadata and supported decoded messages. A `crc_valid: null` frame uses a message ID whose CRC extra is not in the small built-in decoder; it is logged but not authenticated. When MAVLink forwarding is active, the tool sends one harmless ground-station HEARTBEAT to the Backpack UDP listen port (normally 14555) and checks status again. Use `--no-probe` for receive-only observation.

Only one process can normally listen on UDP 14550 on Windows. Close the main control station before running this tool for full reception. A different `--listen-port` permits web/status testing while the main station runs, but the Backpack still sends to its configured port 14550. The Backpack web endpoint proves PC-to-Backpack Wi-Fi/TCP reachability even without an ELRS receiver; it does not prove MAVLink UDP or RF. The `/mavlink` status page exposes forwarding state, GCS IP, ports, and packet counters. It does not expose the TX12 battery or EdgeTX-local state. When RADIO_STATUS (message 109) arrives, the tool reports its validated raw fields. ExpressLRS uses its receiver-generated `RADIO_STATUS.rssi` for uplink LQ and `remrssi` for uplink RSSI; the ESP32 firmware relays it for the PC to see it.

The separate `esp32-elrs-crsf-test` bench sketch uses CRSF at 420000 baud. It sends no MAVLink frames. The control station sends a periodic GCS HEARTBEAT to an active Backpack so it learns the PC address. The production buoy firmware uses MAVLink at 460800 baud.

## Control-Station Layout Contract

The main control-station window uses three panel regions:

- Top-left: `Buoy Visualization` — the buoy shape, motor arrangement, and movement vector.
- Bottom-left: `Buoy Status` — compact operational telemetry and control source selection.
- Full-height right: `Dashboard` — Overview, Communications, Power, Propulsion, Navigation, Mission, Instrument Debugging, and SD Card tabs. Mission contains Mission Creation and Mission Control sub-tabs.

Overview is the default page and shows one compact status tile for each subsystem. Communications owns Wi-Fi source selection, connection controls, link diagnostics, text commands, the link log, and ELRS backpack telemetry. Instrument Debugging contains sensor readings, audio session capture, and microphone diagnostics. Mission creation and mission monitoring are separate dashboard tabs.

## Current Responsibilities

- Monitor buoy telemetry, ELRS link quality, TX12 channel positions, and motor authority
- Keep science capture, audio, SD files, and diagnostics available while TX12 controls propulsion
- Send keyboard drive commands only in explicitly selected Wi-Fi keyboard mode
- Keep a deliberate stop action and future mission/recovery controls

## Current Stack

- Python
- Python 3.13 x64 is currently used for the Windows control-station environment.
- `PySide6` / Qt Quick for the main desktop UI
- `pygame` mixer for audio playback only
- `pyserial` for serial communication
- `bleak` for optional HM-10 BLE communication
- `run_control_station.sh`, `run_control_station.ps1`, and `run_control_station.bat` for the normal launch path

## Working Files

- `main.py` main entry point for the laptop control program
- `station/qt_app.py` runtime state, telemetry, audio, and optional Wi-Fi keyboard drive
- `station/geometry.py` buoy geometry and thrust calculations
- `station/serial_link.py` transport connection handling for Wi-Fi, BLE, and serial
- `station/models.py` command and science sample data structures
- `station/settings.py` default link settings
- `qml/Main.qml` main three-region window layout and dashboard tabs
- `qml/OverviewPanel.qml` default subsystem status tiles
- `qml/CommunicationsPanel.qml` Wi-Fi controls, ELRS backpack telemetry, and link log
- `qml/PowerPanel.qml` uses the same compact instrument layout as Navigation. It shows live battery voltage, current, and calculated electrical power above a history chart. The chart offers 10-second readings over 30 minutes, 30-second readings over two hours, and five-minute readings over a day, with gaps where telemetry is missing. It marks observed voltage minimum and maximum, shows signed net current, and draws an adjustable advisory low line (10.2 V by default). The control station retains these histories while it runs. The buoy normally transmits power about every 500 ms, but slows it to about 30 seconds during audio recording. The buoy also saves voltage and current in each SD session's `telemetry.csv`. The low line is not an active firmware cutoff, and battery voltage under load is not a measured state of charge. ELRS `SYS_STATUS` current does not establish charging direction.
- `qml/InstrumentPanel.qml` instrument readings and microphone debugging subpages
- `qml/ScienceAudioPanel.qml` instrument readings and audio session controls
- `qml/MicrophoneDebugPanel.qml` Instrument Debug dashboard tab, with per-channel microphone waveforms, packet continuity, monitor controls, and laptop-side high-pass, low-pass, and noise-gate filters
- `qml/MotorTestPanel.qml` compact Propulsion dashboard with live electrical readings, motor output and PWM displays, hold-to-run bench controls, and current RAM-only motor settings
- `qml/NavigationDashboardPanel.qml` compact GPS, magnetic heading, and IMU instruments with waterline and compass calibration controls
- `config/PROPULSION-CALIBRATION.md` planned SD backed motor profiles and in water response calibration
- `qml/MissionHubPanel.qml` combines the Mission Creation and Mission Control sub-tabs under the single Mission dashboard tab.
- `qml/MissionsPanel.qml` Mission Creation tab and full-width embedded Leaflet map editor. Boundary editing and mission-point editing have separate modes; the water polygon remains visible while boundary vertices hide in point mode. The boundary is emphasized in red with a purple fill. New and moved movement points are restricted to the water polygon. Each movement point stores its own list of science measurements (all enabled by default). Mission settings open in a compact side panel. The Leaflet library is bundled locally. An empty plan opens centered on Redwood Shores; saved plans automatically frame their route. Choose between USGS aerial imagery and OpenTopoMap terrain. Saving a mission prepares USGS National Map USGSTopo tiles for the boundary and route at zoom levels 10–16; the map falls back to this compact package when online tiles fail. Home may be outside the survey boundary; travel to and from home is not routed or safety checked yet. Movement mode can be Manual or Automatic. Automatic lays out alternating east-west survey passes inside the boundary at adjustable spacing (25 m by default), speed (50% by default), and boundary clearance (15 m by default). It places collection checkpoints along the passes and keeps connecting transit segments within the same clearance around concave areas. Light green circles mark the 5 m GPS arrival area for each checkpoint. A collection duration (30 s by default) is stored per generated checkpoint; manual checkpoints can set their own duration. Drafts save as JSON with a matching `<mission UUID>_tiles/` folder; older `logs/missions/` drafts are copied here on startup.
- `qml/MissionProgressPanel.qml` Mission Control tab. The map shows the mission read from the buoy's SD card; local saved plans are chosen in the upload dialog and appear on this map only after the buoy confirms the upload. The SD mission stores its route, optional home/end, boundary polygon, and per-point measurement selection, collection flag, hold duration, and speed. Older V1/V2 mission files remain readable. The firmware can persist and report this mission after reset, but autonomous navigation and measurement execution are still not implemented.
- The mission estimate shows Home-to-first-point and movement-point path length, estimated travel plus collection-stop duration, and approximate battery charge. Enter measured speed and current at 25%, 50%, 75%, and 100% output, idle current during collection stops, and usable battery capacity in Ah. Values between calibrated speeds are linearly interpolated; endpoint values are held outside the calibrated range. Estimates remain marked as needing calibration until the required inputs exist. The dashed final End leg is excluded because it is only a preview; wind, water conditions, turns, and other loads can change actual time and consumption.
- `station/navigation.py` filtered IMU direction comparison and bounded real-time correction algorithm
- `requirements.txt` Python dependencies for the laptop program
- `setup_env.sh` create the local virtual environment and install dependencies
- `activate_env.sh` activate the local virtual environment in the current shell
- `run_control_station.sh` run the program using the local virtual environment

The Radio / ELRS panel shows Backpack discovery, validated MAVLink receive status, receiver uplink LQ/RSSI/SNR, TX12 channel positions, and motor authority/arm/recovery status. Production firmware accepts receiver channels and sends low-rate MAVLink telemetry. Solar input telemetry is not yet available, so Power marks it unavailable.

## Command Format

When a serial port is supplied, the control station sends one ASCII line per update:

```text
CTRL VECTOR <x> <y>
```

Example:

```text
CTRL VECTOR +50 +87
```

Value meaning:

- `x` horizontal movement component from `-100` to `+100`
- `y` vertical movement component from `-100` to `+100`
- Vector magnitude is clamped to 100 (sqrt(x² + y²) ≤ 100), representing 0-100% speed
- Magnitude 0 stops all motors

The Arduino buoy firmware receives the `CTRL VECTOR` command, computes the required motor thrusts using the same geometry model, and applies them to the motors.

Independent yaw uses the three-axis command:

```text
CTRL MOTION <lateral> <thrust> <yaw>
```

All three values range from `-100` to `+100`. Translation-only commands continue using `CTRL VECTOR` for compatibility. When yaw is combined with translation, the firmware scales the complete three-motor mix together if necessary so no motor exceeds its allowed command range.

The ESP32 propulsion watchdog requires fresh control traffic at least once per second while a motor command is active. Normal drive commands are refreshed at the configured send rate, and the control station backend refreshes the Motor Test command every `200 ms` while the test button is held. The command trace records each motor keepalive and marks button release, stop requests, tab changes, link loss, and an observed motor-output drop. Losing Wi-Fi/TCP or exceeding the watchdog interval stops all motors.

## Navigation Test Mode

The `Navigation` dashboard tab compares three body-frame vectors:

- the operator's requested Wi-Fi keyboard vector
- horizontal linear acceleration from the MPU6050 after low-frequency gravity and hull tilt are removed
- the corrected vector actually sent to the buoy

Balance assist is opt-in. Zero the IMU while the buoy is stationary, verify the measured-axis direction with short low-power inputs, and only then enable assist. The algorithm preserves requested magnitude and applies a smoothed angular correction limited by the configured maximum. It automatically disengages when IMU telemetry becomes stale, the connection drops, or the operator leaves the Navigation tab. Active corrections are recorded as `NAV` entries in the communication log.

The installed MPU6050 is mounted 90 degrees clockwise. Navigation Test therefore defaults to swapping X/Y and inverting the mapped X axis: buoy-right is sensor `-Y`, and buoy-forward is sensor `+X`.

Navigation Test includes cockpit-style movement instruments for roll, pitch, linear acceleration, yaw rate, and speed. Speed uses fresh multi-fix GPS ground speed when available and falls back gracefully to a low-confidence, short-term IMU integration when GPS is missing or stale. Select `Dry land` or `In water` before testing. Dry-land mode intentionally caps speed confidence because propeller response in air does not predict thrust or drag in water; a water-response model requires matched water trials with motor command, acceleration, current, yaw, and GPS data.

Enable `Keyboard Drive` in Navigation Test to capture the keyboard globally instead of letting focused controls use it for interface navigation. `W/S` move forward and reverse, `A/D` translate left and right, and the Left/Right arrows command independent yaw. Combined movement is supported. Space stops all motors; release the keys before resuming. This is an explicit Wi-Fi drive mode and disarms TX12 when selected.

The accelerometer measures transient acceleration rather than steady thrust, velocity, or course over water. This mode is intended for low-speed motor-balance experiments; reliable steady-course navigation will require GPS, heading, or water-speed feedback in a later control layer.

Responses from the buoy should use:

```text
ACK ...
ERR ...
TEL STATUS ...
TEL SCI ...
```

Telemetry is intentionally split into small typed messages by purpose. Operational buoy state uses line-based TCP messages (`TEL STATUS ...`, `TEL SCI ...`) when requested, while fast telemetry is streamed over UDP as binary packets.

Default Wi-Fi transport:

```text
ESP32 STA SSID: Bouy
ESP32 STA password: SuperMonkey
TCP control port: 5000
UDP telemetry port: 5001
UDP audio port: 5002
```

When multiple laptop network adapters are active, you can select which local IPv4 address the control station should use for discovery and source binding with `--source-ip <address>` or `--wifi-local-ip <address>`. This covers Wi-Fi and Ethernet uplinks to the router. The Qt UI exposes the same choice in the main status panel as `Source interface`.

UDP packet types currently implemented:

```text
1 IMU
2 RANGE
3 POWER
4 STATE
5 GPS
6 ELRS_CHANNELS (16 channel values, 10 Hz while buoy Wi-Fi telemetry is selected)
10 AUDIO
```

Audio packets carry stereo PCM from the INMP441 pair. The control station plots the received waveform in the main window and includes a mute toggle that turns local playback on or off without stopping reception.

Current science telemetry also includes MPU6050 IMU lines:

```text
TEL SCI IMU_ACCEL <x_g|UNKNOWN> <y_g|UNKNOWN> <z_g|UNKNOWN>
TEL SCI IMU_GYRO <x_dps|UNKNOWN> <y_dps|UNKNOWN> <z_dps|UNKNOWN>
TEL SCI IMU_TEMP <c|UNKNOWN>
```

## Transmission Optimization

To prevent excessive command traffic:
- `CTRL VECTOR` commands are only sent when the movement vector changes or at regular intervals (based on `--send-rate`)
- When the vector is zero (stop command), it is sent up to 10 times to ensure delivery, then transmission stops until movement resumes

## Environment Setup

Use a local virtual environment so the control station dependencies do not affect global Python packages.

```bash
./src/control_station/setup_env.sh
source ./src/control_station/activate_env.sh
./src/control_station/run_control_station.sh
```

On macOS, run the same `setup_env.sh` script from Terminal. It installs `PySide6-Addons` through `requirements.txt`, including the Qt WebEngine and WebChannel modules used by the mission map.

On Windows PowerShell:

```powershell
.\src\control_station\setup_env.ps1
.\src\control_station\activate_env.ps1
.\src\control_station\run_control_station.ps1
```

These Windows setup scripts use `py -3` so they follow the installed Python 3 interpreter on the machine.

On Windows Command Prompt or by double-clicking batch files:

```bat
src\control_station\setup_env.bat
src\control_station\activate_env.bat
src\control_station\run_control_station.bat
```

To run against the default Wi-Fi buoy link:

```bash
./src/control_station/.venv/bin/python ./src/control_station/main.py --transport wifi --wifi-host auto --tcp-port 5000 --udp-port 5001
```

If PySide6 is installed, this launches the Qt Quick control station. If PySide6 is missing, the launcher falls back to the legacy Pygame UI and prints a warning.

The recommended launchers are:

```bash
./src/control_station/run_control_station.sh
```

```powershell
.\src\control_station\run_control_station.ps1
```

```bat
src\control_station\run_control_station.bat
```

To use an explicit audio UDP port as well:

```bash
./src/control_station/.venv/bin/python ./src/control_station/main.py --transport wifi --wifi-host auto --tcp-port 5000 --udp-port 5001 --audio-port 5002
```

To run against a serial link:

```bash
./src/control_station/.venv/bin/python ./src/control_station/main.py --port /dev/tty.HC-05-DevB --baudrate 9600
```


To auto-discover a different HM-10 BLE device name:

```bash
./src/control_station/.venv/bin/python ./src/control_station/main.py --device-name My-Device-Name
```

To run against an HM-10 BLE module explicitly:

```bash
./src/control_station/.venv/bin/python ./src/control_station/main.py --transport ble --device-name "DSD TECH"
```

To scan using multiple fallback names (example):

```bash
./src/control_station/.venv/bin/python ./src/control_station/main.py --transport ble --device-name "ESP32-BUOY,DSD TECH"
```

To run against the ESP32 onboard BLE firmware explicitly:

```bash
./src/control_station/.venv/bin/python ./src/control_station/main.py --transport ble --device-name "ESP32-BUOY" --ble-service-uuid 0000ffe0-0000-1000-8000-00805f9b34fb --ble-characteristic-uuid 0000ffe1-0000-1000-8000-00805f9b34fb
```

If your HM-10 firmware uses different custom UUIDs:

```bash
./src/control_station/.venv/bin/python ./src/control_station/main.py --transport ble --device-name HMSoft --ble-service-uuid 0000ffe0-0000-1000-8000-00805f9b34fb --ble-characteristic-uuid 0000ffe1-0000-1000-8000-00805f9b34fb
```

To use the older Bluetooth serial workflow instead:

```bash
./src/control_station/.venv/bin/python ./src/control_station/main.py --transport serial --port /dev/tty.HC-05-DevB --device-name DSDTECHHC-05
```

Windows PowerShell example with explicit COM port:

```powershell
.\src\control_station\run_control_station.ps1 --port COM5 --baudrate 9600
```

Windows batch example with explicit COM port:

```bat
src\control_station\run_control_station.bat --port COM5 --baudrate 9600
```

The control station now writes communication logs to `src/control_station/logs/comm-YYYYMMDD-HHMMSS.log` by default, including raw TX/RX bytes and decoded text.

To write to a specific log file:

```bash
./src/control_station/.venv/bin/python ./src/control_station/main.py --comm-log-file ./src/control_station/logs/my-test.log
```

To send repeated `hello world` ping messages:

```bash
./src/control_station/.venv/bin/python ./src/control_station/main.py --hello-ping
```

To send your own text command repeatedly over the active transport:

```bash
./src/control_station/.venv/bin/python ./src/control_station/main.py --send-text "banana" --send-interval 1.0
```

To change the repeat interval in seconds:

```bash
./src/control_station/.venv/bin/python ./src/control_station/main.py --send-text "banana" --send-interval 0.5
```

## Science experiments and SD browsing

The firmware mounts the SD card at boot but creates no session until `CTRL SCIENCE START YYYYMMDDTHHMMSSZ`. The control station sends the UTC timestamp automatically. Start creates `/sessionNNN-YYYYMMDDTHHMMSSZ`, begins science and telemetry recording, and writes `manifest.txt` with the start time. Microphone recording starts disabled and can be enabled only where a mission needs audio with `CTRL AUDIO RECORD START`; `CTRL AUDIO RECORD STOP` stops it. While audio is recording, non-GPS sensor sampling and science/telemetry logging run at 30-second intervals and the audio queue is enlarged; this audio-priority profile is active in firmware, but should be revisited during mission design so GPS, drift/current measurements, motor control, and safety-critical current readings retain appropriate rates during autonomous navigation. The microphone's digital input gain is 1x to preserve headroom; laptop playback gain does not alter SD files. Repeated audio start/stop commands currently append to one `audio.wav` with timestamps in `audio.idx`. `CTRL SCIENCE STOP YYYYMMDDTHHMMSSZ` stops recording and stamps the stop time. `CTRL SCIENCE SAVE YYYYMMDDTHHMMSSZ` flushes and closes the files, then stamps the save time. An unfinished session retains `status=recording` in its manifest so it can be identified after power loss.

Audio and IMU Wi-Fi streams start disabled. Use `CTRL STREAM AUDIO START` / `STOP` and `CTRL STREAM IMU START` / `STOP` to control them independently. Lightweight telemetry remains available. The microphone panel controls audio streaming; Navigation Test controls IMU streaming; Science & Audio holds manual mission capture controls.

The SD Card panel requests root session summaries first, including folder sizes. It requests a session's file list only when that session is opened. Downloading or deleting a session sends `CTRL SERVICE START`: motors stop, audio and IMU streams stop, and sensor polling pauses. Downloads and deletion are unavailable while a mission session is open. The panel shows the current session, bytes, speed, elapsed time, estimated time left, and a Stop download button. Session deletion requires confirmation and permanently removes only the selected top-level `sessionNNN-timestamp` folder and its contents. Completion, cancellation, and errors send `CTRL SERVICE STOP`. The control station stores timestamped outgoing commands and incoming Wi-Fi TCP lines, including `DIAG ADS1115` readings, in `src/control_station/logs/system/commands-*.log`. New science sessions do not contain a `system.log` file.

The proposed automatic mission transcript, checkpoint log, and per-checkpoint audio file model are documented in [mission-transcript.md](../mission-transcript.md). Those automation commands are not yet implemented.
