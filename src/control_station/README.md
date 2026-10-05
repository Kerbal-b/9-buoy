# Laptop Control Station

This program will run on the laptop and act as the control station for the buoy.

## Main Procedures To Implement And Reconfirm

Review this section after each control station update and confirm that the code still matches these procedures.

1. Detect and connect to an Xbox controller on the laptop.
2. Read controller input continuously while the program is running.
3. Apply a deadzone so small stick noise near center does not create unwanted motor commands.
4. Convert controller input into manual movement commands for the buoy.
5. Represent the buoy visually as a minimalist three-motor layout over the movement vector.
6. Calculate thrust for the three motors using a 120 degree motor geometry model.
7. Show each motor's live thrust level on the screen so the math can be reviewed visually.
8. Open the configured transport link to the buoy.
9. Send control commands in a simple, repeatable text format.
10. Keep the main interface minimal so it can later hold navigation, sensor, and mission controls.
11. Move controller diagnostics into a separate debug mode that can be opened by command.
12. Fall back to simulation mode when no buoy transport is connected so controller logic can still be tested.
13. Keep the command format and on-screen display simple enough to support later Arduino integration and bench testing.

## Current Functional Reference

- Controller source: Xbox controller connected to the laptop
- Main movement input: left stick X for horizontal movement and left stick Y for forward and reverse movement
- Deadzone behavior: ignore small stick movement near center
- Buoy layout: external tangential motors with the rear water jet pointing
  left, both front motors producing forward buoy force, and pure-yaw motor
  signs `+/-/+`
- Output command formats: `CTRL VECTOR <lateral> <thrust>` and `CTRL MOTION <lateral> <thrust> <yaw>`
- Hello ping mode: optional repeated `hello world` messages for link testing
- Default transport path: Wi-Fi TCP for reliable commands plus UDP for fast telemetry
- Alternate paths: HM-10 BLE or serial when explicitly selected
- Main interface goal: show only the movement vector and buoy layout
- Debug mode launch: `--debug-controller`
- Debug mode goal: show generic controller inputs, with analog values as live bars and digital buttons as labeled on/off indicators
- Current ESP32 Wi-Fi target: the buoy joins SSID `Bouy` with password `SuperMonkey`, then serves TCP `5000`, UDP telemetry `5001`, and UDP audio `5002`
- The Radio / ELRS panel can find an ExpressLRS TX Backpack on the same network. It checks `elrs_txbp.local`, then probes local IPv4 /24 addresses for the TX Backpack web page and shows its IP and MAVLink forwarding status. The station listens for validated MAVLink HEARTBEAT, SYS_STATUS, GLOBAL_POSITION_INT, RADIO_STATUS, RC_CHANNELS, and RC_CHANNELS_RAW messages on UDP `14550`. The ESP32 MAVLink bench sketch relays ELRS receiver uplink LQ/RSSI/SNR and TX12 channel positions to this panel, using smaller eight-channel messages to fit the ELRS telemetry budget. Battery and position require production buoy MAVLink telemetry. Set `Backpack > Telemetry` to `WiFi` in the ExpressLRS Lua script to enable forwarding; `Enable Backpack WiFi` is firmware update mode. Discovery and telemetry reception do not yet send missions over MAVLink.

### TX12 MKII control map

The Radio / ELRS panel puts the light TX12 MKII image beside the buoy telemetry table and shows a compact, two-column control map underneath. The duplicate 16-channel grid has been removed. **Show unmapped** reveals controls with no channel or no defined function; it is off by default. A control counts as mapped when it has a channel and either a `purpose` or at least one switch-position `meaning` in `config/tx12-channel-map.json`. The starter map assumes **Mode 2 / AETR**: right stick horizontal CH1 (aileron), right stick vertical CH2 (elevator), left stick vertical CH3 (throttle), and left stick horizontal CH4 (rudder). Confirm those outputs in the active EdgeTX model's channel monitor. Edit any `channel` value from 1 through 16 and click **Reload map** in the panel. The file only labels the station display; it does not program EdgeTX, transmit controls, or assign buoy commands. Buoy commands will be mapped to channel values in firmware separately.

The template maps SE/SF/SB/SC to CH5-CH8 and SA/SD to CH9-CH10. Those channels are planned assignments that must also be mixed in the active EdgeTX model; S1 and S2 remain unassigned. Physical labels follow the [RadioMaster TX12 MKII manual](https://cdn.shopify.com/s/files/1/0701/8066/7584/files/TX12MKII_A2.0.pdf?v=1770617495), and the starter stick assignments follow the [EdgeTX Mode 2 and channel-order definitions](https://manual.edgetx.org/v2.11/bw-radios/radio-settings/radio-setup). The generated photo and marker locations are visual aids.

The current channel plan, stick functions, switch assignments, and display thresholds are in [config/TX12-CONTROLS.md](config/TX12-CONTROLS.md). The station shows live bars for all four stick axes and discrete channel states for switches; position command meanings remain open in the JSON map.

### TX Backpack bench diagnostic (no buoy receiver required)

From `src/control_station`, run `python diagnose_backpack.py --host 192.168.8.134 --seconds 20 --log logs/backpack_diagnostic.jsonl` (replace the IP if DHCP changes it). This standard-library tool checks the Backpack web page and firmware revision, reads the `/mavlink` forwarding state and counters, listens on UDP 14550, and logs every raw datagram with any MAVLink frame metadata and supported decoded messages. A `crc_valid: null` frame uses a message ID whose CRC extra is not in the small built-in decoder; it is logged but not authenticated. When MAVLink forwarding is active, the tool sends one harmless ground-station HEARTBEAT to the Backpack UDP listen port (normally 14555) and checks status again. Use `--no-probe` for receive-only observation.

Only one process can normally listen on UDP 14550 on Windows. Close the main control station before running this tool for full reception. A different `--listen-port` permits web/status testing while the main station runs, but the Backpack still sends to its configured port 14550. The Backpack web endpoint proves PC-to-Backpack Wi-Fi/TCP reachability even without an ELRS receiver; it does not prove MAVLink UDP or RF. The `/mavlink` status page exposes forwarding state, GCS IP, ports, and packet counters. It does not expose the TX12 battery or EdgeTX-local state. When RADIO_STATUS (message 109) arrives, the tool reports its validated raw fields. ExpressLRS uses its receiver-generated `RADIO_STATUS.rssi` for uplink LQ and `remrssi` for uplink RSSI; the bench ESP32 must relay it for the PC to see it.

The separate `esp32-elrs-crsf-test` bench sketch uses CRSF at 420000 baud. It sends no MAVLink frames, so a zero Backpack MAVLink downlink count is expected even when its TX12 control and EdgeTX telemetry work. The control station sends a periodic GCS HEARTBEAT to an active Backpack so it learns the PC address. The `esp32-elrs-mavlink-test` sketch at 460800 baud is the current bench source for heartbeat, link measurements, and channel telemetry.
- Test fallback: simulation mode when no serial port is supplied
- Display goal: keep diagnostics out of the main interface unless debug mode is opened

## Control-Station Layout Contract

The main control-station window uses three panel regions:

- Top-left: `Buoy Visualization` — the buoy shape, motor arrangement, and movement vector.
- Bottom-left: `Buoy Status` — compact operational telemetry and control source selection.
- Full-height right: `Dashboard` — Overview, Communications, Power, Motor Test, Navigation, Instrument Debugging, and SD Card tabs.

Overview is the default page and shows one compact status tile for each subsystem. Communications owns Wi-Fi source selection, connection controls, link diagnostics, text commands, the link log, and ELRS backpack telemetry. Instrument Debugging contains sensor readings, audio session capture, and microphone diagnostics. The former Map and Bottom Mesh tabs have been removed pending Mission Tracking and Mission Creation.

## Current Responsibilities

- Read Xbox controller input
- Convert joystick movement into manual drive commands
- Calculate thrust targets for the rear, front-left, and front-right motors
- Optionally send those commands to the buoy over Wi-Fi, BLE, or serial
- Keep controller mapping details in a separate debug mode
- Expand later for sensor data display and higher-level control

## Current Stack

- Python
- Python 3.13 x64 is currently required for the Windows control-station environment because the pinned pygame release does not yet provide a Python 3.14 Windows wheel.
- `PySide6` / Qt Quick for the main desktop UI
- `pygame` for Xbox controller input and legacy fallback UI
- `pyserial` for serial communication
- `bleak` for optional HM-10 BLE communication
- `run_control_station.sh`, `run_control_station.ps1`, and `run_control_station.bat` for the normal launch path
- `run_controller_debug.sh`, `run_controller_debug.ps1`, and `run_controller_debug.bat` for the debug controller view

## Working Files

- `main.py` main entry point for the laptop control program
- `station/app.py` runtime loop and mode selection
- `station/controller.py` controller connection and axis reading
- `station/geometry.py` buoy geometry and thrust calculations
- `station/serial_link.py` transport connection handling for Wi-Fi, BLE, and serial
- `station/ui.py` drawing code for the main and debug interfaces
- `station/models.py` shared runtime and command data structures
- `station/settings.py` UI and runtime constants
- `qml/Main.qml` main three-region window layout and dashboard tabs
- `qml/OverviewPanel.qml` default subsystem status tiles
- `qml/CommunicationsPanel.qml` Wi-Fi controls, ELRS backpack telemetry, and link log
- `qml/PowerPanel.qml` battery status, current, and charging indication
- `qml/InstrumentPanel.qml` instrument readings and microphone debugging subpages
- `qml/ScienceAudioPanel.qml` instrument readings and audio session controls
- `qml/MicrophoneDebugPanel.qml` Instrument Debug dashboard tab, with per-channel microphone waveforms, packet continuity, monitor controls, and laptop-side high-pass, low-pass, and noise-gate filters
- `qml/MotorTestPanel.qml` motor testing, output visualization, and calibration dashboard tab
- `qml/NavigationTestPanel.qml` requested-versus-measured movement visualization and opt-in navigation balance-assist controls
- `station/navigation.py` filtered IMU direction comparison and bounded real-time correction algorithm
- `requirements.txt` Python dependencies for the laptop program
- `setup_env.sh` create the local virtual environment and install dependencies
- `activate_env.sh` activate the local virtual environment in the current shell
- `run_control_station.sh` run the program using the local virtual environment
- `run_controller_debug.sh` open the separate controller diagnostics window

The Radio / ELRS panel shows backpack discovery, MAVLink receive status, bench receiver uplink LQ/RSSI/SNR, and TX12 channel positions. The former TX12 USB joystick controls are no longer shown on this card. The separate ESP32 MAVLink sketch is a bench test; production firmware does not yet report RF link status, accept TX12 receiver channels, or send MAVLink telemetry. Solar input telemetry is also not yet available, so Power marks it unavailable.

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

- the operator's requested controller vector
- horizontal linear acceleration from the MPU6050 after low-frequency gravity and hull tilt are removed
- the corrected vector actually sent to the buoy

Balance assist is opt-in. Zero the IMU while the buoy is stationary, verify the measured-axis direction with short low-power inputs, and only then enable assist. The algorithm preserves requested magnitude and applies a smoothed angular correction limited by the configured maximum. It automatically disengages when IMU telemetry becomes stale, the connection drops, or the operator leaves the Navigation tab. Active corrections are recorded as `NAV` entries in the communication log.

The installed MPU6050 is mounted 90 degrees clockwise. Navigation Test therefore defaults to swapping X/Y and inverting the mapped X axis: buoy-right is sensor `-Y`, and buoy-forward is sensor `+X`.

Navigation Test includes cockpit-style movement instruments for roll, pitch, linear acceleration, yaw rate, and speed. Speed uses fresh multi-fix GPS ground speed when available and falls back gracefully to a low-confidence, short-term IMU integration when GPS is missing or stale. Select `Dry land` or `In water` before testing. Dry-land mode intentionally caps speed confidence because propeller response in air does not predict thrust or drag in water; a water-response model requires matched water trials with motor command, acceleration, current, yaw, and GPS data.

Enable `Keyboard Drive` in Navigation Test to capture the keyboard globally instead of letting focused controls use it for interface navigation. `W/S` move forward and reverse, `A/D` translate left and right, and the Left/Right arrows command independent yaw. Combined movement is supported. Space stops all motors, and both Space and leaving the tab require the physical controller to return to center before movement can resume.

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
3 POWER
4 STATE
5 GPS
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

On macOS, run the same `setup_env.sh` script from Terminal. It installs `PySide6-Addons` through `requirements.txt`, so the QML map modules are available there too.

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

The debug controller view uses the matching `run_controller_debug.*` launcher set.

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

To open the separate controller diagnostics window:

```bash
./src/control_station/run_controller_debug.sh
```

On Windows PowerShell:

```powershell
.\src\control_station\run_controller_debug.ps1
```

Windows batch equivalent:

```bat
src\control_station\run_controller_debug.bat
```

## Science experiments and SD browsing

The firmware mounts the SD card at boot but creates no session until `CTRL SCIENCE START YYYYMMDDTHHMMSSZ`. The control station sends the UTC timestamp automatically. Start creates `/sessionNNN-YYYYMMDDTHHMMSSZ`, begins science and telemetry recording, and writes `manifest.txt` with the start time. Microphone recording starts disabled and can be enabled only where a mission needs audio with `CTRL AUDIO RECORD START`; `CTRL AUDIO RECORD STOP` stops it. While audio is recording, non-GPS sensor sampling and science/telemetry logging run at 30-second intervals and the audio queue is enlarged; this audio-priority profile is active in firmware, but should be revisited during mission design so GPS, drift/current measurements, motor control, and safety-critical current readings retain appropriate rates during autonomous navigation. The microphone's digital input gain is 1x to preserve headroom; laptop playback gain does not alter SD files. Repeated audio start/stop commands currently append to one `audio.wav` with timestamps in `audio.idx`. `CTRL SCIENCE STOP YYYYMMDDTHHMMSSZ` stops recording and stamps the stop time. `CTRL SCIENCE SAVE YYYYMMDDTHHMMSSZ` flushes and closes the files, then stamps the save time. An unfinished session retains `status=recording` in its manifest so it can be identified after power loss.

Audio and IMU Wi-Fi streams start disabled. Use `CTRL STREAM AUDIO START` / `STOP` and `CTRL STREAM IMU START` / `STOP` to control them independently. Lightweight telemetry remains available. The microphone panel controls audio streaming; Navigation Test controls IMU streaming; Science & Audio holds manual mission capture controls.

The SD Card panel requests root session summaries first, including folder sizes. It requests a session's file list only when that session is opened. Downloading or deleting a session sends `CTRL SERVICE START`: motors stop, audio and IMU streams stop, and sensor polling pauses. Downloads and deletion are unavailable while a mission session is open. The panel shows the current session, bytes, speed, elapsed time, estimated time left, and a Stop download button. Session deletion requires confirmation and permanently removes only the selected top-level `sessionNNN-timestamp` folder and its contents. Completion, cancellation, and errors send `CTRL SERVICE STOP`. The control station stores timestamped outgoing command traces in `src/control_station/logs/system/commands-*.log`. New science sessions do not contain a `system.log` file.

The proposed automatic mission transcript, checkpoint log, and per-checkpoint audio file model are documented in [mission-transcript.md](../mission-transcript.md). Those automation commands are not yet implemented.
