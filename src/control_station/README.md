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
- Buoy layout: one rear motor and two front motors arranged with 120 degree spacing
- Output command format: `CTRL VECTOR <turn> <thrust>`
- Hello ping mode: optional repeated `hello world` messages for link testing
- Default transport path: Wi-Fi TCP for reliable commands plus UDP for fast telemetry
- Alternate paths: HM-10 BLE or serial when explicitly selected
- Main interface goal: show only the movement vector and buoy layout
- Debug mode launch: `--debug-controller`
- Debug mode goal: show generic controller inputs, with analog values as live bars and digital buttons as labeled on/off indicators
- Current ESP32 Wi-Fi target: the buoy joins SSID `Bouy` with password `SuperMonkey`, then serves TCP `5000`, UDP telemetry `5001`, and UDP audio `5002`
- Test fallback: simulation mode when no serial port is supplied
- Display goal: keep diagnostics out of the main interface unless debug mode is opened

## Control-Station Layout Contract

The main control-station window uses three stable panel regions:

- Top-left: `Buoy Visualization` — the buoy shape, motor arrangement, and movement vector.
- Bottom-left: `Buoy Operational` — connection, Wi-Fi, navigation, movement, power, and operational telemetry.
- Full-height right: `Dashboard` — a tabbed area for Science & Audio, logs, maps/science views, motor testing, and future subtabs.

Future control-station changes must preserve these three regions and their responsibilities. New right-side features should be added as Dashboard subtabs. Change this layout only when the project owner explicitly requests a design change.

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
- `qml/ScienceAudioPanel.qml` Science & Audio dashboard tab
- `qml/MotorTestPanel.qml` motor testing, output visualization, and calibration dashboard tab
- `requirements.txt` Python dependencies for the laptop program
- `setup_env.sh` create the local virtual environment and install dependencies
- `activate_env.sh` activate the local virtual environment in the current shell
- `run_control_station.sh` run the program using the local virtual environment
- `run_controller_debug.sh` open the separate controller diagnostics window

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
