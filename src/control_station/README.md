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
8. Open a Bluetooth serial connection to the buoy when a serial port is provided.
9. Send control commands over Bluetooth in a simple, repeatable text format.
10. Keep the main interface minimal so it can later hold navigation, sensor, and mission controls.
11. Move controller diagnostics into a separate debug mode that can be opened by command.
12. Fall back to simulation mode when no Bluetooth serial port is connected so controller logic can still be tested.
13. Keep the command format and on-screen display simple enough to support later Arduino integration and bench testing.

## Current Functional Reference

- Controller source: Xbox controller connected to the laptop
- Main movement input: left stick X for horizontal movement and left stick Y for forward and reverse movement
- Deadzone behavior: ignore small stick movement near center
- Buoy layout: one rear motor and two front motors arranged with 120 degree spacing
- Output command format: `CTRL VECTOR <turn> <thrust>`
- Hello ping mode: optional repeated `hello world` messages for link testing
- Default transport path: HM-10 BLE through `bleak` using the default HM-10 `FFE0` service and `FFE1` characteristic
- Alternate path: Bluetooth serial when `--transport serial` is selected
- Main interface goal: show only the movement vector and buoy layout
- Debug mode launch: `--debug-controller`
- Debug mode goal: show generic controller inputs, with analog values as live bars and digital buttons as labeled on/off indicators
- Default Bluetooth target name: `DSD TECH`
- Test fallback: simulation mode when no serial port is supplied
- Display goal: keep diagnostics out of the main interface unless debug mode is opened

## Current Responsibilities

- Read Xbox controller input
- Convert joystick movement into manual drive commands
- Calculate thrust targets for the rear, front-left, and front-right motors
- Optionally send those commands to the buoy over Bluetooth serial
- Keep controller mapping details in a separate debug mode
- Expand later for sensor data display and higher-level control

## Current Stack

- Python
- `pygame` for Xbox controller input
- `pyserial` for Bluetooth serial communication
- `bleak` for HM-10 BLE communication

## Working Files

- `main.py` main entry point for the laptop control program
- `station/app.py` runtime loop and mode selection
- `station/controller.py` controller connection and axis reading
- `station/geometry.py` buoy geometry and thrust calculations
- `station/serial_link.py` Bluetooth serial connection and command sending
- `station/ui.py` drawing code for the main and debug interfaces
- `station/models.py` shared runtime and command data structures
- `station/settings.py` UI and runtime constants
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

Telemetry is intentionally split into small typed messages instead of one large snapshot line. Operational buoy state should use `TEL STATUS ...`, while environmental measurements should use `TEL SCI ...`.

## Transmission Optimization

To prevent excessive Bluetooth traffic:
- `CTRL VECTOR` commands are only sent when the movement vector changes or at regular intervals (based on `--send-rate`)
- When the vector is zero (stop command), it is sent up to 10 times to ensure delivery, then transmission stops until movement resumes

## Environment Setup

Use a local virtual environment so the control station dependencies do not affect global Python packages.

```bash
./src/control_station/setup_env.sh
source ./src/control_station/activate_env.sh
./src/control_station/run_control_station.sh
```

On Windows PowerShell:

```powershell
.\src\control_station\setup_env.ps1
.\src\control_station\activate_env.ps1
.\src\control_station\run_control_station.ps1
```

These Windows setup scripts now use `py -3.12` explicitly, because `pygame` may fail to install on newer interpreter versions such as Python 3.14.

On Windows Command Prompt or by double-clicking batch files:

```bat
src\control_station\setup_env.bat
src\control_station\activate_env.bat
src\control_station\run_control_station.bat
```

To run against a Bluetooth serial link:

```bash
./src/control_station/.venv/bin/python ./src/control_station/main.py --port /dev/tty.HC-05-DevB --baudrate 9600
```

To start the control station against the default HM-10 BLE device name `DSD TECH`:

```bash
./src/control_station/run_control_station.sh
```

On Windows PowerShell:

```powershell
.\src\control_station\run_control_station.ps1
```

Windows batch equivalent:

```bat
src\control_station\run_control_station.bat
```

To auto-discover a different HM-10 BLE device name:

```bash
./src/control_station/.venv/bin/python ./src/control_station/main.py --device-name My-Device-Name
```

To run against an HM-10 BLE module explicitly:

```bash
./src/control_station/.venv/bin/python ./src/control_station/main.py --transport ble --device-name "DSD TECH"
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

To send your own text command repeatedly over Bluetooth:

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
