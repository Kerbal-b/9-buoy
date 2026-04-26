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
- Output command format: `CTRL <turn> <thrust> <rear_motor> <front_left_motor> <front_right_motor>`
- Hello ping mode: optional repeated `hello world` messages for link testing
- Transport path: Bluetooth serial when `--port` is supplied, or auto-discovered by Bluetooth device name
- Main interface goal: show only the movement vector and buoy layout
- Debug mode launch: `--debug-controller`
- Debug mode goal: show generic controller inputs, with analog values as live bars and digital buttons as labeled on/off indicators
- Default Bluetooth target name: `DSDTECHHC-05`
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
VECTOR <x> <y>
```

Example:

```text
VECTOR +50 +87
```

Value meaning:

- `x` horizontal movement component from `-100` to `+100`
- `y` vertical movement component from `-100` to `+100`
- Vector magnitude is clamped to 100 (sqrt(x² + y²) ≤ 100), representing 0-100% speed
- Magnitude 0 stops all motors

The Arduino buoy firmware receives the VECTOR command, computes the required motor thrusts using the same geometry model, and applies them to the motors.

## Transmission Optimization

To prevent excessive Bluetooth traffic:
- VECTOR commands are only sent when the movement vector changes or at regular intervals (based on `--send-rate`)
- When the vector is zero (stop command), it is sent up to 10 times to ensure delivery, then transmission stops until movement resumes

## Environment Setup

Use a local virtual environment so the control station dependencies do not affect global Python packages.

```bash
./src/control_station/setup_env.sh
source ./src/control_station/activate_env.sh
./src/control_station/run_control_station.sh
```

To run against a Bluetooth serial link:

```bash
./src/control_station/.venv/bin/python ./src/control_station/main.py --port /dev/tty.HC-05-DevB --baudrate 9600
```

To auto-discover the Bluetooth device named `DSDTECHHC-05`:

```bash
./src/control_station/run_control_station.sh
```

To auto-discover a different Bluetooth device name:

```bash
./src/control_station/.venv/bin/python ./src/control_station/main.py --device-name My-Device-Name
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
