# Source Code Reference

This file explains what the main source files mean and what configuration decisions are currently active.

## Main Source Areas

- `src/arduino/buoy_firmware/`
  - Arduino-side firmware for the buoy hardware.
- `src/control_station/`
  - Laptop-side control station for controller input, debug tools, and future buoy communication.

## Arduino Firmware

### File

- `src/arduino/buoy_firmware/buoy_firmware.ino`

### What It Does

- Defines the Arduino firmware for the buoy motor system.
- Controls three motors in one loop.
- Applies output to each motor through its motor-driver pins.
- Exposes a motor-drive helper that later code can call from joystick or serial logic.
- Runs a simple hardware test mode where all three motors are active at the same time.
- Supports a simple serial toggle command: typing `banana` toggles all motors on or off.
- Prints motor debug information over serial at `9600`.
- Uses a clean motor-channel structure that stores only the three real motor-control pins:
  - `speedPin`
  - `directionPin1`
  - `directionPin2`

### Current Configuration

- Rear motor
  - `ENA`: `D3`
  - `IN1`: `D8`
  - `IN2`: `D4`

- Front-left motor
  - `ENB`: `D5`
  - `IN3`: `D7`
  - `IN4`: `D10`

- Front-right motor
  - `ENA`: `D6`
  - `IN1`: `D11`
  - `IN2`: `D12`

### Important Notes

- The current Arduino firmware is based on the exact Nano wiring that was provided during setup.
- The speed pins are now mapped to valid Arduino Nano PWM outputs: `D3`, `D5`, and `D6`.
- The current firmware no longer reads joystick or analog inputs directly.
- The current firmware is in motor test mode: rear, front-left, and front-right are all driven together.
- The current firmware does not yet parse serial `CTRL ...` commands from the laptop control station, but it does accept the `banana` toggle command.
- Motor labels and current drive values are stored separately from the motor-channel pin structure.

## Control Station

### Entry Files

- `src/control_station/main.py`
  - Thin entry point that starts the control station app.
- `src/control_station/run_control_station.sh`
  - Launches the main control station window.
- `src/control_station/run_controller_debug.sh`
  - Launches the separate controller debug window.

### Main Python Modules

- `src/control_station/station/app.py`
  - Main runtime loop, startup, argument parsing, and mode selection.
- `src/control_station/station/controller.py`
  - Reads controller state from `pygame`.
- `src/control_station/station/geometry.py`
  - Converts movement inputs into buoy motor command values.
- `src/control_station/station/serial_link.py`
  - Handles Bluetooth serial communication to the buoy.
- `src/control_station/station/ui.py`
  - Draws the main interface and the controller debug interface.
- `src/control_station/station/models.py`
  - Shared data structures for commands and controller snapshots.
- `src/control_station/station/settings.py`
  - Centralized constants for colors, window size, and defaults.

### Current Control Station Configuration

- Controller input source
  - Xbox-style controller through `pygame`
- Main movement input
  - left stick X for horizontal movement
  - left stick Y for forward and reverse movement
- Manual command format
  - `CTRL <turn> <thrust> <rear_motor> <front_left_motor> <front_right_motor>`
- Hello ping mode
  - optional repeated `hello world` messages for Bluetooth link testing
- Bluetooth auto-discovery
  - defaults to looking for the device name `DSDTECHHC-05`
- Main interface
  - minimalist movement vector plus buoy layout
- Debug interface
  - generic controller inspector
  - analog inputs shown as live bars
  - digital inputs shown as labeled `ON/OFF` indicators

### Important Notes

- The control station is modular on purpose so future systems can be added without growing one large file.
- The debug window is meant for controller discovery and mapping, not for the main buoy UI.
- The control station already assumes a three-motor buoy layout for movement visualization and command generation.
- The serial layer can now auto-discover a Bluetooth serial port by device name when an explicit `--port` is not provided.
- The app also supports a ping-only send mode to quickly verify serial RX (`--hello-ping`).

## Configuration Decisions We Are Currently Making

- The buoy uses three motors:
  - rear
  - front-left
  - front-right
- The control station and firmware are being developed in parallel.
- The control station is prepared for serial command-based control.
- The joystick logic and the Arduino motor-output logic are now being kept separate.
- The Arduino firmware is currently acting as a simple simultaneous hardware test instead of taking live control commands.
- The next major integration step is likely to connect the laptop `CTRL ...` serial output to Arduino motor control logic.

## When To Update This File

Update this file when:

- a source file is added or renamed
- a motor pin mapping changes
- the serial protocol changes
- the control station behavior changes
- a major code module gets a new responsibility
