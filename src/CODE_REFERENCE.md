# Source Code Reference

This file explains what the main source files mean and what configuration decisions are currently active.

## Change Workflow Rules

- When changing the buoy firmware, change one section at a time whenever possible.
- If two sections are tightly coupled, it is acceptable to change those two sections together in one pass.
- Prefer structural refactors that preserve behavior before making functional changes.
- Keep subsystem boundaries clear:
  - motor control should contain vector math and motor-output behavior
  - communications should parse commands and send responses, not contain motor or sensor logic inline
  - operational sensors should cover buoy-health and navigation hardware
  - scientific sensors should cover environmental and research measurements
  - telemetry/reporting should format and emit status from those subsystems
- Update this reference file when the firmware structure, protocol, or subsystem responsibilities change.
- Update the wiring diagram in this file whenever a new device is added or a connection changes.

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
- Receives Bluetooth text commands and applies buoy motor control from `CTRL VECTOR <turn> <thrust>`.
- Computes individual thrust values for the rear, front-left, and front-right motors from a 120-degree vector model.
- Measures buoy current draw through the ACS712 current sensor.
- Sends protocol acknowledgements and current telemetry back over Bluetooth.
- Uses a sectioned structure so the firmware can grow by subsystem instead of as one long file.

### Current Firmware Sections

- Configuration / Protocol Constants
- Data Structures
- Hardware Configuration
- Runtime State
- Utility Helpers
- Motor Control
- Operational Sensors
- Scientific Sensors
- Telemetry / Reporting
- Communications / Bluetooth Protocol
- Safety / Fault Handling
- Arduino Lifecycle

### Protocol Reference

Commands sent to the buoy should be grouped by purpose.

Control commands:

```text
CTRL STOP
CTRL VECTOR <x> <y>
CTRL HOLD <ON|OFF>
CTRL GOTO <lat> <lon>
```

Transport and request commands:

```text
PING
REQ STATUS ALL
```

Acknowledgements and errors:

```text
ACK CTRL STOP
ACK CTRL VECTOR <x> <y>
ACK CTRL HOLD <ON|OFF>
ACK CTRL GOTO <lat> <lon>
ACK PING
ACK REQ STATUS ALL
ERR <reason>
```

Telemetry is split into short messages by category instead of one large message.

Operational status telemetry:

```text
TEL STATUS MODE <mode>
TEL STATUS POS <lat|UNKNOWN> <lon|UNKNOWN>
TEL STATUS TARGET <lat|UNKNOWN> <lon|UNKNOWN>
TEL STATUS HOLD <ON|OFF>
TEL STATUS BATTERY <volts|UNKNOWN> <amps|UNKNOWN> <percent|UNKNOWN>
TEL STATUS CURRENT <amps|UNKNOWN>
```

Scientific telemetry:

```text
TEL SCI WATER_TEMP <c|UNKNOWN>
TEL SCI AIR_TEMP <c|UNKNOWN>
TEL SCI DEPTH <m|UNKNOWN>
```

### Current Configuration

- Rear motor
  - `PWM`: `D3`
  - `DIR`: `D8`

- Front-left motor
  - `PWM`: `D5`
  - `DIR`: `D7`

- Front-right motor
  - `PWM`: `D6`
  - `DIR`: `D11`

- Bluetooth module
  - `RX`: `D2`
  - `TX`: `D10`

- Current sensor
  - `ACS712 OUT`: `A0`

### Module-Based Wiring Diagram

#### Motor Drive

```text
[ Arduino Nano ]
   |
   |-- D3  (PWM) -> Rear motor driver speed input
   |-- D8  (DIR) -> Rear motor driver direction input
   |               direction reverse path uses 74HC14 inverter
   |
   |-- D5  (PWM) -> Front-left motor driver speed input
   |-- D7  (DIR) -> Front-left motor driver direction input
   |               direction reverse path uses 74HC14 inverter
   |
   |-- D6  (PWM) -> Front-right motor driver speed input
   |-- D11 (DIR) -> Front-right motor driver direction input
                   direction reverse path uses 74HC14 inverter
```

#### Communication

```text
[ Laptop Control Station ]
   |
   |  Bluetooth commands and telemetry
   v
[ Bluetooth Module ]
   |
   |-- TX -> Arduino D2  (SoftwareSerial RX)
   |-- RX <- Arduino D10 (SoftwareSerial TX)
   |-- VCC -> Arduino 5V   assumed
   |-- GND -> Arduino GND  assumed
```

#### Operational Sensors

```text
[ ACS712 Current Sensor ]
   |
   |-- OUT -> Arduino A0
   |-- VCC -> Arduino 5V
   |-- GND -> Arduino GND
```

This is the only operational sensor currently implemented in firmware.

#### Scientific Sensors

```text
Not wired in current firmware yet.

Planned examples:
- Water temperature sensor
- Air temperature sensor
- Depth / pressure sensor
- Other environmental probes
```

### Important Notes

- The current Arduino firmware is based on the exact Nano wiring that was provided during setup.
- The speed pins are now mapped to valid Arduino Nano PWM outputs: `D3`, `D5`, and `D6`.
- The HC-05 Bluetooth module is now intended to use `SoftwareSerial` on `D2` (Arduino RX, module TX) and `D10` (Arduino TX, module RX).
- The current firmware no longer reads joystick or analog inputs directly.
- The current firmware now uses `CTRL ...`, `PING`, and `REQ STATUS ALL` as the primary protocol surface.
- Legacy commands such as `banana`, `start`, `stop`, and `current` are still accepted for transition compatibility.
- The target protocol should use `CTRL`, `ACK`, `ERR`, `TEL STATUS`, and `TEL SCI` prefixes consistently.
- Motor vector math and output application now live in the motor-control section.
- Bluetooth parsing and protocol responses now live in the communications section.
- Current-sensor logic now lives in the operational-sensors section.
- The scientific-sensors and safety sections are placeholders for future expansion.

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
  - `CTRL VECTOR <turn> <thrust>`
- Hello ping mode
  - optional repeated `hello world` messages for Bluetooth link testing
- Bluetooth auto-discovery
  - defaults to looking for the device name `DSD TECH`
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
- The Arduino firmware currently takes live `VECTOR` commands and computes motor thrust onboard.
- Firmware changes should be scoped section-by-section to keep refactors controlled.

## When To Update This File

Update this file when:

- a source file is added or renamed
- a motor pin mapping changes
- a device is added to the buoy wiring
- a wiring connection changes
- the serial protocol changes
- the control station behavior changes
- a major code module gets a new responsibility
