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
- Update the centralized wiring documentation in `docs/wiring/` whenever a device or connection changes; do not duplicate schematics in this file.

## Main Source Areas

- `src/firmware/arduino/`
  - Arduino Nano production firmware for the buoy hardware.
- `src/firmware/esp32/`
  - ESP32 production firmware for the buoy hardware.
- `src/firmware/esp32-test/`
  - Standalone ESP32 bring-up firmware used to validate board upload, serial, and basic GPIO.
- `src/control_station/`
  - Laptop-side control station for controller input, debug tools, and future buoy communication.

## Nano Firmware

### File

- `src/firmware/arduino/arduino-buoy-firmware.ino`

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
TEL SCI IMU_ACCEL <x_g|UNKNOWN> <y_g|UNKNOWN> <z_g|UNKNOWN>
TEL SCI IMU_GYRO <x_dps|UNKNOWN> <y_dps|UNKNOWN> <z_dps|UNKNOWN>
TEL SCI IMU_TEMP <c|UNKNOWN>
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

- Battery voltage sensor
  - `0-25V module OUT`: `A1`

### Legacy Wiring Documentation

The historical Nano, HC-05, motor, and sensor schematic is retained in `docs/wiring/legacy-nano-hc05-wiring.md`. The active ESP32 wiring starts at `docs/wiring/README.md`.

### Important Notes

- The current Nano firmware is based on the exact Nano wiring that was provided during setup.
- The speed pins are now mapped to valid Arduino Nano PWM outputs: `D3`, `D5`, and `D6`.
- The HC-05 Bluetooth module is now intended to use `SoftwareSerial` on `D2` (Arduino RX, module TX) and `D10` (Arduino TX, module RX).
- The current firmware no longer reads joystick or analog inputs directly.
- The current firmware now uses `CTRL ...`, `PING`, and `REQ STATUS ALL` as the primary protocol surface.
- Legacy commands such as `banana`, `start`, `stop`, and `current` are still accepted for transition compatibility.
- The target protocol should use `CTRL`, `ACK`, `ERR`, `TEL STATUS`, and `TEL SCI` prefixes consistently.
- Motor vector math and output application now live in the motor-control section.
- Bluetooth parsing and protocol responses now live in the communications section.
- Current-sensor logic now lives in the operational-sensors section.
- The battery voltage sensor is currently assumed to be a standard 0-25V analog module on `A1`.
- Battery percentage is currently estimated from a 3S lithium-ion range of `12.6V` full to `9.6V` empty.
- Keepalive `PING` responses should send a status snapshot so newly added operational sensors appear in normal telemetry without requiring a separate request.
- The scientific-sensors and safety sections are placeholders for future expansion.

## ESP32 Controller Wiring

The active firmware assigns UART1 to GPS, keeps UART0 for USB/debug, and leaves UART2 reserved. The former migration draft and duplicated pin list were removed so they cannot drift from the production firmware and centralized wiring documents.

### ESP32 Wiring Documentation

All active-build schematics, device connection tables, physical verification notes, and the consolidated ESP32 pin map are maintained in `docs/wiring/`. Start with `docs/wiring/README.md`.

The production firmware remains the source of truth for active GPIO assignments. Do not add a second schematic here; update the device file and index in `docs/wiring/` instead.

### ESP32 Pin-Selection Notes

- Avoid `GPIO6` to `GPIO11` (connected to onboard flash on most dev boards).
- Avoid boot strap pins for critical outputs during reset (`GPIO0`, `GPIO2`, `GPIO12`, `GPIO15`).
- Keep analog sensing on `ADC1` input-only pins when possible; the current plan uses `GPIO34` and `GPIO35` for sensors.
- `GPIO34` and `GPIO35` are input-only, which is ideal for analog sensors.
- `GPIO14` is used for the DS18B20 1-Wire data line in the current wiring plan.
- `GPIO36` (`VP`) is used for the INMP441 microphone data line and is input-only, which matches the I2S microphone output.
- `GPIO39` is input-only, which is ideal for the RCWL-1655 echo pulse.
- `GPIO13` is reserved for the RCWL-1655 trigger pulse in the current wiring plan.
- Motor control is currently assigned to `GPIO18/19/21/22/32/33` to match the latest wiring layout.
- I2C is currently assigned to `GPIO23/27` so it does not conflict with the front-right motor on `GPIO21/22`.
- The current control architecture uses onboard Wi-Fi for command and telemetry transport, so `UART2` remains available for GPS swap, secondary radio, or diagnostics.
- The RCWL-1655 now feeds both periodic UDP `RANGE` packets and text `TEL SCI DEPTH` updates.
- The ESP32 firmware uses `OneWire` + `DallasTemperature` on `GPIO14` for `TEL SCI WATER_TEMP` support.

## ESP32 Production Firmware

### File

- `src/firmware/esp32/esp32-buoy-firmware/esp32-buoy-firmware.ino`

### What It Does

- Implements buoy motor control and telemetry protocol directly on ESP32.
- Uses all three ESP32 UART controllers with explicit roles:
  - `UART0`: USB programming/debug monitor
  - `UART1`: GPS
  - `UART2`: reserved for future expansion
- Joins the buoy Wi-Fi network as a station in the current build.
- Accepts reliable control commands over TCP.
- Streams fast telemetry over UDP using compact binary packets for IMU, range, power, state, GPS, and stereo audio data, and emits DS18B20 water-temperature text telemetry.
- Uses grouped motor pins to simplify physical harness routing.
- Inverts motor polarity in software to match the current water-tested buoy wiring and propeller orientation.
- Keeps protocol compatibility with existing `CTRL ...`, `PING`, and `REQ STATUS ALL` command flow.

## ESP32 Bring-Up Test Firmware

### File

- `src/firmware/esp32-test/esp32-firmware-test.ino`

### What It Does

- Boots on ESP32 and starts USB serial at `115200`.
- Starts onboard BLE as `ESP32-BUOY`.
- Exposes HM-10 compatible BLE UUIDs (`FFE0` service and `FFE1` characteristic) so the existing control station can connect.
- Accepts buoy protocol commands over BLE write:
  - `CTRL VECTOR <turn> <thrust>`
  - `CTRL STOP`
  - `CTRL HOLD <ON|OFF>`
  - `CTRL GOTO <lat> <lon>`
  - `PING`
  - `REQ STATUS ALL`
- Returns protocol-compatible messages (`ACK ...`, `ERR ...`, `TEL STATUS ...`, `TEL SCI ...`) for control-station integration testing.

### Notes

- This firmware is intentionally isolated from buoy production logic.
- Use it only for controller bring-up and quick board health checks.

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
  - Handles Wi-Fi, BLE, and serial communication to the buoy.
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
  - still available for the legacy Bluetooth serial path
- Wi-Fi control target
  - defaults to auto-discovery of the buoy host for the current ESP32 Wi-Fi link
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
- The control station is prepared for serial, BLE, and Wi-Fi command-based control.
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
