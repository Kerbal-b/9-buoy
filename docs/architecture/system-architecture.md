# Buoy System Architecture

## Purpose

The buoy is a small, ESP32-controlled surface platform for collecting georeferenced environmental observations in the Redwood Shores Lagoon. It combines water measurements, movement, remote operation, onboard recording, and a physical platform that can carry the equipment safely.

## Main System Groups

| System | Current design |
| --- | --- |
| Controller | ESP32 development board. It reads sensors, drives motor controllers, runs Wi-Fi communication, captures audio, and writes logs to microSD. |
| Propulsion | Three 7.4 V ICHIFIT underwater thrusters, each controlled by an IBT-4-style dual-PWM MOSFET H-bridge. Module 0.8 bevel gears and 3 mm shafts transfer drive to the propellers. |
| Navigation and control links | HGLRC M100-5883 GPS provides position and speed; its QMC5883 provides raw compass axes for heading calibration. Wi-Fi links the ESP32 with the laptop control station. The SkyGuy Nano ExpressLRS receiver provides direct TX12 motor control over MAVLink on UART2; Backpack telemetry is optional for control. |
| Science | DS18B20 water temperature; Keyestudio KS0429 TDS and KS0414 turbidity sensors through ADS1115 channels A2 and A3; JSN-SR04T V3.0 range measurement from above the water; two INMP441 microphones for stereo acoustic recordings; MPU6050 motion and vibration data. |
| Power | Battery supply and distribution, electrical monitoring, and planned solar charging. The ADS1115 reads battery voltage and current-sensor signals. The listed battery is a `12 V 2600 mAh` rechargeable Li-ion pack with a listed `12.6 V 1 A` charger. A separate `MPPT Solar Charge Controller Li-ion LiFePO4 Lithium Battery Charger Module 1A 12V` and a `5.5 V 190 mA 70 × 70 mm` mini polycrystalline panel are identified; compatibility across the charger, panel, and pack is unverified. Regulators, fusing, and final distribution still require definition. |
| Storage | Mini TF/microSD SPI module stores timestamps, logs, telemetry, scientific measurements, and audio when a card is available. |
| Operator Equipment | Laptop control station and game controller; TX12 and ELRS receiver for direct manual control; GL.iNet Opal router hosting the receive-only MAVLink phone dashboard. |

The hull, enclosure, flotation, motor mounts, and wiring supports form the physical platform that carries these systems. The K'NEX frame and layered hull remain prototype/build decisions documented in [Physical Design](physical-design.md).

## Data And Command Flow

1. Sensors and the ADS1115 provide measurements to the ESP32; the GPS supplies position over UART.
2. The ESP32 accepts TX12 channel control directly from the ELRS receiver over UART2. It also sends telemetry to the laptop over Wi-Fi and accepts optional keyboard movement commands from the control station.
3. The ESP32 writes available logs, science data, telemetry, and audio to microSD.
4. Motor PWM signals pass from the ESP32 to the three H-bridge drivers and thrusters.

The field telemetry path is ESP32 → ELRS MAVLink receiver → TX Backpack UDP forwarding → GL.iNet Opal bridge (UDP 14550) → phone browser (HTTP 8080). The bridge only receives MAVLink and serves the dashboard/API; it does not send MAVLink or control the buoy. The PC control station is a separate two-way Wi-Fi control path. Its interfaces and mission/session behavior are documented in the [software behavior guide](../software/system-behavior.md), [MAVLink reference](../software/mavlink-reference.md), and [router API contract](../../src/router_bridge/API.md).

## Current Boundaries

- Manual control, sensor acquisition, SD logging, and the local router dashboard have been exercised in stages. The bridge/API returned data and the page was viewed on an iPhone on November 8, 2026; that packet sample did not contain GPS position. Concurrent PC QGC and phone dashboard operation has not been proven.
- Mission route planning, upload/readback, status, execution logic, and start interlocks exist in software. The November 8, 2026 water test did not pass the start gate: nine attempts were blocked because calibrated heading was unavailable. No successful mission run is validated. Treat mission navigation, collection checkpoints, and return behavior as unvalidated until a successful end-to-end water test is recorded.
- TDS and turbidity are currently sensor-voltage measurements; defensible ppm and NTU values require calibration. The isolated TDS bench test can show a manufacturer-equation estimate using assumed 25 °C.
- The JSN-SR04T measures range through air. It is not an immersed sonar transducer; water depth would require a surveyed reference height and an air-gap calculation.
- The active firmware uses ELRS MAVLink for direct TX12 control. The separate CRSF sketch is a bench test and is not the production receiver protocol.
- Solar charging and the complete battery/power-distribution design are not finalized. Do not charge the pack with the separate MPPT module until battery chemistry/cell configuration and charger specifications are verified as compatible; confirm that the 5.5 V panel meets the controller's solar-input requirements. The listing identifies a separate 12.6 V, 1 A charger with the battery.
- microSD chip select uses ESP32 GPIO1, which can interfere with USB serial logging and flashing while connected.

## Reference Documents

- [Component inventory and identification status](../hardware/components/README.md)
- [Firmware interface map](../hardware/components/firmware-interface-map.md)
- [Wiring index](../hardware/wiring/README.md)
- [Physical Design](physical-design.md)
- [Testing Plan](testing-plan.md)

This document summarizes the system. The firmware and current wiring pages govern implemented interfaces; the inventory identifies which physical parts are confirmed.
