# Buoy Component Documentation Library

This folder is the organized starting point for identifying, wiring, testing, and documenting the hardware used by the ESP32 buoy.

## What Is Complete

- The active firmware interfaces and pin assignments have been extracted into [Firmware interface map](firmware-interface-map.md).
- Existing project wiring pages are linked from the inventory below.
- Manufacturer or vendor documentation links are collected in [Electronics and sensors](electronics-and-sensors.md) and [Propulsion, power, and structure](propulsion-power-and-structure.md).
- Current Amazon purchase references from the live component sheet are collected in [Current component product links](product-links.md).
- The repository now contains the existing ESP32 photograph, exact Keyestudio manufacturer images, user-supplied product images, and a remaining representative DS18B20 image in [`images/`](images/).

## Important Limitation

We do **not** yet have an actual photograph or exact model number for every physical part. A product or representative image is useful for recognition, but it is not proof that the installed breakout has the same regulator, level shifter, supply voltage, or pin order. See [Photo and identification checklist](photo-and-identification-checklist.md).

## Inventory Summary

| Subsystem | Component | Quantity | Firmware status | Documentation status |
| --- | --- | ---: | --- | --- |
| Controller | ESP32 development board | 1 | Active | Photo and pin map available; exact board manufacturer still needs confirmation |
| Movement | ICHIFIT 7.4 V, 16800 RPM underwater thrusters | 3 | Installed / active propulsion | Installed model confirmed; product image available |
| Movement | GODIYMODULES/IBT-4-style dual-PWM MOSFET H-bridges | 3 | Installed / active | Installed model confirmed; purchase link, product image, and wiring sheet available |
| Movement | Module 0.8 bevel gears | 3 sets | Mechanical | Basic description only; exact dimensions/source need recording |
| Movement | 3 mm drive shafts | 3 | Mechanical | Basic description only; material and length need recording |
| Navigation | HGLRC M100-5883 GPS and QMC5883 compass | 1 | Installed / active | GPS parsing and raw compass telemetry supported; calibration and mission-heading validity logic implemented, but heading reliability is unverified and blocked the Nov 8, 2026 mission start attempts |
| Motion | MPU6050 / GY-521 IMU | 1 | Active | Wiring, chip documentation, and representative photo available |
| Depth/range | JSN-SR04T V3.0 waterproof ultrasonic module | 1 | Installed / active | Model confirmed; firmware and wiring documentation updated |
| Temperature | Waterproof DS18B20 probe | 1 | Active | Wiring, datasheet, and representative photo available |
| Water quality | Keyestudio KS0414 Turbidity Sensor V1.0 | 1 | Read by firmware as voltage | Exact official documentation and photo available; wet calibration pending |
| Water quality | Keyestudio KS0429 TDS Meter V1.0 | 1 | Read by firmware as voltage | Exact official documentation and photo available; temperature compensation/calibration pending |
| Audio | Qoroos INMP441 microphone breakouts | 2 | Installed / active stereo I2S | Installed model confirmed; actual-board L/R straps still need recording |
| Analog acquisition | ADS1115 16-bit ADC breakout | 1 | Installed / active at I2C `0x48` | Installed board model and chip documentation available |
| Power sensing | ACS712 30 A current-sensor module | 1 | Active through ADS1115 A0 | Wiring and chip datasheet available; actual module photo needed |
| Power sensing | 0-25 V voltage-divider module | 1 | Active through ADS1115 A1 | Wiring and purchase link available; actual module photo needed |
| Storage | Mini TF/microSD SPI module and card | 1 | Installed / active | Installed module model and firmware pins documented; electrical interface still requires care |
| Radio | ESP32 Wi-Fi | Built in | Active | No external wiring |
| Radio | SkyGuy Nano 2.4 GHz ExpressLRS receiver | 1 | Active TX12 control through production ELRS MAVLink firmware | Exact model confirmed as `SG-NANO-2G4-RX`; UART2 wiring and channel/arming behavior documented; verify installed radio settings and physical wiring |
| Structure | IP67 ABS junction box | 1 | Mechanical | Purchase link available; actual installation photo needed |
| Structure | K'NEX prototype frame | As required | Mechanical | General description available |
| Power source | `12 V 2600 mAh` rechargeable Li-ion pack with listed `12.6 V 1 A` charger; separate 1 A, 12 V MPPT Li-ion/LiFePO4 charger module and 5.5 V, 190 mA, 70 × 70 mm mini polycrystalline panel identified | Battery, charger, and panel listed; compatibility of MPPT module/panel/pack unverified | Required | Exact models, cell configuration, discharge/protection details, panel/MPPT input compatibility, fuse, regulator, switch, connectors, and wire sizes still need documentation |
| Operator equipment | Laptop and Xbox-style controller | 1 each | Active | Control-station software is documented under `src/control_station/` |
| Operator network | GL.iNet Opal GL-SFT1200 router | 1 | Installed / active field access point and local dashboard/API host | Firmware v4.3.28 release 2, LuCI, Python 3.6.5-1, ttyd browser terminal, and the receive-only MAVLink bridge are installed. UDP 14550 in; HTTP 8080 out. See [software inventory](../../software/software-inventory.md) and [software behavior guide](../../software/system-behavior.md). Live GPS display and PC QGC coexistence remain unverified. |

## Recommended Reading Order

1. [Firmware interface map](firmware-interface-map.md) - what the current code actually expects.
2. [Electronics and sensors](electronics-and-sensors.md) - pictures, purpose, wiring links, and datasheets.
3. [Propulsion, power, and structure](propulsion-power-and-structure.md) - motors, drivers, enclosure, and missing power details.
4. [Current component product links](product-links.md) - purchase listings recorded in the live inventory sheet.
5. [Photo and identification checklist](photo-and-identification-checklist.md) - what must still be photographed or identified.
6. [Detailed device wiring pages](../wiring/README.md).

## Source Of Truth

For GPIO assignments, the current source of truth is:

`src/firmware/esp32/esp32-buoy-firmware/esp32-buoy-firmware.ino`

Physical wire colors, connector order, board revisions, supply voltages, and installed components must be verified on the actual buoy.
