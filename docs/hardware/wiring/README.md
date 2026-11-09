# Current ESP32 Buoy Wiring

This folder contains only the active, firmware-compatible wiring. Start with the consolidated [ESP32 pin map](esp32-pin-map.md), then use the page for each device.

## Source Of Truth

Signal assignments were checked against:

`src/firmware/esp32/esp32-buoy-firmware/esp32-buoy-firmware.ino`

The firmware cannot prove wire colors, connector order, supply voltage, or which board revision is physically installed. Verify those details on the actual module before power-up.

## Active Connections

| Device or subsystem | Interface | Current wiring page |
| --- | --- | --- |
| ESP32 development board | All signals | [Consolidated ESP32 pin map](esp32-pin-map.md) |
| Three motors and three dual-PWM H-bridges | Six PWM outputs | [Motor wiring](motor-wiring-sheet.md) |
| HGLRC M100-5883 GPS and compass | UART1 receive + shared I2C | [GPS and compass wiring](gps-wiring-map.md) |
| MPU6050 | I2C at `0x68` | [MPU6050 wiring](mpu6050-wiring-map.md) |
| ADS1115 | I2C at `0x48` | [ADS1115 wiring](ads1115-wiring-map.md) |
| ACS712 30 A sensor | ADS1115 `A0` through divider | [Current-sensor wiring](acs712-current-sensor-wiring-map.md) |
| Battery-voltage module | ADS1115 `A1` | [Battery-voltage wiring](battery-voltage-sensor-wiring-map.md) |
| TDS and turbidity modules | ADS1115 `A2` and `A3` | [Water-quality wiring](water-quality-wiring-map.md) |
| JSN-SR04T V3.0 | Trigger and echo | [JSN-SR04T V3.0 wiring](jsn-sr04t-v3-wiring-map.md) |
| DS18B20 | 1-Wire | [DS18B20 wiring](DS18B20-wiring-map.md) |
| Two INMP441 microphones | Shared stereo I2S | [Microphone wiring](inmp441-microphones-wiring-map.md) |
| microSD module | Dedicated SPI | [microSD wiring](microsd-wiring-map.md) |
| SkyGuy Nano ELRS receiver | UART2 MAVLink | [ELRS receiver wiring](elrs-mavlink-wiring-map.md) |

Wi-Fi is built into the ESP32 and has no external signal wiring. The active firmware uses the ELRS receiver in MAVLink mode; the CRSF sketch remains a separate bench test.

## Important SD / USB Limitation

The SD chip-select line uses `GPIO1`, which is normally UART0 transmit. With the SD controller wired, normal USB-serial logging is unavailable and programming may be inconvenient or fail depending on the attached module. Disconnect the SD controller from `GPIO1` when flashing if required by the bench setup.

## Archived Designs

The following are design history, not current instructions:

- [Pre-ADS1115 ESP32 pin map](../../archive/hardware/wiring/esp32-pin-map-pre-ads1115.md)
- [Earlier ADS1115/ELRS/microSD expansion proposal](../../archive/hardware/wiring/alternative-ads1115-elrs-microsd-water-quality-wiring.md)
- [Legacy Arduino Nano and HC-05 wiring](../../archive/hardware/wiring/legacy-nano-hc05-wiring.md)

## Before Power-Up

- Confirm a common logic ground where required.
- Confirm that every ESP32 and ADS1115 input remains within its electrical limits.
- Verify each module's supply rail and pin order from its exact markings or datasheet.
- Keep motor current out of the breadboard and ESP32 ground paths.
- Commission one subsystem at a time, beginning with power off and continuity checks.
