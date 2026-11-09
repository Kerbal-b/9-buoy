# Firmware Interface Map

This page records the interfaces currently compiled into `esp32-buoy-firmware.ino` as reviewed on 2026-11-08. It supersedes older GPIO tables when they disagree with the firmware. For control modes, mission start interlocks, telemetry, and SD recording behavior, see the [software behavior guide](../../software/system-behavior.md).

## ESP32 GPIO Assignments

| GPIO | Direction | Current function | Component |
| ---: | --- | --- | --- |
| `1` | Output | SPI chip select | microSD module |
| `2` | Output | Status LED | ESP32 onboard LED |
| `4` | Output | I2S word select | INMP441 microphone pair |
| `5` | Output | I2S bit clock | INMP441 microphone pair |
| `13` | Output | Trigger | JSN-SR04T V3.0 |
| `14` | Bidirectional | 1-Wire data | DS18B20 |
| `16` | Input | UART1 receive at 115200 baud | HGLRC M100-5883 GPS TX |
| `17` | Output | SPI clock | microSD module |
| `18` | PWM output | IN2 | Front-right motor driver |
| `19` | PWM output | IN1 | Front-right motor driver |
| `21` | PWM output | IN2 | Front-left motor driver |
| `22` | PWM output | IN1 | Front-left motor driver |
| `23` | Bidirectional | I2C SDA | MPU6050, ADS1115, and QMC5883 compass |
| `26` | Output | SPI MOSI | microSD module |
| `27` | Output | I2C SCL | MPU6050, ADS1115, and QMC5883 compass |
| `32` | PWM output | IN2 | Rear motor driver |
| `33` | PWM output | IN1 | Rear motor driver |
| `35` | Input only | SPI MISO | microSD module |
| `36` / `VP` | Input only | I2S data | INMP441 microphone pair |
| `39` | Input only | Echo | JSN-SR04T V3.0 |

The HGLRC M100-5883 GPS is receive-only at `115200 baud`. `GPS_TX_PIN` is `-1`, so the ESP32 does not transmit configuration data to it. GPS parsing includes position, fix, satellite count, HDOP, speed/course, and age/validity diagnostics. The M100-5883 QMC5883 compass is initialized at `0x0D`; raw X/Y/Z counts stream for diagnostics, and stored calibration is used to compute mission heading. Heading availability/reliability is not yet validated under motor load; mission start attempts on 2026-11-08 were blocked when the heading was unavailable.

## I2C Devices

| Address | Device | Role |
| --- | --- | --- |
| `0x48` | ADS1115 | Four analog input channels |
| `0x68` | MPU6050 | Acceleration, angular rate, and internal temperature |
| `0x0D` | QMC5883 | Raw three-axis magnetic-field counts; stored calibration and mission-heading validity checks are implemented |

## ADS1115 Channel Assignments

| Channel | Sensor | Firmware interpretation |
| ---: | --- | --- |
| `A0` | ACS712 30 A current sensor through a `0.60` divider | Amps after zero-offset calibration and `0.066 V/A` sensitivity |
| `A1` | Battery voltage-divider module | System volts using the stored battery calibration |
| `A2` | Keyestudio KS0429 TDS module | Sensor voltage only; wet calibration is not complete |
| `A3` | Keyestudio KS0414 turbidity module through a `0.60` divider | Reconstructed sensor voltage only; NTU calibration is not complete |

## microSD Interface

| microSD signal | ESP32 pin |
| --- | ---: |
| `CS` | `GPIO1` |
| `SCK` | `GPIO17` |
| `MOSI` | `GPIO26` |
| `MISO` | `GPIO35` |

The firmware requests a `20 MHz` SPI clock. Because `GPIO1` is normally the ESP32 UART0 transmit pin, this configuration interferes with ordinary USB-serial logging and can make flashing/debugging inconvenient while the SD controller is attached.

## Devices Not Implemented In The Active Firmware

- pH sensor
- dissolved-oxygen sensor
- tilt-compensated magnetic heading (the mission heading implementation uses its calibrated heading path; tilt compensation is not claimed here)

Do not infer wiring for these devices from unused pins. Their interfaces need a deliberate design and firmware change first.

## Related Project Documents

- [Current ESP32 pin map](../wiring/esp32-pin-map.md)
- [Archived ADS1115, ELRS, microSD, and water-quality proposal](../../archive/hardware/wiring/alternative-ads1115-elrs-microsd-water-quality-wiring.md) - design history only; the ELRS portion was not implemented.
- [Firmware source](../../../src/firmware/esp32/esp32-buoy-firmware/esp32-buoy-firmware.ino)
