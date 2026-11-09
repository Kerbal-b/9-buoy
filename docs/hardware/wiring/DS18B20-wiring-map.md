# DS18B20 Wiring Map

The DS18B20 is a 1-Wire digital temperature sensor. In this project it is a good fit for water temperature or other environmental temperature measurements.

Firmware source: `src/firmware/esp32/esp32-buoy-firmware/esp32-buoy-firmware.ino` (`DS18B20_DATA_PIN`).

## Pin Functions

- `VDD`: power input, `3.0V` to `5.5V`
- `GND`: ground
- `DQ`: 1-Wire data line

## Buoy Project Map

Recommended wiring for the current `ESP32` buoy controller:

| DS18B20 Pin | Connect To | Notes |
| --- | --- | --- |
| `VDD` | `ESP32 3.3V` | Power it from `3.3V` for safe logic-level compatibility. |
| `GND` | `ESP32 GND` | Common ground is required. |
| `DQ` | `GPIO14` | Dedicated 1-Wire data pin with a `4.7 kOhm` pull-up to `3.3V`. |

## Simple Wiring Diagram

```text
ESP32 3.3V ---> VDD
ESP32 GND   ---> GND
ESP32 GPIO14 <--> DQ
               |
               +--- 4.7 kOhm pull-up to 3.3V
```

## Notes For This Project

- Use `3.3V` power so the data line stays within safe ESP32 logic levels.
- Add a `4.7 kOhm` pull-up resistor between `DQ` and `3.3V`.
- `GPIO14` is used here because the microphone data line has been moved to `VP / GPIO36`.
- Keep the DS18B20 line separate from the motor PWM pins, I2C bus, JSN-SR04T trigger/echo pins, and current/battery analog sensing pins.
- The firmware uses this sensor for `TEL SCI WATER_TEMP` updates.

## 1-Wire Reminder

The DS18B20 uses 1-Wire, so it is not wired like an analog sensor:

- one data wire carries commands and temperature readings
- the pull-up resistor is required for reliable bus operation
- multiple DS18B20 sensors can share the same data pin if needed later
