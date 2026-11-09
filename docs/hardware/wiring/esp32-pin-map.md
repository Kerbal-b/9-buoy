# ESP32 Pin Map — Active Firmware

This is the consolidated connection map for the current production firmware.

| GPIO | Direction | Connection |
| ---: | --- | --- |
| `1` | Output | microSD `CS`; conflicts with normal UART0 TX/USB-serial logging |
| `2` | Output | Onboard status LED |
| `4` | Output | INMP441 pair `WS/LRCLK` |
| `5` | Output | INMP441 pair `SCK/BCLK` |
| `13` | Output | JSN-SR04T V3.0 `TRIG` |
| `14` | Bidirectional | DS18B20 1-Wire data |
| `16` | Input | HGLRC M100-5883 GPS `TX` to ESP32 UART1 RX at 115200 baud |
| `17` | Output | microSD `SCK` |
| `18` | PWM output | Front-right motor driver `IN2` |
| `19` | PWM output | Front-right motor driver `IN1` |
| `21` | PWM output | Front-left motor driver `IN2` |
| `22` | PWM output | Front-left motor driver `IN1` |
| `23` | Bidirectional | I2C `SDA`: ADS1115, MPU6050, and M100-5883 QMC5883 compass |
| `25` | Output | ELRS receiver `RX`: UART2 TX |
| `26` | Output | microSD `MOSI` |
| `27` | Output | I2C `SCL`: ADS1115, MPU6050, and M100-5883 QMC5883 compass |
| `32` | PWM output | Rear motor driver `IN2` |
| `33` | PWM output | Rear motor driver `IN1` |
| `34` | Input | ELRS receiver `TX`: UART2 RX |
| `35` | Input | microSD `MISO` |
| `36` / `VP` | Input | INMP441 pair `SD` data |
| `39` / `VN` | Input | JSN-SR04T V3.0 `ECHO` |

## External ADC Channels

The analog sensors do not connect to the ESP32's internal ADC pins.

| ADS1115 input | Sensor |
| --- | --- |
| `A0` | ACS712 output through a `0.60` voltage divider |
| `A1` | Battery-voltage module output |
| `A2` | Keyestudio KS0429 TDS analog output |
| `A3` | Keyestudio KS0414 turbidity analog output through a `0.60` voltage divider |

## Bus Summary

- I2C: `SDA GPIO23`, `SCL GPIO27`; ADS1115 `0x48`, MPU6050 `0x68`, QMC5883 compass `0x0D`.
- HGLRC M100-5883 GPS: receive-only at `115200 baud`; module `RX` is not connected.
- microSD: `CS 1`, `SCK 17`, `MOSI 26`, `MISO 35`, requested clock `20 MHz`.
- ExpressLRS receiver: UART2 MAVLink at 460800 baud, RX `GPIO34`, TX `GPIO25`.

See the individual device pages for power, divider, and safety details.
