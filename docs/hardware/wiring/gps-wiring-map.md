# HGLRC M100-5883 GPS and Compass Wiring Map

The installed HGLRC M100-5883 combines an M10 GNSS receiver and QMC5883 magnetometer. The production firmware reads NMEA data through ESP32 UART1 at `115200 baud` and reads compass axes over I2C at address `0x0D`. GPS receive is one-way because `GPIO17` is used as the microSD clock.

| M100-5883 pin | Connect to | Notes |
| --- | --- | --- |
| `TX` | ESP32 `GPIO16` (`UART1 RX`) | GPS data into the ESP32. |
| `RX` | Not connected | ESP32 does not configure the module over UART. |
| `SDA` | ESP32 `GPIO23` | Shared I2C data bus with MPU6050 and ADS1115. |
| `SCL` | ESP32 `GPIO27` | Shared I2C clock bus with MPU6050 and ADS1115. |
| `5V` | Regulated `5 V` | HGLRC specifies 3.3–5 V input; use the board's labeled supply pin. |
| `GND` | ESP32/common `GND` | Required common reference. |

```text
[ HGLRC M100-5883 ]                [ ESP32 ]
 TX  ----------------------------> GPIO16 / UART1 RX (115200 baud)
 RX  ----------------------------- not connected
 SDA <---------------------------> GPIO23 / I2C SDA
 SCL <----------------------------- GPIO27 / I2C SCL
 5V  ----------------------------> regulated 5 V
 GND ----------------------------> common GND
```

The compass raw X/Y/Z readings are streamed over buoy Wi-Fi when a control-station client is connected. The firmware identifies the compass as ready only after the QMC5883 at `0x0D` responds and accepts configuration. The exact board labels govern connector order; verify the pin names on the module before power-up.
