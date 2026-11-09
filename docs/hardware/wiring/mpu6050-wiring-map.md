# MPU6050 Wiring Map

The active firmware communicates directly with an MPU6050 at I2C address `0x68`. The interrupt line is not used.

## Connections

| MPU6050 pin | Connect to | Notes |
| --- | --- | --- |
| `VCC` | ESP32 `3.3V` | Preferred safe logic supply for the current design. |
| `GND` | ESP32 `GND` | Common ground. |
| `SDA` | ESP32 `GPIO23` | Firmware I2C data pin. |
| `SCL` | ESP32 `GPIO27` | Firmware I2C clock pin. |
| `AD0` | `GND` | Selects the firmware address `0x68`. |
| `INT` | Not connected | `MPU6050_INT_PIN` is `-1` in firmware. |

## Schematic

```text
[ MPU6050 ]                        [ ESP32 ]
 VCC ----------------------------- 3.3V
 GND ----------------------------- GND
 SDA <---------------------------> GPIO23
 SCL <----------------------------- GPIO27
 AD0 ----------------------------- GND       address 0x68
 INT ----------------------------- NC
```

If the breakout already contains I2C pull-ups, confirm they pull SDA and SCL to `3.3 V`, not `5 V`.
