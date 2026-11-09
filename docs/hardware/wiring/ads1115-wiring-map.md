# ADS1115 Wiring Map

The ADS1115 is the external four-channel ADC for power and water-quality sensors. The firmware expects I2C address `0x48` and a `4.096 V` programmable full-scale setting.

## Connections

| ADS1115 pin | Connect to | Notes |
| --- | --- | --- |
| `VDD` | Regulated `5 V` | User-confirmed installed wiring. Check I2C pull-up voltage before connecting SDA/SCL to ESP32 pins. |
| `GND` | ESP32/common `GND` | Required reference for all four measurements. |
| `SDA` | ESP32 `GPIO23` directly | Green wire; shared with MPU6050. Pull-up voltage needs verification. |
| `SCL` | ESP32 `GPIO27` directly | White wire; shared with MPU6050. Pull-up voltage needs verification. |
| `ADDR` | `GND` | Selects firmware address `0x48`; many boards already default to this. Verify the board. |
| `ALERT/RDY` | Not connected | Not used by the firmware. |
| `A0` | ACS712 divider output | Current measurement. |
| `A1` | Battery-voltage module output | Battery/system voltage. |
| `A2` | KS0429 TDS module analog output directly | No divider; firmware reports the A2 voltage. |
| `A3` | Turbidity divider output | Voltage only until wet calibration. |

The firmware selects a `4.096 V` ADC range. With the ADS1115 powered at `5 V`, inputs above 4.096 V cannot be measured at that setting. Measure every analog signal before connection and keep it within the ADC supply limits. The current and turbidity paths use dividers in the installed wiring; TDS goes directly to A2.

Both installed Keyestudio sensor interfaces, KS0429 TDS and KS0414 turbidity, are powered from regulated `5 V` and share common ground with the ADS1115. KS0429 output goes directly to `A2`; KS0414 output passes through a 10 kΩ / 15 kΩ divider to `A3`, with nominal ratio 0.60. The firmware matches this arrangement: it reports A2 voltage directly and corrects A3 by the divider ratio. Verify the actual signal voltages before interpreting either as a water measurement.

The installed SDA/SCL wires go directly to the ESP32. Its I2C pins use 3.3 V logic. Before powering this wiring again, identify the ADS1115 board's pull-ups and verify that neither bus wire rises above the ESP32 pin rating. If the board pulls SDA/SCL to 5 V, use an I2C level shifter or correct the pull-up arrangement; do not apply a 5 V I2C high level to ESP32 GPIO23/GPIO27.
