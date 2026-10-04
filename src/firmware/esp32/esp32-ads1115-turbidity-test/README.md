# ESP32 ADS1115 Water-Quality Sensor Test

This isolated bench sketch verifies the installed ADS1115, Keyestudio KS0429 TDS meter, and Keyestudio KS0414 turbidity module without starting motors, Wi-Fi, audio, GPS, or SD logging.

## ADS1115 Connections

| Connection | Destination |
| --- | --- |
| ADS1115 `VDD` | ESP32 `3.3 V` |
| ADS1115 `GND` | Common `GND` |
| ADS1115 `SDA` | ESP32 `GPIO23` |
| ADS1115 `SCL` | ESP32 `GPIO27` |
| ADS1115 `ADDR` | `GND` for address `0x48` |

## TDS Connections

| TDS connection | Destination |
| --- | --- |
| KS0429 `VCC` / `+` | ESP32 `3.3 V` |
| KS0429 `GND` / `-` | Common `GND` |
| KS0429 analog output / `A` | ADS1115 `A2` |
| Probe connector | TDS probe |

The TDS output is connected directly to `A2`; no voltage divider is used. The sketch median-filters 31 readings and reports the measured voltage plus the manufacturer's temperature-compensated ppm estimate. Because this isolated test does not read the DS18B20, it assumes `25.0 C`. Treat ppm as an uncalibrated estimate until the probe is calibrated with a reference solution and actual water temperature is supplied.

## Turbidity Connections

| Connection | Destination |
| --- | --- |
| KS0414 `VCC` / `+` | Regulated `5 V` |
| KS0414 `GND` / `-` | Common `GND` |
| Series resistor | KS0414 `OUT` -> `10 kOhm` -> junction `A3_NODE` |
| ADC input | ADS1115 `A3` -> junction `A3_NODE` |
| Pull-down resistor | Junction `A3_NODE` -> `15 kOhm` -> common `GND` |

The ADS1115 uses `3.3 V`, but the KS0414 uses `5 V`. Set the KS0414 slide switch to `A` for analog output and keep its interface board dry.

```text
ESP32 3.3 V -------------------- ADS1115 VDD
ESP32 3.3 V -------------------- KS0429 VCC / +
KS0429 OUT --------------------> ADS1115 A2

regulated 5 V ------------------ KS0414 VCC / +
KS0414 OUT ----[ 10 kOhm ]---o A3_NODE ---- ADS1115 A3
                            |
                         [ 15 kOhm ]
                            |
COMMON GND -----------------o------------- ADS1115 GND
                            +------------- KS0429 GND / -
                            +------------- KS0414 GND / -
```

Important: the `15 kOhm` resistor starts at `A3_NODE`, on the ADS1115 side of the `10 kOhm` resistor. Do not attach it directly to KS0414 `OUT`.

## Before Uploading

Disconnect the SD module's `CS` wire from ESP32 `GPIO1`. The production SD wiring shares the normal UART0 TX pin, which can prevent flashing and serial output. The SD module is not needed for this test.

## Test Procedure

1. Upload `esp32-ads1115-turbidity-test.ino` to the ESP32.
2. Open the serial monitor at `115200 baud`.
3. Confirm `PASS: ADS1115 acknowledged at I2C address 0x48`.
4. Confirm the TDS signal appears in the `tds_voltage` column.
5. Place the TDS probe in a reference solution or water sample and allow the reading to settle.
6. Observe `a3_voltage` and `turbidity_sensor_voltage` with the turbidity probe in clear water.
7. Introduce a repeatable cloudy-water sample and watch for a stable turbidity-voltage change.
8. Rinse the probes and repeat each condition several times.

The sketch reports once every 800 ms. It median-filters 31 TDS conversions and averages 16 turbidity conversions. `turbidity_sensor_voltage` reconstructs the KS0414 module output using the firmware's `0.60` divider ratio.

Do not treat the reported TDS estimate as calibrated ppm or the turbidity voltage as NTU yet. Scientific measurements require calibration standards, repeated measurements, temperature records, consistent probe geometry, and fitted calibration curves.
