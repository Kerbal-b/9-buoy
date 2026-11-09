# Battery Voltage Sensor Wiring Map

The active firmware reads the `0-25 V` resistor-divider module through ADS1115 channel `A1`. Its system-voltage conversion is calibrated to `16.52 V` at full ADC scale; battery percentage currently assumes a nominal 3-cell pack (`12.6 V` full and `9.6 V` empty).

## Connections

| Voltage-module connection | Connect to | Notes |
| --- | --- | --- |
| Input `+` | Battery positive | Measured system voltage. |
| Input `-` | Battery/common ground | Measurement return. |
| Signal `S` or `OUT` | ADS1115 `A1` | No ESP32 internal ADC connection. |
| Output `-` or `GND` | ADS1115/ESP32 common `GND` | Required reference. |
| Output `+` | Usually not connected | Passive divider boards do not require logic power; verify the exact module. |

```text
Battery + ------------------------> Module input +
Battery - / common GND -----------> Module input - / GND
Module S / OUT -------------------> ADS1115 A1
```

## Safety And Calibration

- Measure the module output before attaching `A1`; it must remain within the input limit of the ADS1115 powered at `3.3 V`.
- The firmware's `16.52 V` scale is a project calibration, not the generic module's advertised `25 V` input rating.
- Recalibrate if the divider module, ADS1115, resistor values, or battery range changes.
