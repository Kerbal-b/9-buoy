# TDS and Turbidity Sensor Wiring

The active firmware records both sensors as voltages. It does not yet convert them into calibrated TDS or turbidity units.

For isolated turbidity bench testing, use the [ADS1115 turbidity test firmware](../../../src/firmware/esp32/esp32-ads1115-turbidity-test/README.md).

## Keyestudio KS0429 TDS Module

| TDS connection | Connect to | Notes |
| --- | --- | --- |
| Analog output | ADS1115 `A2` directly | No divider; firmware reports the A2 voltage. |
| `GND` | Common `GND` | Must share the ADC reference. |
| Module power | Regulated `5 V` | Installed wiring. KS0429 accepts 3.3-5.5 V; its specified analog output is 0-2.3 V. This is separate from the ADS1115 supply. |
| Probe connector | TDS probe | Keep the probe connector dry. |

## Keyestudio KS0414 Turbidity Module

| Turbidity connection | Connect to | Notes |
| --- | --- | --- |
| Analog output | Voltage divider, then ADS1115 `A3` | Firmware reconstructs the module voltage using a `0.60` divider ratio. |
| `GND` | Common `GND` | Must share the ADC reference. |
| `VCC` / `+` | Regulated `5 V` | Keyestudio specifies DC `5 V`; do not power the KS0414 from the ADS1115's 3.3 V rail. |
| Probe connector | Turbidity probe | Keep non-waterproof electronics dry. |

## Required `0.60` Divider

Use the same divider ratio assumed by the firmware on the turbidity analog output. The `15 kΩ` pull-down attaches to the **A3 junction after the 10 kΩ resistor**, not directly to KS0414 OUT.

| Divider connection | Exact endpoints |
| --- | --- |
| Series resistor | KS0414 `OUT` → `10 kΩ` → junction `A3_NODE` |
| ADC input | ADS1115 `A3` → junction `A3_NODE` |
| Pull-down resistor | Junction `A3_NODE` → `15 kΩ` → common `GND` |

```text
regulated 5 V ------------------ ADS1115 VDD
           +-------------------- KS0414 VCC / +
           +-------------------- KS0429 VCC / +

KS0414 OUT ----[ 10 kΩ ]----o A3_NODE ---- ADS1115 A3
                            |
                         [ 15 kΩ ]
                            |
COMMON GND -----------------o------------- ADS1115 GND
                            +------------- KS0414 GND / -
                            +------------- KS0429 GND / -
```

KS0429 analog output goes directly to ADS1115 `A2`; only KS0414 analog output passes through the nominal 0.60 divider to `A3`. The ADS1115 is powered at 5 V, while the firmware's measurement range is 0-4.096 V. With both probes connected but dry, the installed firmware recently reported about 3.8 V at A2, above KS0429's specified 0-2.3 V output range. It also reported about 3.8 V at A3, which the firmware reconstructs as about 6.3 V at the KS0414 output, above its 5 V supply. Check the actual sensor output and ADC pin voltages, the A3 divider, and the I2C bus high level with a multimeter before interpreting either reading as a water measurement.

There is no direct `15 kΩ` connection from KS0414 `OUT` to ground. Both ADS1115 `A3` and the top of the `15 kΩ` resistor meet at `A3_NODE`.

The actual resistor values may vary if their ratio remains correct and the sensor is not excessively loaded. Measure and record the real ratio for calibration.

Set the KS0414 slide switch to `A` for analog output. Keep the TDS and turbidity probes separated enough that one measurement does not disturb the other, and develop wet-calibration procedures before reporting scientific units.
