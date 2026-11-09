# ACS712 Current Sensor Wiring Map

The active firmware is configured for an ACS712 `30 A` module with `0.066 V/A` sensitivity. It reads the sensor through ADS1115 channel `A0`, calibrates the zero-current level during startup, and assumes a `0.60` input-divider ratio.

## Logic-Side Connections

| ACS712 connection | Connect to | Notes |
| --- | --- | --- |
| `OUT` | `10 kΩ` series resistor to ADS1115 `A0`, with `15 kΩ` from `A0` to ground | Produces the firmware's `0.60` divider ratio. |
| `GND` | ESP32/ADS1115 common `GND` | Required reference. |
| `VCC` | Supply required by the exact ACS712 module | Many modules require `5 V`; verify the board. |

```text
ACS712 OUT ----[ 10 kΩ ]----o A0_NODE ---- ADS1115 A0
                             |
                          [ 15 kΩ ]
                             |
common GND ------------------o
```

## Current Path

Place the isolated high-current terminals in series with the load being measured. Terminal labels vary by module.

```text
Battery positive ---- IP+  [ ACS712 current path ]  IP- ---- Motor-driver positive
```

## Safety And Verification

- Measure the divided `A0` voltage at zero current and a controlled test load before connecting the ADS1115.
- Keep the ADS1115 input within the limit imposed by its `3.3 V` supply.
- Firmware negates the measured polarity. If positive load produces negative telemetry, reverse the high-current path or deliberately change the software convention.
- Do not route motor current through an ESP32 board, breadboard rail, or thin signal wire.
