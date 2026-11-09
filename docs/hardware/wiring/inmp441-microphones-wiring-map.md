# INMP441 Stereo Microphones Wiring Map

The production firmware configures ESP32 I2S0 as a stereo receiver at `16 kHz`, using one shared clock bus and one data input for an INMP441 pair.

## Shared Bus Connections

| INMP441 pin | Connect to | Notes |
| --- | --- | --- |
| `VDD` | ESP32 `3.3V` | Both microphones. |
| `GND` | ESP32 `GND` | Both microphones. |
| `SCK` / `BCLK` | ESP32 `GPIO5` | Shared I2S bit clock. |
| `WS` / `LRCL` | ESP32 `GPIO4` | Shared word-select clock. |
| `SD` | ESP32 `GPIO36` / `VP` | Shared data input in the firmware design. |
| `L/R` | Opposite selection on the two microphones | One microphone selects left, the other right; **verify actual breakout polarity**. |

## Schematic

```text
                         +--> Mic A SCK
ESP32 GPIO5  BCLK -------+--> Mic B SCK

                         +--> Mic A WS
ESP32 GPIO4  WS ---------+--> Mic B WS

                         +--- Mic A SD
ESP32 GPIO36 DATA <-------+--- Mic B SD

ESP32 3.3V --------------+--> both VDD
ESP32 GND ----------------+--> both GND

Mic A L/R ---------------- left-channel selection
Mic B L/R ---------------- right-channel selection
```

The firmware requests both left and right I2S slots, but software cannot confirm which physical microphone is strapped to each slot. Check the exact INMP441 breakout documentation: commonly `L/R` low selects one channel and high selects the other, but labels and conventions must be verified before hard-wiring.
