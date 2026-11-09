# ELRS Receiver Wiring Map

The production ESP32 firmware uses the SkyGuy Nano 2.4 GHz receiver in MAVLink UART mode. Configure both the receiver and TX12 for MAVLink, with receiver TX to ESP32 RX and receiver RX to ESP32 TX.

| Receiver pin | ESP32 connection |
| --- | --- |
| `TX` | `GPIO34` (UART2 RX) |
| `RX` | `GPIO25` (UART2 TX) |
| `5V` / `VCC` | Regulated `5V` |
| `GND` | Common `GND` |

UART2 uses non-inverted 460800 baud, 8N1. The receiver power input is 5 V; the ADS1115 remains on 3.3 V. The radio outputs are channel mapped and guarded as described in [production ELRS MAVLink](../../../src/firmware/esp32/esp32-buoy-firmware/ELRS-MAVLINK.md).
