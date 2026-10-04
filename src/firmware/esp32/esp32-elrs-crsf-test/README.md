# ESP32 ELRS / CRSF Full-Duplex Test

This bench sketch tests control input from the RadioMaster TX12 through the SkyGuy Nano 2.4 GHz ExpressLRS receiver and sends CRSF telemetry back to the radio.

## Connections

| ELRS receiver | ESP32 development board |
| --- | --- |
| `TX` | `GPIO34` (`UART2 RX`) |
| `RX` | `GPIO25` (`UART2 TX`) |
| `5V` / `VCC` | `5V` |
| `GND` | Common `GND` |

The signal wires cross: receiver `TX` goes to ESP32 receive, and receiver `RX` goes to ESP32 transmit. The sketch uses standard, non-inverted CRSF at 420000 baud.

## What The Test Does

- Decodes 16 packed CRSF control channels.
- Prints channels 1–8 as approximately 1000–2000 microseconds over USB serial at 115200 baud.
- Reports link quality, RSSI, SNR, valid frames, CRC failures, and a 500 ms RC-loss failsafe state.
- Sends only the selected CRSF telemetry set to the TX12: battery, GPS, and flight/operating mode.
- Does not send attitude, audio, scientific sensor streams, or duplicate ELRS radio-link statistics. ELRS supplies its own RSSI/LQ/SNR telemetry; high-volume buoy data remains better suited to Wi-Fi and microSD.

The telemetry values are intentionally static bench-test values. Battery reports 12.0 V, 0 A, 0 mAh, and 95%. GPS reports a fixed test location. They prove the return path but are not live buoy measurements.

## TX12 Test

1. Remove or mechanically disable all propellers. This sketch never drives motors.
2. Select the bound ExpressLRS model on the TX12 and power the receiver from the ESP32 `5V` supply.
3. Open the Arduino serial monitor at 115200 baud. Moving sticks and switches should change channel values; stopping the transmitter or receiver data should produce `RC=FAILSAFE` within 500 ms.
4. In EdgeTX, open the model's **Telemetry** page and run **Discover new sensors**. Battery voltage, current/capacity/remaining, GPS, flight mode, and ELRS's own link statistics should appear.

If controls work but telemetry does not, confirm receiver `RX` is connected to ESP32 `GPIO25`, telemetry is enabled in the ELRS configuration, and the model telemetry ratio is not set to `OFF`.

## Important Scope

This is an isolated test firmware. It does not alter the production buoy firmware and does not control motors. Live battery, GPS, and motion values should only replace the test constants after this bidirectional CRSF link is verified.
