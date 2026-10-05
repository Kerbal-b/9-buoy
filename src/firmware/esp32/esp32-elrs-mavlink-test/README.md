# ESP32 ELRS / MAVLink heartbeat bench test

This sketch is a separate follow-up to `esp32-elrs-crsf-test`. It sends one MAVLink HEARTBEAT per second through the SkyGuy ELRS receiver. It validates the receiver's inbound MAVLink `RADIO_STATUS` and `RC_CHANNELS_OVERRIDE` checksums, relays one fresh link-status sample per second, and relays changed channel banks in 30-byte `RC_CHANNELS_RAW` messages (at most one bank every 500 ms, with an idle refresh every six seconds per bank). It does not drive motors or implement missions.

## Wiring

Use the same test wiring: receiver `TX` to ESP32 `GPIO34` (UART2 RX), receiver `RX` to ESP32 `GPIO25` (UART2 TX), receiver power to the correct supply, and common ground. The UART is non-inverted, 460800 baud, 8N1.

## Before flashing

Confirm TX and RX ELRS firmware support MAVLink (minimum 3.5.0). Configure the receiver serial protocol as MAVLink, set the TX12 Link Mode to MAVLink, and keep Backpack Telemetry set to WiFi. These are radio settings; this sketch does not change them. Flash `esp32-elrs-mavlink-test.ino` to the bench ESP32, not to the production buoy controller.

## Expected result

The ESP32 USB serial monitor at 115200 baud prints one `MAVLink HEARTBEAT sent` line per second, the ELRS receiver's uplink LQ, RSSI, and SNR, and all 16 channel positions when fresh data arrives. The TX Backpack `/mavlink` downlink counter should rise by about two packets per second at idle, plus an occasional channel bank refresh. The control station should show LQ, RSSI, SNR, and up to 16 channel positions on Communications > Radio / ELRS after a restart and Backpack discovery. The receiver's `RADIO_STATUS` carries uplink LQ scaled to 0–255 in `rssi`, the magnitude of negative RSSI dBm in `remrssi`, and signed SNR dB in `noise`. The sketch re-publishes these as system ID 1, component ID 68. It re-publishes received `RC_CHANNELS_OVERRIDE` as two `RC_CHANNELS_RAW` banks from system ID 1, component ID 1. These values describe the TX12-to-receiver uplink; the TX12 may show different downlink readings.

Close the control station before running the standalone diagnostic on UDP 14550 because Windows normally permits only one listener on that port. A bound receiver with the old CRSF sketch may still show RSSI/LQ on the TX12, but it cannot produce this MAVLink heartbeat. The receiver link values become unknown if fresh `RADIO_STATUS` stops arriving. This relay is a bench test; production buoy firmware still needs its own MAVLink and ELRS integration.

At the TX12's 50Hz packet rate, ExpressLRS lists only about 110 bytes/s of MAVLink downlink capacity. The banked channel relay keeps background traffic under that rate, but it cannot make 50Hz downlink instantaneous. For a faster bench response, use `100Hz Full` or a faster compatible packet rate in the TX12 ELRS Lua settings; this trades some RF sensitivity/range for throughput.
