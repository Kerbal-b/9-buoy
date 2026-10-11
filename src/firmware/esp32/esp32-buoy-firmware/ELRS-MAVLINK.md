# Production ELRS MAVLink link

## Firmware version

The firmware reports four numeric fields as `TEL STATUS FIRMWARE <major> <minor> <patch> <YYYYMMDD>`. The control station displays them as `v1.2.0 (2026-10-10)` for this build, and microSD manifests store `firmware_version=1.2.0+20261010` plus the separate fields. Increment major for incompatible wiring or control protocol changes, minor for new compatible features, and patch for fixes. The date identifies the build and does not replace the version numbers. Change the constants in `firmware_version.h` before each release.

The buoy sends `TEL SCI TDS_V` from ADS1115 A2 and `TEL SCI TURBIDITY_V` from ADS1115 A3 over the Wi-Fi TCP control connection every two seconds, even without a science recording session. It also includes them in status snapshots. These are module output voltages; the A3 value is corrected for its 0.60 divider. `UNKNOWN` means the ADC could not be read. A numeric voltage confirms only an electrical reading, not calibrated conductivity or turbidity. Audio recording lowers the live update rate to 30 seconds.

The production buoy firmware uses the SkyGuy Nano 2.4 GHz ExpressLRS receiver on ESP32 UART2. Configure the receiver serial protocol and TX12 Link Mode for **MAVLink** (ELRS 3.5.0 or newer). Set TX Backpack Telemetry to **WiFi** for the control station's UDP 14550 listener. CRSF is available only in its separate bench sketch; one receiver UART cannot run both serial protocols at once.

| Receiver | ESP32 |
| --- | --- |
| `TX` | `GPIO34` (UART2 RX) |
| `RX` | `GPIO25` (UART2 TX) |
| `5V` / `VCC` | Regulated `5V` |
| `GND` | Common `GND` |

The UART is non-inverted, 460800 baud, 8N1. Power the receiver from the regulated 5 V rail. The installed ADS1115 and both Keyestudio water-sensor interfaces also use regulated 5 V; TDS output goes directly to A2, while turbidity output reaches A3 through a 10 kΩ / 15 kΩ divider. The ADS1115 SDA/SCL wires currently connect directly to ESP32 GPIO23/GPIO27, so verify that their pull-ups do not bring 5 V onto the ESP32 pins.

For future ESP32 firmware flashes in this project, the owner does not require a pre-flash copy of the chip's existing firmware.

## Control

The TX12 mapping is CH1 lateral, CH2 forward/reverse, CH3 power limit, CH4 yaw, and CH5 arm request. CH5 below 1300 microseconds disarms. CH5 above 1700 microseconds can arm only after the firmware has observed it low, CH3 is below 1100 microseconds, and the movement sticks are neutral. The receiver's RC_CHANNELS_OVERRIDE must target MAVLink system 1, component 1 or all components. The firmware rejects missing/ignored primary channels.

ELRS propulsion is capped at 50% while physical motor directions are verified. Wi-Fi and ELRS cannot command motors at the same time. A Wi-Fi `CTRL STOP` or `CTRL MOTOR STOP` disarms ELRS and requires another CH5 low-to-high cycle. RC data older than 500 ms, link status older than 2 seconds, or zero link quality stops ELRS-driven motors. Test with propellers removed and confirm channel direction in the EdgeTX channel monitor before using propulsion.

## Telemetry

The buoy sends MAVLink HEARTBEAT and relayed receiver RADIO_STATUS every second, changed RC_CHANNELS_RAW banks at most twice per second, SYS_STATUS battery/current every five seconds when the ADS1115 battery reading is available, and GLOBAL_POSITION_INT every five seconds when GPS is fresh. The Backpack downlink at a 50 Hz ELRS packet rate has little spare capacity; high-rate IMU and audio remain on Wi-Fi and microSD. TDS and turbidity remain uncalibrated voltages in `science.csv` on microSD.

This firmware does not implement MAVLink mission upload or autonomous mission execution. The station's Backpack path is for link, channel, battery, and GPS telemetry.
