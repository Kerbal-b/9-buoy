# Production ELRS MAVLink link

## Firmware version

The firmware reports four numeric fields as `TEL STATUS FIRMWARE <major> <minor> <patch> <YYYYMMDD>`. The control station displays them as `v1.6.0 (2026-10-18)` for this build, and microSD manifests store `firmware_version=1.6.0+20261018` plus the separate fields. Increment major for incompatible wiring or control protocol changes, minor for new compatible features, and patch for fixes. The date identifies the build and does not replace the version numbers. Change the constants in `firmware_version.h` before each release.

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

`TEL STATUS ELRS_RC_DIAG` reports cumulative receiver-side RC override parsing counts since boot: `OVERRIDE` frames seen, `CRC_OK` / `CRC_BAD`, short payloads, target-ID rejects, invalid primary channels, accepted frames, and the last rejection with its target or bad channel value. If `OVERRIDE` remains zero while `RADIO` increases, no RC override packets are arriving at the ESP32. If overrides arrive but `ACCEPTED` stays zero, the rejection counters identify why.

ELRS CH3 now scales propulsion from zero to the full configured motor range. At CH3 maximum, a full movement stick can request 255 PWM from the motor mix; individual motors may remain below 255 for directions that need less thrust, and the ramp takes time to reach the target. Wi-Fi keyboard drive retains its adjustable default 50% mixer limit. Wi-Fi and ELRS cannot command motors at the same time. A Wi-Fi `CTRL STOP` or `CTRL MOTOR STOP` disarms ELRS and requires another CH5 low-to-high cycle. RC data older than 500 ms or a fresh zero-link-quality report stops ELRS-driven motors and automatically stops and saves an active microSD science/audio session. Missing `RADIO_STATUS` alone does not stop propulsion. The control station, buoy Wi-Fi, and TX Backpack are not required for TX12 control; the ESP32 reads receiver MAVLink directly on UART2. The recovery status reports `MANUAL_STOP` until the radio is armed again. No recovery switch is reserved; mission return-to-home is future work. Test with propellers removed and confirm channel direction in the EdgeTX channel monitor before using propulsion.

On an ELRS failsafe, the buoy sends `TEL EVENT ELRS_FAILSAFE COUNT <n> REASON <reason> RC_AGE_MS <ms> LQ <percent> LINK_AGE_MS <ms> RC <count> RADIO <count> CRCERR <count> UPTIME_MS <ms>` over the Wi-Fi control connection. Reasons are `RC_TIMEOUT`, `ZERO_LQ`, `RC_TIMEOUT_AND_ZERO_LQ`, or `ARM_STATE_LOST`. The event captures receiver state at the stop. `TEL STATUS ELRS_FAILSAFE` repeats the last event in every status snapshot, including after rearming or a Wi-Fi reconnect; `NONE` means no failsafe since reboot. The control station explains the reason in its system communication log. Wi-Fi is not required for TX12 control, but it must connect eventually to read this diagnostic.

Wi-Fi telemetry uses a bounded nonblocking TCP queue. If Wi-Fi becomes slow or unavailable, telemetry may be dropped; it cannot hold up ELRS receiver polling. The 500 ms RC timeout and zero-link-quality motor stops remain active for a real receiver or radio failure.

SF on CH6 selects telemetry routing independently of SE arming and TX12 motor control. Low or middle selects full buoy Wi-Fi telemetry, including receiver link status and channels in status snapshots, and only a MAVLink heartbeat. High selects low-rate MAVLink state, link, channel, battery/current, GPS, and rotating science telemetry for the TX Backpack; Wi-Fi still accepts commands and reports a compact control/safety status on PING. The switch does not select a different motor controller or disable the ELRS failsafe. If Wi-Fi drops, TX12 propulsion remains available as long as receiver control frames stay fresh. `TEL EVENT TELEMETRY_MODE` announces a switch change and `TEL STATUS TELEMETRY_MODE` confirms the active route when Wi-Fi is available. High-rate IMU, audio, and SD file downloads remain on buoy Wi-Fi or local recording.

## Telemetry

The buoy sends MAVLink HEARTBEAT, a compact `BUOY_STATE` NAMED_VALUE_INT, and relayed receiver RADIO_STATUS every second; changed RC_CHANNELS_RAW banks at most twice per second; SYS_STATUS battery/current every five seconds when the ADS1115 battery reading is available; and GLOBAL_POSITION_INT every five seconds when GPS is fresh. It also sends one NAMED_VALUE_FLOAT science field per second, rotating through `WATER_TEMP` (°C), `DEPTH_M` (m), `TDS_V` (ADS1115 A2 module output volts), and `TURBIDITY` (ADS1115 A3 probe output volts with divider correction). Each value refreshes about every four seconds; unavailable readings are omitted and the station marks them stale after twelve seconds. These water sensor voltages are not calibrated conductivity or turbidity measurements. The GPS message includes measured ground speed and track when both are valid. `BUOY_STATE` packs rear, front-left, and front-right signed applied PWM scaled from ±255 to ±127 into its first three value bytes; the station reconstructs the PWM to within one count, including exact 255 at maximum. Its fourth byte has mode in bits 0–2 (idle, manual, hold, goto), source in bits 3–4 (none, Wi-Fi, TX12), arm in bit 5, science recording in bit 6, and manual link-loss stop in bit 7. This reports what the buoy is doing without a buoy Wi-Fi connection. GPS track describes motion over the ground; it is not a measured hull heading. Mission mode is not implemented, so the station reports no active mission. High-rate IMU, audio, and SD file downloads remain on Wi-Fi.

This firmware does not implement MAVLink mission upload or autonomous mission execution. The station's Backpack path is for link, channel, battery, and GPS telemetry.
