# Production ELRS MAVLink link

## Firmware 1.16.26 telemetry routing

SF/CH6 selects detailed Wi-Fi or detailed radio telemetry. Both positions retain
essential ELRS telemetry: heartbeat, buoy state/audio, battery/current, GPS when
fixed, 1 Hz attitude/tilt heading, and versioned `B1` MAVLink `STATUSTEXT` every
two seconds. `B1` contains arm, assist mode, mission state/progress/waypoint,
session/audio, heading, Home distance/source and GPS fix. Calibration adds `BC1`
with state, step, power, ceiling and duration. ExpressLRS 4.1 converts these
to CRSF passthrough text for the TX12 Lua screen; Backpack clients receive raw
MAVLink. SF high additionally enables rotating science, raw calibration values,
detailed navigation and RC channel telemetry. `CAPA` remains unavailable because
consumption is not measured.

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

The TX12 mapping is CH1 lateral, CH2 forward/reverse, CH3 power limit, CH4 yaw, and CH5 arm request. CH5 below 1300 microseconds disarms. CH5 above 1700 microseconds can arm only after the firmware has observed it low, CH3 is below 1100 microseconds, and the movement sticks are neutral. The receiver's RC_CHANNELS_OVERRIDE must target MAVLink system 1, component 1 or all components. The firmware rejects missing/ignored primary channels. Firmware v1.16.17 and later reserves CH11–CH16 for the optional TX12 calibration mix: enable, command, mode, test index, test power, and selected duration. Firmware v1.16.18 records calibration data at 20 Hz, samples current and battery voltage for each row with validity flags, and pauses optional Wi-Fi UDP sensor streams and standalone SD logging while required sensor reads, GPS parsing, and ELRS control/safety remain active. Firmware v1.16.20 supports 5-second default pulses or 10-second large-pool pulses, with a 3-second neutral interval. Firmware v1.16.21 expands Wi-Fi `ELRS_CHANNELS` status to all 16 channels and records every calibration-start interlock and CH11–CH16 value in both the station event trace and the buoy system log. Firmware v1.16.22 accepts EdgeTX extended output endpoints of 881–2119 microseconds for CH13–CH16, while still rejecting ignored channels, and logs the decoded mode, power cap, and pulse duration after accepting a start. Firmware v1.16.23 accepts those same extended endpoints when validating each Lua step and logs raw values when rejecting a step. Normal control remains on CH1–CH10. GPS fix is reported for diagnosis but is not a calibration-start requirement. A powered step additionally requires SE high after the low-to-high arm interlock.

The calibration script starts a separate structured file under `/calibration/` on the SD card. Its header snapshots the complete active `/config/propulsion.cfg` profile (including points and CRC), current motor calibration values and safety parameters. Sample rows record the requested test, motor commands/targets/applied PWM, current, IMU/gyro, raw and corrected compass heading, GPS, and RC channels. This file is intended to be imported later to calculate adjustments against the settings used for that test.

`TEL STATUS ELRS_RC_DIAG` reports cumulative receiver-side RC override parsing counts since boot: `OVERRIDE` frames seen, `CRC_OK` / `CRC_BAD`, short payloads, target-ID rejects, invalid primary channels, accepted frames, and the last rejection with its target or bad channel value. If `OVERRIDE` remains zero while `RADIO` increases, no RC override packets are arriving at the ESP32. If overrides arrive but `ACCEPTED` stays zero, the rejection counters identify why.

ELRS CH3 now scales propulsion from zero to the full configured motor range. At CH3 maximum, a full movement stick can request 255 PWM from the motor mix; individual motors may remain below 255 for directions that need less thrust, and the ramp takes time to reach the target. Wi-Fi keyboard drive retains its adjustable default 50% mixer limit. Wi-Fi and ELRS cannot command motors at the same time. A Wi-Fi `CTRL STOP` or `CTRL MOTOR STOP` disarms ELRS and requires another CH5 low-to-high cycle. RC data older than 500 ms or a fresh zero-link-quality report stops ELRS-driven motors and automatically stops and saves an active microSD science/audio session. Missing `RADIO_STATUS` alone does not stop propulsion. The control station, buoy Wi-Fi, and TX Backpack are not required for TX12 control; the ESP32 reads receiver MAVLink directly on UART2. The recovery status reports `MANUAL_STOP` until the radio is armed again. No recovery switch is reserved; mission return-to-home is future work. Test with propellers removed and confirm channel direction in the EdgeTX channel monitor before using propulsion.

On an ELRS failsafe, the buoy sends `TEL EVENT ELRS_FAILSAFE COUNT <n> REASON <reason> RC_AGE_MS <ms> LQ <percent> LINK_AGE_MS <ms> RC <count> RADIO <count> CRCERR <count> UPTIME_MS <ms>` over the Wi-Fi control connection. Reasons are `RC_TIMEOUT`, `ZERO_LQ`, `RC_TIMEOUT_AND_ZERO_LQ`, or `ARM_STATE_LOST`. The event captures receiver state at the stop. `TEL STATUS ELRS_FAILSAFE` repeats the last event in every status snapshot, including after rearming or a Wi-Fi reconnect; `NONE` means no failsafe since reboot. The control station explains the reason in its system communication log. Wi-Fi is not required for TX12 control, but it must connect eventually to read this diagnostic.

Wi-Fi telemetry uses a bounded nonblocking TCP queue. If Wi-Fi becomes slow or unavailable, telemetry may be dropped; it cannot hold up ELRS receiver polling. The 500 ms RC timeout and zero-link-quality motor stops remain active for a real receiver or radio failure.

SF/CH6 low or middle keeps detailed buoy Wi-Fi telemetry; high adds detailed rotating ELRS telemetry. Essential ELRS status, battery, GPS when fixed, attitude, and versioned buoy status text are sent in both positions. Wi-Fi remains available for commands and file transfer. The switch does not select a motor controller or disable receiver failsafe.

## Telemetry

RC channels CH11-CH16 carry calibration commands; telemetry returns separately as MAVLink. The buoy sends `B1` and `BC1` status text for the TX12 Lua display and raw Backpack clients. `NAMED_VALUE_FLOAT` science and calibration detail is transmitted only in SF-high mode and does not become ordinary EdgeTX sensors under ExpressLRS 4.1. The control station decodes those raw values and the compact status. `CAPA` is unavailable because consumed mAh is not measured.

The buoy sends MAVLink ATTITUDE at 1 Hz in both SF positions when magnetometer calibration is loaded. Roll and pitch come from the accelerometer; yaw is tilt-corrected magnetic heading. This is a low-rate snapshot affected by hull acceleration. SYS_STATUS and BATTERY_STATUS provide voltage/current and estimated battery percentage every five seconds when battery voltage is available.

The buoy sends MAVLink HEARTBEAT and `BUOY_STATE`/`BUOY_AUDIO` every second in both SF positions, plus `B1` status every two seconds. Battery/current is sent every five seconds; GPS and Home are sent every five seconds with a fresh fix. SF high adds receiver RADIO_STATUS, changed RC channel banks, rotating science, and detailed heading readings. The active Home is manual Home in Manual or uploaded mission Home in Mission. The TX12 reads Home source/distance from `B1` and Home bearing from ExpressLRS passthrough when GPS has a fix.

With SD magnetometer calibration loaded, `MAG_HEAD` reports calibrated magnetic heading and `TILT_HEAD` an accelerometer tilt-corrected heading about every five seconds. Tilt correction is a snapshot and can be disturbed by hull acceleration. The buoy samples MPU6050 acceleration and rotation at about 10 Hz in MAVLink mode and sends five-second RMS summaries as `SEA_ACC` (dynamic acceleration in g) and `SEA_GYR` (angular rate in degrees/second). These indicate relative hull motion; they are not wave height or real-time attitude, and motor vibration can contribute. `BUOY_STATE` carries buoy mode, control source, arming, science-session recording, link-loss recovery, and motor outputs. `BUOY_AUDIO` carries separate microphone-to-SD recording state. Rotating NAMED_VALUE_FLOAT readings include water temperature, depth, TDS voltage, and turbidity voltage; while a mission runs, mission progress fields also rotate. The GPS message includes measured ground speed and track when valid. High-rate IMU, audio, and SD file downloads remain on Wi-Fi.

Missions are uploaded over buoy Wi-Fi and stored on SD. Autonomous execution uses the uploaded boundary and waypoint route, GPS position, and calibrated tilt-corrected magnetic heading. SB low selects direct Manual, middle selects Manual Assist, and high requests Mission. In Manual Assist, calibrated compass heading adds a bounded yaw correction only while the buoy is moving and the yaw stick is centered; moving the yaw stick turns directly, and centering it captures the new heading. This does not require GPS. The TX12 `B1` status reports Direct (0), Assist (1), Assist without calibrated heading (2), or Mission (3). SE/CH5 arms after the standard arm-low cycle, minimum CH3, and centered sticks. Deselecting Mission or disarming stops the run. The station's Backpack path carries mission progress and low-rate link, channel, battery, GPS, calibrated heading, and motion telemetry; upload and Wi-Fi Stop remain Wi-Fi-only.
