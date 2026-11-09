# Software Configuration Reference

This is a reproducibility checklist, not a place for credentials. Values come from the current source code and installed development tools unless marked as a test-only setting.

For end-to-end operation, control modes, mission start interlocks, session/log lifecycles, MAVLink message layouts, and the router-to-phone telemetry path, see the [software behavior guide](system-behavior.md) and [MAVLink reference](mavlink-reference.md). The deployed Opal bridge's JSON contract remains [`src/router_bridge/API.md`](../../src/router_bridge/API.md).

## ESP32 Firmware Build

| Setting | Current value |
| --- | --- |
| Arduino CLI | 1.5.1 |
| Arduino IDE | 2.3.10 |
| Board package | `esp32:esp32` 3.3.11 |
| Fully qualified board name | `esp32:esp32:esp32` |
| Main sketch | `src/firmware/esp32/esp32-buoy-firmware/esp32-buoy-firmware.ino` |
| ELRS test sketch | `src/firmware/esp32/esp32-elrs-crsf-test/esp32-elrs-crsf-test.ino` |
| Bench USB port | Usually `COM8`; always confirm before uploading |

Example compile command:

```powershell
arduino-cli compile --fqbn esp32:esp32:esp32 src/firmware/esp32/esp32-buoy-firmware
```

Example upload command after confirming the port:

```powershell
arduino-cli upload -p COM8 --fqbn esp32:esp32:esp32 src/firmware/esp32/esp32-buoy-firmware
```

The buoy partition table has two OTA application slots. Firmware `1.16.9` adds authenticated ArduinoOTA support over the buoy Wi-Fi network. The first update that installs this support must still be uploaded over USB. After that, run the repository helper with the buoy's Wi-Fi IP address:

```powershell
.\scripts\upload-buoy-ota.ps1 -BuoyAddress 192.168.8.XX
```

The helper compiles the sketch, prompts securely for the buoy Wi-Fi password, and uploads the application image over Wi-Fi. Keep the buoy powered, disarmed, and without an active science session during an update. OTA is accepted only while motors are stopped and the buoy is idle. Disarmed ELRS source is allowed; Wi-Fi control ownership is not. The bootloader and partition table are not changed by OTA updates.

The installed Arduino libraries verified on September 27, 2026 were TinyGPSPlus 1.0.3, OneWire 2.3.8, and DallasTemperature 4.0.5. Core-supplied libraries follow ESP32 core 3.3.11.

Deployment record: the buoy reported firmware `1.16.27+20261108` during the November 8, 2026 field setup. The source was subsequently advanced locally; that newer source version is not part of this deployment record and must not be described as flashed until an upload is confirmed.

## Main Communications Configuration

| Link | Current configuration | Purpose |
| --- | --- | --- |
| Wi-Fi station | Network credentials are presently configured in firmware; deliberately omitted here | ESP32 joins the buoy network |
| TCP | Port `5000` | Reliable commands and line-based status/science messages |
| UDP telemetry | Port `5001` | Fast binary IMU, power, state, and GPS packets |
| UDP audio | Port `5002` | Stereo PCM audio stream |
| Backpack MAVLink forwarding | UDP destination port `14550` | Backpack forwards the receiver MAVLink stream to a ground-station listener; destination IP/unicast-versus-broadcast must be verified in the Backpack forwarding settings. The Opal bridge listens here and is receive-only. |
| Opal dashboard/API | HTTP port `8080` | Local phone page and JSON API; Python 3.6 standard-library bridge. See software inventory and API contract. |
| Opal ttyd | HTTP port `7681` | Router browser terminal for local administration; available only on the buoy LAN. |
| Optional serial link | `9600` baud by default in the laptop application | Alternate control/testing path |
| Optional BLE | Device-name discovery plus HM-10 service `FFE0` and characteristic `FFE1` UUIDs | Alternate test transport |

Security note: Wi-Fi credentials should be moved from tracked source into an untracked local configuration before the repository is shared publicly. Rotate any credential that has already appeared in project documentation or source history.

## Laptop Control Station Defaults

| Setting | Current value |
| --- | --- |
| Default transport | Wi-Fi |
| Wi-Fi host | Automatic discovery |
| Controller deadzone | `0.12` |
| Command send rate | `10 Hz` |
| Serial baud rate | `9600` |
| Manual-control safety power limit | `50%` |
| Propulsion watchdog | Fresh commands required at least once per second while motors are active |
| Controller input | Left stick X/Y for translation; right stick X for yaw |

Create and run the Windows environment with:

```powershell
.\src\control_station\setup_env.ps1
.\src\control_station\run_control_station.ps1
```

The validated project environment is Python 3.13.15 with pygame 2.6.1, pyserial 3.5, Bleak 3.0.2, and PySide6/PySide6-Addons 6.11.1.

## ExpressLRS and CRSF Bench Configuration

| Setting | Current test value |
| --- | --- |
| ESP32 UART | UART2 |
| CRSF receiver-to-ESP32 input | ESP32 GPIO 34 |
| CRSF ESP32-to-receiver telemetry output | ESP32 GPIO 25 |
| Serial format | `420000` baud, 8 data bits, no parity, 1 stop bit, non-inverted |
| RC failsafe timeout in test firmware | `500 ms` |
| Selected return telemetry | Battery approximately 1 Hz; GPS approximately 0.5 Hz; flight mode approximately 1 Hz |
| TX12 telemetry setup | Telemetry ratio must not be `OFF`; discover new sensors after the link is active |

The ELRS test firmware sends fixed demonstration values for some fields. Do not treat its reported GPS position, battery percentage, or flight mode as scientific data until those fields are connected to live production measurements.

Record these radio settings from the TX12 and ELRS tools before field testing: firmware versions, regulatory domain, packet rate, telemetry ratio, switch/channel map, failsafe positions, model match, and power settings. Keep binding secrets private.

## Data and File Formats

| Output | Format or protocol | Notes |
| --- | --- | --- |
| SD operational log | Text log | Events and diagnostic messages; no card means no SD write |
| SD telemetry | CSV with microsecond timestamp field | Operational state, power, GPS, motor output, and SD health |
| SD science data | CSV | Calibrated units where available; TDS and turbidity remain voltage readings until wet calibration |
| SD audio | WAV/PCM session recording | High-rate audio is buffered separately from slower logs |
| Laptop commands | ASCII lines | `CTRL VECTOR` and `CTRL MOTION` commands |
| Fast telemetry | Binary UDP packets | Packet types are documented in the control-station README and code reference |
| ELRS radio link | MAVLink receiver protocol in production; CRSF in a separate bench sketch | Production RC channels toward ESP32 and selected telemetry toward TX12 |

The active firmware remains the source of truth for GPIO assignments and embedded timing values. The control-station source remains the source of truth for desktop defaults.
