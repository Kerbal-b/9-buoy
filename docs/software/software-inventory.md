# Buoy Project Software Inventory

This list records software resources actually used or directly represented in the current project. An exact version is shown when it could be verified on the development computer. `Record needed` means the resource is part of the workflow, but its installed version has not yet been documented.

## Firmware Development

| Resource | Version or status | How it is used | Project evidence or setup |
| --- | --- | --- | --- |
| [Arduino IDE](https://www.arduino.cc/en/software) | 2.3.10 | Edit, compile, upload, and monitor ESP32 test and production firmware | Firmware sketches under `src/firmware/esp32/` |
| Arduino CLI | 1.5.1 | Repeatable command-line compile, upload, board detection, and serial monitoring | Board target and commands are in the configuration reference |
| [Arduino ESP32 core by Espressif](https://docs.espressif.com/projects/arduino-esp32/en/latest/) | 3.3.11; board target `esp32:esp32:esp32` | ESP32 framework, Wi-Fi, UDP, SPI, I2C, UART, SD, I2S audio, timers, and FreeRTOS support | Production firmware includes core-provided libraries |
| TinyGPSPlus | 1.0.3 | Parses NMEA data from the HGLRC M100-5883 GPS | `#include <TinyGPSPlus.h>` in production firmware |
| OneWire | 2.3.8 | One-wire bus support for the DS18B20 temperature probe | `#include <OneWire.h>` |
| DallasTemperature | 4.0.5 installed; 4.0.6 available when checked | Reads DS18B20 temperature sensors | `#include <DallasTemperature.h>` |
| Built-in ESP32 libraries | Supplied by ESP32 core 3.3.11 | `Wire`, `WiFi`, `WiFiUdp`, `FS`, `SD`, `SPI`, I2S driver, timers, and Arduino runtime | Included by the production firmware; version follows the ESP32 core |
| Custom CRSF implementation | Project source | Reads ExpressLRS controller channels and returns selected telemetry without a separate Arduino CRSF library | `src/firmware/esp32/esp32-elrs-crsf-test/` |

## Laptop Control Station

| Resource | Verified version | How it is used | Project evidence or setup |
| --- | --- | --- | --- |
| [Python](https://www.python.org/) | 3.13.15 in the project virtual environment | Runs the control station, communications, telemetry processing, and diagnostics | `src/control_station/.venv/` and setup scripts |
| [PySide6 / Qt for Python](https://doc.qt.io/qtforpython-6/) | 6.11.1 | Main Qt Quick desktop interface and QML dashboard | `src/control_station/station/qt_app.py` and `qml/` |
| PySide6-Addons | 6.11.1 | Additional Qt/QML modules, including map-related support | `src/control_station/requirements.txt` |
| [pygame](https://www.pygame.org/) | 2.6.1 | Xbox-style controller input and legacy fallback interface | Pinned in `requirements.txt` |
| [pyserial](https://pyserial.readthedocs.io/) | 3.5 | Optional USB/serial transport and serial diagnostics | `station/serial_link.py` |
| [Bleak](https://bleak.readthedocs.io/) | 3.0.2 | Optional Bluetooth Low Energy transport for HM-10-style testing | `station/serial_link.py` |
| Windows launcher scripts | Project source | Create the virtual environment and start the control station or controller diagnostics | `setup_env.ps1`, `run_control_station.ps1`, and matching `.bat` files |
| macOS/Linux launcher scripts | Project source | Create the virtual environment and start the same application | `setup_env.sh` and `run_control_station.sh` |

The virtual environment contains a few document/PDF packages used for project documentation in addition to the control-station requirements. The authoritative runtime dependencies are the five entries in `src/control_station/requirements.txt`.

## Field Router And Phone Dashboard

| Resource | Version or status | How it is used | Project evidence or setup |
| --- | --- | --- | --- |
| GL.iNet Opal (`GL-SFT1200`) firmware | `4.3.28`, release 2; router panel reports build date `2025-06-25` | Field access point and local host for the receive-only dashboard/API | `src/router_bridge/README.md`; admin address `192.168.8.1` |
| GL.iNet LuCI | Installed | OpenWrt system configuration and package management | Router UI; installation verified on November 8, 2026 |
| OpenWrt Python 3 | `3.6.5-1`, full `python3` package | Runs the bridge without pip or third-party Python modules | `/usr/bin/python3` on the Opal; Python 3.6 syntax is maintained in `src/router_bridge/` |
| `mavlink_bridge.py` | Project source, standard library only; deployed November 8, 2026 | Receives MAVLink UDP on port `14550`, decodes telemetry, serves the dashboard and read-only JSON API | `src/router_bridge/mavlink_bridge.py`; deployed copy `/root/mavlink-bridge/mavlink_bridge.py` |
| `mavlink-bridge` procd service | Installed and enabled | Starts/restarts the bridge on router boot | `src/router_bridge/mavlink-bridge.init`; deployed at `/etc/init.d/mavlink-bridge` |
| Router HTTP service | Port `8080` | Serves dashboard at `/`, compact state at `/api/state`, raw feed at `/api/raw` and `/api/telemetry` | Full contract: `src/router_bridge/API.md` |
| GL.iNet ttyd terminal | Available at `http://192.168.8.1:7681/` on the local network | Browser shell used to install/manage the bridge when SSH key access was unavailable | Router service; keep access limited to the trusted buoy LAN |
| Phone browser | No app/build dependency; tested rendering the dashboard on iPhone | Displays router-served page and polls router JSON; emits no MAVLink | `http://192.168.8.1:8080`; map works without internet tiles |

The bridge and API returned successful responses from the PC on November 8, 2026, and the dashboard page was viewed on the iPhone. The sampled stream included buoy state, science values, attitude, radio and RC data from the Backpack at `192.168.8.134`; it did not include GPS position in that check. Live GPS display and simultaneous QGC operation remain unverified. The bridge is listen-only and does not request parameters or send commands.

## Radio and Operator Software

| Resource | Version or status | How it is used | Configuration record needed |
| --- | --- | --- | --- |
| [EdgeTX](https://edgetx.org/) on RadioMaster TX12 MKII | `HTX12MK2`, version `2.12.5`, radio name `Queen Anne's Revenge`, firmware date `2026-10-18` (reported by user on 2026-11-01). Model Settings currently shows Setup, Heli Setup, Flight Modes, Inputs, Mixes, Outputs, Curves, Logical Switches, Special Functions, Telemetry, Display, and Model Cell; no Custom Scripts tab | Stores the buoy radio model, channel mapping, switches, telemetry sensors, and failsafe behavior | Check why Custom Scripts is absent on this recorded build before configuring `mixcal.lua`; verify whether Lua Mixes (`LUA_MIXER`) is built in and whether its model-menu tab is hidden/disabled |
| [ExpressLRS](https://www.expresslrs.org/) transmitter and receiver firmware | TX12 internal module reported as `RMTX12` version `4.1.0` on 2026-11-01; buoy receiver version not yet verified. Production uses MAVLink mode | Direct TX12 control through the buoy receiver; Backpack forwarding is optional for control-station telemetry | Record receiver firmware version, regulatory domain, binding method, packet rate, telemetry ratio, and receiver UID/binding phrase handling without publishing secrets |
| ExpressLRS flashing/configuration tool | Exact application and version must be recorded | Used to flash and configure the ELRS receiver | Record whether ExpressLRS Configurator, Wi-Fi Update, or another method was used |

## Design, Documentation, Data, and Collaboration

| Resource | Version or status | How it is used | Project record |
| --- | --- | --- | --- |
| [Autodesk Fusion](https://www.autodesk.com/products/fusion-360/overview) | Cloud-updated; exact build not recorded | Hull, component placement, and mechanical/CAD design | `docs/architecture/physical-design.md` and CAD files |
| [Git](https://git-scm.com/) | 2.55.0.windows.3 | Source version control | Repository history and working tree |
| [GitHub](https://github.com/Kerbal-b/9-buoy) | Hosted service | Remote source repository | Remote `origin` points to `Kerbal-b/9-buoy` |
| Google Drive, Docs, and Sheets | Hosted services | Component inventory, research notes, collaboration, and printable documents | Component Inventory sheet and project Docs folder |
| [GLEON Lake Observer](https://www.lakeobserver.org/) | Project account created | Records georeferenced lake observations and supports comparison with an established citizen-science format | Redwood Shores Lagoon observation project |

## Items To Record Before Field Release

- [ ] EdgeTX version on the TX12.
- [ ] ExpressLRS transmitter and receiver firmware versions.
- [ ] ExpressLRS regulatory domain, packet rate, telemetry ratio, and model-match setting.
- [ ] Exact ELRS flashing method and tool version.
- [ ] Fusion build number used for the final mechanical design.
- [ ] Freeze the final Python dependency versions with `pip freeze` after the control station is validated.
- [ ] Save the Arduino CLI core and library lists with the final firmware release.

Last verified: November 8, 2026. The GL.iNet router state comes from the router UI and deployed-service check; development-tool versions are shown as last recorded where they were not rechecked today.
