# TX12 Lua telemetry display

## Essential telemetry (firmware 1.16.27)

SF/CH6 now selects the **detail route**. In either position the buoy sends arm, assist,
mission state and progress, waypoint, session/audio, calibrated tilt heading, GPS fix,
Home source and distance, battery/current, and attitude over ELRS. Every two seconds
it sends a versioned `B1` MAVLink status text; during calibration it also sends `BC1`
with state, step, current power, power limit, and pulse duration. ExpressLRS 4.1
converts status text to CRSF passthrough text, which `rxscan.lua` reads directly;
these fields do not need EdgeTX sensor discovery. SF high additionally enables
rotating science, detailed navigation, and receiver channel reports. `CAPA` is
shown as unavailable because the buoy does not measure consumed mAh.

This requires both the new buoy firmware and the updated `rxscan.lua` on the TX12.
The Backpack forwards the same MAVLink status to the control station when its
telemetry forwarding is enabled. Link budget and packet loss can delay individual
updates; the TX12 marks status stale after six seconds.

This folder is for EdgeTX Lua scripts and notes for the RadioMaster TX12 MKII used with the buoy. The documented model setup is Mode 2 with AETR channel order:

| Channel | Control | Buoy use |
| --- | --- | --- |
| CH1 | Right stick horizontal | Lateral movement |
| CH2 | Right stick vertical | Forward/reverse |
| CH3 | Left stick vertical | Motor power ceiling |
| CH4 | Left stick horizontal | Yaw |
| CH5 / SE | Arm / disarm | Low disarms; high requests arm after the safety checks |
| CH6 / SF | Telemetry route | Low/middle selects Wi-Fi telemetry; high selects MAVLink telemetry |
| CH7 / SB | Mode request | Low direct manual; middle Manual Assist with heading hold; high requests the loaded mission |
| CH8 / SC | Manual session control | Low saves/stops; middle records science; high records science with audio |
| CH9 / SA | Manual Home / Return Home | Hold while arming to set Home; after arming, hold with SD for one second to request Return Home |
| CH10 / SD | Return Home confirmation | Hold with SA for one second; SD alone does nothing |

The active EdgeTX model must output these channels. Confirm the model name and channel values in the TX12 channel monitor before operation. `rxscan.lua` is a read-only telemetry display. The optional `mixcal.lua` Lua Mixes script described below emits calibration commands on CH11–CH16; it does not change CH1–CH10.

## Manual Home and Return Home

For manual control, put SB low or middle. Put SE low, hold SA, keep the sticks centered and CH3 at zero, then raise SE to arm. The buoy captures its current location as manual Home only if GPS is fresh, has at least 5 satellites, and HDOP is 3.0 or lower. The captured Home is saved in ESP32 nonvolatile storage and survives power cycles; mission Home remains separate and comes from the loaded mission. After arming, release both SA and SD; hold them together for one second to start Return Home. Either button alone does nothing. SE away from high stops and disarms. Manual Return Home needs GPS and a calibrated magnetometer heading, uses a direct route without a geofence, and stops if GPS, heading, or RC control is lost.

## Install a telemetry Lua script

1. On the TX12, select the buoy model. Confirm its model name and keep a backup of the radio SD card before changing files.
2. Connect the radio to the computer in USB storage mode, or remove the microSD card and use a card reader. Open the radio's SD card root (the folder containing `SCRIPTS`, `MODELS`, and other EdgeTX folders).
3. Copy the script file to `SCRIPTS/TELEMETRY/`. Create the `TELEMETRY` folder if it does not exist. EdgeTX requires the filename before `.lua` to be six characters or fewer, so use a short name such as `buoy.lua`.
4. Safely eject the storage device/card and restart the radio.
5. With the buoy model selected, open Model Settings > Display. Set one of the telemetry screens to **Script**, then choose the installed script. On the TX12 MKII, use the **Tele** button from the home screen to open the configured telemetry screens. EdgeTX menu names vary slightly by firmware version; the script must appear in the model's telemetry script list before it can be selected.
6. With the receiver and buoy powered, check each displayed value against the control station and the model's telemetry monitor. Missing sensor fields need to be made available to EdgeTX before a Lua screen can show them.

EdgeTX telemetry scripts are stored under `/SCRIPTS/TELEMETRY/`, limited to six-character names (before `.lua`), and configured per model. See the [EdgeTX Lua telemetry script guide](https://luadoc.edgetx.org/2.11/overview/script-types/telemetry-scripts/) and the [EdgeTX B&W telemetry display instructions](https://manual.edgetx.org/v2.8/b-and-w-radios/model-select/display).

## Inspect what the TX12 receives

This repository includes [`telemetry/rxscan.lua`](telemetry/rxscan.lua), a read-only four-page flight display. Use `+` and `-` to move between pages (or the configured virtual page keys):

| Page | Shows |
| --- | --- |
| 1 - Status/Mission | Arm, assist mode, mission state/progress/waypoint and start block, plus session/audio from the versioned buoy status packet. |
| 2 — Navigation | GPS fix, Home source and distance, compass heading, Home bearing, GPS speed and satellite count. Unknown values are labeled clearly. |
| 3 — Power/Radio | Pack voltage, estimated battery percentage, link quality, RSSI and SNR. Capacity and the per-value LIVE/OLD column are omitted. |
| 4 - Mixer Calibration | Local CH11-CH16 requests and received buoy calibration state, step and power. Press ENTER for expected pulse levels. |

Each page uses one title line. Status/Mission shows `NO BUOY STATUS PACKET` plus received text and B1 counts if the buoy status stream is missing. Values without a valid source show `--` or `UNKNOWN`; the screen does not spend a column on per-value freshness. The pack percentage is a voltage-based estimate for a 3S battery (9.6–12.6 V), so a reading below 9.6 V is flagged as a power-input problem. Raw sensor discovery and inspection remain in EdgeTX Telemetry settings.

1. Copy `telemetry/rxscan.lua` to the radio SD card as `SCRIPTS/TELEMETRY/rxscan.lua`.
2. With the buoy model selected, open its **Telemetry** settings and choose **Discover new sensors**. Keep the radio and buoy powered while discovery runs. Discover standard GPS, battery, attitude, and radio-link sensors. The buoy sends `MIS_*`, `MAN_AST`, `CAL_*`, magnetic-heading, and science fields as MAVLink `NAMED_VALUE_FLOAT`; ExpressLRS forwards these to Backpack MAVLink clients but does not turn them into discoverable EdgeTX sensors. Repeating sensor discovery will not make those names appear.
3. Either SF/CH6 position carries essential status. SF high also enables detailed rotating radio telemetry. Keep SE/CH5 low and the motors disarmed for setup.
4. Add a telemetry **Script** screen for `rxscan` under the model's **Display** settings. Open it with the radio's **Tele** screen button.
5. Leave Status/Mission open for at least six seconds. `B1` should increase, and arm/session/mission changes should appear within a few seconds. If `B1` stays zero, report the `TEXT RX`, `CRSF RX`, and `LAST` lines; this distinguishes missing RF text from a format mismatch. Standard battery readings normally update about every five seconds.
6. If standard battery, GPS, attitude or link values are missing, check sensor discovery. Buoy-specific mission and calibration status arrives as CRSF passthrough text and does not appear as an ordinary sensor.

The display script reads the model sensor list and values via EdgeTX's Lua API, plus Home passthrough frames. The Backpack MAVLink link remains telemetry-only; calibration commands travel uplink through the existing RC channel override path. Stop/Return Home web commands still use the buoy Wi-Fi connection. EdgeTX documents `model.getSensor()` and `getSourceValue()` for reading configured sensor metadata and values, including freshness status ([sensor API](https://luadoc.edgetx.org/dev/api-reference/model-functions/model-get-sensor/), [value API](https://luadoc.edgetx.org/2.11/api-reference/variables/getsourcevalue/)).

## Telemetry source and limits

The TX12 reads versioned buoy status and calibration text directly from ExpressLRS CRSF passthrough in either SF position. Standard GPS, battery, attitude, and link sensors remain available when their source data is valid. Home distance requires a valid GPS fix; Home source and mission state do not. Raw science values are available to the Backpack/control station in SF-high mode but are not ordinary EdgeTX sensors.

Backpack MAVLink forwarding to the control station over Wi-Fi and TX12 telemetry over the RF link are separate paths. A radio display can remain useful when Backpack Wi-Fi is unavailable, for values the receiver/radio link exposes to EdgeTX. `rxscan.lua` stays read-only. The optional `mixcal.lua` script is a dedicated calibration sequencer; normal arm, drive, session, and telemetry controls stay on CH1–CH10.

## Run a buoy calibration from the TX12

The calibration mix script needs an EdgeTX radio build with Lua Mixes enabled. The project radio is a RadioMaster TX12 MKII running build `HTX12MK2`, EdgeTX `2.12.5`, dated `2026-10-18` (radio name `Queen Anne's Revenge`, reported 2026-11-01). The user reports no **Custom Scripts** tab in Model Settings; enable it under **Setup → Enabled Features → Model Menu Tabs** if present. If the option itself is absent, verify whether Lua Mixes (`LUA_MIXER`) is included in the build before proceeding. Do not try to add the Lua file as a normal Mixes line. Copy [`mixes/mixcal.lua`](mixes/mixcal.lua) to the radio SD card as `SCRIPTS/MIXES/mixcal.lua`, and [`tools/calgo.lua`](tools/calgo.lua) to `SCRIPTS/TOOLS/calgo.lua`. The buoy firmware must be v1.16.20 or newer and the receiver must be sending a full 16-channel `RC_CHANNELS_OVERRIDE` frame.

In the buoy model's Mixes setup, add the `mixcal` script and set its inputs:

- `Action`: select the radio source **G9**. The `calgo.lua` tool provides a three-page setup guide; ENTER on its final page sends the start trigger and closes the tool.
- `Arm`: the SE arm switch source.
- `Mode`: 0 for mixer response, 1 for individual motor and compass load.
- `MaxPwr`: highest test power in 10% increments. It defaults to 30%. A 100% setting runs all six directions at 30%, 40%, and so on through 100%; it does not make the first step 100%.
- `Duration`: 0 for a 10-second pulse, 1 for a 20-second pulse. The installed `BOUY` model is set to 0 and keeps its existing 70% maximum.

Set the `Action` source to **G9** (reserve G9 for calibration and do not use it in another mix). The `calgo.lua` one-time tool in **SYS → Tools** shows the test plan and preflight reminders. On its final page, press ENTER to raise G9 and close the tool; the mixer restarts and consumes the trigger. EdgeTX suspends other Lua scripts while a one-time tool is open, so the tool cannot stay open to show live progress. Use page 4 of `rxscan.lua` or the buoy calibration CSV for run status. This is a radio-local trigger; it does not use buoy telemetry.

Route the six script outputs, each at 100% weight with no offset, to CH11 through CH16 in order: `CalEn`, `CalCmd`, `CalMode`, `CalIdx`, `CalPwr`, `CalDur`. Keep all existing CH1–CH10 mixes. **The internal ExpressLRS switch mode must carry all 16 proportional channels.** The earlier mode passed CH11/CH12 but left CH13/CH15/CH16 at 881 µs and mirrored the arm switch on CH14; the buoy therefore rejected every calibration step. After the radio configuration change, the buoy received CH13–CH16 near the expected 988–2011 µs range and accepted the first 10% step with `MaxPwr=100`. The buoy log did not record the actual RF packet rate, so confirm the radio's packet-rate setting separately. ExpressLRS documentation lists 100Hz Full and 333Hz Full as packet rates that support Full Res 16ch Rate/2; regular 150Hz carries fewer usable channels. Change switch mode while the receiver is disconnected and SE is low; ExpressLRS can reject a mode change while connected. Set the Lua `Duration` input to 0 for the normal 10-second run or 1 for the 20-second large-pool run; it must remain unchanged during a run. Use the TX12 channel monitor to verify CH11 is low while idle; CH12 rests near 1350 µs, briefly moves near 1200 µs on start, then holds near 1500 µs for each 10 or 20 second test; CH13 selects the two modes; CH14 steps through six test indices; CH15 carries the configured 10% power steps; and CH16 selects 10 or 20 seconds. Confirm the buoy's received CH13–CH16 in its ELRS channel status before testing motors; TX12 channel monitor values alone do not prove transmission.

For a test, select SB Manual, leave SE low, keep CH1/CH2/CH4 centered, CH3 at minimum, and ensure no science session, mission, Wi-Fi motor control, or SD transfer is active. Run Cal Trigger and press ENTER on its final screen. The buoy creates a file under `/calibration/` and waits for SE. Raise SE to arm; the script then runs six directions at each 10% power level from 30% through `MaxPwr`, with 3 seconds neutral between steps. The default pulse is 10 seconds (13 seconds between step commands); select `Duration=1` for a 20-second pulse (23 seconds between commands) in a large pool. During calibration, firmware prioritizes control and calibration reads and pauses optional Wi-Fi UDP sensor streams and standalone SD logging. Required sensors remain active: the buoy polls the gyro/accelerometer, magnetometer, current and battery sensors at 20 Hz (every 50 ms), while GPS parsing and ELRS safety/status continue. Current and voltage validity flags distinguish failed sensor reads from real zero values. Lowering SE aborts and disarms. Mixer response mode tests forward, reverse, right strafe, left strafe, clockwise yaw, and counterclockwise yaw. Individual motor mode tests each motor forward and reverse while the buoy is held steady for compass correction measurements. The buoy automatically closes the file and disarms after the last step. An interrupted or incomplete step sequence is marked incomplete and also disarms.

Each CSV starts with `#key=value` metadata, including firmware, the active mixer profile CRC and all 60 profile points, every per-motor PWM/deadband/boost/curve/scale/offset/ramp/mixing setting, safety limits, and the test protocol. `SAMPLE` rows then record the requested step and power, commanded axis, drive targets, ramped drive and applied PWM, current and battery, accelerometer and gyro, raw magnetometer and calibrated heading, GPS, and the transmitted RC channel values. A `#completed=0` footer means the run was interrupted. The data stays separate from science session files and can be copied from SD through the existing file access. This gives later analysis the exact baseline configuration to calculate adjustments on top of.

The buoy sends calibration state, step, current power, limit and duration in the `BC1` status packet every two seconds while a calibration result is available. `rxscan.lua` shows the received state on page 4. CH11-CH16 are still the outgoing command values; compare them with the received state. The buoy CSV preserves the exact baseline profile and motor settings. See [`skills/mixer-calibration-analysis/SKILL.md`](skills/mixer-calibration-analysis/SKILL.md) for analysis and profile upload.

### Announce the calibration start on the TX12

[`functions/calbeg.lua`](functions/calbeg.lua) is a Function Script example that plays a spoken prompt once when **Cal Trigger** is confirmed. It uses a shared-memory mailbox because EdgeTX pauses Function Scripts while the one-time tool is open; the tool leaves the request, closes, and the Function Script speaks after it resumes. It does not use a buoy telemetry link or reserve another physical switch.

1. Copy `functions/calbeg.lua` to the radio SD card as `SCRIPTS/FUNCTIONS/calbeg.lua`.
2. Put a spoken WAV saying **“Calibration starting”** at `SOUNDS/en/calbeg.wav`. The project does not include this recording yet; use a recording or a generated voice clip with that filename.
3. In the selected model's **Special Functions**, add a row with switch `ON`, function **Lua Script**, value `calbeg`, and repeat `ON`. This keeps the lightweight listener active without taking a buoy control switch.
4. Keep the existing `calgo.lua` Tool and `mixcal.lua` Mixes setup. When you run Cal Trigger and confirm its last page, the TX12 says “Calibration starting.” The buoy still waits for SE to be raised before it begins the motor test; use the existing preflight and disarm procedure.

The Function Script reads and clears shared-memory slot 0. Keep that slot reserved for this start announcement if adding other radio Lua scripts that use shared memory.

## Development notes

- Put project telemetry scripts in this directory and copy the selected `.lua` file to the radio SD card using the steps above.
- Keep each script lightweight and compatible with the TX12's installed EdgeTX version and screen size.
- Check fields with EdgeTX's telemetry discovery/monitor before adding them to a screen. Treat unavailable or stale readings as unavailable; don't reuse an old value as if it were live.
- The existing control map is documented in [`../control_station/config/TX12-CONTROLS.md`](../control_station/config/TX12-CONTROLS.md).
