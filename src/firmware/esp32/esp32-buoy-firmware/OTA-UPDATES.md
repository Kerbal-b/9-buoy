# Updating buoy firmware over Wi-Fi

The buoy accepts Arduino OTA updates starting with firmware `1.16.9+20261031`.
The first install was flashed over COM8 on 2026-10-31, then an upload over Wi-Fi
to `192.168.8.120` was completed and verified. The address was the buoy's address
that day; check its current address before each update.

Build `1.16.20+20261101` was uploaded over Wi-Fi to `192.168.8.120` on
2026-11-01. Build `1.16.21+20261102` was uploaded over Wi-Fi to the same
address on 2026-11-02. `REQ STATUS ALL` verified the new version, disarmed
state, and zero motor outputs. The new status line includes CH11–CH16; a
calibration-start interlock event now records each pass/fail flag and all six
calibration channel values in the control-station trace and buoy system log.
Build `1.16.22+20261102` accepts the EdgeTX extended channel endpoints found
during mixer calibration and records the decoded test settings when a start is
accepted. Build `1.16.23+20261102` applies that endpoint range to each test-step
request and records the channel values if a step is rejected. Build
`1.16.24+20261102` fixes the calibration arm latch so subsequent radio frames
do not stop a run with `SAFETY_INTERLOCK`. It was uploaded over Wi-Fi to
`192.168.8.120` and verified with `REQ STATUS ALL` on 2026-11-02.
Build `1.16.25+20261102` latches the verified low-SE start state for the first
armed frame and reports calibration state and exact channel values on a step
mismatch. It was uploaded over Wi-Fi to the same address and verified disarmed
with zero motor output on 2026-11-02.
Build `1.16.27+20261108` was compiled and uploaded over Wi-Fi to
`192.168.8.120` on 2026-11-08. The buoy was powered by USB with the 12 V pack
disconnected. After reboot, `REQ STATUS ALL` reported firmware 1.16.27,
`ELRS_ARM DISARMED`, `MISSION_CONTROL MANUAL`, calibration idle with a 30% power
floor, and all six motor outputs at zero. The reported 4.31 V battery-input
reading reflects the USB-only setup and must not be used for a water test.

## Update procedure

1. Power the buoy from a stable supply. Put SE/CH5 in disarmed, stop any mission
   and science session, release Wi-Fi motor control, and wait for motor outputs
   to reach zero. Finish any SD transfer or service operation.
2. Put the computer on a network that can reach the buoy's Wi-Fi IP address.
   Find the current address in the control station's **Communications → Network**
   page or its `WIFI:<address>:5000` connection target. The Backpack telemetry
   link cannot carry firmware updates.
3. From the project root, run:

   ```powershell
   .\scripts\upload-buoy-ota.ps1 -BuoyAddress 192.168.8.120
   ```

   Substitute the current address. The helper compiles the sketch using Arduino
   CLI, prompts for the buoy Wi-Fi password, then uploads the build. If Arduino
   CLI loses network-port discovery, it retries by IP with the ESP32 OTA tool
   from the installed board package and the control station Python environment.
   It does not save the password. Keep the buoy powered and disarmed through
   the reboot.
4. Reconnect in the control station and check the reported firmware version,
   disarmed state, and zero motor outputs. On the initial verified update,
   `REQ STATUS ALL` returned `TEL STATUS FIRMWARE 1 16 9 20261031`.

The helper uses the installed `arduino-cli`, or the CLI bundled with Arduino IDE,
and writes build output to `.build\buoy-firmware`.

## If the upload does not connect

- Confirm that the IP is current and reachable from the computer.
- If Arduino CLI reports `port not found` after an interrupted transfer, use
  the helper again. Its direct-IP fallback does not depend on port discovery.
- Confirm the buoy is disarmed, no mission or session is active, Wi-Fi motor
  control is released, and no SD transfer or service mode is active. The OTA
  service only accepts a new update under those conditions.
- If authentication fails, enter the buoy Wi-Fi password at the prompt.
- If the installed firmware cannot start OTA or a recovery is needed, flash
  over COM8 with Arduino CLI or Arduino IDE.

OTA updates the application partition. Bootloader or partition-table changes
still require a wired flash. The `1.16.9` image used about 94% of its OTA app
slot, so check image size before adding much more firmware.
