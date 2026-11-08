# Propulsion calibration

## Mixer profile workflow

The calibration CSV is analyzed deterministically and then interpreted with the rules in [`../tx12/skills/mixer-calibration-analysis/SKILL.md`](../../tx12/skills/mixer-calibration-analysis/SKILL.md). The processed artifact is a `buoy_mixer_candidate_v1` JSON containing a complete 60-point profile, the exact tested baseline and CRC, and the measured response. AI processing changes only values supported by the recorded response and documents those changes in `candidate_deltas`.

In **Propulsion → Mixer calibration**, connect to the buoy, disarm it, and leave Mission mode. Press **Upload processed mixer profile** and select the processed `.mixer-candidate.json`. The station validates run metadata, all 60 baseline points, and every unique movement/power value; it then fills the adjustment table and sends all 60 profile points to the buoy. Firmware saves `/config/propulsion.cfg`. The buoy receives the values over the existing control protocol and does not parse JSON. Wait for the buoy's save confirmation and profile CRC in the station.

The table displays the profile from the imported file and its measured evidence. The station requires a complete profile; incomplete drafts cannot be uploaded. A complete profile loads at boot and drives TX12 manual mixing. Motor PWM startup/ramp settings remain separate from this mixer table.

## Calibration data and interpretation

Calibration mode records at 20 Hz and suspends optional sensor streaming and standalone SD logging while keeping ELRS control/safety, GPS parsing, direct IMU/magnetometer/current/battery reads, and TCP status alive. It records current, battery voltage, motor commands, applied PWM, IMU/gyro, compass, GPS, and RC channels. After the November 8 water test, the sweep starts at 30% power and uses 10-second pulses by default or 20-second pulses in a large pool, with a neutral interval between pulses. Compare repeated trials with the same movement, duration, power, battery range, and buoy setup. Each run records the exact mixer CRC and motor settings.

Check direction signs first. Use gyro Z for yaw during short pulses; compass heading can be distorted by motor fields. Use axis-aligned acceleration and valid GPS speed/course for translation when fix and sample interval support it. Current and PWM reveal stalls and unequal loading but do not prove movement direction. Compare repeated runs and require a change to exceed the run-to-run spread. Ground speed is affected by current and wind, and acceleration can include tilt.

The deterministic analyzer parses and summarizes the data. AI interprets the measured response and creates the processed candidate; it must preserve the tested baseline and explain every supported adjustment. It must retain baseline values when measurements are inconclusive. See the mixer calibration analysis skill for the full procedure and output schema.

## Buoy profile format

`/config/propulsion.cfg` uses `BUOY_MIXER_PROFILE_V1` with `POINT,power,axis,rear,front_left,front_right` rows. Axis ids are 0 forward, 1 reverse, 2 strafe right, 3 strafe left, 4 clockwise yaw, and 5 counter-clockwise yaw. Motor values are signed command percentages. Firmware validates ranges, writes and verifies a temporary file, then atomically replaces the active file with backup recovery. Runtime interpolates adjacent power rows, adds simultaneous axis contributions, and scales the complete vector together at saturation. It uses the empirical profile for TX12 manual mixing only when all 60 points exist. Incomplete profiles and mission control retain their safe existing mix paths.

## Motor-load compass calibration

Navigation keeps motors-off base calibration separate from water-only motor correction capture. For a load survey, hold the buoy at a known fixed heading, record a motors-off baseline, then capture each motor separately in forward and reverse at selected water power levels while confirming gyro yaw stays near baseline. The capture records raw magnetometer XYZ, battery current and voltage, commanded motor/PWM, gyro, and timestamps. These captures do not modify the base compass calibration.

Fit any compensation against measured current, not commanded throttle alone. With one total-current sensor, isolate each motor in turn to estimate its contribution, then validate combined-motor cases. If the field change is not repeatable or a current-based fit leaves large heading errors, move the magnetometer farther from motors and power wiring or rely on gyro during those loads. Propellers unloaded on dry land may draw less current than in water, so validate accepted compensation under representative tethered water loads.
