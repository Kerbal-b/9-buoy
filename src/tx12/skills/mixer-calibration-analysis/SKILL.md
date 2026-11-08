---
name: mixer-calibration-analysis
description: Analyze a completed BUOY_ELRS_CALIBRATION_V1 CSV, interpret measured response, produce a complete processed mixer profile JSON while preserving the tested baseline, and apply it through the control station.
---

# Mixer calibration CSV workflow

Use this process when the user supplies or downloads a buoy calibration CSV and wants the mixer profile values processed for upload.

## Deterministic analysis

Run the repository analyzer:

```powershell
python scripts/analyze_mixer_calibration.py "path\to\calibration.csv"
```

It validates the format, requires `test_mode=mixer_response`, requires `completed=1`, and requires all 60 baseline mixer points in the metadata. It groups active samples by direction and power and writes a `.mixer-candidate.json` beside the source unless an output path is supplied with `--output`. The bundle contains source firmware and baseline CRC, motor settings, the exact baseline profile, measured response summaries, and candidate deltas.

The generated profile starts as a copy of the profile recorded in the CSV. This preserves the exact settings used during the test. The measurements are evidence for processing; the deterministic analyzer does not infer thrust from IMU acceleration or silently rewrite motor commands. A partial or interrupted run is rejected. `magnetometer_motor_load` runs are not mixer candidates.

## Interpret the response

Check direction signs first. For yaw, use gyro Z during the short motor pulse; do not infer rotation from compass heading alone while motors are energized. For translation, use axis-aligned acceleration and valid GPS speed/course where the fix and sample interval support it. Current and applied PWM help identify stalls, dead zones, and unequal motor loading; they do not by themselves prove movement direction.

Compare repeated runs before changing values when available. The report groups trials by mixer CRC and gives the per-direction/power mean and run-to-run spread for acceleration, gyro Z, current, battery voltage, and valid GPS speed. Compare before/after means with repeat spread. Different firmware or motor settings are not direct comparisons. Ground speed is affected by current and wind, and acceleration can include tilt.

Use the CSV's exact tested baseline as the starting configuration. Change only movement/power points supported by observed response. Prefer smooth changes across adjacent power steps, keep all motor commands in -100..100, and preserve a direction that already behaves correctly. If evidence is weak or inconsistent, retain the baseline at those points and say no change is supported there.

## Produce the processed profile

After analysis, use this skill's rules to decide the final profile values and write one processed JSON bundle for the control station. Use schema `buoy_mixer_candidate_v1`. Preserve the original `analysis` object, including its exact 60-point `baseline_profile`, source firmware, baseline CRC, test metadata, and response summaries.

The final `profile` must contain exactly one row for each of six movements at each 10..100% power step: 60 unique rows total. Every row must have `power`, `axis`, `axis_index`, `rear`, `front_left`, `front_right`, and `set: true`. Preserve measured response fields from the analyzer. Keep all motor values within -100..100. Record each change from the original baseline in `candidate_deltas` using `{ "power": N, "axis": "...", "delta_percent": {"rear": N, "front_left": N, "front_right": N} }`. Save the processed artifact with a `.mixer-candidate.json` suffix. The file must retain the complete baseline even when the proposed profile changes many points.

If the evidence does not support a change, the processed file may retain the baseline values. Do not invent adjustments merely to make the profile look different. State the reasoning and evidence used for every set of changes.

## Upload through the control station

Connect to the buoy, confirm it is disarmed and not in Mission mode, then open **Propulsion → Mixer calibration** and press **Upload processed mixer profile**. Select the skill-produced JSON. The station validates the schema, completed-run metadata, 60 baseline points, and complete 60-point profile; it then fills the adjustment table and sends the profile over the existing control protocol. Firmware saves `/config/propulsion.cfg`. Wait for the buoy's save confirmation and profile CRC in the station. The buoy does not parse JSON; JSON is the station-side interchange file.

## Role of deterministic code and AI

Keep parsing, completeness checks, aggregation, and source JSON generation deterministic so repeated analysis produces reproducible evidence. AI interprets response patterns, decides whether the measurements justify corrections, and writes the processed profile bundle. The station applies only a valid complete bundle while the buoy is disarmed and outside Mission mode.
