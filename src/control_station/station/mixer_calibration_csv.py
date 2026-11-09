"""Read TX12 buoy calibration CSV files and build portable mixer candidates."""

from __future__ import annotations

import csv
import json
import math
from collections import defaultdict
from pathlib import Path
from statistics import mean, median, pstdev
from math import ceil
from typing import Any


AXES = ("forward", "reverse", "strafe_right", "strafe_left", "yaw_clockwise", "yaw_counterclockwise")
AXIS_INDEX = {name: index for index, name in enumerate(AXES)}
AXIS_INDEX.update({"yaw_cw": 4, "yaw_ccw": 5})
POWER_LEVELS = tuple(range(10, 101, 10))
MOTORS = ("rear", "front_left", "front_right")
PROFILE_FIELDS = ("rear", "front_left", "front_right")
FORMAT = "BUOY_ELRS_CALIBRATION_V1"
SCHEMA = "buoy_mixer_candidate_v1"


def _number(value: str | None) -> float | None:
    if value is None or not value.strip():
        return None
    try:
        result = float(value)
    except ValueError:
        return None
    return result if result == result and abs(result) != float("inf") else None


def _empty_rows() -> list[dict[str, Any]]:
    return [
        {
            "power": power,
            "axis": axis,
            "axis_index": axis_index,
            "rear": 0,
            "front_left": 0,
            "front_right": 0,
            "set": False,
            "samples": 0,
            "accel_x_g": None,
            "accel_y_g": None,
            "yaw_rate_dps": None,
            "gyro_x_dps": None,
            "gyro_y_dps": None,
            "current_a": None,
            "battery_voltage_v": None,
            "heading_deg": None,
            "actual_drive_rear": None,
            "actual_drive_front_left": None,
            "actual_drive_front_right": None,
            "applied_pwm_rear": None,
            "applied_pwm_front_left": None,
            "applied_pwm_front_right": None,
            "gps_speed_mps": None,
            "gps_course_deg": None,
            "mag_x": None,
            "mag_y": None,
            "mag_z": None,
        }
        for axis_index, axis in enumerate(AXES)
        for power in POWER_LEVELS
    ]


def analyze_calibration_csv(path: str | Path) -> dict[str, Any]:
    """Validate a complete mixer-response CSV and bundle its baseline/profile data."""
    source = Path(path)
    metadata: dict[str, str] = {}
    profile: dict[tuple[int, int], dict[str, Any]] = {}
    header: list[str] | None = None
    groups: dict[tuple[int, int], dict[str, list[float]]] = defaultdict(lambda: defaultdict(list))
    raw_rows = 0
    active_rows = 0
    elapsed_samples: list[float] = []
    with source.open("r", encoding="utf-8-sig", newline="") as handle:
        for line in handle:
            if line.startswith("#"):
                key, separator, value = line[1:].strip().partition("=")
                if separator:
                    if key == "profile_point":
                        parts = value.split(",")
                        if len(parts) == 6:
                            try:
                                power, axis = int(parts[0]), AXIS_INDEX[parts[1]]
                                profile[power, axis] = {
                                    "power": power, "axis": AXES[axis], "axis_index": axis,
                                    "rear": int(parts[2]), "front_left": int(parts[3]),
                                    "front_right": int(parts[4]), "set": parts[5] == "1",
                                }
                            except (KeyError, ValueError):
                                pass
                    else:
                        metadata[key] = value
                continue
            if not line.strip():
                continue
            parsed = next(csv.reader([line]))
            if parsed and parsed[0] == "record_type":
                header = parsed
                continue
            if not header:
                continue
            record = dict(zip(header, parsed))
            if record.get("record_type") != "SAMPLE":
                continue
            raw_rows += 1
            elapsed = _number(record.get("elapsed_ms"))
            if elapsed is not None:
                elapsed_samples.append(elapsed)
            if record.get("test_active") != "1":
                continue
            try:
                axis = int(record["test_index"])
                power = int(record["requested_power_pct"])
            except (KeyError, ValueError):
                continue
            if axis not in range(6) or power not in POWER_LEVELS:
                continue
            active_rows += 1
            group = groups[axis, power]
            mapping = {
                "accel_x_g": "accel_x_g", "accel_y_g": "accel_y_g", "yaw_rate_dps": "gyro_z_dps",
                "gyro_x_dps": "gyro_x_dps", "gyro_y_dps": "gyro_y_dps", "current_a": "current_a",
                "battery_voltage_v": "battery_v", "heading_deg": "mag_heading_deg",
                "gps_speed_mps": "speed_mps", "gps_course_deg": "course_deg",
                "mag_x": "mag_x_raw", "mag_y": "mag_y_raw", "mag_z": "mag_z_raw",
                "actual_drive_rear": "drive_rear", "actual_drive_front_left": "drive_front_left",
                "actual_drive_front_right": "drive_front_right", "applied_pwm_rear": "pwm_rear",
                "applied_pwm_front_left": "pwm_front_left", "applied_pwm_front_right": "pwm_front_right",
            }
            for target, column in mapping.items():
                if target == "current_a" and "current_valid" in record and record.get("current_valid") != "1":
                    continue
                if target == "battery_voltage_v" and "battery_valid" in record and record.get("battery_valid") != "1":
                    continue
                if target in {"gps_speed_mps", "gps_course_deg"} and record.get("gps_valid") != "1":
                    continue
                value = _number(record.get(column))
                if value is not None:
                    group[target].append(value)

    if metadata.get("format") != FORMAT:
        raise ValueError(f"Unsupported calibration CSV format: {metadata.get('format', 'missing')}")
    if metadata.get("test_mode") != "mixer_response":
        raise ValueError("This file is not a mixer_response run; motor-load runs cannot generate a mixer profile")
    if metadata.get("completed") != "1":
        raise ValueError("The calibration run is incomplete; complete all requested steps before generating a candidate")
    if len(profile) != 60:
        raise ValueError(f"Expected all 60 baseline profile points in CSV metadata; found {len(profile)}")
    if not active_rows:
        raise ValueError("No active motor test samples were found in the CSV")

    rows = _empty_rows()
    summaries: list[dict[str, Any]] = []
    populated = 0
    for row in rows:
        key = (row["axis_index"], row["power"])
        baseline = profile.get((row["power"], row["axis_index"]))
        if baseline:
            row.update({field: baseline[field] for field in PROFILE_FIELDS})
            row["set"] = bool(baseline["set"])
        samples = groups.get(key, {})
        if samples:
            row["samples"] = len(samples.get("yaw_rate_dps", [])) or max((len(v) for v in samples.values()), default=0)
            for field, values in samples.items():
                if values:
                    row[field] = round(mean(values), 5)
            populated += 1
            summaries.append({
                "axis": row["axis"], "power_percent": row["power"], "sample_count": row["samples"],
                "mean_accel_x_g": row["accel_x_g"], "mean_accel_y_g": row["accel_y_g"],
                "mean_gyro_z_dps": row["yaw_rate_dps"], "mean_current_a": row["current_a"],
                "mean_battery_v": row["battery_voltage_v"],
                "mean_gps_speed_mps": row["gps_speed_mps"],
                "stddev_gyro_z_dps": round(pstdev(samples["yaw_rate_dps"]), 5) if len(samples.get("yaw_rate_dps", [])) > 1 else None,
            })
    if populated < 6:
        raise ValueError(f"Only {populated} movement/power groups contain active data; at least one full six-direction power sweep is required")

    observed_intervals = sorted(
        later - earlier for earlier, later in zip(elapsed_samples, elapsed_samples[1:]) if later > earlier
    )
    interval_p95 = observed_intervals[min(len(observed_intervals) - 1, ceil(len(observed_intervals) * 0.95) - 1)] if observed_intervals else None
    target_interval = int(metadata.get("sample_interval_ms", "0") or 0)

    return {
        "schema": SCHEMA,
        "analysis": {
            "source_file": source.name,
            "source_format": FORMAT,
            "source_firmware_version": metadata.get("firmware_version", ""),
            "started_utc": metadata.get("started_utc", ""),
            "test_mode": metadata.get("test_mode"),
            "completed": True,
            "mixer_profile_crc32": metadata.get("mixer_profile_crc32", ""),
            "mixer_profile_source": metadata.get("mixer_profile_source", ""),
            "max_test_power_percent": int(metadata.get("max_test_power_percent", "0") or 0),
            "test_duration_ms": int(metadata.get("test_duration_ms", "0") or 0),
            "sample_interval_ms": int(metadata.get("sample_interval_ms", "0") or 0),
            "observed_sample_interval_ms": {
                "median": round(median(observed_intervals), 2) if observed_intervals else None,
                "p95": round(interval_p95, 2) if interval_p95 is not None else None,
                "intervals_over_1_5x_target": sum(delta > target_interval * 1.5 for delta in observed_intervals) if target_interval else None,
            },
            "raw_sample_rows": raw_rows,
            "active_sample_rows": active_rows,
            "measured_groups": populated,
            "motor_configuration": {key: value for key, value in metadata.items() if key.startswith("motor_")},
            "baseline_settings": {key: value for key, value in metadata.items() if key not in {"profile_point", "format"}},
            "baseline_profile": [profile[power, axis] for power in POWER_LEVELS for axis in range(6)],
            "response_summary": summaries,
            "candidate_note": "Profile seeded from the exact 60-point baseline recorded by the buoy. Review measured response and edit candidate values before upload. This CSV records IMU/GPS response, not direct thrust; no automatic thrust correction is inferred.",
        },
        "profile": rows,
        "candidate_deltas": [],
    }


def load_candidate_json(path: str | Path) -> dict[str, Any]:
    with Path(path).open("r", encoding="utf-8-sig") as handle:
        data = json.load(handle)
    if not isinstance(data, dict) or data.get("schema") != SCHEMA or not isinstance(data.get("profile"), list):
        raise ValueError(f"Expected a {SCHEMA} configuration JSON")
    if len(data["profile"]) != 60:
        raise ValueError("Configuration JSON must contain exactly 60 profile rows")
    return data


def make_candidate_json(rows_json: str, analysis_json: str = "") -> dict[str, Any]:
    rows = json.loads(rows_json)
    if not isinstance(rows, list) or len(rows) != 60:
        raise ValueError("Candidate profile must contain exactly 60 rows")
    analysis: dict[str, Any] = {}
    if analysis_json:
        try:
            parsed = json.loads(analysis_json)
            if isinstance(parsed, dict):
                analysis = parsed
        except ValueError:
            pass
    baseline = analysis.get("baseline_profile", []) if isinstance(analysis, dict) else []
    deltas: list[dict[str, Any]] = []
    baseline_lookup = {
        (int(row.get("power", -1)), int(row.get("axis_index", -1))): row
        for row in baseline if isinstance(row, dict)
    }
    for row in rows:
        base = baseline_lookup.get((int(row.get("power", -1)), int(row.get("axis_index", -1))))
        if not base:
            continue
        change = {field: int(row[field]) - int(base.get(field, 0)) for field in PROFILE_FIELDS}
        if any(change.values()):
            deltas.append({"power": row["power"], "axis": row.get("axis"), "delta_percent": change})
    return {"schema": SCHEMA, "analysis": analysis, "profile": rows, "candidate_deltas": deltas}


def compare_calibration_trials(paths: list[str | Path]) -> dict[str, Any]:
    """Compare repeated calibration runs grouped by the exact mixer profile CRC."""
    if len(paths) < 2:
        raise ValueError("Select at least two completed calibration CSV files")
    metrics = ("mean_accel_x_g", "mean_accel_y_g", "mean_gyro_z_dps", "mean_current_a", "mean_battery_v", "mean_gps_speed_mps")
    runs: list[dict[str, Any]] = []
    grouped: dict[str, dict[tuple[str, int], dict[str, list[float]]]] = {}
    config_order: list[str] = []
    reference_settings: tuple[str, str, int] | None = None
    for path in paths:
        bundle = analyze_calibration_csv(path)
        analysis = bundle["analysis"]
        settings_signature = json.dumps(analysis.get("motor_configuration", {}), sort_keys=True)
        run_settings = (
            str(analysis.get("source_firmware_version", "")), settings_signature,
            int(analysis.get("test_duration_ms", 0)),
        )
        if reference_settings is None:
            reference_settings = run_settings
        elif run_settings != reference_settings:
            raise ValueError("Trial files use different firmware, motor settings, or pulse durations; compare those separately")
        crc = str(analysis.get("mixer_profile_crc32") or "NONE").upper()
        if crc not in grouped:
            grouped[crc] = defaultdict(lambda: defaultdict(list))
            config_order.append(crc)
        run_record = {
            "source_file": analysis["source_file"],
            "started_utc": analysis["started_utc"],
            "firmware_version": analysis["source_firmware_version"],
            "mixer_profile_crc32": crc,
        }
        runs.append(run_record)
        for point in analysis["response_summary"]:
            key = (str(point["axis"]), int(point["power_percent"]))
            values = grouped[crc][key]
            for metric in metrics:
                value = point.get(metric)
                if isinstance(value, (int, float)) and math.isfinite(value):
                    values[metric].append(float(value))

    configurations: list[dict[str, Any]] = []
    for crc in config_order:
        points = []
        for (axis, power), values in sorted(grouped[crc].items(), key=lambda item: (item[0][1], item[0][0])):
            measured = {name: {
                "mean_across_runs": round(mean(series), 6) if series else None,
                "run_to_run_stddev": round(pstdev(series), 6) if len(series) > 1 else None,
            } for name, series in values.items()}
            points.append({"axis": axis, "power_percent": power, "repeat_count": max((len(v) for v in values.values()), default=0), "metrics": measured})
        configurations.append({
            "mixer_profile_crc32": crc,
            "trial_count": sum(run["mixer_profile_crc32"] == crc for run in runs),
            "points": points,
        })

    comparisons: list[dict[str, Any]] = []
    for before_index, before in enumerate(configurations):
        for after in configurations[before_index + 1:]:
            before_points = {(p["axis"], p["power_percent"]): p for p in before["points"]}
            after_points = {(p["axis"], p["power_percent"]): p for p in after["points"]}
            changes = []
            for key in sorted(before_points.keys() & after_points.keys(), key=lambda item: (item[1], item[0])):
                deltas = {}
                for metric in metrics:
                    old = before_points[key]["metrics"].get(metric, {}).get("mean_across_runs")
                    new = after_points[key]["metrics"].get(metric, {}).get("mean_across_runs")
                    deltas[metric] = round(new - old, 6) if old is not None and new is not None else None
                changes.append({"axis": key[0], "power_percent": key[1], "delta_after_minus_before": deltas})
            comparisons.append({
                "before_crc32": before["mixer_profile_crc32"],
                "after_crc32": after["mixer_profile_crc32"],
                "paired_points": len(changes),
                "changes": changes,
            })
    return {
        "schema": "buoy_mixer_trial_comparison_v1",
        "method": "Independent completed runs are grouped by the logged mixer profile CRC. Means and repeat-to-repeat population standard deviations are computed from per-run point means.",
        "runs": runs,
        "configurations": configurations,
        "comparisons": comparisons,
        "interpretation_note": "Use at least three repeated runs per configuration and compare the change with the before/after run-to-run spread. Same CRC means the same mixer table; firmware and motor settings should also match. GPS speed is ground-relative and may be missing or unsuitable for short pulses.",
    }
