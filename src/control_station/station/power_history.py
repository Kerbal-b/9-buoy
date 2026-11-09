"""Time-binned live power summaries for the Power tab."""

from __future__ import annotations

import math
import json
from pathlib import Path


POWER_HISTORY_WINDOWS = {10: 1800, 30: 7200, 300: 86400}


def load_power_histories(path: Path) -> dict[int, list[dict[str, float | int]]]:
    histories: dict[int, list[dict[str, float | int]]] = {10: [], 30: [], 300: []}
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError, TypeError):
        return histories
    if not isinstance(payload, dict) or payload.get("version") != 1:
        return histories
    stored = payload.get("histories")
    if not isinstance(stored, dict):
        return histories
    required = {"t", "v", "vmin", "vmax", "tmin", "tmax", "a", "amin", "amax", "sum_a", "count"}
    for interval in histories:
        rows = stored.get(str(interval), [])
        if not isinstance(rows, list):
            continue
        for row in rows:
            if not isinstance(row, dict) or not required <= row.keys():
                continue
            try:
                normalized = {key: (int(row[key]) if key == "count" else float(row[key])) for key in required}
            except (ValueError, TypeError, OverflowError):
                continue
            if normalized["count"] < 1 or not all(math.isfinite(float(value)) for value in normalized.values()):
                continue
            histories[interval].append(normalized)
        histories[interval].sort(key=lambda point: float(point["t"]))
    return histories


def save_power_histories(path: Path, histories: dict[int, list[dict[str, float | int]]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "version": 1,
        "histories": {str(interval): points for interval, points in histories.items()},
    }
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(json.dumps(payload, separators=(",", ":")) + "\n", encoding="utf-8")
    temporary.replace(path)


def add_power_sample(points: list[dict[str, float | int]], timestamp: float,
                     voltage: float, current: float, bin_seconds: int = 300) -> None:
    if not (math.isfinite(timestamp) and math.isfinite(voltage) and math.isfinite(current)) or voltage <= 0:
        return
    start = int(timestamp // bin_seconds) * bin_seconds
    if points and points[-1]["t"] == start:
        point = points[-1]
        point["v"] = round(voltage, 3)
        if voltage < float(point["vmin"]):
            point["tmin"] = round(timestamp, 3)
        if voltage > float(point["vmax"]):
            point["tmax"] = round(timestamp, 3)
        point["vmin"] = min(float(point["vmin"]), voltage)
        point["vmax"] = max(float(point["vmax"]), voltage)
        point["sum_a"] = float(point["sum_a"]) + current
        point["count"] = int(point["count"]) + 1
        point["a"] = round(float(point["sum_a"]) / int(point["count"]), 3)
        point["amin"] = min(float(point["amin"]), current)
        point["amax"] = max(float(point["amax"]), current)
    else:
        points.append({
            "t": start, "v": round(voltage, 3), "vmin": round(voltage, 3),
            "vmax": round(voltage, 3), "tmin": round(timestamp, 3),
            "tmax": round(timestamp, 3), "a": round(current, 3),
            "amin": round(current, 3), "amax": round(current, 3),
            "sum_a": current, "count": 1,
        })
