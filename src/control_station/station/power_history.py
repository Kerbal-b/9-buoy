"""Time-binned live power summaries for the Power tab."""

from __future__ import annotations

import math


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
