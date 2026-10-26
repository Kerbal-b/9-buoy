"""Local mission plans with derived, boundary-constrained navigation routes."""

from __future__ import annotations

import copy
import heapq
import json
import math
import os
from pathlib import Path
import shutil
import tempfile
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from urllib.request import Request, urlopen
from uuid import uuid4


def new_plan() -> dict:
    return {
        "schema_version": 1,
        "id": str(uuid4()),
        "name": "Untitled mission",
        "home": None,
        "end": None,
        "boundary": [],
        "waypoints": [],
        "planned_route": [],
        "return_route": [],
        "route_generation_error": "",
        "movement_mode": "manual",
        "auto_spacing_m": 25,
        "auto_speed_percent": 50,
        "auto_clearance_m": 15,
        "arrival_radius_m": 5,
        "auto_collection_seconds": 30,
        "audio_capture": "checkpoint_segments",
        "calibration": {
            "speed_mps": {"25": None, "50": None, "75": None, "100": None},
            "current_a": {"25": None, "50": None, "75": None, "100": None},
            "idle_current_a": None,
            "battery_capacity_ah": None,
        },
        "failsafes": {
            "link_loss": "return_home",
            "boundary_breach": "return_home",
            "low_battery": "return_home",
            "battery_threshold_percent": 20,
        },
    }


def mission_estimate(plan: dict) -> dict:
    """Estimate planned survey-route length, time and charge from user calibration."""
    route = []
    if valid_point(plan.get("home")):
        route.append(plan["home"])
    route.extend(point for point in plan.get("planned_route", plan.get("waypoints", [])) if valid_point(point))
    distance = 0.0
    legs = []
    for a, b in zip(route, route[1:]):
        lat1, lat2 = math.radians(a["lat"]), math.radians(b["lat"])
        dlat, dlon = lat2 - lat1, math.radians(b["lon"] - a["lon"])
        hav = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
        leg = 6371000.0 * 2 * math.asin(min(1.0, math.sqrt(hav)))
        distance += leg
        legs.append((leg, b))

    calibration = plan.get("calibration", {})
    speeds = calibration.get("speed_mps", {}) if isinstance(calibration, dict) else {}
    currents = calibration.get("current_a", {}) if isinstance(calibration, dict) else {}

    def calibrated(profile: dict, percent: int) -> float | None:
        points = sorted((int(k), float(v)) for k, v in profile.items()
                        if k.isdigit() and isinstance(v, (int, float)) and math.isfinite(v) and v > 0)
        if not points:
            return None
        if percent <= points[0][0]:
            return points[0][1]
        if percent >= points[-1][0]:
            return points[-1][1]
        for (lo, lv), (hi, hv) in zip(points, points[1:]):
            if lo <= percent <= hi:
                return lv + (hv - lv) * (percent - lo) / (hi - lo)
        return None

    travel_seconds = 0.0
    travel_ah = 0.0
    time_ready = bool(legs)
    battery_ready = bool(legs)
    for leg, waypoint in legs:
        speed = calibrated(speeds, int(waypoint.get("speed_percent", 50)))
        current = calibrated(currents, int(waypoint.get("speed_percent", 50)))
        if speed is None:
            time_ready = False
            battery_ready = False
        else:
            seconds = leg / speed
            travel_seconds += seconds
            if current is None:
                battery_ready = False
            else:
                travel_ah += current * seconds / 3600
        if current is None:
            battery_ready = False
    holds_seconds = sum(max(0, int(p.get("hold_seconds", 0))) for p in plan.get("waypoints", [])
                        if isinstance(p.get("hold_seconds", 0), (int, float)))
    total_seconds = travel_seconds + holds_seconds if time_ready else None
    capacity = calibration.get("battery_capacity_ah") if isinstance(calibration, dict) else None
    idle_current = calibration.get("idle_current_a") if isinstance(calibration, dict) else None
    if (capacity is None or not isinstance(capacity, (int, float)) or capacity <= 0
            or idle_current is None or not isinstance(idle_current, (int, float)) or idle_current < 0):
        battery_ready = False
    charge_ah = travel_ah if battery_ready else None
    if battery_ready and charge_ah is not None:
        charge_ah += idle_current * holds_seconds / 3600
        battery_percent = charge_ah / capacity * 100
    else:
        battery_percent = None
    return {"distance_m": distance, "travel_seconds": travel_seconds if time_ready else None,
            "hold_seconds": holds_seconds, "total_seconds": total_seconds,
            "battery_ah": charge_ah, "battery_percent": battery_percent}


def valid_point(point: object) -> bool:
    if not isinstance(point, dict):
        return False
    if not isinstance(point.get("lat"), (int, float)) or not isinstance(point.get("lon"), (int, float)):
        return False
    try:
        lat, lon = float(point["lat"]), float(point["lon"])
    except (KeyError, TypeError, ValueError):
        return False
    return math.isfinite(lat) and math.isfinite(lon) and -90 <= lat <= 90 and -180 <= lon <= 180


def mission_end(plan: dict) -> dict | None:
    """Return the explicit end, or Home when no end was placed."""
    return plan.get("end") if plan.get("end") is not None else plan.get("home")


def _segments_intersect(a: dict, b: dict, c: dict, d: dict) -> bool:
    # Longitude is x and latitude is y. Local plans should not cross the date line.
    def orient(p: dict, q: dict, r: dict) -> float:
        return (q["lon"] - p["lon"]) * (r["lat"] - p["lat"]) - (q["lat"] - p["lat"]) * (r["lon"] - p["lon"])

    def on_segment(p: dict, q: dict, r: dict) -> bool:
        return min(p["lon"], r["lon"]) <= q["lon"] <= max(p["lon"], r["lon"]) and min(p["lat"], r["lat"]) <= q["lat"] <= max(p["lat"], r["lat"])

    o1, o2, o3, o4 = orient(a, b, c), orient(a, b, d), orient(c, d, a), orient(c, d, b)
    eps = 1e-12
    if ((o1 > eps and o2 < -eps) or (o1 < -eps and o2 > eps)) and ((o3 > eps and o4 < -eps) or (o3 < -eps and o4 > eps)):
        return True
    return (abs(o1) <= eps and on_segment(a, c, b)) or (abs(o2) <= eps and on_segment(a, d, b)) or (abs(o3) <= eps and on_segment(c, a, d)) or (abs(o4) <= eps and on_segment(c, b, d))


def boundary_point_near(points: list[dict], point: dict, *, skip_index: int = -1,
                        minimum_m: float = 2.0) -> bool:
    """Keep separate boundary vertices from landing on top of one another."""
    for index, existing in enumerate(points):
        if index == skip_index:
            continue
        mean_lat = math.radians((existing["lat"] + point["lat"]) / 2)
        distance_m = 111320.0 * math.hypot(
            (existing["lon"] - point["lon"]) * math.cos(mean_lat),
            existing["lat"] - point["lat"],
        )
        if distance_m < minimum_m:
            return True
    return False


def untangle_boundary(points: list[dict]) -> list[dict]:
    """Remove crossing polygon edges with local 2-opt reversals; keep all vertices."""
    def crosses(a: dict, b: dict, c: dict, d: dict) -> bool:
        def side(p: dict, q: dict, r: dict) -> float:
            return (q["lon"] - p["lon"]) * (r["lat"] - p["lat"]) - (q["lat"] - p["lat"]) * (r["lon"] - p["lon"])

        ab_c, ab_d = side(a, b, c), side(a, b, d)
        cd_a, cd_b = side(c, d, a), side(c, d, b)
        return ab_c * ab_d < 0 and cd_a * cd_b < 0

    ordered = list(points)
    if len(ordered) < 4:
        return ordered
    for _ in range(len(ordered) * len(ordered)):
        crossing = None
        for i in range(len(ordered)):
            for j in range(i + 2, len(ordered)):
                if i == 0 and j == len(ordered) - 1:
                    continue  # These edges share the first vertex.
                if crosses(ordered[i], ordered[(i + 1) % len(ordered)],
                           ordered[j], ordered[(j + 1) % len(ordered)]):
                    crossing = (i, j)
                    break
            if crossing is not None:
                break
        if crossing is None:
            break
        i, j = crossing
        ordered[i + 1:j + 1] = reversed(ordered[i + 1:j + 1])
    return ordered


def _inside(point: dict, polygon: list[dict]) -> bool:
    x, y = point["lon"], point["lat"]
    inside = False
    for i, a in enumerate(polygon):
        b = polygon[(i + 1) % len(polygon)]
        dx, dy = b["lon"] - a["lon"], b["lat"] - a["lat"]
        tolerance = 2e-8  # About 2 mm of coordinate rounding near the equator.
        if math.hypot(dx, dy) and abs(dx * (y - a["lat"]) - dy * (x - a["lon"])) <= tolerance * math.hypot(dx, dy) and min(a["lon"], b["lon"]) - tolerance <= x <= max(a["lon"], b["lon"]) + tolerance and min(a["lat"], b["lat"]) - tolerance <= y <= max(a["lat"], b["lat"]) + tolerance:
            return True
        if (a["lat"] > y) != (b["lat"] > y):
            crossing = a["lon"] + (y - a["lat"]) * (b["lon"] - a["lon"]) / (b["lat"] - a["lat"])
            if x < crossing:
                inside = not inside
    return inside


def point_inside_boundary(point: dict, boundary: list[dict]) -> bool:
    """Return whether a valid coordinate is on or inside a mission boundary."""
    return valid_point(point) and len(boundary) >= 3 and all(valid_point(p) for p in boundary) and _inside(point, boundary)


def validate_plan(plan: dict) -> list[str]:
    issues: list[str] = []
    boundary = plan.get("boundary", [])
    waypoints = plan.get("waypoints", [])
    home = plan.get("home")
    if not valid_point(home):
        issues.append("Set a valid home position.")
    elif len(boundary) >= 3 and all(valid_point(point) for point in boundary) and not point_inside_boundary(home, boundary):
        issues.append("Home must be inside the mission boundary.")
    if plan.get("end") is not None and not valid_point(plan["end"]):
        issues.append("Set a valid end position or remove it to finish at Home.")
    elif plan.get("end") is not None and len(boundary) >= 3 and all(valid_point(point) for point in boundary) and not point_inside_boundary(plan["end"], boundary):
        issues.append("End must be inside the mission boundary.")
    if len(boundary) < 3:
        issues.append("Place at least three boundary points.")
    elif not all(valid_point(p) for p in boundary):
        issues.append("Boundary contains an invalid position.")
    else:
        if max(p["lon"] for p in boundary) - min(p["lon"] for p in boundary) > 180:
            issues.append("A boundary cannot cross the date line.")
        if len({(p["lat"], p["lon"]) for p in boundary}) != len(boundary):
            issues.append("Boundary points must be distinct.")
        area = sum(a["lon"] * b["lat"] - b["lon"] * a["lat"] for a, b in zip(boundary, boundary[1:] + boundary[:1]))
        if abs(area) < 1e-14:
            issues.append("Boundary must enclose an area.")
        for i in range(len(boundary)):
            for j in range(i + 2, len(boundary)):
                if i == 0 and j == len(boundary) - 1:
                    continue
                if _segments_intersect(boundary[i], boundary[(i + 1) % len(boundary)], boundary[j], boundary[(j + 1) % len(boundary)]):
                    issues.append("Boundary crosses itself.")
                    break
            if "Boundary crosses itself." in issues:
                break
    if not waypoints:
        issues.append("Place at least one movement point.")
    if plan.get("movement_mode", "manual") not in ("manual", "automatic"):
        issues.append("Choose Manual or Automatic movement.")
    spacing = plan.get("auto_spacing_m", 25)
    if not isinstance(spacing, int) or not 5 <= spacing <= 500:
        issues.append("Automatic pass spacing must be 5–500 m.")
    auto_speed = plan.get("auto_speed_percent", 50)
    if not isinstance(auto_speed, int) or not 1 <= auto_speed <= 100:
        issues.append("Automatic route speed must be 1–100%.")
    clearance = plan.get("auto_clearance_m", 15)
    if not isinstance(clearance, int) or not 5 <= clearance <= 100:
        issues.append("Boundary clearance must be 5–100 m so arrival circles fit inside it.")
    if plan.get("arrival_radius_m", 5) != 5:
        issues.append("Checkpoint arrival radius must be 5 m.")
    collection_seconds = plan.get("auto_collection_seconds", 30)
    if not isinstance(collection_seconds, int) or not 1 <= collection_seconds <= 3600:
        issues.append("Automatic collection duration must be 1–3600 seconds.")
    if plan.get("audio_capture", "checkpoint_segments") != "checkpoint_segments":
        issues.append("Audio must be recorded as checkpoint segments.")
    for i, waypoint in enumerate(waypoints, 1):
        if not valid_point(waypoint):
            issues.append(f"Movement point {i} has an invalid position.")
            continue
        if not isinstance(waypoint.get("speed_percent"), int) or not 1 <= waypoint["speed_percent"] <= 100:
            issues.append(f"Movement point {i} needs a speed from 1 to 100%.")
        if not isinstance(waypoint.get("hold_seconds"), int) or not 0 <= waypoint["hold_seconds"] <= 3600:
            issues.append(f"Movement point {i} needs a hold from 0 to 3600 seconds.")
        if "collect_data" in waypoint and not isinstance(waypoint["collect_data"], bool):
            issues.append(f"Movement point {i} has an invalid collection setting.")
        if waypoint.get("collect_data") is True and waypoint.get("hold_seconds") == 0:
            issues.append(f"Collection checkpoint {i} needs a duration greater than zero.")
        if waypoint.get("collect_data") is False and isinstance(waypoint.get("hold_seconds"), int) and waypoint["hold_seconds"] > 0:
            issues.append(f"Transit point {i} cannot have a collection duration.")
        measurements = waypoint.get("measurements")
        allowed_measurements = {"water_temperature", "depth", "conductivity", "turbidity", "air_temperature"}
        if measurements is not None and (not isinstance(measurements, list) or any(item not in allowed_measurements for item in measurements)):
            issues.append(f"Movement point {i} has an invalid measurement selection.")
    if len(boundary) >= 3 and all(valid_point(p) for p in boundary):
        for i, waypoint in enumerate(waypoints, 1):
            if valid_point(waypoint) and not _inside(waypoint, boundary):
                issues.append(f"Movement point {i} is outside the boundary.")
        if not any("boundary" in issue.lower() for issue in issues):
            mean_lat = sum(point["lat"] for point in boundary) / len(boundary)
            mean_lon = sum(point["lon"] for point in boundary) / len(boundary)
            x_scale = 111320.0 * max(0.01, math.cos(math.radians(mean_lat)))
            polygon_m = [((p["lon"] - mean_lon) * x_scale, (p["lat"] - mean_lat) * 111320.0) for p in boundary]
            for i, waypoint in enumerate(waypoints, 1):
                if (valid_point(waypoint) and (waypoint.get("collect_data") is True or
                        ("collect_data" not in waypoint and isinstance(waypoint.get("hold_seconds"), int)
                         and waypoint["hold_seconds"] > 0))):
                    xy = ((waypoint["lon"] - mean_lon) * x_scale,
                          (waypoint["lat"] - mean_lat) * 111320.0)
                    if not _safe_point_m(xy, polygon_m, 5):
                        issues.append(f"Collection checkpoint {i} needs a full 5 m arrival circle inside the boundary.")
        if (plan.get("movement_mode") == "automatic" and waypoints
                and all(valid_point(p) for p in waypoints)
                and isinstance(clearance, int) and 5 <= clearance <= 100
                and not any("boundary" in issue.lower() for issue in issues)):
            route_m = [((p["lon"] - mean_lon) * x_scale, (p["lat"] - mean_lat) * 111320.0) for p in waypoints]
            if (route_m and not _safe_point_m(route_m[0], polygon_m, clearance)) or any(
                    not _safe_segment_m(a, b, polygon_m, clearance) for a, b in zip(route_m, route_m[1:])):
                issues.append("Automatic route violates boundary clearance.")
    failsafes = plan.get("failsafes", {})
    for key in ("link_loss", "boundary_breach", "low_battery"):
        if failsafes.get(key) not in ("stop", "return_home"):
            issues.append(f"Choose a valid {key.replace('_', ' ')} action.")
    threshold = failsafes.get("battery_threshold_percent")
    if not isinstance(threshold, int) or not 5 <= threshold <= 80:
        issues.append("Low battery threshold must be 5–80%.")
    route_error = plan.get("route_generation_error")
    if isinstance(route_error, str) and route_error and route_error not in issues:
        issues.append(route_error)
    planned_route = plan.get("planned_route", [])
    if isinstance(planned_route, list) and len(planned_route) > 64:
        issues.append("The safe planned route exceeds the buoy's 64-point limit; simplify or space out the mission.")
    return issues


def _inside_m(point: tuple[float, float], polygon: list[tuple[float, float]]) -> bool:
    x, y = point
    inside = False
    for i, a in enumerate(polygon):
        b = polygon[(i + 1) % len(polygon)]
        dx, dy = b[0] - a[0], b[1] - a[1]
        length = math.hypot(dx, dy)
        if length and abs(dx * (y - a[1]) - dy * (x - a[0])) <= 0.0001 * length:
            if min(a[0], b[0]) - 0.0001 <= x <= max(a[0], b[0]) + 0.0001 and min(a[1], b[1]) - 0.0001 <= y <= max(a[1], b[1]) + 0.0001:
                return True
        if (a[1] > y) != (b[1] > y):
            crossing = a[0] + (y - a[1]) * dx / dy
            if x < crossing:
                inside = not inside
    return inside


def _segment_inside_m(a: tuple[float, float], b: tuple[float, float], polygon: list[tuple[float, float]]) -> bool:
    if not _inside_m(a, polygon) or not _inside_m(b, polygon):
        return False
    rx, ry = b[0] - a[0], b[1] - a[1]
    length_sq = rx * rx + ry * ry
    if length_sq < 1e-8:
        return True
    cuts = [0.0, 1.0]
    for i, c in enumerate(polygon):
        d = polygon[(i + 1) % len(polygon)]
        sx, sy = d[0] - c[0], d[1] - c[1]
        qx, qy = c[0] - a[0], c[1] - a[1]
        cross = rx * sy - ry * sx
        if abs(cross) < 1e-9:
            if abs(qx * ry - qy * rx) < 1e-6:
                cuts.extend(((qx * rx + qy * ry) / length_sq,
                             ((d[0] - a[0]) * rx + (d[1] - a[1]) * ry) / length_sq))
            continue
        t = (qx * sy - qy * sx) / cross
        u = (qx * ry - qy * rx) / cross
        if -1e-9 <= t <= 1 + 1e-9 and -1e-9 <= u <= 1 + 1e-9:
            cuts.append(max(0.0, min(1.0, t)))
    cuts = sorted(set(cut for cut in cuts if 0 <= cut <= 1))
    return all(
        _inside_m((a[0] + rx * (left + right) / 2, a[1] + ry * (left + right) / 2), polygon)
        for left, right in zip(cuts, cuts[1:]) if right - left > 1e-8
    )


def _point_segment_distance_m(point: tuple[float, float], a: tuple[float, float], b: tuple[float, float]) -> float:
    dx, dy = b[0] - a[0], b[1] - a[1]
    length_sq = dx * dx + dy * dy
    if length_sq == 0:
        return math.dist(point, a)
    fraction = max(0.0, min(1.0, ((point[0] - a[0]) * dx + (point[1] - a[1]) * dy) / length_sq))
    return math.dist(point, (a[0] + fraction * dx, a[1] + fraction * dy))


def _segments_cross_m(a: tuple[float, float], b: tuple[float, float], c: tuple[float, float], d: tuple[float, float]) -> bool:
    def side(p: tuple[float, float], q: tuple[float, float], r: tuple[float, float]) -> float:
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])

    o1, o2, o3, o4 = side(a, b, c), side(a, b, d), side(c, d, a), side(c, d, b)
    if o1 * o2 < 0 and o3 * o4 < 0:
        return True
    return (
        (abs(o1) < 1e-8 and _point_segment_distance_m(c, a, b) < 1e-4)
        or (abs(o2) < 1e-8 and _point_segment_distance_m(d, a, b) < 1e-4)
        or (abs(o3) < 1e-8 and _point_segment_distance_m(a, c, d) < 1e-4)
        or (abs(o4) < 1e-8 and _point_segment_distance_m(b, c, d) < 1e-4)
    )


def _segment_boundary_distance_m(a: tuple[float, float], b: tuple[float, float], polygon: list[tuple[float, float]]) -> float:
    closest = math.inf
    for i, c in enumerate(polygon):
        d = polygon[(i + 1) % len(polygon)]
        if _segments_cross_m(a, b, c, d):
            return 0.0
        closest = min(closest,
                      _point_segment_distance_m(a, c, d), _point_segment_distance_m(b, c, d),
                      _point_segment_distance_m(c, a, b), _point_segment_distance_m(d, a, b))
    return closest


def _safe_point_m(point: tuple[float, float], polygon: list[tuple[float, float]], clearance_m: int) -> bool:
    if not _inside_m(point, polygon):
        return False
    return all(
        _point_segment_distance_m(point, edge, polygon[(i + 1) % len(polygon)]) >= clearance_m - 0.02
        for i, edge in enumerate(polygon)
    )


def _safe_segment_m(a: tuple[float, float], b: tuple[float, float], polygon: list[tuple[float, float]], clearance_m: int) -> bool:
    return _segment_inside_m(a, b, polygon) and _segment_boundary_distance_m(a, b, polygon) >= clearance_m - 0.02


def _inset_navigation_nodes(polygon: list[tuple[float, float]], clearance_m: int) -> list[tuple[float, float]]:
    area = sum(a[0] * b[1] - b[0] * a[1] for a, b in zip(polygon, polygon[1:] + polygon[:1]))
    orientation = 1 if area > 0 else -1
    normals = []
    for i, a in enumerate(polygon):
        b = polygon[(i + 1) % len(polygon)]
        dx, dy = b[0] - a[0], b[1] - a[1]
        length = math.hypot(dx, dy)
        normals.append((-orientation * dy / length, orientation * dx / length))
    nodes = []
    inset = clearance_m + 0.25
    for i, vertex in enumerate(polygon):
        previous, following = normals[i - 1], normals[i]
        divisor = 1 + previous[0] * following[0] + previous[1] * following[1]
        if divisor > 0.05:
            candidate = (vertex[0] + inset * (previous[0] + following[0]) / divisor,
                         vertex[1] + inset * (previous[1] + following[1]) / divisor)
            if _safe_point_m(candidate, polygon, clearance_m):
                nodes.append(candidate)
        next_vertex = polygon[(i + 1) % len(polygon)]
        midpoint = ((vertex[0] + next_vertex[0]) / 2 + inset * following[0],
                    (vertex[1] + next_vertex[1]) / 2 + inset * following[1])
        if _safe_point_m(midpoint, polygon, clearance_m):
            nodes.append(midpoint)
    return nodes


def _visibility_graph(nodes: list[tuple[float, float]], polygon: list[tuple[float, float]], clearance_m: int) -> list[list[tuple[float, int]]]:
    graph: list[list[tuple[float, int]]] = [[] for _ in nodes]
    for i in range(len(nodes)):
        for j in range(i + 1, len(nodes)):
            if _safe_segment_m(nodes[i], nodes[j], polygon, clearance_m):
                distance = math.dist(nodes[i], nodes[j])
                graph[i].append((distance, j))
                graph[j].append((distance, i))
    return graph


def _inside_connector(
    a: tuple[float, float], b: tuple[float, float], polygon: list[tuple[float, float]],
    clearance_m: int, navigation_nodes: list[tuple[float, float]],
    vertex_graph: list[list[tuple[float, int]]] | None = None,
) -> list[tuple[float, float]]:
    """Shortest route whose segments keep the selected boundary clearance."""
    if _safe_segment_m(a, b, polygon, clearance_m):
        return [a, b]
    if vertex_graph is None:
        vertex_graph = _visibility_graph(navigation_nodes, polygon, clearance_m)
    nodes = [a, b, *navigation_nodes]
    graph: list[list[tuple[float, int]]] = [[], []]
    graph.extend([[(distance, neighbor + 2) for distance, neighbor in edges] for edges in vertex_graph])
    for index, vertex in enumerate(navigation_nodes, start=2):
        for endpoint in (0, 1):
            if _safe_segment_m(nodes[endpoint], vertex, polygon, clearance_m):
                distance = math.dist(nodes[endpoint], vertex)
                graph[endpoint].append((distance, index))
                graph[index].append((distance, endpoint))
    best = [math.inf] * len(nodes)
    previous = [-1] * len(nodes)
    best[0] = 0.0
    pending = [(0.0, 0)]
    while pending:
        distance, node = heapq.heappop(pending)
        if node == 1:
            break
        if distance > best[node]:
            continue
        for edge_distance, neighbor in graph[node]:
            candidate = distance + edge_distance
            if candidate < best[neighbor]:
                best[neighbor] = candidate
                previous[neighbor] = node
                heapq.heappush(pending, (candidate, neighbor))
    if not math.isfinite(best[1]):
        raise ValueError("Could not connect passes with the selected boundary clearance")
    path = []
    node = 1
    while node != -1:
        path.append(nodes[node])
        node = previous[node]
    return list(reversed(path))


def rebuild_mission_routes(plan: dict) -> str:
    """Build an in-boundary route through all mission points and back to End/Home."""
    plan["planned_route"] = []
    plan["return_route"] = []
    plan["route_generation_error"] = ""
    boundary = plan.get("boundary", [])
    tasks = plan.get("waypoints", [])
    home = plan.get("home")
    endpoint = mission_end(plan)
    if len(boundary) < 3 or not all(valid_point(point) for point in boundary):
        return ""
    if not valid_point(home) or not valid_point(endpoint) or not tasks or not all(valid_point(point) for point in tasks):
        return ""
    if any(abs(point["lat"]) > 85 for point in boundary):
        error = "Safe route planning is limited to latitudes within 85 degrees."
        plan["route_generation_error"] = error
        return error

    clearance = max(5, int(plan.get("auto_clearance_m", 15))) if plan.get("movement_mode") == "automatic" else 5
    mean_lat = sum(point["lat"] for point in boundary) / len(boundary)
    mean_lon = sum(point["lon"] for point in boundary) / len(boundary)
    x_scale = 111320.0 * max(0.01, math.cos(math.radians(mean_lat)))
    y_scale = 111320.0
    polygon = [((p["lon"] - mean_lon) * x_scale, (p["lat"] - mean_lat) * y_scale) for p in boundary]
    if (len(set(polygon)) != len(polygon)
            or any(math.dist(polygon[i], polygon[(i + 1) % len(polygon)]) < 0.01 for i in range(len(polygon)))
            or abs(sum(a[0] * b[1] - b[0] * a[1] for a, b in zip(polygon, polygon[1:] + polygon[:1]))) < 0.01):
        return ""

    def xy(point: dict) -> tuple[float, float]:
        return ((point["lon"] - mean_lon) * x_scale, (point["lat"] - mean_lat) * y_scale)

    def coordinate(point: tuple[float, float]) -> dict:
        return {"lat": round(mean_lat + point[1] / y_scale, 9),
                "lon": round(mean_lon + point[0] / x_scale, 9)}

    destinations = [(point, xy(point)) for point in [home, *tasks, endpoint]]
    for index, (point, local) in enumerate(destinations):
        if not _safe_point_m(local, polygon, clearance):
            label = "Home" if index == 0 else "End" if index == len(destinations) - 1 else "A mission point"
            error = f"{label} must be at least {clearance} m inside the boundary for safe routing."
            plan["route_generation_error"] = error
            return error

    try:
        navigation_nodes = _inset_navigation_nodes(polygon, clearance)
        vertex_graph = _visibility_graph(navigation_nodes, polygon, clearance)
    except (ValueError, ZeroDivisionError):
        error = "Could not build a safe navigation path for this boundary."
        plan["route_generation_error"] = error
        return error
    planned_route: list[dict] = []
    return_route: list[dict] = []
    transit_speed = max(1, min(100, int(plan.get("auto_speed_percent", 50))))

    def add_leg(start: tuple[float, float], finish: tuple[float, float],
                destination: dict, output: list[dict], *, returning: bool = False) -> None:
        path = _inside_connector(start, finish, polygon, clearance, navigation_nodes, vertex_graph)
        for node in path[1:-1]:
            output.append({**coordinate(node), "speed_percent": transit_speed, "hold_seconds": 0,
                           "collect_data": False, "measurements": [],
                           "route_role": "return_transit" if returning else "boundary_transit"})
        point = destination
        if returning:
            point = {**point, "speed_percent": transit_speed, "hold_seconds": 0,
                     "collect_data": False, "measurements": [], "route_role": "mission_end"}
        output.append(dict(point))

    try:
        cursor = destinations[0][1]
        for task, target in destinations[1:-1]:
            add_leg(cursor, target, task, planned_route)
            cursor = target

        final_target = destinations[-1][1]
        if math.dist(cursor, final_target) > 0.1:
            add_leg(cursor, final_target, endpoint, return_route, returning=True)
            planned_route.extend(dict(point) for point in return_route)
    except ValueError as exc:
        error = f"Could not create a safe route inside the boundary: {exc}"
        plan["route_generation_error"] = error
        return error

    plan["planned_route"] = planned_route
    plan["return_route"] = return_route
    return ""


def generate_automatic_waypoints(
    boundary: list[dict], home: dict | None, spacing_m: int, speed_percent: int = 50,
    clearance_m: int = 15, max_points: int = 300, collection_seconds: int = 30,
) -> list[dict]:
    """Generate collection checkpoints and safe transit connectors inside the boundary."""
    if not isinstance(spacing_m, int) or not 5 <= spacing_m <= 500:
        raise ValueError("Automatic pass spacing must be 5–500 m")
    if not isinstance(speed_percent, int) or not 1 <= speed_percent <= 100:
        raise ValueError("Automatic route speed must be 1–100%")
    if not isinstance(clearance_m, int) or not 5 <= clearance_m <= 100:
        raise ValueError("Boundary clearance must be 5–100 m so arrival circles fit inside it")
    if not isinstance(collection_seconds, int) or not 1 <= collection_seconds <= 3600:
        raise ValueError("Collection duration must be 1–3600 seconds")
    if len(boundary) < 3:
        raise ValueError("Place at least three boundary points")
    if len(boundary) > 80:
        raise ValueError("Automatic planning supports at most 80 boundary points")
    probe = new_plan()
    probe.update(boundary=boundary, home=boundary[0], waypoints=[{**boundary[0], "speed_percent": 50, "hold_seconds": 0}])
    boundary_issues = [issue for issue in validate_plan(probe) if "boundary" in issue.lower()]
    if boundary_issues:
        raise ValueError(boundary_issues[0])
    if any(abs(point["lat"]) > 85 for point in boundary):
        raise ValueError("Automatic planning is limited to latitudes within 85 degrees")
    mean_lat = sum(point["lat"] for point in boundary) / len(boundary)
    mean_lon = sum(point["lon"] for point in boundary) / len(boundary)
    x_scale = 111320.0 * math.cos(math.radians(mean_lat))
    y_scale = 111320.0
    polygon = [((point["lon"] - mean_lon) * x_scale, (point["lat"] - mean_lat) * y_scale) for point in boundary]
    south = min(point[1] for point in polygon)
    north = max(point[1] for point in polygon)
    rows = max(1, math.ceil((north - south) / spacing_m))
    if rows * 2 > max_points:
        raise ValueError("Too many passes; increase automatic spacing")
    home_xy = (((home["lon"] - mean_lon) * x_scale, (home["lat"] - mean_lat) * y_scale)
               if valid_point(home) else None)
    targets: list[tuple[float, float]] = []
    sample_step = max(1.0, min(5.0, spacing_m / 4, clearance_m / 2))
    total_samples = 0
    for row in range(rows):
        y = south + (row + 0.5) * (north - south) / rows
        intersections = []
        for i, a in enumerate(polygon):
            b = polygon[(i + 1) % len(polygon)]
            if (a[1] <= y < b[1]) or (b[1] <= y < a[1]):
                intersections.append(a[0] + (y - a[1]) * (b[0] - a[0]) / (b[1] - a[1]))
        intersections.sort()
        segments = []
        for left, right in zip(intersections[::2], intersections[1::2]):
            inner_left, inner_right = left + clearance_m + 0.25, right - clearance_m - 0.25
            if inner_right <= inner_left:
                continue
            count = max(2, math.ceil((inner_right - inner_left) / sample_step) + 1)
            total_samples += count
            if total_samples > 25000:
                raise ValueError("Area is too large for this spacing; increase automatic spacing")
            safe_run: list[tuple[float, float]] = []

            def finish_run() -> None:
                if not safe_run:
                    return
                start = safe_run[0]
                previous = start
                for point in safe_run[1:]:
                    if not _safe_segment_m(start, point, polygon, clearance_m):
                        segments.append((start, previous))
                        start = point
                    previous = point
                segments.append((start, previous))
                safe_run.clear()

            for index in range(count):
                point = (inner_left + (inner_right - inner_left) * index / (count - 1), y)
                if _safe_point_m(point, polygon, clearance_m):
                    safe_run.append(point)
                else:
                    finish_run()
            finish_run()
        if row % 2:
            segments = [(right, left) for left, right in reversed(segments)]
        for start, end in segments:
            steps = max(1, math.ceil(math.dist(start, end) / spacing_m))
            for step in range(steps + 1):
                fraction = step / steps
                point = (start[0] + (end[0] - start[0]) * fraction,
                         start[1] + (end[1] - start[1]) * fraction)
                if not targets or math.dist(targets[-1], point) > 0.1:
                    targets.append(point)
    if not targets:
        raise ValueError("No room remains inside the selected boundary clearance; reduce clearance or pass spacing")
    if home_xy is not None and math.dist(home_xy, targets[-1]) < math.dist(home_xy, targets[0]):
        targets.reverse()
    route: list[tuple[tuple[float, float], bool]] = []
    current = None
    vertex_graph = None
    navigation_nodes = None
    for target in targets:
        if current is None or _safe_segment_m(current, target, polygon, clearance_m):
            connector = [target]
        else:
            if navigation_nodes is None:
                navigation_nodes = _inset_navigation_nodes(polygon, clearance_m)
            if vertex_graph is None:
                vertex_graph = _visibility_graph(navigation_nodes, polygon, clearance_m)
            connector = _inside_connector(current, target, polygon, clearance_m, navigation_nodes, vertex_graph)[1:]
        for index, point in enumerate(connector):
            checkpoint = index == len(connector) - 1
            if route and math.dist(route[-1][0], point) <= 0.1:
                route[-1] = (route[-1][0], route[-1][1] or checkpoint)
            else:
                route.append((point, checkpoint))
        current = target
        if len(route) > max_points:
            raise ValueError("Too many movement points; increase automatic pass spacing")
    return [
        {"lat": round(mean_lat + point[1] / y_scale, 9),
         "lon": round(mean_lon + point[0] / x_scale, 9),
         "speed_percent": speed_percent,
         "hold_seconds": collection_seconds if checkpoint else 0,
         "collect_data": checkpoint}
        for point, checkpoint in route
    ]


class MissionStore:
    def __init__(self, folder: Path) -> None:
        self.folder = folder

    def copy_legacy_plans(self, legacy_folder: Path) -> None:
        """Copy existing mission drafts into the dedicated project folder."""
        if not legacy_folder.is_dir() or legacy_folder.resolve() == self.folder.resolve():
            return
        self.folder.mkdir(parents=True, exist_ok=True)
        for source in legacy_folder.glob("*.json"):
            try:
                uuid4_from_string(source.stem)
            except ValueError:
                continue
            destination = self.folder / source.name
            if not destination.exists():
                shutil.copy2(source, destination)

    def list_plans(self) -> list[dict]:
        if not self.folder.exists():
            return []
        plans = []
        for path in self.folder.glob("*.json"):
            try:
                plan = self.load(path.stem)
                plans.append({"id": plan["id"], "name": plan["name"], "points": len(plan["waypoints"]),
                              "offline_tiles_available": plan["offline_tiles_available"]})
            except (OSError, ValueError, KeyError, TypeError):
                continue
        return sorted(plans, key=lambda p: p["name"].casefold())

    def load(self, plan_id: str) -> dict:
        if str(uuid4_from_string(plan_id)) != plan_id:
            raise ValueError("Invalid mission identifier")
        with (self.folder / f"{plan_id}.json").open(encoding="utf-8") as handle:
            plan = json.load(handle)
        if not isinstance(plan, dict) or plan.get("schema_version") != 1 or plan.get("id") != plan_id:
            raise ValueError("Unsupported mission file")
        if not isinstance(plan.get("name"), str) or not isinstance(plan.get("boundary"), list) or not isinstance(plan.get("waypoints"), list) or not isinstance(plan.get("failsafes"), dict):
            raise ValueError("Malformed mission file")
        plan.setdefault("calibration", copy.deepcopy(new_plan()["calibration"]))
        calibration_defaults = new_plan()["calibration"]
        for key, values in calibration_defaults.items():
            if not isinstance(plan["calibration"].get(key), dict):
                plan["calibration"][key] = values
            else:
                for level, value in values.items():
                    plan["calibration"][key].setdefault(level, value)
        if any(not valid_point(p) for p in plan["boundary"] + plan["waypoints"]):
            raise ValueError("Malformed mission point")
        if plan.get("home") is not None and not valid_point(plan["home"]):
            raise ValueError("Malformed home position")
        if plan.get("end") is not None and not valid_point(plan["end"]):
            raise ValueError("Malformed end position")
        plan.setdefault("planned_route", [])
        plan.setdefault("return_route", [])
        plan.setdefault("route_generation_error", "")
        rebuild_mission_routes(plan)
        tile_dir = self.folder / f"{plan_id}_tiles"
        plan["offline_tiles_available"] = (tile_dir / "_complete").is_file()
        plan["offline_tile_template"] = (tile_dir.resolve().as_uri() + "/{z}/{x}/{y}.jpg"
                                         if plan["offline_tiles_available"] else "")
        return plan

    def save(self, plan: dict) -> None:
        plan_id = str(uuid4_from_string(plan["id"]))
        rebuild_mission_routes(plan)
        self.folder.mkdir(parents=True, exist_ok=True)
        with tempfile.NamedTemporaryFile("w", encoding="utf-8", dir=self.folder, suffix=".tmp", delete=False) as handle:
            temp_path = Path(handle.name)
            persisted = copy.deepcopy(plan)
            persisted.pop("offline_tiles_available", None)
            persisted.pop("offline_tile_template", None)
            json.dump(persisted, handle, indent=2, allow_nan=False)
            handle.write("\n")
        try:
            os.replace(temp_path, self.folder / f"{plan_id}.json")
        finally:
            temp_path.unlink(missing_ok=True)
        (self.folder / f"{plan_id}_tiles" / "_complete").unlink(missing_ok=True)

    def prepare_offline_tiles(self, plan_id: str) -> int:
        """Cache compact USGS topo coverage for a mission's full boundary and route."""
        plan_id = str(uuid4_from_string(plan_id))
        plan = self.load(plan_id)
        points = [point for point in [plan.get("home"), plan.get("end")]
                  + plan.get("boundary", []) + plan.get("waypoints", []) if valid_point(point)]
        if not points:
            raise ValueError("Add a mission location before preparing its offline map")

        south, north = min(p["lat"] for p in points), max(p["lat"] for p in points)
        west, east = min(p["lon"] for p in points), max(p["lon"] for p in points)
        lat_pad = max((north - south) * 0.2, 0.001)
        lon_pad = max((east - west) * 0.2, 0.001)
        south, north = max(-85.0, south - lat_pad), min(85.0, north + lat_pad)
        west, east = max(-180.0, west - lon_pad), min(180.0, east + lon_pad)

        def tile_x(lon: float, zoom: int) -> int:
            return int((lon + 180.0) / 360.0 * (1 << zoom))

        def tile_y(lat: float, zoom: int) -> int:
            lat_rad = math.radians(max(-85.05112878, min(85.05112878, lat)))
            return int((1.0 - math.asinh(math.tan(lat_rad)) / math.pi) / 2.0 * (1 << zoom))

        jobs = []
        for zoom in range(10, 17):
            limit = (1 << zoom) - 1
            for x in range(max(0, tile_x(west, zoom)), min(limit, tile_x(east, zoom)) + 1):
                for y in range(max(0, tile_y(north, zoom)), min(limit, tile_y(south, zoom)) + 1):
                    path = self.folder / f"{plan_id}_tiles" / str(zoom) / str(x) / f"{y}.jpg"
                    if not path.is_file():
                        jobs.append((zoom, x, y, path))
        if len(jobs) > 2500:
            raise ValueError("Mission area is too large for the offline map package (limit 2,500 tiles)")

        def download(job: tuple[int, int, int, Path]) -> None:
            zoom, x, y, path = job
            url = f"https://basemap.nationalmap.gov/arcgis/rest/services/USGSTopo/MapServer/tile/{zoom}/{y}/{x}"
            request = Request(url, headers={"User-Agent": "BuoyControlStation/1.0 (offline mission map)"})
            last_error = None
            for attempt in range(3):
                try:
                    with urlopen(request, timeout=15) as response:
                        content = response.read()
                    if not content.startswith(b"\xff\xd8\xff"):
                        raise OSError("USGS tile response was not a JPEG image")
                    path.parent.mkdir(parents=True, exist_ok=True)
                    temporary = path.with_suffix(".tmp")
                    temporary.write_bytes(content)
                    os.replace(temporary, path)
                    return
                except Exception as exc:
                    last_error = exc
                    if attempt < 2:
                        time.sleep(0.4 * (attempt + 1))
            raise OSError(str(last_error))

        failures = []
        with ThreadPoolExecutor(max_workers=3) as pool:
            futures = [pool.submit(download, job) for job in jobs]
            for future in as_completed(futures):
                try:
                    future.result()
                except Exception as exc:
                    failures.append(exc)
        if failures:
            raise OSError(f"Could not download {len(failures)} map tile(s): {failures[0]}")
        marker = self.folder / f"{plan_id}_tiles" / "_complete"
        marker.parent.mkdir(parents=True, exist_ok=True)
        marker.write_text("USGS National Map USGSTopo JPEG tiles; zooms 10-16\n", encoding="utf-8")
        return len(jobs)

    def delete(self, plan_id: str) -> None:
        mission_id = str(uuid4_from_string(plan_id))
        self.folder.joinpath(f"{mission_id}.json").unlink()
        shutil.rmtree(self.folder / f"{mission_id}_tiles", ignore_errors=True)


def uuid4_from_string(value: str):
    from uuid import UUID
    parsed = UUID(str(value))
    if parsed.version != 4:
        raise ValueError("Invalid mission identifier")
    return parsed
