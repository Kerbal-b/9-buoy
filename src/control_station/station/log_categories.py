from __future__ import annotations

import time
import re


SYSTEM_LOG = "system"
NAVIGATION_LOG = "navigation"
SCIENTIFIC_LOG = "scientific"


def classify_log_entry(entry: str) -> str:
    """Assign a communication entry to the most useful operator-facing stream."""
    normalized = entry.strip().upper()

    if (
        "TEL SCI " in normalized
        or "BIN IMU" in normalized
        or "BIN RANGE" in normalized
        or "BIN AUDIO" in normalized
        or normalized.startswith("SCI ")
    ):
        return SCIENTIFIC_LOG

    if (
        normalized.startswith("NAV ")
        or "CTRL VECTOR" in normalized
        or "CTRL MOTION" in normalized
        or "CTRL HOLD" in normalized
        or "CTRL GOTO" in normalized
        or "CTRL TARGET" in normalized
        or "CTRL STOP" in normalized
        or "TEL STATUS POS " in normalized
        or "TEL STATUS TARGET " in normalized
        or "TEL STATUS HOLD " in normalized
        or "TEL STATUS GPS " in normalized
        or "TEL GPS " in normalized
    ):
        return NAVIGATION_LOG

    return SYSTEM_LOG


def display_timestamp(timestamp: float) -> str:
    whole_seconds = int(timestamp)
    milliseconds = int((timestamp - whole_seconds) * 1000.0)
    local_time = time.localtime(timestamp)
    return f"{time.strftime('%H:%M:%S', local_time)}.{milliseconds:03d}"


def _friendly_name(value: str) -> str:
    return value.replace("_", " ").strip().title()


def _number(value: str, digits: int = 2) -> str:
    try:
        return f"{float(value):.{digits}f}"
    except ValueError:
        return value


def interpret_log_entry(entry: str) -> str:
    """Return an operator-friendly explanation for known protocol messages."""
    normalized = entry.strip()
    upper = normalized.upper()

    match = re.search(r"CTRL VECTOR\s+([+-]?\d+)\s+([+-]?\d+)", normalized, re.IGNORECASE)
    if match:
        action = "accepted" if upper.startswith("RX ACK") else "requested"
        return f"Movement {action} — lateral {int(match.group(1)):+d}%, thrust {int(match.group(2)):+d}%"

    match = re.search(r"CTRL MOTION\s+([+-]?\d+)\s+([+-]?\d+)\s+([+-]?\d+)", normalized, re.IGNORECASE)
    if match:
        action = "accepted" if upper.startswith("RX ACK") else "requested"
        return (
            f"Movement {action} — lateral {int(match.group(1)):+d}%, "
            f"thrust {int(match.group(2)):+d}%, yaw {int(match.group(3)):+d}%"
        )

    match = re.search(r"CTRL MOTOR PERCENT\s+(\S+)\s+([+-]?\d+)", normalized, re.IGNORECASE)
    if match:
        percent = int(match.group(2))
        direction = "forward" if percent > 0 else "reverse" if percent < 0 else "stopped"
        return f"Motor test â€” {_friendly_name(match.group(1))}: {direction}, {abs(percent)}%"

    match = re.search(r"CTRL MOTOR\s+(\S+)\s+([+-]?\d+)", normalized, re.IGNORECASE)
    if match:
        drive = int(match.group(2))
        percent = round(abs(drive) / 255.0 * 100.0)
        direction = "forward" if drive > 0 else "reverse" if drive < 0 else "stopped"
        return f"Motor test — {_friendly_name(match.group(1))}: {direction}, {abs(drive)}/255 ({percent}%)"

    match = re.search(
        r"TEL MOTOR CAL\s+(\S+)\s+(FORWARD|REVERSE)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+([0-9.]+)\s+([0-9.]+)\s+([01])",
        normalized,
        re.IGNORECASE,
    )
    if match:
        return (
            f"{_friendly_name(match.group(1))} {match.group(2).lower()} calibration — "
            f"start {match.group(3)} for {match.group(4)} ms, minimum {match.group(5)}, "
            f"maximum {match.group(6)}, curve {match.group(7)}, ramp {match.group(8)} s, "
            f"polarity {'inverted' if match.group(9) == '1' else 'normal'}"
        )

    match = re.search(
        r"TEL STATUS BATTERY\s+(\S+)\s+(\S+)\s+(\S+)", normalized, re.IGNORECASE
    )
    if match:
        if "UNKNOWN" in {value.upper() for value in match.groups()}:
            return "Battery measurement unavailable"
        return (
            f"Battery — {_number(match.group(1))} V, {_number(match.group(2))} A, "
            f"{_number(match.group(3), 0)}% remaining"
        )

    match = re.search(r"TEL STATUS CURRENT\s+(\S+)", normalized, re.IGNORECASE)
    if match:
        return "Current measurement unavailable" if match.group(1).upper() == "UNKNOWN" else f"Current draw — {_number(match.group(1))} A"

    for key, label, unit in (
        ("WATER_TEMP", "Water temperature", "°C"),
        ("AIR_TEMP", "Air temperature", "°C"),
        ("IMU_TEMP", "IMU die temperature", "°C"),
        ("DEPTH", "Measured depth", "m"),
    ):
        match = re.search(rf"TEL SCI {key}\s+(\S+)", normalized, re.IGNORECASE)
        if match:
            value = match.group(1)
            return f"{label} — unavailable" if value.upper() == "UNKNOWN" else f"{label} — {_number(value)} {unit}"

    match = re.search(r"TEL SCI IMU_ACCEL\s+(\S+)\s+(\S+)\s+(\S+)", normalized, re.IGNORECASE)
    if match:
        return f"Acceleration — X {_number(match.group(1), 3)} g, Y {_number(match.group(2), 3)} g, Z {_number(match.group(3), 3)} g"

    match = re.search(r"TEL SCI IMU_GYRO\s+(\S+)\s+(\S+)\s+(\S+)", normalized, re.IGNORECASE)
    if match:
        return f"Angular velocity — X {_number(match.group(1), 2)}, Y {_number(match.group(2), 2)}, Z {_number(match.group(3), 2)} °/s"

    for key, label in (("POS", "Current position"), ("TARGET", "Navigation target")):
        match = re.search(rf"TEL STATUS {key}\s+(\S+)\s+(\S+)", normalized, re.IGNORECASE)
        if match:
            if "UNKNOWN" in {value.upper() for value in match.groups()}:
                return f"{label} — unavailable"
            return f"{label} — latitude {match.group(1)}, longitude {match.group(2)}"

    match = re.search(r"TEL STATUS HOLD\s+(ON|OFF)", normalized, re.IGNORECASE)
    if match:
        return f"Position hold — {match.group(1).lower()}"

    match = re.search(r"TEL STATUS MODE\s+(\S+)", normalized, re.IGNORECASE)
    if match:
        return f"Controller mode — {_friendly_name(match.group(1))}"

    if upper.startswith("NAV "):
        match = re.search(
            r"requested=([+-]?\d+),([+-]?\d+),yaw=([+-]?\d+).*"
            r"corrected=([+-]?\d+),([+-]?\d+),yaw=([+-]?\d+).*"
            r"error_deg=([+-]?[0-9.]+).*correction_deg=([+-]?[0-9.]+)",
            normalized,
            re.IGNORECASE,
        )
        if match:
            return (
                f"Navigation correction — requested {match.group(1)}/{match.group(2)}, yaw {match.group(3)}; "
                f"output {match.group(4)}/{match.group(5)}, yaw {match.group(6)}; "
                f"direction error {match.group(7)}°, correction {match.group(8)}°"
            )
        return "Navigation event — " + normalized[4:]

    if upper == "TX PING":
        return "Heartbeat sent to controller"
    if upper == "RX ACK PING":
        return "Controller heartbeat acknowledged"
    if upper.startswith("LINK TIMEOUT"):
        return "Connection lost — controller stopped responding"
    if upper.startswith("PING TIMEOUT"):
        return "Heartbeat timed out — controller response missing"
    if upper.startswith("CONNECT START"):
        return "Connecting to the buoy controller"
    if upper.startswith("CONNECT SUCCESS") or upper.startswith("RECONNECTED"):
        return "Buoy controller connection established"
    if upper.startswith("CONNECT FAILED") or upper.startswith("RECONNECT FAILED"):
        return "Unable to connect to the buoy controller"
    if upper.startswith("RX ERR") or upper.startswith("TX FAILED"):
        return "Controller communication error"

    return ""
