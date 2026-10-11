from __future__ import annotations

from datetime import date


def format_vector_command(turn: int, thrust: int) -> str:
    return f"CTRL VECTOR {turn:+d} {thrust:+d}\n"


def format_motion_command(lateral: int, thrust: int, yaw: int) -> str:
    return f"CTRL MOTION {lateral:+d} {thrust:+d} {yaw:+d}\n"


def is_protocol_message(text: str) -> bool:
    return text.startswith(("ACK ", "ERR ", "TEL "))


def parse_acknowledgement(text: str) -> tuple[str, tuple[str, ...]] | None:
    if not text.startswith("ACK "):
        return None

    parts = text.split()
    if len(parts) < 2:
        return None

    return parts[1], tuple(parts[2:])


def parse_error(text: str) -> str | None:
    if not text.startswith("ERR "):
        return None
    return text[4:].strip() or "UNKNOWN"


def parse_status_update(text: str) -> tuple[str, tuple[str, ...]] | None:
    prefix = "TEL STATUS "
    if not text.startswith(prefix):
        return None

    parts = text[len(prefix):].split()
    if len(parts) < 1:
        return None

    return parts[0], tuple(parts[1:])


def format_firmware_version(values: tuple[str, ...]) -> str:
    """Format numeric firmware fields, preserving older banner responses."""
    if len(values) == 4 and all(value.isdecimal() for value in values):
        major, minor, patch, build_date = values
        if len(build_date) == 8:
            try:
                built = date(int(build_date[:4]), int(build_date[4:6]), int(build_date[6:]))
            except ValueError:
                pass
            else:
                return f"v{int(major)}.{int(minor)}.{int(patch)} ({built.isoformat()})"
    return " ".join(values)


def parse_science_update(text: str) -> tuple[str, tuple[str, ...]] | None:
    prefix = "TEL SCI "
    if not text.startswith(prefix):
        return None

    parts = text[len(prefix):].split()
    if len(parts) < 1:
        return None

    return parts[0], tuple(parts[1:])
