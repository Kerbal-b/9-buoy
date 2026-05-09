from __future__ import annotations


def format_vector_command(turn: int, thrust: int) -> str:
    return f"CTRL VECTOR {turn:+d} {thrust:+d}\n"


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


def parse_science_update(text: str) -> tuple[str, tuple[str, ...]] | None:
    prefix = "TEL SCI "
    if not text.startswith(prefix):
        return None

    parts = text[len(prefix):].split()
    if len(parts) < 1:
        return None

    return parts[0], tuple(parts[1:])
