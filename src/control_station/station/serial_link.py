from __future__ import annotations

import serial
from serial import SerialException

from .models import ManualCommand


def open_serial_connection(
    port: str | None, baudrate: int
) -> tuple[serial.Serial | None, str]:
    if not port:
        return None, "Simulation only"

    try:
        return serial.Serial(port, baudrate=baudrate, timeout=0.1), "Connected"
    except SerialException as exc:
        return None, f"Open failed: {exc}"


def send_command(
    link: serial.Serial | None,
    command: ManualCommand,
) -> str:
    if link is None:
        return "No serial link"

    try:
        link.write(command.to_line().encode("ascii"))
        return "Sent"
    except SerialException as exc:
        return f"Send failed: {exc}"
