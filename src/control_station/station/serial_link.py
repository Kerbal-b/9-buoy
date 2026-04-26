from __future__ import annotations

import serial
from serial import SerialException
from serial.tools import list_ports

from .models import ManualCommand


def find_port_by_device_name(device_name: str) -> tuple[str | None, str]:
    lowered_name = device_name.lower()

    for port in list_ports.comports():
        fields = [
            port.device or "",
            port.name or "",
            port.description or "",
            port.manufacturer or "",
            port.product or "",
        ]

        if any(lowered_name in field.lower() for field in fields):
            return port.device, f"Matched device name: {device_name}"

    return None, f"Device not found: {device_name}"


def open_serial_connection(
    port: str | None,
    baudrate: int,
    device_name: str | None = None,
) -> tuple[serial.Serial | None, str, str]:
    resolved_port = port
    status = "Simulation only"

    if not resolved_port and device_name:
        resolved_port, status = find_port_by_device_name(device_name)

    if not resolved_port:
        if device_name:
            return None, status, f"Device name: {device_name}"
        return None, "Simulation only", "No port selected"

    try:
        return serial.Serial(resolved_port, baudrate=baudrate, timeout=0.1), "Connected", resolved_port
    except SerialException as exc:
        return None, f"Open failed: {exc}", resolved_port


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


def send_text(link: serial.Serial | None, text: str) -> str:
    if link is None:
        return "No serial link"

    try:
        if not text.endswith("\n"):
            text = f"{text}\n"
        link.write(text.encode("ascii"))
        return "Sent"
    except SerialException as exc:
        return f"Send failed: {exc}"


def read_text(link: serial.Serial | None) -> str:
    if link is None:
        return ""

    try:
        if link.in_waiting == 0:
            return ""

        line = link.readline()
        if not line:
            return ""
        return line.decode("ascii", errors="replace").strip("\r\n")
    except SerialException:
        return ""
