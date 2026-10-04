from __future__ import annotations

import socket
import time
from pathlib import Path
from typing import Callable


WIFI_SD_FILE_PORT = 5003
_CHUNK_BYTES = 32768
SCIENCE_FILE_NAMES = frozenset({"audio.wav", "audio.idx", "science.csv", "telemetry.csv", "manifest.txt"})


def _connect(host: str, local_ip: str | None, timeout: float) -> socket.socket:
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client.settimeout(timeout)
    if local_ip and local_ip.lower() != "auto":
        client.bind((local_ip, 0))
    client.connect((host, WIFI_SD_FILE_PORT))
    return client


def list_sd_files(host: str, local_ip: str | None = None) -> dict[str, object]:
    return _list_sd_directory(host, "/", local_ip)


def list_sd_session(host: str, session_path: str, local_ip: str | None = None) -> dict[str, object]:
    if not session_path.startswith("/session") or ".." in session_path or "|" in session_path:
        raise ValueError("Invalid SD session path")
    return _list_sd_directory(host, session_path, local_ip)


def delete_sd_session(host: str, session_path: str, local_ip: str | None = None) -> str:
    name = session_path.rsplit("/", 1)[-1]
    tail = name[7:] if name.startswith("session") else ""
    if tail.isdigit() and len(tail) >= 3:
        valid_name = True  # legacy sessionNNN folder
    elif tail.startswith("-"):
        stamp = tail[1:]
        valid_name = len(stamp) == 16 and all(char in "0123456789TZ" for char in stamp)
    else:
        separator = tail.find("-")
        stamp = tail[separator + 1 :] if separator >= 3 else ""
        valid_name = (
            separator >= 3
            and tail[:separator].isdigit()
            and len(stamp) == 16
            and all(char in "0123456789TZ" for char in stamp)
        )
    if (
        not session_path.startswith("/session")
        or session_path.count("/") != 1
        or ".." in session_path
        or "|" in session_path
        or not valid_name
    ):
        raise ValueError("Invalid SD session path")
    with _connect(host, local_ip, 8.0) as client:
        client.sendall(f"DELETE {session_path}\n".encode("utf-8"))
        response_bytes = bytearray()
        while b"\n" not in response_bytes:
            chunk = client.recv(128)
            if not chunk:
                break
            response_bytes.extend(chunk)
        response = bytes(response_bytes).split(b"\n", 1)[0].decode("utf-8", errors="replace").strip()
    if response.startswith("ERR "):
        raise RuntimeError(response[4:])
    if response != "OK DELETE":
        raise RuntimeError(f"Unexpected SD delete response: {response or 'connection closed'}")
    return session_path


def _list_sd_directory(host: str, directory: str, local_ip: str | None) -> dict[str, object]:
    entries: list[dict[str, object]] = []
    capacity: dict[str, int] = {}
    with _connect(host, local_ip, 8.0) as client:
        request = "LIST\n" if directory == "/" else f"LIST {directory}\n"
        client.sendall(request.encode("utf-8"))
        buffer = bytearray()
        while True:
            while b"\n" not in buffer:
                chunk = client.recv(4096)
                if not chunk:
                    raise RuntimeError("SD card connection closed before the listing finished")
                buffer.extend(chunk)
            line, _, remainder = buffer.partition(b"\n")
            buffer = bytearray(remainder)
            response = line.decode("utf-8", errors="replace").rstrip("\r")
            if response.startswith("ERR "):
                raise RuntimeError(response[4:])
            fields = response.split("|")
            if fields[0] == "INFO" and len(fields) >= 6 and fields[1] == "CARD":
                capacity = {"cardBytes": int(fields[2]), "totalBytes": int(fields[3]), "usedBytes": int(fields[4]), "freeBytes": int(fields[5])}
                continue
            if fields[0] == "OK" and len(fields) >= 2:
                continue
            if fields[0] == "END":
                break
            if fields[0] == "FILE" and len(fields) >= 4:
                entries.append({"path": fields[1], "name": fields[2], "size": int(fields[3]), "isDirectory": False})
            elif fields[0] == "DIR" and len(fields) >= 3:
                entries.append({"path": fields[1], "name": fields[2], "size": int(fields[3]) if len(fields) >= 4 else 0, "isDirectory": True})
    return {"files": entries, "capacity": capacity}


def download_sd_file(
    host: str,
    remote_path: str,
    destination: str,
    local_ip: str | None = None,
    progress: Callable[[int, int], None] | None = None,
    cancelled: Callable[[], bool] | None = None,
) -> Path:
    if cancelled and cancelled():
        raise RuntimeError("Download cancelled")
    target = Path(destination)
    target.parent.mkdir(parents=True, exist_ok=True)
    received = 0
    last_progress_at = time.monotonic()
    temporary = target.with_name(target.name + ".part")
    try:
        with _connect(host, local_ip, 15.0) as client:
            client.settimeout(1.0)
            client.sendall(f"GET {remote_path}\n".encode("utf-8"))
            header = bytearray()
            last_data_at = time.monotonic()
            while b"\n" not in header:
                if cancelled and cancelled():
                    raise RuntimeError("Download cancelled")
                try:
                    chunk = client.recv(256)
                except socket.timeout:
                    if time.monotonic() - last_data_at > 15.0:
                        raise RuntimeError("SD download timed out")
                    continue
                if not chunk:
                    raise RuntimeError("SD card connection closed before the download started")
                header.extend(chunk)
                last_data_at = time.monotonic()
            line, _, remainder = header.partition(b"\n")
            fields = line.decode("utf-8", errors="replace").rstrip("\r").split(" ", 3)
            if not fields or fields[0] != "OK" or len(fields) < 3 or fields[1] != "FILE":
                raise RuntimeError(" ".join(fields[1:]) or "SD download failed")
            expected = int(fields[2])
            with temporary.open("wb") as output:
                if remainder:
                    payload = bytes(remainder[:expected])
                    output.write(payload)
                    received += len(payload)
                while received < expected:
                    if cancelled and cancelled():
                        raise RuntimeError("Download cancelled")
                    try:
                        chunk = client.recv(min(_CHUNK_BYTES, expected - received))
                    except socket.timeout:
                        if time.monotonic() - last_data_at > 15.0:
                            raise RuntimeError("SD download timed out")
                        continue
                    if not chunk:
                        raise RuntimeError(f"Download ended early ({received} of {expected} bytes)")
                    last_data_at = time.monotonic()
                    output.write(chunk)
                    received += len(chunk)
                    if progress and (received == expected or time.monotonic() - last_progress_at >= 0.2):
                        progress(received, expected)
                        last_progress_at = time.monotonic()
        if cancelled and cancelled():
            raise RuntimeError("Download cancelled")
        temporary.replace(target)
        if progress:
            progress(expected, expected)
        return target
    except Exception:
        temporary.unlink(missing_ok=True)
        raise


def download_sd_session(
    host: str,
    session_path: str,
    destination_parent: str,
    local_ip: str | None = None,
    progress: Callable[[int, int, str], None] | None = None,
    cancelled: Callable[[], bool] | None = None,
) -> Path:
    if cancelled and cancelled():
        raise RuntimeError("Download cancelled")
    listing = list_sd_session(host, session_path, local_ip)
    files = [entry for entry in listing["files"] if not entry["isDirectory"] and str(entry["name"]).lower() in SCIENCE_FILE_NAMES]
    total = sum(int(entry["size"]) for entry in files)
    session_name = session_path.rstrip("/").rsplit("/", 1)[-1]
    parent = Path(destination_parent)
    parent.mkdir(parents=True, exist_ok=True)
    target = parent / session_name
    suffix = 2
    while target.exists():
        target = parent / f"{session_name}-{suffix}"
        suffix += 1
    target.mkdir()
    incomplete = target / ".download-incomplete"
    incomplete.write_text("This session download was interrupted.\n", encoding="utf-8")
    completed = 0
    if progress:
        progress(0, total, "Starting session download")
    for entry in files:
        if cancelled and cancelled():
            raise RuntimeError("Download cancelled")
        name = str(entry["name"])
        if Path(name).name != name:
            raise RuntimeError(f"Invalid filename on SD card: {name}")
        offset = completed
        download_sd_file(
            host,
            str(entry["path"]),
            str(target / name),
            local_ip,
            lambda received, expected, file_name=name, base=offset: progress(base + received, total, file_name) if progress else None,
            cancelled,
        )
        completed += int(entry["size"])
    if progress:
        progress(total, total, "Complete")
    incomplete.unlink()
    return target
