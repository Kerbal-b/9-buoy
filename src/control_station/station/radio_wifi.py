"""Locate an ExpressLRS TX Backpack on the laptop's Wi-Fi network."""

from __future__ import annotations

import concurrent.futures
import gzip
import ipaddress
import json
import socket


def _http_get(address: str, path: str, source_ip: str | None) -> bytes:
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as connection:
            connection.settimeout(0.35)
            if source_ip:
                connection.bind((source_ip, 0))
            connection.connect((address, 80))
            connection.sendall(f"GET {path} HTTP/1.0\r\nHost: elrs_txbp.local\r\nConnection: close\r\n\r\n".encode("ascii"))
            response = bytearray()
            expected_size = None
            while len(response) < 65536:
                try:
                    chunk = connection.recv(min(4096, 65536 - len(response)))
                except socket.timeout:
                    break
                if not chunk:
                    break
                response.extend(chunk)
                if expected_size is None and b"\r\n\r\n" in response:
                    header, _, _ = response.partition(b"\r\n\r\n")
                    for line in header.split(b"\r\n"):
                        if line.lower().startswith(b"content-length:"):
                            try:
                                expected_size = len(header) + 4 + int(line.partition(b":")[2].strip())
                            except ValueError:
                                pass
                            break
                if expected_size is not None and len(response) >= expected_size:
                    break
        return bytes(response)
    except OSError:
        return b""


def _is_tx_backpack(address: str, source_ip: str | None = None) -> bool:
    for path in ("/", "/index.html"):
        response = _http_get(address, path, source_ip)
        if not response:
            return False
        header, _, body = response.partition(b"\r\n\r\n")
        if not header.startswith(b"HTTP/1.") or b" 200 " not in header.split(b"\r\n", 1)[0]:
            continue
        if b"content-encoding: gzip" in header.lower():
            try:
                body = gzip.decompress(body)
            except (OSError, EOFError):
                continue
        page = body.lower()
        if b"expresslrs" in page and b"tx backpack" in page:
            return True
    return False


def _mavlink_status(address: str, source_ip: str | None) -> str:
    response = _http_get(address, "/mavlink", source_ip)
    header, _, body = response.partition(b"\r\n\r\n")
    if not header.startswith(b"HTTP/1.") or b" 200 " not in header.split(b"\r\n", 1)[0]:
        return "MAVLink status unavailable"
    try:
        data = json.loads(body)
    except (ValueError, UnicodeDecodeError):
        return "MAVLink status unavailable"
    if not isinstance(data, dict):
        return "MAVLink status unavailable"
    if data.get("enabled") is True:
        return "MAVLink forwarding active"
    if data.get("enabled") is False:
        return "MAVLink forwarding disabled"
    return "MAVLink status unavailable"


def discover_tx_backpack(local_ips: list[str], selected_ip: str = "auto") -> tuple[str | None, str]:
    """Check the documented mDNS name, then probe the selected local /24 network."""
    source_ip = selected_ip if selected_ip != "auto" else None
    try:
        address = socket.gethostbyname("elrs_txbp.local")
        if _is_tx_backpack(address, source_ip):
            return address, f"TX Backpack found via elrs_txbp.local; {_mavlink_status(address, source_ip)}"
    except OSError:
        pass

    networks: set[ipaddress.IPv4Network] = set()
    for value in ([source_ip] if source_ip else local_ips):
        if not value:
            continue
        try:
            address = ipaddress.IPv4Address(value)
        except ipaddress.AddressValueError:
            continue
        if address.is_private and not address.is_loopback and not address.is_link_local:
            networks.add(ipaddress.IPv4Network(f"{address}/24", strict=False))

    for network in sorted(networks, key=str):
        candidates = [str(host) for host in network.hosts() if str(host) not in local_ips]
        executor = concurrent.futures.ThreadPoolExecutor(max_workers=32)
        futures = {executor.submit(_is_tx_backpack, host, source_ip): host for host in candidates}
        try:
            for future in concurrent.futures.as_completed(futures):
                if future.result():
                    address = futures[future]
                    return address, f"TX Backpack found on {network}; {_mavlink_status(address, source_ip)}"
        finally:
            executor.shutdown(wait=False, cancel_futures=True)
    return None, "TX Backpack not found; check Backpack telemetry Wi-Fi and the selected network"
