from __future__ import annotations

import asyncio
import concurrent.futures
import json
import re
import socket
import struct
import subprocess
import sys
import threading
from dataclasses import dataclass
from typing import Protocol

import serial
from serial import SerialException
from serial.tools import list_ports

from .models import ManualCommand

try:
    from bleak import BleakClient, BleakScanner
except ImportError:  # pragma: no cover - depends on optional BLE install
    BleakClient = None
    BleakScanner = None


HM10_DEFAULT_SERVICE_UUID = "0000ffe0-0000-1000-8000-00805f9b34fb"
HM10_DEFAULT_CHARACTERISTIC_UUID = "0000ffe1-0000-1000-8000-00805f9b34fb"
HM10_MAX_WRITE_CHUNK = 20

WIFI_DEFAULT_HOST = "auto"
WIFI_DEFAULT_TCP_PORT = 5000
WIFI_DEFAULT_UDP_PORT = 5001
WIFI_DEFAULT_AUDIO_PORT = 5002

PACKET_TYPE_IMU = 1
PACKET_TYPE_RANGE = 2
PACKET_TYPE_POWER = 3
PACKET_TYPE_STATE = 4
PACKET_TYPE_GPS = 5
PACKET_TYPE_AUDIO = 10

HEADER_STRUCT = struct.Struct("<BBHI")
HEADER_AUDIO_STRUCT = struct.Struct("<BBHIHH")
IMU_STRUCT = struct.Struct("<hhhhhhh")
RANGE_STRUCT = struct.Struct("<HBB")
POWER_STRUCT = struct.Struct("<HhBB")
STATE_STRUCT = struct.Struct("<BBBB")
GPS_STRUCT = struct.Struct("<ii")


@dataclass(frozen=True)
class TelemetryPacket:
    packet_type: str
    sequence: int
    timestamp_ms: int
    values: tuple[object, ...]


class LinkTransport(Protocol):
    @property
    def is_open(self) -> bool:
        ...

    def write(self, payload: bytes) -> None:
        ...

    def read_available(self) -> bytes:
        ...

    def read_telemetry_packets(self) -> list[TelemetryPacket]:
        ...

    def close(self) -> None:
        ...


@dataclass
class SerialTransport:
    port: serial.Serial

    @property
    def is_open(self) -> bool:
        return self.port.is_open

    def write(self, payload: bytes) -> None:
        self.port.write(payload)

    def read_available(self) -> bytes:
        waiting = self.port.in_waiting
        if waiting == 0:
            return b""
        return self.port.read(waiting)

    def read_telemetry_packets(self) -> list[TelemetryPacket]:
        return []

    def close(self) -> None:
        self.port.close()


class HM10BleTransport:
    def __init__(
        self,
        device_name: str,
        service_uuid: str,
        characteristic_uuid: str,
        scan_timeout: float = 5.0,
        connect_timeout: float = 15.0,
    ) -> None:
        if BleakClient is None or BleakScanner is None:
            raise RuntimeError("BLE support is not installed. Install bleak in the control station environment.")

        self.device_name = device_name
        self.service_uuid = service_uuid.lower()
        self.characteristic_uuid = characteristic_uuid.lower()
        self.scan_timeout = scan_timeout
        self.connect_timeout = connect_timeout
        self._rx_buffer = bytearray()
        self._rx_lock = threading.Lock()
        self._is_open = False
        self._loop = asyncio.new_event_loop()
        self._loop_ready = threading.Event()
        self._thread = threading.Thread(target=self._run_loop, name="hm10-ble-loop", daemon=True)
        self._thread.start()
        self._loop_ready.wait(timeout=2.0)
        connect_future = asyncio.run_coroutine_threadsafe(self._connect_async(), self._loop)
        connect_future.result(timeout=self.connect_timeout)

    def _run_loop(self) -> None:
        asyncio.set_event_loop(self._loop)
        self._loop_ready.set()
        self._loop.run_forever()

    async def _connect_async(self) -> None:
        assert BleakClient is not None
        assert BleakScanner is not None

        devices = await BleakScanner.discover(timeout=self.scan_timeout)
        candidate_names = [part.strip().lower() for part in self.device_name.split(",") if part.strip()]
        if not candidate_names:
            candidate_names = [self.device_name.lower()]
        matched_device = None
        for device in devices:
            device_name = (device.name or "").lower()
            if any(candidate in device_name for candidate in candidate_names):
                matched_device = device
                break

        if matched_device is None:
            raise RuntimeError(f"BLE device '{self.device_name}' not found")

        self._client = BleakClient(matched_device, disconnected_callback=self._on_disconnect)
        await self._client.connect()

        service = self._client.services.get_service(self.service_uuid)
        if service is None:
            await self._client.disconnect()
            raise RuntimeError(f"BLE service not found: {self.service_uuid}")

        characteristic = service.get_characteristic(self.characteristic_uuid)
        if characteristic is None:
            await self._client.disconnect()
            raise RuntimeError(f"BLE characteristic not found: {self.characteristic_uuid}")

        await self._client.start_notify(self.characteristic_uuid, self._notification_handler)
        self._characteristic = characteristic
        self._is_open = True

    def _notification_handler(self, _sender: object, data: bytearray) -> None:
        with self._rx_lock:
            self._rx_buffer.extend(data)

    def _on_disconnect(self, _client: object) -> None:
        self._is_open = False

    @property
    def is_open(self) -> bool:
        return self._is_open

    async def _write_async(self, payload: bytes) -> None:
        if not self._is_open:
            raise RuntimeError("BLE link is not connected")

        properties = set(getattr(self._characteristic, "properties", []))
        response = "write-without-response" not in properties
        for index in range(0, len(payload), HM10_MAX_WRITE_CHUNK):
            chunk = payload[index:index + HM10_MAX_WRITE_CHUNK]
            await self._client.write_gatt_char(self.characteristic_uuid, chunk, response=response)

    def write(self, payload: bytes) -> None:
        if not self._is_open:
            raise RuntimeError("BLE link is not connected")

        future = asyncio.run_coroutine_threadsafe(self._write_async(payload), self._loop)
        future.result(timeout=5.0)

    def read_available(self) -> bytes:
        with self._rx_lock:
            if not self._rx_buffer:
                return b""
            payload = bytes(self._rx_buffer)
            self._rx_buffer.clear()
            return payload

    def read_telemetry_packets(self) -> list[TelemetryPacket]:
        return []

    async def _close_async(self) -> None:
        if hasattr(self, "_client"):
            try:
                if self._is_open:
                    await self._client.stop_notify(self.characteristic_uuid)
            except Exception:
                pass
            try:
                await self._client.disconnect()
            except Exception:
                pass
        self._is_open = False

    def close(self) -> None:
        if self._loop.is_closed():
            self._is_open = False
            return

        future = asyncio.run_coroutine_threadsafe(self._close_async(), self._loop)
        try:
            future.result(timeout=5.0)
        except Exception:
            pass
        self._loop.call_soon_threadsafe(self._loop.stop)
        self._thread.join(timeout=2.0)
        self._is_open = False


class WiFiTransport:
    def __init__(
        self,
        host: str,
        tcp_port: int,
        udp_port: int,
        audio_port: int,
        connect_timeout: float = 5.0,
    ) -> None:
        self.host = host
        self.tcp_port = tcp_port
        self.udp_port = udp_port
        self.audio_port = audio_port
        self._tcp = socket.create_connection((host, tcp_port), timeout=connect_timeout)
        self._tcp.setblocking(False)
        self._udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._udp.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._udp.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
        self._udp.bind(("", udp_port))
        self._udp.setblocking(False)
        self._audio_udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._audio_udp.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._audio_udp.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
        self._audio_udp.bind(("", audio_port))
        self._audio_udp.setblocking(False)
        self._is_open = True

    @property
    def is_open(self) -> bool:
        return self._is_open

    def write(self, payload: bytes) -> None:
        if not self._is_open:
            raise RuntimeError("Wi-Fi TCP link is not connected")
        self._tcp.sendall(payload)

    def read_available(self) -> bytes:
        if not self._is_open:
            return b""

        chunks = bytearray()
        while True:
            try:
                payload = self._tcp.recv(4096)
            except BlockingIOError:
                break
            except TimeoutError:
                break
            except OSError as exc:
                self._is_open = False
                raise RuntimeError(f"TCP receive failed: {exc}") from exc

            if not payload:
                self._is_open = False
                break
            chunks.extend(payload)

        return bytes(chunks)

    def read_telemetry_packets(self) -> list[TelemetryPacket]:
        if not self._is_open:
            return []

        packets: list[TelemetryPacket] = []
        while True:
            try:
                payload, address = self._udp.recvfrom(2048)
            except BlockingIOError:
                break
            except TimeoutError:
                break
            except OSError:
                break

            if address[0] != self.host:
                continue

            packet = _decode_telemetry_packet(payload)
            if packet is not None:
                packets.append(packet)

        while True:
            try:
                payload, address = self._audio_udp.recvfrom(4096)
            except BlockingIOError:
                break
            except TimeoutError:
                break
            except OSError:
                break

            if address[0] != self.host:
                continue

            packet = _decode_audio_packet(payload)
            if packet is not None:
                packets.append(packet)

        return packets

    def close(self) -> None:
        self._is_open = False
        try:
            self._tcp.close()
        except OSError:
            pass
        try:
            self._udp.close()
        except OSError:
            pass
        try:
            self._audio_udp.close()
        except OSError:
            pass


def _probe_tcp_host(host: str, port: int, timeout: float = 0.15) -> bool:
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False


def _get_local_ipv4() -> str | None:
    try:
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        probe.connect(("8.8.8.8", 80))
        local_ip = probe.getsockname()[0]
        probe.close()
        return local_ip
    except OSError:
        return None


def _get_local_ipv4s() -> list[str]:
    addresses: list[str] = []
    if sys.platform != "win32":
        try:
            result = subprocess.run(["ifconfig"], capture_output=True, text=True, check=True)
            for match in re.finditer(r"\binet (\d+\.\d+\.\d+\.\d+)\b", result.stdout):
                address = match.group(1)
                if address != "127.0.0.1":
                    addresses.append(address)
        except Exception:
            pass

    if not addresses:
        fallback = _get_local_ipv4()
        if fallback is not None:
            addresses.append(fallback)

    seen: set[str] = set()
    unique_addresses: list[str] = []
    for address in addresses:
        if address not in seen:
            seen.add(address)
            unique_addresses.append(address)
    return unique_addresses


def discover_wifi_host(tcp_port: int) -> tuple[str | None, str]:
    mdns_candidates = ("esp32-buoy.local", "buoy.local")
    for candidate in mdns_candidates:
        try:
            resolved = socket.gethostbyname(candidate)
        except OSError:
            continue
        if _probe_tcp_host(resolved, tcp_port):
            return resolved, f"Auto-discovered via mDNS: {candidate} -> {resolved}"

    local_ips = _get_local_ipv4s()
    if not local_ips:
        return None, "Auto-discovery failed: no local IPv4 interface"

    seen_prefixes: set[str] = set()
    prefixes: list[str] = []
    for local_ip in local_ips:
        parts = local_ip.split(".")
        if len(parts) != 4:
            continue
        prefix = ".".join(parts[:3])
        if prefix not in seen_prefixes:
            seen_prefixes.add(prefix)
            prefixes.append(prefix)

    if not prefixes:
        return None, f"Auto-discovery failed: no valid IPv4 prefix from {', '.join(local_ips)}"

    for prefix in prefixes:
        local_hosts = {ip.rsplit(".", 1)[-1] for ip in local_ips if ip.startswith(f"{prefix}.")}
        candidates = [f"{prefix}.{host}" for host in range(1, 255) if str(host) not in local_hosts]

        with concurrent.futures.ThreadPoolExecutor(max_workers=64) as executor:
            futures = {
                executor.submit(_probe_tcp_host, host, tcp_port): host
                for host in candidates
            }
            for future in concurrent.futures.as_completed(futures):
                host = futures[future]
                try:
                    if future.result():
                        return host, f"Auto-discovered on subnet {prefix}.0/24"
                except Exception:
                    continue

    return None, f"Auto-discovery failed on subnets: {', '.join(f'{prefix}.0/24' for prefix in prefixes)}"


def _decode_telemetry_packet(payload: bytes) -> TelemetryPacket | None:
    if len(payload) < HEADER_STRUCT.size:
        return None

    version, packet_type, sequence, timestamp_ms = HEADER_STRUCT.unpack_from(payload, 0)
    if version != 1:
        return None

    payload_bytes = payload[HEADER_STRUCT.size:]

    if packet_type == PACKET_TYPE_IMU:
        if len(payload_bytes) != IMU_STRUCT.size:
            return None
        ax_mg, ay_mg, az_mg, gx_dps_x10, gy_dps_x10, gz_dps_x10, temp_c_x100 = IMU_STRUCT.unpack(payload_bytes)
        return TelemetryPacket(
            packet_type="IMU",
            sequence=sequence,
            timestamp_ms=timestamp_ms,
            values=(
                ax_mg / 1000.0,
                ay_mg / 1000.0,
                az_mg / 1000.0,
                gx_dps_x10 / 10.0,
                gy_dps_x10 / 10.0,
                gz_dps_x10 / 10.0,
                temp_c_x100 / 100.0,
            ),
        )

    if packet_type == PACKET_TYPE_RANGE:
        if len(payload_bytes) != RANGE_STRUCT.size:
            return None
        distance_mm, valid, _reserved = RANGE_STRUCT.unpack(payload_bytes)
        return TelemetryPacket(
            packet_type="RANGE",
            sequence=sequence,
            timestamp_ms=timestamp_ms,
            values=(distance_mm, bool(valid)),
        )

    if packet_type == PACKET_TYPE_POWER:
        if len(payload_bytes) != POWER_STRUCT.size:
            return None
        battery_mv, current_ma, battery_pct, _reserved = POWER_STRUCT.unpack(payload_bytes)
        return TelemetryPacket(
            packet_type="POWER",
            sequence=sequence,
            timestamp_ms=timestamp_ms,
            values=(battery_mv / 1000.0, current_ma / 1000.0, battery_pct),
        )

    if packet_type == PACKET_TYPE_STATE:
        if len(payload_bytes) != STATE_STRUCT.size:
            return None
        control_mode, hold_enabled, gps_valid, motors_enabled = STATE_STRUCT.unpack(payload_bytes)
        return TelemetryPacket(
            packet_type="STATE",
            sequence=sequence,
            timestamp_ms=timestamp_ms,
            values=(control_mode, bool(hold_enabled), bool(gps_valid), bool(motors_enabled)),
        )

    if packet_type == PACKET_TYPE_GPS:
        if len(payload_bytes) != GPS_STRUCT.size:
            return None
        lat_e7, lon_e7 = GPS_STRUCT.unpack(payload_bytes)
        return TelemetryPacket(
            packet_type="GPS",
            sequence=sequence,
            timestamp_ms=timestamp_ms,
            values=(lat_e7 / 1e7, lon_e7 / 1e7),
        )

    return None


def _decode_audio_packet(payload: bytes) -> TelemetryPacket | None:
    if len(payload) < HEADER_AUDIO_STRUCT.size:
        return None

    version, packet_type, sequence, timestamp_ms, sample_count, channels = HEADER_AUDIO_STRUCT.unpack_from(payload, 0)
    if version != 1 or packet_type != PACKET_TYPE_AUDIO:
        return None

    pcm_bytes = payload[HEADER_AUDIO_STRUCT.size:]
    expected_bytes = sample_count * channels * 2
    if len(pcm_bytes) != expected_bytes:
        return None

    return TelemetryPacket(
        packet_type="AUDIO",
        sequence=sequence,
        timestamp_ms=timestamp_ms,
        values=(pcm_bytes, sample_count, channels),
    )


def _find_windows_bluetooth_port(device_name: str) -> tuple[str | None, str]:
    try:
        device_script = r"""
$devices = Get-PnpDevice |
    Where-Object { $_.FriendlyName -like '*%s*' } |
    Select-Object FriendlyName, InstanceId
$devices | ConvertTo-Json -Compress
""" % device_name
        device_result = subprocess.run(
            ["powershell", "-NoProfile", "-Command", device_script],
            capture_output=True,
            text=True,
            check=True,
        )
        if not device_result.stdout.strip():
            return None, "Disconnected"

        devices = json.loads(device_result.stdout)
        if isinstance(devices, dict):
            devices = [devices]

        address_tokens: list[str] = []
        for device in devices:
            instance_id = str(device.get("InstanceId", ""))
            if "DEV_" in instance_id:
                address_tokens.append(instance_id.split("DEV_", 1)[1].split("\\", 1)[0].upper())

        if not address_tokens:
            return None, f"Matched device name: {device_name}, but no COM port mapping found"

        ports_script = r"""
$ports = Get-CimInstance Win32_SerialPort |
    Select-Object DeviceID, Name, Description, PNPDeviceID
$ports | ConvertTo-Json -Compress
"""
        ports_result = subprocess.run(
            ["powershell", "-NoProfile", "-Command", ports_script],
            capture_output=True,
            text=True,
            check=True,
        )
        if not ports_result.stdout.strip():
            return None, f"Matched device name: {device_name}, but no serial ports found"

        ports = json.loads(ports_result.stdout)
        if isinstance(ports, dict):
            ports = [ports]

        for token in address_tokens:
            for port in ports:
                pnp_device_id = str(port.get("PNPDeviceID", "")).upper()
                if token in pnp_device_id:
                    return str(port.get("DeviceID", "")), f"Matched device name: {device_name}"

        return None, f"Matched device name: {device_name}, but no COM port mapping found"
    except Exception:
        return None, "Disconnected"


def find_port_by_device_name(device_name: str) -> tuple[str | None, str]:
    candidate_names = [part.strip().lower() for part in device_name.split(",") if part.strip()]
    if not candidate_names:
        candidate_names = [device_name.lower()]

    for port in list_ports.comports():
        fields = [
            port.device or "",
            port.name or "",
            port.description or "",
            port.manufacturer or "",
            port.product or "",
        ]

        if any(candidate in field.lower() for candidate in candidate_names for field in fields):
            return port.device, f"Matched device name: {device_name}"

    if sys.platform == "win32":
        return _find_windows_bluetooth_port(device_name)

    return None, "Disconnected"


def open_serial_connection(
    port: str | None,
    baudrate: int,
    device_name: str | None = None,
    *,
    use_ble: bool = False,
    use_wifi: bool = False,
    wifi_host: str = WIFI_DEFAULT_HOST,
    tcp_port: int = WIFI_DEFAULT_TCP_PORT,
    udp_port: int = WIFI_DEFAULT_UDP_PORT,
    audio_port: int = WIFI_DEFAULT_AUDIO_PORT,
    ble_service_uuid: str = HM10_DEFAULT_SERVICE_UUID,
    ble_characteristic_uuid: str = HM10_DEFAULT_CHARACTERISTIC_UUID,
) -> tuple[LinkTransport | None, str, str]:
    if use_wifi:
        resolved_host = wifi_host
        if wifi_host.strip().lower() == "auto":
            resolved_host, discovery_status = discover_wifi_host(tcp_port)
            if resolved_host is None:
                target = f"WIFI:auto:{tcp_port}/udp:{udp_port}"
                return None, f"Open failed: {discovery_status}", target

        try:
            transport = WiFiTransport(
                host=resolved_host,
                tcp_port=tcp_port,
                udp_port=udp_port,
                audio_port=audio_port,
            )
            return transport, "Connected", f"WIFI:{resolved_host}:{tcp_port}/udp:{udp_port}/audio:{audio_port}"
        except Exception as exc:
            return None, f"Open failed: {exc}", f"WIFI:{resolved_host}:{tcp_port}/udp:{udp_port}/audio:{audio_port}"

    if use_ble:
        if not device_name:
            return None, "BLE target missing", "No BLE device name selected"

        try:
            transport = HM10BleTransport(
                device_name=device_name,
                service_uuid=ble_service_uuid,
                characteristic_uuid=ble_characteristic_uuid,
            )
            return transport, "Connected", f"BLE:{device_name}"
        except Exception as exc:
            return None, f"Open failed: {exc}", f"BLE:{device_name}"

    resolved_port = port
    status = "Simulation only"

    if not resolved_port and device_name:
        resolved_port, status = find_port_by_device_name(device_name)

    if not resolved_port:
        if device_name:
            return None, status, f"Device name: {device_name}"
        return None, "Simulation only", "No port selected"

    try:
        return SerialTransport(serial.Serial(resolved_port, baudrate=baudrate, timeout=0.1)), "Connected", resolved_port
    except SerialException as exc:
        return None, f"Open failed: {exc}", resolved_port


def send_command(
    link: LinkTransport | None,
    command: ManualCommand,
) -> str:
    if link is None:
        return "No serial link"

    try:
        link.write(command.to_line().encode("ascii"))
        return "OK"
    except Exception as exc:
        return f"Send failed: {exc}"


def send_text(link: LinkTransport | None, text: str) -> str:
    if link is None:
        return "No serial link"

    try:
        if not text.endswith("\n"):
            text = f"{text}\n"
        link.write(text.encode("ascii"))
        return "OK"
    except Exception as exc:
        return f"Send failed: {exc}"


def read_text(link: LinkTransport | None) -> str:
    if link is None:
        return ""

    payload = read_available_bytes(link)
    if not payload:
        return ""
    return payload.decode("ascii", errors="replace").strip("\r\n")


def read_available_bytes(link: LinkTransport | None) -> bytes:
    if link is None:
        return b""

    try:
        return link.read_available()
    except Exception:
        return b""


def read_telemetry_packets(link: LinkTransport | None) -> list[TelemetryPacket]:
    if link is None:
        return []

    try:
        return link.read_telemetry_packets()
    except Exception:
        return []
