"""Small, standard-library-only TX Backpack Wi-Fi/MAVLink diagnostic.

Run from this directory: python diagnose_backpack.py --host 192.168.8.134
This does not need a buoy receiver or change Backpack configuration.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import gzip
import json
import re
import socket
import struct
import time
from urllib.error import URLError
from urllib.request import urlopen

from station.mavlink_telemetry import CRC_EXTRAS, _crc_x25, decode_datagram, gcs_heartbeat


def mavlink_frames(data: bytes) -> list[dict]:
    """Describe every complete MAVLink frame, including unknown message IDs."""
    frames = []
    pos = 0
    while pos < len(data):
        if data[pos] not in (0xFD, 0xFE):
            pos += 1
            continue
        v2 = data[pos] == 0xFD
        header = 10 if v2 else 6
        if len(data) - pos < header:
            break
        size = data[pos + 1]
        signed = bool(v2 and data[pos + 2] & 1)
        end = pos + header + size + 2 + (13 if signed else 0)
        if end > len(data):
            break
        raw = data[pos:end]
        message_id = int.from_bytes(raw[7:10], "little") if v2 else raw[5]
        extra = 185 if message_id == 109 else CRC_EXTRAS.get(message_id)
        received_crc = struct.unpack_from("<H", raw, header + size)[0]
        valid = None if extra is None else received_crc == _crc_x25(raw[1:header + size], extra)
        frame_info = {
            "offset": pos, "version": 2 if v2 else 1,
            "message_id": message_id, "sequence": raw[4] if v2 else raw[2],
            "system_id": raw[5] if v2 else raw[3],
            "component_id": raw[6] if v2 else raw[4],
            "signed": signed, "crc_valid": valid,
            "payload_hex": raw[header:header + size].hex(),
        }
        if message_id == 109 and valid and size >= 9:
            rxerrors, fixed, rssi, remote_rssi, txbuf, noise, remote_noise = struct.unpack_from(
                "<HHBBBBB", raw, header
            )
            frame_info["radio_status"] = {
                "rssi_raw": None if rssi == 255 else rssi,
                "remote_rssi_raw": None if remote_rssi == 255 else remote_rssi,
                "tx_buffer_free_percent": txbuf,
                "noise_raw": None if noise == 255 else noise,
                "remote_noise_raw": None if remote_noise == 255 else remote_noise,
                "receive_errors": rxerrors, "corrected_packets": fixed,
            }
        frames.append(frame_info)
        pos = end
    return frames


def get_http(host: str, path: str) -> bytes:
    with urlopen(f"http://{host}{path}", timeout=3) as response:
        body = response.read()
        return gzip.decompress(body) if response.headers.get("Content-Encoding") == "gzip" else body


def get_status(host: str) -> dict | None:
    try:
        status = json.loads(get_http(host, "/mavlink"))
        return status if isinstance(status, dict) else None
    except (OSError, URLError, ValueError):
        return None


def get_config(host: str) -> dict | None:
    try:
        config = json.loads(get_http(host, "/config"))
        return config.get("config") if isinstance(config, dict) else None
    except (OSError, URLError, ValueError):
        return None


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="elrs_txbp.local", help="Backpack hostname or IP")
    parser.add_argument("--bind", default="0.0.0.0", help="local IPv4 address for UDP receive")
    parser.add_argument("--listen-port", type=int, default=14550)
    parser.add_argument("--seconds", type=float, default=20)
    parser.add_argument("--log", help="append JSON lines to this file")
    parser.add_argument("--no-probe", action="store_true", help="skip one GCS heartbeat UDP probe")
    args = parser.parse_args()
    log = open(args.log, "a", encoding="utf-8") if args.log else None

    def emit(event: dict) -> None:
        event = {"time": datetime.now(timezone.utc).isoformat(), **event}
        line = json.dumps(event, sort_keys=True)
        print(line, flush=True)
        if log:
            log.write(line + "\n")
            log.flush()

    try:
        try:
            address = socket.gethostbyname(args.host)
            page = get_http(args.host, "/")
            identified = b"expresslrs" in page.lower() and b"tx backpack" in page.lower()
            firmware = re.search(rb"Firmware Rev\.\s*</b>\s*([^<]+)", page, re.IGNORECASE)
            emit({"event": "web", "address": address, "reachable": True,
                  "tx_backpack_page": identified,
                  "backpack_firmware": firmware.group(1).decode("utf-8", "replace").strip() if firmware else None})
        except (OSError, URLError) as exc:
            emit({"event": "web", "reachable": False, "error": str(exc)})
            return 2
        if not identified:
            emit({"event": "warning", "detail": "Web page did not identify as an ExpressLRS TX Backpack"})
        emit({"event": "backpack_config", "config": get_config(args.host)})
        before = get_status(args.host)
        emit({"event": "mavlink_status_before", "status": before})
        listen_port = args.listen_port
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver:
            try:
                receiver.bind((args.bind, listen_port))
            except OSError as exc:
                emit({"event": "udp_bind_failed", "port": listen_port, "error": str(exc),
                      "detail": "Close the other listener on this port, or select another port (which may not receive Backpack traffic)"})
                return 1
            receiver.settimeout(0.25)
            emit({"event": "udp_listening", "address": args.bind, "port": listen_port})
            if before and before.get("enabled") is True and not args.no_probe:
                port = before.get("ports", {}).get("listen", 14555)
                receiver.sendto(gcs_heartbeat(), (address, port))
                emit({"event": "udp_probe_sent", "address": address, "port": port,
                      "detail": "one MAVLink GCS HEARTBEAT; sending alone does not prove receipt"})
            elif before and before.get("enabled") is False:
                emit({"event": "udp_probe_skipped", "reason": "Backpack MAVLink forwarding is disabled"})
            count = 0
            deadline = time.monotonic() + max(0, args.seconds)
            while time.monotonic() < deadline:
                try:
                    data, source = receiver.recvfrom(65535)
                except socket.timeout:
                    continue
                count += 1
                frames = mavlink_frames(data)
                emit({"event": "udp_packet", "source": list(source), "bytes": len(data),
                      "raw_hex": data.hex(), "mavlink_frames": frames,
                      "decoded": decode_datagram(data)})
        after = get_status(args.host)
        emit({"event": "mavlink_status_after", "status": after})
        old_count = (before or {}).get("counters", {}).get("packets_up")
        new_count = (after or {}).get("counters", {}).get("packets_up")
        gcs = (after or {}).get("ip", {}).get("gcs")
        if count:
            verdict = "UDP datagrams received; inspect each source and CRC before attributing them to the buoy"
        elif before and before.get("enabled") is False:
            verdict = "Backpack web endpoint reached; MAVLINK_TX Wi-Fi service is inactive, so the UDP bridge cannot forward packets"
        elif gcs == receiver_ip(address) and gcs != (before or {}).get("ip", {}).get("gcs"):
            verdict = "Backpack learned this PC as GCS; UDP reached its bridge (RF delivery is unproven)"
        elif isinstance(old_count, int) and isinstance(new_count, int) and new_count > old_count:
            verdict = "Backpack uplink counter rose; consistent with UDP receipt, but other GCS traffic is possible"
        else:
            verdict = "Web endpoint reached; no UDP acknowledgement or telemetry observed"
        emit({"event": "result", "udp_packets": count, "verdict": verdict})
        return 0
    except OSError as exc:
        emit({"event": "error", "detail": str(exc)})
        return 1
    finally:
        if log:
            log.close()


def receiver_ip(target: str) -> str:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
        probe.connect((target, 14555))
        return probe.getsockname()[0]


if __name__ == "__main__":
    raise SystemExit(main())
