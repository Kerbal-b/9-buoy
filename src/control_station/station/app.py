from __future__ import annotations

import argparse
from array import array
from collections import deque
import sys
import time
from dataclasses import replace
from pathlib import Path

import pygame

from .controller import get_controller, read_axes, read_controller_snapshot
from .geometry import build_manual_command
from .models import ManualCommand, RuntimeState, ScienceSample, TelemetryState
from .protocol import is_protocol_message, parse_acknowledgement, parse_error, parse_science_update, parse_status_update
from .serial_link import (
    CONNECTION_PREFS_FILENAME,
    HM10_DEFAULT_CHARACTERISTIC_UUID,
    HM10_DEFAULT_SERVICE_UUID,
    WIFI_DEFAULT_AUDIO_PORT,
    WIFI_DEFAULT_HOST,
    WIFI_DEFAULT_TCP_PORT,
    WIFI_DEFAULT_UDP_PORT,
    apply_connection_preferences,
    extract_wifi_host_from_target,
    open_serial_connection,
    read_available_bytes,
    read_telemetry_packets,
    save_connection_preferences,
    send_command,
    send_text,
)
from .settings import DEFAULT_BAUDRATE, DEFAULT_DEADZONE, DEFAULT_SEND_RATE, WINDOW_HEIGHT, WINDOW_WIDTH
from .ui import (
    get_audio_mute_button_rect,
    get_dashboard_tab_rects,
    get_main_layout_rects,
    render_debug_interface,
    render_main_interface,
)


def format_bytes_for_log(payload: bytes) -> str:
    hex_part = " ".join(f"{byte:02X}" for byte in payload)
    ascii_part = "".join(chr(byte) if 32 <= byte <= 126 else "." for byte in payload)
    return f"hex=[{hex_part}] ascii=[{ascii_part}]"


PING_BYTES = b"PING\n"
AUDIO_SAMPLE_RATE = 16000
AUDIO_CHANNELS = 2
AUDIO_WAVEFORM_SAMPLES = 480


def append_comm_log(comm_log: list[str], log_file: Path, entry: str) -> None:
    comm_log.append(entry)
    if len(comm_log) > 50:
        comm_log.pop(0)
    with log_file.open("a", encoding="utf-8") as handle:
        handle.write(f"{entry}\n")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Laptop control station for manual buoy control."
    )
    parser.add_argument(
        "--port",
        help="Serial port for the Bluetooth link, for example /dev/tty.HC-05-DevB",
    )
    parser.add_argument(
        "--transport",
        choices=("serial", "ble", "wifi"),
        default="wifi",
        help="Link transport to use for the buoy connection",
    )
    parser.add_argument(
        "--device-name",
        default="ESP32-BUOY",
        help="Device name(s) to auto-discover when --port is not supplied or when using BLE/serial (comma-separated)",
    )
    parser.add_argument(
        "--wifi-host",
        default=WIFI_DEFAULT_HOST,
        help="ESP32 Wi-Fi host or IP address",
    )
    parser.add_argument(
        "--wifi-local-ip",
        "--source-ip",
        dest="wifi_local_ip",
        default="auto",
        help="Local IPv4 address to use as the source interface (auto selects all interfaces)",
    )
    parser.add_argument(
        "--tcp-port",
        type=int,
        default=WIFI_DEFAULT_TCP_PORT,
        help="TCP port used for reliable command/control",
    )
    parser.add_argument(
        "--udp-port",
        type=int,
        default=WIFI_DEFAULT_UDP_PORT,
        help="Local UDP port used for fast telemetry",
    )
    parser.add_argument(
        "--audio-port",
        type=int,
        default=WIFI_DEFAULT_AUDIO_PORT,
        help="Local UDP port used for streamed audio",
    )
    parser.add_argument(
        "--ble-service-uuid",
        default=HM10_DEFAULT_SERVICE_UUID,
        help="HM-10 BLE service UUID",
    )
    parser.add_argument(
        "--ble-characteristic-uuid",
        default=HM10_DEFAULT_CHARACTERISTIC_UUID,
        help="HM-10 BLE characteristic UUID used for write and notify",
    )
    parser.add_argument(
        "--baudrate",
        type=int,
        default=DEFAULT_BAUDRATE,
        help="Serial baudrate used by the serial link",
    )
    parser.add_argument(
        "--deadzone",
        type=float,
        default=DEFAULT_DEADZONE,
        help="Ignore small joystick movement around center",
    )
    parser.add_argument(
        "--send-rate",
        type=float,
        default=DEFAULT_SEND_RATE,
        help="How many command updates to send per second",
    )
    parser.add_argument(
        "--hello-ping",
        action="store_true",
        help="Send hello world text messages over the active transport instead of CTRL commands",
    )
    parser.add_argument(
        "--send-text",
        help="Send this text over the active transport instead of CTRL commands",
    )
    parser.add_argument(
        "--send-interval",
        type=float,
        default=1.0,
        help="Seconds between repeated text messages when --send-text or --hello-ping is enabled",
    )
    parser.add_argument(
        "--debug-controller",
        action="store_true",
        help="Open the controller diagnostics window instead of the main interface",
    )
    parser.add_argument(
        "--comm-log-file",
        help="Write raw transport communication logs to this file",
    )
    return parser.parse_args()


def build_runtime_state(
    joystick: pygame.joystick.Joystick | None,
    controller_snapshot,
    serial_status: str,
    serial_target: str,
    last_send_result: str,
    last_sent_line: str,
    command: ManualCommand,
    command_input: str,
    last_response: str,
    telemetry: TelemetryState,
    ack_vector: str,
    comm_log: list[str],
    *,
    audio_muted: bool,
    audio_waveform: tuple[float, ...],
) -> RuntimeState:
    controller_status = "Connected" if joystick is not None else "Not connected"
    controller_name = joystick.get_name() if joystick is not None else "Connect Xbox controller"
    return RuntimeState(
        command=command,
        controller_status=controller_status,
        controller_name=controller_name,
        controller_snapshot=controller_snapshot,
        serial_status=serial_status,
        serial_target=serial_target,
        last_send_result=last_send_result,
        last_sent_line=last_sent_line,
        command_input=command_input,
        last_response=last_response,
        telemetry=telemetry,
        ack_vector=ack_vector,
        comm_log=comm_log,
        audio_muted=audio_muted,
        audio_waveform=audio_waveform,
    )


def format_position(values: tuple[str, ...]) -> str:
    if len(values) < 2:
        return "Unknown"
    if values[0] == "UNKNOWN" or values[1] == "UNKNOWN":
        return "Unknown"
    return f"{values[0]}, {values[1]}"


def parse_battery_status(values: tuple[str, ...]) -> str:
    if len(values) < 3:
        return "N/A"
    if any(value == "UNKNOWN" for value in values[:3]):
        return "N/A"
    return f"{values[0]} V / {float(values[1]):+.3f} A / {values[2]}%"


def parse_current_draw(values: tuple[str, ...]) -> str:
    if not values:
        return "N/A"
    if values[0] == "UNKNOWN":
        return "N/A"
    try:
        return f"{float(values[0]):+.3f} A"
    except ValueError:
        return "N/A"


def parse_science_value(values: tuple[str, ...], unit: str) -> str:
    if not values or values[0] == "UNKNOWN":
        return "N/A"
    return f"{values[0]} {unit}".strip()


def _parse_float_prefix(value: str) -> float | None:
    if not value or value == "N/A" or value == "Unknown":
        return None
    token = value.split()[0].strip().rstrip(",")
    try:
        return float(token)
    except ValueError:
        return None


def parse_location_pair(location_text: str) -> tuple[float, float] | None:
    if not location_text or location_text in {"N/A", "Unknown"}:
        return None

    parts = [part.strip() for part in location_text.split(",")]
    if len(parts) != 2:
        return None

    try:
        return float(parts[0]), float(parts[1])
    except ValueError:
        return None


def parse_science_sample(
    telemetry: TelemetryState,
    timestamp: float,
) -> ScienceSample | None:
    location = parse_location_pair(telemetry.current_location)
    depth_m = _parse_float_prefix(telemetry.current_depth)
    water_temperature_c = _parse_float_prefix(telemetry.water_temperature)
    audio_level = telemetry.audio_level.rstrip("%").strip()

    if location is None or depth_m is None:
        return None

    try:
        audio_level_percent = float(audio_level)
    except ValueError:
        return None

    latitude, longitude = location
    return ScienceSample(
        timestamp=timestamp,
        latitude=latitude,
        longitude=longitude,
        depth_m=depth_m,
        audio_level_percent=audio_level_percent,
        water_temperature_c=water_temperature_c,
    )


def parse_vector3(values: tuple[str, ...], unit: str) -> str:
    if len(values) < 3 or any(value == "UNKNOWN" for value in values[:3]):
        return "N/A"
    return f"{values[0]}, {values[1]}, {values[2]} {unit}".strip()


def format_udp_battery_status(voltage: float, current: float, percent: int) -> str:
    return f"{voltage:.3f} V / {current:+.3f} A / {percent}%"


def format_udp_position(latitude: float, longitude: float) -> str:
    return f"{latitude:.6f}, {longitude:.6f}"


def decode_control_mode(mode_value: int) -> str:
    return {
        0: "idle",
        1: "manual",
        2: "hold",
        3: "goto",
    }.get(mode_value, "unknown")


def decode_pcm_samples(pcm_bytes: bytes) -> array:
    samples = array("h")
    samples.frombytes(pcm_bytes)
    if sys.byteorder != "little":
        samples.byteswap()
    return samples


def update_audio_waveform_buffer(
    waveform_buffer: deque[float],
    samples: array,
    channels: int,
) -> float:
    if channels <= 0 or len(samples) == 0:
        return 0.0

    # Build mono samples from interleaved channels and track amplitude.
    mono_samples: list[float] = []
    peak = 0.0
    for index in range(0, len(samples), channels):
        frame = samples[index:index + channels]
        if not frame:
            continue
        sample = sum(frame) / len(frame)
        sample_abs = abs(sample)
        mono_samples.append(sample)
        if sample_abs > peak:
            peak = sample_abs

    if not mono_samples:
        return 0.0

    # Remove DC bias and apply adaptive gain so low-level mic activity is visible.
    dc_offset = sum(mono_samples) / len(mono_samples)
    centered_peak = 0.0
    centered_samples: list[float] = []
    for sample in mono_samples:
        centered = sample - dc_offset
        centered_samples.append(centered)
        sample_abs = abs(centered)
        if sample_abs > centered_peak:
            centered_peak = sample_abs

    normalized_peak = centered_peak / 32768.0
    gain = 1.0
    if centered_peak > 1.0:
        target_peak = 0.75
        gain = min(12.0, target_peak / max(normalized_peak, 1e-6))

    for sample in centered_samples:
        value = (sample / 32768.0) * gain
        waveform_buffer.append(max(-1.0, min(1.0, value)))

    return normalized_peak


def read_arrow_key_axes() -> tuple[float, float]:
    keys = pygame.key.get_pressed()
    turn = 0.0
    thrust = 0.0

    if keys[pygame.K_LEFT]:
        turn -= 1.0
    if keys[pygame.K_RIGHT]:
        turn += 1.0
    if keys[pygame.K_UP]:
        thrust += 1.0
    if keys[pygame.K_DOWN]:
        thrust -= 1.0

    return turn, thrust


def run() -> None:
    args = parse_args()
    app_dir = Path(__file__).resolve().parent.parent
    log_dir = app_dir / "logs"
    log_dir.mkdir(parents=True, exist_ok=True)
    timestamp = time.strftime("%Y%m%d-%H%M%S")
    log_file = Path(args.comm_log_file) if args.comm_log_file else (log_dir / f"comm-{timestamp}.log")
    prefs_path = log_dir / CONNECTION_PREFS_FILENAME
    apply_connection_preferences(args, prefs_path)

    pygame.mixer.pre_init(AUDIO_SAMPLE_RATE, -16, AUDIO_CHANNELS, 512)
    pygame.init()
    pygame.joystick.init()
    pygame.font.init()
    mixer_ready = pygame.mixer.get_init() is not None
    if not mixer_ready:
        try:
            pygame.mixer.init(frequency=AUDIO_SAMPLE_RATE, size=-16, channels=AUDIO_CHANNELS, buffer=512)
            mixer_ready = True
        except pygame.error:
            mixer_ready = False

    screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
    pygame.display.set_caption("Controller Debug" if args.debug_controller else "Buoy Control Station")
    clock = pygame.time.Clock()

    title_font = pygame.font.SysFont("arial", 30, bold=True)
    body_font = pygame.font.SysFont("arial", 22)
    small_font = pygame.font.SysFont("arial", 16)
    table_title_font = pygame.font.SysFont("arial", 20, bold=True)
    table_caption_font = pygame.font.SysFont("arial", 16)
    table_value_font = pygame.font.SysFont("arial", 14)

    joystick = get_controller()
    serial_link, serial_status, serial_target = open_serial_connection(
        args.port,
        args.baudrate,
        args.device_name,
        use_ble=(args.transport == "ble"),
        use_wifi=(args.transport == "wifi"),
        wifi_host=args.wifi_host,
        local_ip=args.wifi_local_ip,
        tcp_port=args.tcp_port,
        udp_port=args.udp_port,
        audio_port=args.audio_port,
        ble_service_uuid=args.ble_service_uuid,
        ble_characteristic_uuid=args.ble_characteristic_uuid,
    )
    connected_device_name = None
    if serial_link:
        if args.transport == "wifi":
            connected_device_name = args.wifi_host
            serial_status = f"{serial_status} ({args.wifi_host})"
            save_connection_preferences(
                prefs_path,
                wifi_host=extract_wifi_host_from_target(serial_target) or args.wifi_host,
                wifi_local_ip=args.wifi_local_ip if args.wifi_local_ip and args.wifi_local_ip.lower() != "auto" else None,
            )
            send_text(serial_link, "CTRL STOP")
        elif args.device_name:
            connected_device_name = args.device_name
            serial_status = f"{serial_status} ({args.device_name})"
    last_send_time = 0.0
    last_sent_line = "Nothing sent yet"
    last_send_result = "Waiting for first command"
    last_command = build_manual_command(0.0, 0.0)
    last_hello_time = 0.0
    zero_send_count = 0
    command_input = ""
    last_response = "No response yet"
    telemetry = TelemetryState(
        control_mode="idle",
        current_location="Unknown",
        target_location="Not set",
        hold_position=False,
        battery_status="N/A",
        current_draw="N/A",
        current_depth="N/A",
        water_temperature="N/A",
        air_temperature="N/A",
    )
    ack_vector = "N/A"
    comm_log = []
    last_connection_check = time.time()
    last_reconnect_attempt = 0.0
    reconnect_interval = 2.0
    last_ping_time = 0.0
    ping_interval = 5.0
    ping_sent = False
    link_timeout = 3.0
    last_protocol_response_time = time.time() if serial_link is not None else 0.0
    last_protocol_send_time = 0.0
    awaiting_protocol_response = False
    rx_buffer = bytearray()
    imu_last_sequence: int | None = None
    imu_received_count = 0
    imu_missing_count = 0
    audio_muted = False
    audio_waveform_buffer: deque[float] = deque([0.0] * AUDIO_WAVEFORM_SAMPLES, maxlen=AUDIO_WAVEFORM_SAMPLES)
    science_history: deque[ScienceSample] = deque(maxlen=180)
    dashboard_tab = "logs"
    last_science_sample_time = 0.0
    audio_channel = pygame.mixer.Channel(0) if mixer_ready else None
    running = True

    append_comm_log(comm_log, log_file, f"{time.time()}: COMM LOG START file={log_file}")

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if event.type == pygame.JOYDEVICEADDED and joystick is None:
                joystick = get_controller()
            if event.type == pygame.JOYDEVICEREMOVED and joystick is not None:
                joystick.quit()
                joystick = get_controller()
            if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1 and not args.debug_controller:
                layout_rects = get_main_layout_rects()
                audio_button_rect = get_audio_mute_button_rect(layout_rects["science"])
                logs_tab_rect, analysis_tab_rect = get_dashboard_tab_rects(layout_rects["dashboard"])
                if audio_button_rect.collidepoint(event.pos):
                    audio_muted = not audio_muted
                    if audio_muted and audio_channel is not None:
                        audio_channel.stop()
                elif logs_tab_rect.collidepoint(event.pos):
                    dashboard_tab = "logs"
                elif analysis_tab_rect.collidepoint(event.pos):
                    dashboard_tab = "analysis"
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_RETURN:
                    if command_input.strip():
                        last_send_result = send_text(serial_link, command_input.strip())
                        last_sent_line = command_input.strip()
                        if serial_link is not None:
                            outgoing = command_input.strip().encode("ascii", errors="replace") + b"\n"
                            append_comm_log(comm_log, log_file, f"{time.time()}: TX RAW {format_bytes_for_log(outgoing)}")
                        command_input = ""
                elif event.key == pygame.K_BACKSPACE:
                    command_input = command_input[:-1]
                elif event.key == pygame.K_ESCAPE:
                    command_input = ""
                else:
                    if event.unicode and event.unicode.isprintable():
                        command_input += event.unicode

        current_time = time.time()
        if serial_link is None and (current_time - last_reconnect_attempt) >= reconnect_interval:
            serial_link, reopened_status, reopened_target = open_serial_connection(
                args.port,
                args.baudrate,
                args.device_name,
                use_ble=(args.transport == "ble"),
                use_wifi=(args.transport == "wifi"),
                wifi_host=args.wifi_host,
                local_ip=args.wifi_local_ip,
                tcp_port=args.tcp_port,
                udp_port=args.udp_port,
                audio_port=args.audio_port,
                ble_service_uuid=args.ble_service_uuid,
                ble_characteristic_uuid=args.ble_characteristic_uuid,
            )
            last_reconnect_attempt = current_time
            if serial_link is not None:
                serial_status = reopened_status
                serial_target = reopened_target
                if args.transport == "wifi":
                    connected_device_name = args.wifi_host
                    save_connection_preferences(
                        prefs_path,
                        wifi_host=extract_wifi_host_from_target(serial_target) or args.wifi_host,
                        wifi_local_ip=args.wifi_local_ip if args.wifi_local_ip and args.wifi_local_ip.lower() != "auto" else None,
                    )
                    send_text(serial_link, "CTRL STOP")
                else:
                    connected_device_name = args.device_name if args.device_name else None
                serial_status = "Reconnecting" if connected_device_name else reopened_status
                rx_buffer.clear()
                ping_sent = False
                last_protocol_response_time = current_time
                last_protocol_send_time = 0.0
                awaiting_protocol_response = False
                append_comm_log(comm_log, log_file, f"{time.time()}: RECONNECTED target={serial_target}")
            else:
                serial_status = "Reconnecting"
                serial_target = reopened_target

        if current_time - last_connection_check > 10:
            if serial_link and serial_link.is_open:
                recent_response = (current_time - last_protocol_response_time) <= max(link_timeout, ping_interval + 1.0)
                if recent_response and not awaiting_protocol_response:
                    serial_status = "Connected"
                    if connected_device_name:
                        serial_status = f"Connected ({connected_device_name})"
                else:
                    serial_status = "Reconnecting" if args.device_name else "Disconnected"
            else:
                serial_status = "Reconnecting" if args.device_name else "Disconnected"
                serial_link = None
                ack_vector = "N/A"
                telemetry = TelemetryState(
                    control_mode=telemetry.control_mode,
                    current_location=telemetry.current_location,
                    target_location=telemetry.target_location,
                    hold_position=telemetry.hold_position,
                    battery_status=telemetry.battery_status,
                    current_draw="N/A",
                    current_depth=telemetry.current_depth,
                    water_temperature=telemetry.water_temperature,
                    air_temperature=telemetry.air_temperature,
                )
            last_connection_check = current_time

        turn, thrust = read_axes(joystick, args.deadzone)
        key_turn, key_thrust = read_arrow_key_axes()
        if key_turn != 0.0 or key_thrust != 0.0:
            turn, thrust = key_turn, key_thrust
        controller_snapshot = read_controller_snapshot(joystick, args.deadzone)
        command = build_manual_command(turn, thrust)
        now = time.monotonic()
        min_send_interval = 1.0 / max(args.send_rate, 0.1)

        is_zero = (command.turn == 0 and command.thrust == 0)
        if not is_zero:
            zero_send_count = 0

        should_send_keepalive_ping = (
            serial_link is not None
            and not args.hello_ping
            and not args.send_text
            and is_zero
            and not awaiting_protocol_response
        )
        if should_send_keepalive_ping and current_time - last_ping_time > ping_interval:
            result = send_text(serial_link, "PING")
            if "failed" not in result.lower():
                ping_sent = True
                last_ping_time = current_time
                last_protocol_send_time = current_time
                awaiting_protocol_response = True
                append_comm_log(
                    comm_log,
                    log_file,
                    f"{time.time()}: TX RAW {format_bytes_for_log(PING_BYTES)}",
                )

        if args.hello_ping or args.send_text:
            hello_interval = max(args.send_interval, 0.1)
            if (now - last_hello_time) >= hello_interval:
                ping_text = "hello world" if args.hello_ping else args.send_text
                last_send_result = send_text(serial_link, ping_text)
                last_hello_time = now
                last_sent_line = ping_text
                if serial_link is not None:
                    outgoing = ping_text.encode("ascii", errors="replace") + b"\n"
                    append_comm_log(comm_log, log_file, f"{time.time()}: TX RAW {format_bytes_for_log(outgoing)}")
                if "failed" not in last_send_result.lower():
                    last_protocol_send_time = current_time
                    awaiting_protocol_response = True
                if "failed" in last_send_result.lower() and serial_status == "Connected":
                    serial_status = "Disconnected"
                    serial_link = None
                    ack_vector = "N/A"
                    telemetry = TelemetryState(
                        control_mode=telemetry.control_mode,
                        current_location=telemetry.current_location,
                        target_location=telemetry.target_location,
                        hold_position=telemetry.hold_position,
                        battery_status=telemetry.battery_status,
                        current_draw="N/A",
                        current_depth=telemetry.current_depth,
                        water_temperature=telemetry.water_temperature,
                        air_temperature=telemetry.air_temperature,
                    )
                if last_send_result == "No serial link" and serial_status == "Connected":
                    serial_status = "Disconnected"
                    serial_link = None
                    ack_vector = "N/A"
                    telemetry = TelemetryState(
                        control_mode=telemetry.control_mode,
                        current_location=telemetry.current_location,
                        target_location=telemetry.target_location,
                        hold_position=telemetry.hold_position,
                        battery_status=telemetry.battery_status,
                        current_draw="N/A",
                        current_depth=telemetry.current_depth,
                        water_temperature=telemetry.water_temperature,
                        air_temperature=telemetry.air_temperature,
                    )
        else:
            send_command_flag = False
            if command != last_command:
                send_command_flag = True
            elif (now - last_send_time) >= min_send_interval:
                if not is_zero:
                    send_command_flag = True

            if send_command_flag:
                last_send_result = send_command(serial_link, command)
                last_send_time = now
                last_command = command
                last_sent_line = command.to_line().strip()
                if serial_link is not None:
                    outgoing = command.to_line().encode("ascii", errors="replace")
                    append_comm_log(comm_log, log_file, f"{time.time()}: TX RAW {format_bytes_for_log(outgoing)}")
                if "failed" not in last_send_result.lower():
                    last_protocol_send_time = current_time
                    awaiting_protocol_response = True
                ack_vector = f"{command.turn},{command.thrust}"
                if "failed" in last_send_result.lower() and serial_status == "Connected":
                    serial_status = "Disconnected"
                    serial_link = None
                    ack_vector = "N/A"
                    telemetry = TelemetryState(
                        control_mode=telemetry.control_mode,
                        current_location=telemetry.current_location,
                        target_location=telemetry.target_location,
                        hold_position=telemetry.hold_position,
                        battery_status=telemetry.battery_status,
                        current_draw="N/A",
                        current_depth=telemetry.current_depth,
                        water_temperature=telemetry.water_temperature,
                        air_temperature=telemetry.air_temperature,
                    )
                if last_send_result == "No serial link" and serial_status == "Connected":
                    serial_status = "Disconnected"
                    serial_link = None
                    ack_vector = "N/A"
                    telemetry = TelemetryState(
                        control_mode=telemetry.control_mode,
                        current_location=telemetry.current_location,
                        target_location=telemetry.target_location,
                        hold_position=telemetry.hold_position,
                        battery_status=telemetry.battery_status,
                        current_draw="N/A",
                        current_depth=telemetry.current_depth,
                        water_temperature=telemetry.water_temperature,
                        air_temperature=telemetry.air_temperature,
                    )

        incoming_bytes = read_available_bytes(serial_link)
        if incoming_bytes:
            append_comm_log(comm_log, log_file, f"{time.time()}: RX RAW {format_bytes_for_log(incoming_bytes)}")

            rx_buffer.extend(incoming_bytes)
            while True:
                newline_index = rx_buffer.find(b"\n")
                if newline_index == -1:
                    break

                line_bytes = bytes(rx_buffer[:newline_index]).rstrip(b"\r")
                del rx_buffer[:newline_index + 1]
                response_text = line_bytes.decode("ascii", errors="replace")
                if not response_text:
                    continue

                last_response = response_text
                append_comm_log(comm_log, log_file, f"{time.time()}: RX TXT {response_text}")
                ack = parse_acknowledgement(response_text)
                if ack is not None:
                    ack_kind, ack_values = ack
                    if ack_kind == "CTRL" and len(ack_values) >= 3 and ack_values[0] == "VECTOR":
                        ack_vector = f"{ack_values[1]},{ack_values[2]}"
                    elif ack_kind == "CTRL" and len(ack_values) >= 2 and ack_values[0] == "HOLD":
                        telemetry = TelemetryState(
                            control_mode=telemetry.control_mode,
                            current_location=telemetry.current_location,
                            target_location=telemetry.target_location,
                            hold_position=(ack_values[1] == "ON"),
                            battery_status=telemetry.battery_status,
                            current_draw=telemetry.current_draw,
                            current_depth=telemetry.current_depth,
                            water_temperature=telemetry.water_temperature,
                            air_temperature=telemetry.air_temperature,
                        )

                status_update = parse_status_update(response_text)
                if status_update is not None:
                    status_key, status_values = status_update
                    if status_key == "MODE" and status_values:
                        telemetry = TelemetryState(
                            control_mode=status_values[0].lower(),
                            current_location=telemetry.current_location,
                            target_location=telemetry.target_location,
                            hold_position=telemetry.hold_position,
                            battery_status=telemetry.battery_status,
                            current_draw=telemetry.current_draw,
                            current_depth=telemetry.current_depth,
                            water_temperature=telemetry.water_temperature,
                            air_temperature=telemetry.air_temperature,
                        )
                    elif status_key == "POS":
                        telemetry = TelemetryState(
                            control_mode=telemetry.control_mode,
                            current_location=format_position(status_values),
                            target_location=telemetry.target_location,
                            hold_position=telemetry.hold_position,
                            battery_status=telemetry.battery_status,
                            current_draw=telemetry.current_draw,
                            current_depth=telemetry.current_depth,
                            water_temperature=telemetry.water_temperature,
                            air_temperature=telemetry.air_temperature,
                        )
                    elif status_key == "TARGET":
                        telemetry = TelemetryState(
                            control_mode=telemetry.control_mode,
                            current_location=telemetry.current_location,
                            target_location=format_position(status_values),
                            hold_position=telemetry.hold_position,
                            battery_status=telemetry.battery_status,
                            current_draw=telemetry.current_draw,
                            current_depth=telemetry.current_depth,
                            water_temperature=telemetry.water_temperature,
                            air_temperature=telemetry.air_temperature,
                        )
                    elif status_key == "HOLD" and status_values:
                        telemetry = TelemetryState(
                            control_mode=telemetry.control_mode,
                            current_location=telemetry.current_location,
                            target_location=telemetry.target_location,
                            hold_position=(status_values[0] == "ON"),
                            battery_status=telemetry.battery_status,
                            current_draw=telemetry.current_draw,
                            current_depth=telemetry.current_depth,
                            water_temperature=telemetry.water_temperature,
                            air_temperature=telemetry.air_temperature,
                        )
                    elif status_key == "BATTERY":
                        telemetry = TelemetryState(
                            control_mode=telemetry.control_mode,
                            current_location=telemetry.current_location,
                            target_location=telemetry.target_location,
                            hold_position=telemetry.hold_position,
                            battery_status=parse_battery_status(status_values),
                            current_draw=telemetry.current_draw,
                            current_depth=telemetry.current_depth,
                            water_temperature=telemetry.water_temperature,
                            air_temperature=telemetry.air_temperature,
                        )
                    elif status_key == "CURRENT":
                        telemetry = TelemetryState(
                            control_mode=telemetry.control_mode,
                            current_location=telemetry.current_location,
                            target_location=telemetry.target_location,
                            hold_position=telemetry.hold_position,
                            battery_status=telemetry.battery_status,
                            current_draw=parse_current_draw(status_values),
                            current_depth=telemetry.current_depth,
                            water_temperature=telemetry.water_temperature,
                            air_temperature=telemetry.air_temperature,
                        )

                science_update = parse_science_update(response_text)
                if science_update is not None:
                    science_key, science_values = science_update
                    if science_key == "WATER_TEMP":
                        telemetry = TelemetryState(
                            control_mode=telemetry.control_mode,
                            current_location=telemetry.current_location,
                            target_location=telemetry.target_location,
                            hold_position=telemetry.hold_position,
                            battery_status=telemetry.battery_status,
                            current_draw=telemetry.current_draw,
                            current_depth=telemetry.current_depth,
                            water_temperature=parse_science_value(science_values, "C"),
                            air_temperature=telemetry.air_temperature,
                        )
                    elif science_key == "AIR_TEMP":
                        telemetry = TelemetryState(
                            control_mode=telemetry.control_mode,
                            current_location=telemetry.current_location,
                            target_location=telemetry.target_location,
                            hold_position=telemetry.hold_position,
                            battery_status=telemetry.battery_status,
                            current_draw=telemetry.current_draw,
                            current_depth=telemetry.current_depth,
                            water_temperature=telemetry.water_temperature,
                            air_temperature=parse_science_value(science_values, "C"),
                        )
                    elif science_key == "DEPTH":
                        telemetry = TelemetryState(
                            control_mode=telemetry.control_mode,
                            current_location=telemetry.current_location,
                            target_location=telemetry.target_location,
                            hold_position=telemetry.hold_position,
                            battery_status=telemetry.battery_status,
                            current_draw=telemetry.current_draw,
                            current_depth=parse_science_value(science_values, "m"),
                            water_temperature=telemetry.water_temperature,
                            air_temperature=telemetry.air_temperature,
                        )
                    elif science_key == "IMU_ACCEL":
                        telemetry = TelemetryState(
                            control_mode=telemetry.control_mode,
                            current_location=telemetry.current_location,
                            target_location=telemetry.target_location,
                            hold_position=telemetry.hold_position,
                            battery_status=telemetry.battery_status,
                            current_draw=telemetry.current_draw,
                            current_depth=telemetry.current_depth,
                            water_temperature=telemetry.water_temperature,
                            air_temperature=telemetry.air_temperature,
                            imu_accel=parse_vector3(science_values, "g"),
                            imu_gyro=telemetry.imu_gyro,
                            imu_temperature=telemetry.imu_temperature,
                        )
                    elif science_key == "IMU_GYRO":
                        telemetry = TelemetryState(
                            control_mode=telemetry.control_mode,
                            current_location=telemetry.current_location,
                            target_location=telemetry.target_location,
                            hold_position=telemetry.hold_position,
                            battery_status=telemetry.battery_status,
                            current_draw=telemetry.current_draw,
                            current_depth=telemetry.current_depth,
                            water_temperature=telemetry.water_temperature,
                            air_temperature=telemetry.air_temperature,
                            imu_accel=telemetry.imu_accel,
                            imu_gyro=parse_vector3(science_values, "dps"),
                            imu_temperature=telemetry.imu_temperature,
                        )
                    elif science_key == "IMU_TEMP":
                        telemetry = TelemetryState(
                            control_mode=telemetry.control_mode,
                            current_location=telemetry.current_location,
                            target_location=telemetry.target_location,
                            hold_position=telemetry.hold_position,
                            battery_status=telemetry.battery_status,
                            current_draw=telemetry.current_draw,
                            current_depth=telemetry.current_depth,
                            water_temperature=telemetry.water_temperature,
                            air_temperature=telemetry.air_temperature,
                            imu_accel=telemetry.imu_accel,
                            imu_gyro=telemetry.imu_gyro,
                            imu_temperature=parse_science_value(science_values, "C"),
                        )

                if is_protocol_message(response_text):
                    last_protocol_response_time = current_time
                    awaiting_protocol_response = False
                    serial_status = "Connected"
                    if connected_device_name:
                        serial_status = f"Connected ({connected_device_name})"
                    ping_sent = False
                error_text = parse_error(response_text)
                if error_text is not None:
                    last_send_result = f"ERR {error_text}"

        telemetry_packets = read_telemetry_packets(serial_link)
        for packet in telemetry_packets:
            last_protocol_response_time = current_time
            serial_status = "Connected"
            if connected_device_name:
                serial_status = f"Connected ({connected_device_name})"

            if packet.packet_type == "IMU":
                if imu_last_sequence is not None:
                    expected_next = (imu_last_sequence + 1) & 0xFFFF
                    if packet.sequence != expected_next:
                        gap = (packet.sequence - expected_next) & 0xFFFF
                        imu_missing_count += gap
                imu_last_sequence = packet.sequence
                imu_received_count += 1
                total_imu = imu_received_count + imu_missing_count
                loss_pct = (imu_missing_count / total_imu * 100.0) if total_imu > 0 else 0.0

                ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps, temperature_c = packet.values
                telemetry = replace(
                    telemetry,
                    imu_accel=f"{ax_g:.3f}, {ay_g:.3f}, {az_g:.3f} g",
                    imu_gyro=f"{gx_dps:.1f}, {gy_dps:.1f}, {gz_dps:.1f} dps",
                    imu_temperature=f"{temperature_c:.2f} C",
                    imu_udp_loss=f"{loss_pct:.1f}% ({imu_missing_count}/{total_imu})",
                )
            elif packet.packet_type == "POWER":
                battery_volts, current_amps, battery_pct = packet.values
                telemetry = replace(
                    telemetry,
                    battery_status=format_udp_battery_status(battery_volts, current_amps, battery_pct),
                    current_draw=f"{current_amps:+.3f} A",
                )
            elif packet.packet_type == "STATE":
                mode_value, hold_enabled, gps_valid, _motors_enabled = packet.values
                current_location = telemetry.current_location if gps_valid else "Unknown"
                telemetry = replace(
                    telemetry,
                    control_mode=decode_control_mode(mode_value),
                    hold_position=hold_enabled,
                    current_location=current_location,
                )
            elif packet.packet_type == "GPS":
                latitude, longitude = packet.values
                telemetry = replace(
                    telemetry,
                    current_location=format_udp_position(latitude, longitude),
                )
            elif packet.packet_type == "RANGE":
                distance_mm, valid = packet.values
                telemetry = replace(
                    telemetry,
                    current_depth=(f"{distance_mm / 1000.0:.3f} m" if valid else "N/A"),
                )
            elif packet.packet_type == "AUDIO":
                pcm_bytes, _sample_count, channels = packet.values
                samples = decode_pcm_samples(pcm_bytes)
                peak = update_audio_waveform_buffer(audio_waveform_buffer, samples, channels)
                telemetry = replace(
                    telemetry,
                    audio_stream=(
                        f"{AUDIO_SAMPLE_RATE / 1000.0:.1f} kHz stereo"
                        if channels == 2
                        else f"{AUDIO_SAMPLE_RATE / 1000.0:.1f} kHz mono"
                    ),
                    audio_level=f"{peak * 100.0:.1f}%",
                )

                if not audio_muted and mixer_ready and audio_channel is not None:
                    try:
                        if not audio_channel.get_busy():
                            sound = pygame.mixer.Sound(buffer=pcm_bytes)
                            audio_channel.play(sound)
                    except pygame.error:
                        pass

        if current_time - last_science_sample_time >= 0.5:
            science_sample = parse_science_sample(telemetry, current_time)
            if science_sample is not None:
                science_history.append(science_sample)
                last_science_sample_time = current_time

        if serial_link is not None and awaiting_protocol_response and (current_time - last_protocol_send_time) > link_timeout:
            append_comm_log(comm_log, log_file, f"{time.time()}: LINK TIMEOUT closing serial handle")
            serial_status = "Reconnecting" if args.device_name else "Disconnected"
            try:
                serial_link.close()
            except Exception:
                pass
            serial_link = None
            rx_buffer.clear()
            ack_vector = "N/A"
            telemetry = TelemetryState(
                control_mode=telemetry.control_mode,
                current_location=telemetry.current_location,
                target_location=telemetry.target_location,
                hold_position=telemetry.hold_position,
                battery_status=telemetry.battery_status,
                current_draw="N/A",
                current_depth=telemetry.current_depth,
                water_temperature=telemetry.water_temperature,
                air_temperature=telemetry.air_temperature,
            )
            awaiting_protocol_response = False
            ping_sent = False

        if ping_sent and current_time - last_ping_time > 3.0:
            serial_status = "Reconnecting" if args.device_name else "Disconnected"
            try:
                if serial_link is not None:
                    serial_link.close()
            except Exception:
                pass
            serial_link = None
            rx_buffer.clear()
            ack_vector = "N/A"
            telemetry = TelemetryState(
                control_mode=telemetry.control_mode,
                current_location=telemetry.current_location,
                target_location=telemetry.target_location,
                hold_position=telemetry.hold_position,
                battery_status=telemetry.battery_status,
                current_draw="N/A",
                current_depth=telemetry.current_depth,
                water_temperature=telemetry.water_temperature,
                air_temperature=telemetry.air_temperature,
            )
            ping_sent = False
            awaiting_protocol_response = False

        state = build_runtime_state(
            joystick=joystick,
            controller_snapshot=controller_snapshot,
            serial_status=serial_status,
            serial_target=serial_target,
            last_send_result=last_send_result,
            last_sent_line=last_sent_line,
            command=command,
            command_input=command_input,
            last_response=last_response,
            telemetry=telemetry,
            ack_vector=ack_vector,
            comm_log=comm_log,
            audio_muted=audio_muted,
            audio_waveform=tuple(audio_waveform_buffer),
        )

        if args.debug_controller:
            render_debug_interface(
                screen=screen,
                title_font=title_font,
                body_font=body_font,
                small_font=small_font,
                state=state,
                turn=turn,
                thrust=thrust,
                baudrate=args.baudrate,
            )
        else:
            render_main_interface(
                screen=screen,
                title_font=title_font,
                body_font=body_font,
                small_font=small_font,
                table_title_font=table_title_font,
                table_caption_font=table_caption_font,
                table_value_font=table_value_font,
                state=state,
                dashboard_tab=dashboard_tab,
                science_history=tuple(science_history),
                turn=turn,
                thrust=thrust,
            )

        pygame.display.flip()
        clock.tick(30)

    if joystick is not None:
        joystick.quit()
    if serial_link is not None:
        serial_link.close()
    if mixer_ready:
        pygame.mixer.quit()
    append_comm_log(comm_log, log_file, f"{time.time()}: COMM LOG END")
    pygame.quit()


