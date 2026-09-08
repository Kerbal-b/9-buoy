from __future__ import annotations

import argparse
from array import array
from collections import deque
from dataclasses import asdict
from pathlib import Path
import sys
import time
import threading

try:
    import pygame
except ModuleNotFoundError:
    pygame = None
from PySide6.QtCore import QObject, Property, QTimer, Signal, Slot, QUrl
from PySide6.QtGui import QGuiApplication
from PySide6.QtQml import QQmlApplicationEngine

from .controller import get_controller, read_axes
from .geometry import build_manual_command
from .models import ScienceSample
from .protocol import is_protocol_message, parse_acknowledgement, parse_error, parse_science_update, parse_status_update
from .serial_link import (
    CONNECTION_PREFS_FILENAME,
    WIFI_DEFAULT_AUDIO_PORT,
    WIFI_DEFAULT_HOST,
    WIFI_DEFAULT_TCP_PORT,
    WIFI_DEFAULT_UDP_PORT,
    apply_connection_preferences,
    extract_wifi_host_from_target,
    list_wifi_interfaces,
    open_serial_connection,
    read_available_bytes,
    read_telemetry_packets,
    save_connection_preferences,
    send_command,
    send_text,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Laptop control station for manual buoy control.")
    parser.add_argument("--port", help="Serial port for the Bluetooth link, for example /dev/tty.HC-05-DevB")
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
    parser.add_argument("--wifi-host", default=WIFI_DEFAULT_HOST, help="ESP32 Wi-Fi host or IP address")
    parser.add_argument(
        "--wifi-local-ip",
        "--source-ip",
        dest="wifi_local_ip",
        default="auto",
        help="Local IPv4 address used to select the source interface",
    )
    parser.add_argument("--tcp-port", type=int, default=WIFI_DEFAULT_TCP_PORT, help="TCP port used for reliable command/control")
    parser.add_argument("--udp-port", type=int, default=WIFI_DEFAULT_UDP_PORT, help="Local UDP port used for fast telemetry")
    parser.add_argument("--audio-port", type=int, default=WIFI_DEFAULT_AUDIO_PORT, help="Local UDP port used for streamed audio")
    parser.add_argument(
        "--ble-service-uuid",
        default="0000ffe0-0000-1000-8000-00805f9b34fb",
        help="HM-10 BLE service UUID",
    )
    parser.add_argument(
        "--ble-characteristic-uuid",
        default="0000ffe1-0000-1000-8000-00805f9b34fb",
        help="HM-10 BLE characteristic UUID used for write and notify",
    )
    parser.add_argument("--baudrate", type=int, default=9600, help="Serial baudrate used by the serial link")
    parser.add_argument("--deadzone", type=float, default=0.12, help="Ignore small joystick movement around center")
    parser.add_argument("--send-rate", type=float, default=10.0, help="How many command updates to send per second")
    parser.add_argument("--hello-ping", action="store_true", help="Send hello world text messages over the active transport instead of CTRL commands")
    parser.add_argument("--send-text", help="Send this text over the active transport instead of CTRL commands")
    parser.add_argument("--send-interval", type=float, default=1.0, help="Seconds between repeated text messages when --send-text or --hello-ping is enabled")
    parser.add_argument("--debug-controller", action="store_true", help="Open the controller diagnostics window instead of the main interface")
    parser.add_argument("--comm-log-file", help="Write raw transport communication logs to this file")
    return parser.parse_args()


def _parse_float_prefix(value: str) -> float | None:
    if not value or value in {"N/A", "Unknown"}:
        return None
    token = value.split()[0].strip().rstrip(",")
    try:
        return float(token)
    except ValueError:
        return None


def _parse_location_pair(location_text: str) -> tuple[float, float] | None:
    if not location_text or location_text in {"N/A", "Unknown"}:
        return None
    parts = [part.strip() for part in location_text.split(",")]
    if len(parts) != 2:
        return None
    try:
        return float(parts[0]), float(parts[1])
    except ValueError:
        return None


def _parse_science_sample(state: dict[str, object], timestamp: float) -> ScienceSample | None:
    location = _parse_location_pair(str(state.get("currentLocation", "")))
    depth_m = _parse_float_prefix(str(state.get("currentDepth", "")))
    water_temperature_c = _parse_float_prefix(str(state.get("waterTemperature", "")))
    audio_level_text = str(state.get("audioLevel", ""))

    if location is None or depth_m is None:
        return None

    try:
        audio_level_percent = float(audio_level_text.rstrip("%").strip())
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


def _parse_vector3_text(text: str, unit: str) -> str:
    if not text or text == "N/A":
        return "N/A"
    parts = [part.strip() for part in text.replace(",", " ").split() if part.strip()]
    if len(parts) < 3:
        return "N/A"
    return f"{parts[0]}, {parts[1]}, {parts[2]} {unit}"


def _parse_battery_status(text: str) -> str:
    if not text or text == "N/A":
        return "N/A"
    return text


def _format_udp_battery_status(voltage: float, current: float, percent: int) -> str:
    return f"{voltage:.3f} V / {current:.3f} A / {percent}%"


def _format_udp_position(latitude: float, longitude: float) -> str:
    return f"{latitude:.6f}, {longitude:.6f}"


def _decode_control_mode(mode_value: int) -> str:
    return {
        0: "idle",
        1: "manual",
        2: "hold",
        3: "goto",
    }.get(mode_value, "unknown")


def _decode_pcm_samples(pcm_bytes: bytes) -> array:
    samples = array("h")
    samples.frombytes(pcm_bytes)
    if sys.byteorder != "little":
        samples.byteswap()
    return samples


def _update_audio_waveform_buffer(waveform_buffer: deque[float], samples: array, channels: int) -> float:
    if channels <= 0 or len(samples) == 0:
        return 0.0

    mono_samples: list[float] = []
    centered_samples: list[float] = []
    peak = 0.0
    for index in range(0, len(samples), channels):
        frame = samples[index:index + channels]
        if not frame:
            continue
        sample = sum(frame) / len(frame)
        mono_samples.append(sample)
        peak = max(peak, abs(sample))

    if not mono_samples:
        return 0.0

    dc_offset = sum(mono_samples) / len(mono_samples)
    centered_peak = 0.0
    for sample in mono_samples:
        centered = sample - dc_offset
        centered_samples.append(centered)
        centered_peak = max(centered_peak, abs(centered))

    normalized_peak = centered_peak / 32768.0
    gain = 1.0
    if centered_peak > 1.0:
        target_peak = 0.75
        gain = min(12.0, target_peak / max(normalized_peak, 1e-6))

    for sample in centered_samples:
        value = (sample / 32768.0) * gain
        waveform_buffer.append(max(-1.0, min(1.0, value)))

    return normalized_peak


def _clamp_axis(value: float) -> float:
    return max(-1.0, min(1.0, value))


class ControlStationBackend(QObject):
    stateChanged = Signal()
    dashboardTabChanged = Signal()
    audioMutedChanged = Signal()

    def __init__(self, args: argparse.Namespace, log_file: Path, prefs_path: Path) -> None:
        super().__init__()
        self._args = args
        self._log_file = log_file
        self._prefs_path = prefs_path
        self._state: dict[str, object] = {
            "controllerStatus": "Not connected",
            "controllerName": "Connect Xbox controller",
            "controllerMode": "idle",
            "serialStatus": "Disconnected",
            "serialTarget": "No port selected",
            "lastSendResult": "Waiting for first command",
            "lastSentLine": "Nothing sent yet",
            "lastResponse": "No response yet",
            "ackVector": "N/A",
            "turn": 0,
            "thrust": 0,
            "rearMotor": 0,
            "frontLeftMotor": 0,
            "frontRightMotor": 0,
            "batteryStatus": "N/A",
            "currentLocation": "Unknown",
            "targetLocation": "Not set",
            "holdPosition": False,
            "currentDraw": "N/A",
            "currentDepth": "N/A",
            "waterTemperature": "N/A",
            "airTemperature": "N/A",
            "imuAccel": "N/A",
            "imuGyro": "N/A",
            "imuTemperature": "N/A",
            "imuUdpLoss": "N/A",
            "audioStream": "N/A",
            "audioLevel": "N/A",
            "motorConfigFetchId": 0,
            "motorConfigMotor": "",
            "motorConfigDirection": "",
            "motorConfigStartBoostPwm": 0,
            "motorConfigStartBoostMs": 0,
            "motorConfigSustainMinPwm": 0,
            "motorConfigMaxPwm": 0,
            "motorConfigCurveTimes100": 0,
            "motorConfigRampTenths": 0,
        }
        self._comm_log: list[str] = []
        self._science_history: list[dict[str, object]] = []
        self._audio_waveform: list[float] = [0.0] * 480
        self._dashboard_tab = "logs"
        self._audio_muted = False
        self._network_interfaces = list_wifi_interfaces()
        self._wifi_local_ip = getattr(args, "wifi_local_ip", "auto") or "auto"
        self._waveform_buffer: deque[float] = deque([0.0] * 480, maxlen=480)
        self._audio_channel = None
        self._mixer_ready = False

        self._transport = None
        self._connection_thread: threading.Thread | None = None
        self._connection_lock = threading.Lock()
        self._connection_in_flight = False
        self._pending_connection: tuple[int, LinkTransport | None, str, str] | None = None
        self._connection_request_id = 0
        self._joystick = None
        self._rx_buffer = bytearray()
        self._command = build_manual_command(0.0, 0.0)
        self._keyboard_turn = 0.0
        self._keyboard_thrust = 0.0
        self._last_send_time = 0.0
        self._last_hello_time = 0.0
        self._last_ping_time = 0.0
        self._last_connection_check = time.time()
        self._last_protocol_response_time = 0.0
        self._last_protocol_send_time = 0.0
        self._awaiting_protocol_response = False
        self._ping_sent = False
        self._connected_device_name: str | None = None
        self._last_reconnect_attempt = 0.0
        self._reconnect_interval = 2.0
        self._ping_interval = 5.0
        self._link_timeout = 3.0
        self._imu_last_sequence: int | None = None
        self._imu_received_count = 0
        self._imu_missing_count = 0
        self._last_science_sample_time = 0.0

        self._timer = QTimer(self)
        self._timer.setInterval(33)
        self._timer.timeout.connect(self._tick)

    @Property(dict, notify=stateChanged)
    def state(self) -> dict[str, object]:
        return self._state

    @Property(list, notify=stateChanged)
    def commLog(self) -> list[str]:
        return self._comm_log

    @Property(list, notify=stateChanged)
    def scienceHistory(self) -> list[dict[str, object]]:
        return self._science_history

    @Property(list, notify=stateChanged)
    def audioWaveform(self) -> list[float]:
        return self._audio_waveform

    @Property(str, notify=dashboardTabChanged)
    def dashboardTab(self) -> str:
        return self._dashboard_tab

    @Property(bool, notify=audioMutedChanged)
    def audioMuted(self) -> bool:
        return self._audio_muted

    @Property(list, notify=stateChanged)
    def networkInterfaces(self) -> list[dict[str, str]]:
        return self._network_interfaces

    @Property(str, notify=stateChanged)
    def wifiLocalIp(self) -> str:
        return self._wifi_local_ip

    @Property(str, notify=stateChanged)
    def connectionButtonLabel(self) -> str:
        serial_status = str(self._state.get("serialStatus", "Disconnected"))
        if serial_status.startswith("Connecting"):
            return "Connecting..."
        if serial_status.startswith("Reconnecting"):
            return "Reconnecting..."
        if serial_status.startswith("Connected"):
            return "Disconnect"
        return "Reconnect"

    @Slot(str)
    def setDashboardTab(self, tab: str) -> None:
        if tab not in {"science", "logs", "map", "analysis", "motors"}:
            return
        if self._dashboard_tab != tab:
            leaving_motor_debug = self._dashboard_tab == "motors" and tab != "motors"
            self._dashboard_tab = tab
            self.dashboardTabChanged.emit()
            if leaving_motor_debug:
                self.stopAllMotors()

    @Slot()
    def toggleAudioMute(self) -> None:
        self._audio_muted = not self._audio_muted
        if self._audio_muted and self._audio_channel is not None:
            try:
                self._audio_channel.stop()
            except Exception:
                pass
        self.audioMutedChanged.emit()
        self.stateChanged.emit()

    @Slot(str)
    def setWifiLocalIp(self, local_ip: str) -> None:
        normalized = local_ip or "auto"
        if normalized == self._wifi_local_ip:
            return
        self._wifi_local_ip = normalized
        self._args.wifi_local_ip = normalized
        self._connection_request_id += 1
        with self._connection_lock:
            self._pending_connection = None
            self._connection_in_flight = False
        if self._transport is not None:
            try:
                self._transport.close()
            except Exception:
                pass
            self._transport = None
        self._set_state(serialStatus="Reconnecting", serialTarget="No port selected")
        self._queue_transport_connection(initial=False)
        self.stateChanged.emit()

    @Slot()
    def toggleConnection(self) -> None:
        if self._transport is not None:
            try:
                self._transport.close()
            except Exception:
                pass
            self._transport = None
            self._connection_request_id += 1
            self._set_state(serialStatus="Disconnected", serialTarget="No port selected")
            self.stateChanged.emit()
            return

        self._queue_transport_connection(initial=False)
        self.stateChanged.emit()

    @Slot()
    def stopAllMotors(self) -> None:
        if self._transport is None:
            self._set_state(lastSendResult="No serial link", lastSentLine="")
            return

        command = "CTRL STOP"
        result = send_text(self._transport, command)
        self._set_state(lastSendResult=result, lastSentLine=command)
        if "failed" not in result.lower():
            self._append_comm_log(f"TX {command}")

    @Slot()
    def stopMotorTest(self) -> None:
        self.stopAllMotors()

    @Slot(str)
    def sendText(self, text: str) -> None:
        message = (text or "").strip()
        if not message:
            self._set_state(lastSendResult="Empty text", lastSentLine="")
            return
        if self._transport is None:
            self._set_state(lastSendResult="No serial link", lastSentLine=message)
            return

        result = send_text(self._transport, message)
        self._set_state(lastSendResult=result, lastSentLine=message)
        if "failed" not in result.lower():
            self._append_comm_log(f"TX {message}")

    @Slot(str, str, int, int, int, int, float, float, bool, int)
    def startMotorTest(
        self,
        motor: str,
        direction: str,
        start_boost_pwm: int,
        start_boost_ms: int,
        sustain_min_pwm: int,
        max_pwm: int,
        curve: float,
        ramp_seconds: float,
        default_reversed: bool,
        throttle_percent: int,
    ) -> None:
        if self._transport is None:
            self._set_state(lastSendResult="No serial link", lastSentLine="")
            return

        normalized_motor = (motor or "").strip().lower().replace("-", "_")
        normalized_direction = (direction or "").strip().lower()
        start_boost_ms_value = max(0, min(5000, int(start_boost_ms)))
        sustain_min_pwm_value = max(0, min(255, int(sustain_min_pwm)))
        max_pwm_value = max(sustain_min_pwm_value, min(255, int(max_pwm)))
        start_boost_pwm_value = max(sustain_min_pwm_value, min(max_pwm_value, int(start_boost_pwm)))
        curve_value = max(0.1, float(curve))
        ramp_seconds_value = max(0.1, float(ramp_seconds))

        calibration = (
            f"CTRL MOTOR CAL {normalized_motor} {normalized_direction} "
            f"{start_boost_pwm_value:d} {start_boost_ms_value:d} {sustain_min_pwm_value:d} "
            f"{max_pwm_value:d} {curve_value:.2f} {ramp_seconds_value:.2f} "
            f"{1 if default_reversed else 0:d}"
        )
        self._append_comm_log(f"TX {calibration}")
        result = send_text(self._transport, calibration)
        self._set_state(lastSendResult=result, lastSentLine=calibration)
        if "failed" in result.lower():
            self._append_comm_log(f"TX failed {calibration}: {result}")
            return

        self.sendMotorThrottle(normalized_motor, normalized_direction, curve_value, throttle_percent)

    @Slot(str, str, float, int)
    def sendMotorThrottle(
        self,
        motor: str,
        direction: str,
        curve: float,
        throttle_percent: int,
    ) -> None:
        if self._transport is None:
            self._set_state(lastSendResult="No serial link", lastSentLine="")
            return

        normalized_motor = (motor or "").strip().lower().replace("-", "_")
        normalized_direction = (direction or "").strip().lower()
        throttle_value = max(0, min(100, int(throttle_percent)))
        curve_value = max(0.1, float(curve))
        drive_floor = 11

        if throttle_value <= 0:
            drive_value = 0
        else:
            drive_fraction = pow(throttle_value / 100.0, 1.0 / curve_value)
            drive_value = int(round(drive_floor + ((255 - drive_floor) * drive_fraction)))
            drive_value = max(drive_floor, min(255, drive_value))

        if normalized_direction == "reverse":
            drive_value = -drive_value

        command = f"CTRL MOTOR {normalized_motor} {drive_value:+d}"
        result = send_text(self._transport, command)
        self._set_state(lastSendResult=result, lastSentLine=command)
        if "failed" not in result.lower():
            self._append_comm_log(f"TX {command}")

    @Slot(str, int)
    def sendMotorTestPower(self, motor: str, power_percent: int) -> None:
        """Send signed test power and let firmware apply direction-specific calibration."""
        if self._transport is None:
            self._set_state(lastSendResult="No serial link", lastSentLine="")
            return

        normalized_motor = (motor or "").strip().lower().replace("-", "_")
        power_value = max(-100, min(100, int(power_percent)))
        drive_floor = 11

        if power_value == 0:
            drive_value = 0
        else:
            drive_magnitude = int(round(drive_floor + ((255 - drive_floor) * (abs(power_value) / 100.0))))
            drive_value = -drive_magnitude if power_value < 0 else drive_magnitude

        command = f"CTRL MOTOR {normalized_motor} {drive_value:+d}"
        result = send_text(self._transport, command)
        self._set_state(lastSendResult=result, lastSentLine=command)
        if "failed" not in result.lower():
            self._append_comm_log(f"TX {command}")

    @Slot(str, str, int, int, int, int, float, float, bool)
    def sendMotorCalibration(
        self,
        motor: str,
        direction: str,
        start_boost_pwm: int,
        start_boost_ms: int,
        sustain_min_pwm: int,
        max_pwm: int,
        curve: float,
        ramp_seconds: float,
        default_reversed: bool,
    ) -> None:
        if self._transport is None:
            self._set_state(lastSendResult="No serial link", lastSentLine="")
            return

        normalized_motor = (motor or "").strip().lower().replace("-", "_")
        normalized_direction = (direction or "").strip().lower()
        start_boost_ms_value = max(0, min(5000, int(start_boost_ms)))
        sustain_min_pwm_value = max(0, min(255, int(sustain_min_pwm)))
        max_pwm_value = max(sustain_min_pwm_value, min(255, int(max_pwm)))
        start_boost_pwm_value = max(sustain_min_pwm_value, min(max_pwm_value, int(start_boost_pwm)))
        curve_value = max(0.1, float(curve))
        ramp_seconds_value = max(0.1, float(ramp_seconds))
        command = (
            f"CTRL MOTOR CAL {normalized_motor} {normalized_direction} "
            f"{start_boost_pwm_value:d} {start_boost_ms_value:d} {sustain_min_pwm_value:d} "
            f"{max_pwm_value:d} {curve_value:.2f} {ramp_seconds_value:.2f} "
            f"{1 if default_reversed else 0:d}"
        )
        result = send_text(self._transport, command)
        self._set_state(lastSendResult=result, lastSentLine=command)
        if "failed" not in result.lower():
            self._append_comm_log(f"TX {command}")

    @Slot(str, int, int, int, int, float, float, bool)
    def sendSharedMotorCalibration(
        self,
        motor: str,
        start_boost_pwm: int,
        start_boost_ms: int,
        sustain_min_pwm: int,
        max_pwm: int,
        curve: float,
        ramp_seconds: float,
        default_reversed: bool,
    ) -> None:
        """Apply one calibration profile to both motor directions."""
        if self._transport is None:
            self._set_state(lastSendResult="No serial link", lastSentLine="")
            return

        for direction in ("forward", "reverse"):
            self.sendMotorCalibration(
                motor,
                direction,
                start_boost_pwm,
                start_boost_ms,
                sustain_min_pwm,
                max_pwm,
                curve,
                ramp_seconds,
                default_reversed,
            )

    @Slot(str, str)
    def requestMotorConfiguration(self, motor: str, direction: str) -> None:
        if self._transport is None:
            self._set_state(lastSendResult="No serial link", lastSentLine="")
            return

        normalized_motor = (motor or "").strip().lower().replace("-", "_")
        normalized_direction = (direction or "").strip().lower()
        command = f"REQ MOTOR CAL {normalized_motor} {normalized_direction}"
        result = send_text(self._transport, command)
        self._set_state(lastSendResult=result, lastSentLine=command)
        if "failed" not in result.lower():
            self._append_comm_log(f"TX {command}")

    @Slot(str, bool)
    def setKeyboardInput(self, key: str, pressed: bool) -> None:
        value = 1.0 if pressed else 0.0
        if key == "left":
            self._keyboard_turn = -value if pressed else (1.0 if self._keyboard_turn > 0 else 0.0)
        elif key == "right":
            self._keyboard_turn = value if pressed else (-1.0 if self._keyboard_turn < 0 else 0.0)
        elif key == "up":
            self._keyboard_thrust = value if pressed else (-1.0 if self._keyboard_thrust < 0 else 0.0)
        elif key == "down":
            self._keyboard_thrust = -value if pressed else (1.0 if self._keyboard_thrust > 0 else 0.0)
        else:
            return
        self.stateChanged.emit()

    def _append_comm_log(self, entry: str) -> None:
        self._comm_log = (self._comm_log + [entry])[-120:]
        with self._log_file.open("a", encoding="utf-8") as handle:
            handle.write(f"{time.time()}: {entry}\n")
        self.stateChanged.emit()

    def _set_state(self, **updates: object) -> None:
        self._state = {**self._state, **updates}
        self.stateChanged.emit()

    def _set_transport_disconnected(self, status: str = "Disconnected") -> None:
        self._transport = None
        self._connected_device_name = None
        self._set_state(serialStatus=status, serialTarget="No port selected", currentDraw="N/A")

    def start(self) -> None:
        if pygame is not None:
            pygame.init()
            pygame.joystick.init()
            self._mixer_ready = pygame.mixer.get_init() is not None
            if not self._mixer_ready:
                try:
                    pygame.mixer.init(frequency=16000, size=-16, channels=2, buffer=512)
                    self._mixer_ready = True
                except pygame.error:
                    self._mixer_ready = False
            self._audio_channel = pygame.mixer.Channel(0) if self._mixer_ready else None
        self._joystick = get_controller()
        self._queue_transport_connection(initial=True)
        self._timer.start()

    def shutdown(self) -> None:
        self._timer.stop()
        if self._transport is not None:
            try:
                self._transport.close()
            except Exception:
                pass
        try:
            if self._joystick is not None:
                self._joystick.quit()
        except Exception:
            pass
        if pygame is not None:
            try:
                pygame.quit()
            except Exception:
                pass

    def _build_transport_target(self) -> str:
        if self._args.transport == "wifi":
            source_label = f" src:{self._wifi_local_ip}" if self._wifi_local_ip and self._wifi_local_ip.lower() != "auto" else ""
            return f"WIFI:{self._args.wifi_host}:{self._args.tcp_port}/udp:{self._args.udp_port}/audio:{self._args.audio_port}{source_label}"
        if self._args.transport == "ble":
            return f"BLE:{self._args.device_name}"
        if self._args.port:
            return self._args.port
        if self._args.device_name:
            return f"Device name: {self._args.device_name}"
        return "No port selected"

    def _queue_transport_connection(self, *, initial: bool = False) -> None:
        if self._transport is not None:
            return
        if self._connection_in_flight:
            return

        self._connection_in_flight = True
        self._connection_request_id += 1
        request_id = self._connection_request_id
        target = self._build_transport_target()
        self._set_state(serialStatus="Connecting...", serialTarget=target)
        source_ip = self._wifi_local_ip if self._args.transport == "wifi" else "n/a"
        self._append_comm_log(f"CONNECT start target={target} source={source_ip}")

        def worker() -> None:
            transport, status, resolved_target = open_serial_connection(
                self._args.port,
                self._args.baudrate,
                self._args.device_name,
            use_ble=(self._args.transport == "ble"),
            use_wifi=(self._args.transport == "wifi"),
            wifi_host=self._args.wifi_host,
            local_ip=self._wifi_local_ip,
            tcp_port=self._args.tcp_port,
            udp_port=self._args.udp_port,
            audio_port=self._args.audio_port,
            ble_service_uuid=self._args.ble_service_uuid,
            ble_characteristic_uuid=self._args.ble_characteristic_uuid,
            )
            with self._connection_lock:
                self._pending_connection = (request_id, transport, status, resolved_target)
                self._connection_in_flight = False

        self._connection_thread = threading.Thread(target=worker, name="buoy-connect", daemon=True)
        self._connection_thread.start()

    def _drain_pending_connection(self, *, initial: bool = False) -> None:
        with self._connection_lock:
            pending = self._pending_connection
            self._pending_connection = None

        if pending is None:
            return

        request_id, transport, status, target = pending
        if request_id != self._connection_request_id:
            if transport is not None:
                try:
                    transport.close()
                except Exception:
                    pass
            return

        if transport is None:
            self._set_state(serialStatus="Reconnecting", serialTarget=target)
            if not initial:
                self._append_comm_log(f"RECONNECT failed target={target}: {status}")
            else:
                self._append_comm_log(f"CONNECT failed target={target}: {status}")
            return

        self._transport = transport
        self._last_protocol_response_time = time.time()
        self._last_protocol_send_time = 0.0
        self._awaiting_protocol_response = False
        self._ping_sent = False

        if self._args.transport == "wifi":
            self._connected_device_name = self._args.wifi_host
        else:
            self._connected_device_name = self._args.device_name if self._args.device_name else None

        if self._args.transport == "wifi":
            resolved_host = extract_wifi_host_from_target(target) or self._args.wifi_host
            save_connection_preferences(
                self._prefs_path,
                wifi_host=resolved_host,
                wifi_local_ip=self._wifi_local_ip if self._wifi_local_ip and self._wifi_local_ip.lower() != "auto" else None,
            )

        serial_status = "Connected"
        if self._connected_device_name:
            serial_status = f"Connected ({self._connected_device_name})"
        self._set_state(serialStatus=serial_status, serialTarget=target)
        if self._args.transport == "wifi":
            stop_result = send_text(self._transport, "CTRL STOP")
            self._set_state(lastSendResult=stop_result, lastSentLine="CTRL STOP")
            self._append_comm_log(f"TX CTRL STOP target={target}")
        if initial:
            self._append_comm_log(f"CONNECT success target={target}")
        else:
            self._append_comm_log(f"RECONNECTED target={target}")

    def _send_current_command(self) -> None:
        joystick_turn, joystick_thrust = read_axes(self._joystick, self._args.deadzone)
        turn = _clamp_axis(joystick_turn + self._keyboard_turn)
        thrust = _clamp_axis(joystick_thrust + self._keyboard_thrust)
        self._command = build_manual_command(turn, thrust)
        self._set_state(
            turn=self._command.turn,
            thrust=self._command.thrust,
            rearMotor=self._command.rear_motor,
            frontLeftMotor=self._command.front_left_motor,
            frontRightMotor=self._command.front_right_motor,
        )

        now = time.monotonic()
        min_send_interval = 1.0 / max(self._args.send_rate, 0.1)
        should_send_keepalive_ping = (
            self._transport is not None
            and not self._args.hello_ping
            and not self._args.send_text
            and self._command.turn == 0
            and self._command.thrust == 0
            and not self._awaiting_protocol_response
        )

        if should_send_keepalive_ping and time.time() - self._last_ping_time > self._ping_interval:
            result = send_text(self._transport, "PING")
            self._set_state(lastSendResult=result, lastSentLine="PING")
            if "failed" not in result.lower():
                self._ping_sent = True
                self._last_ping_time = time.time()
                self._last_protocol_send_time = time.time()
                self._awaiting_protocol_response = True
                self._append_comm_log("TX PING")

        if self._args.hello_ping or self._args.send_text:
            hello_interval = max(self._args.send_interval, 0.1)
            if now - self._last_hello_time >= hello_interval:
                text = "hello world" if self._args.hello_ping else self._args.send_text
                result = send_text(self._transport, text)
                self._set_state(lastSendResult=result, lastSentLine=text)
                self._last_hello_time = now
                if "failed" not in result.lower():
                    self._last_protocol_send_time = time.time()
                    self._awaiting_protocol_response = True
                self._append_comm_log(f"TX {text}")
            return

        if self._dashboard_tab != "motors":
            send_command_flag = False
            if self._command != getattr(self, "_last_command", None):
                send_command_flag = True
            elif (now - self._last_send_time) >= min_send_interval and not (self._command.turn == 0 and self._command.thrust == 0):
                send_command_flag = True

            if send_command_flag and self._transport is not None:
                result = send_command(self._transport, self._command)
                self._set_state(lastSendResult=result, lastSentLine=self._command.to_line().strip())
                self._last_send_time = now
                self._last_command = self._command
                if "failed" not in result.lower():
                    self._last_protocol_send_time = time.time()
                    self._awaiting_protocol_response = True
                self._append_comm_log(f"TX {self._command.to_line().strip()}")

    def _process_text_line(self, response_text: str) -> None:
        self._set_state(lastResponse=response_text)
        self._append_comm_log(f"RX {response_text}")

        ack = parse_acknowledgement(response_text)
        if ack is not None:
            ack_kind, ack_values = ack
            if ack_kind == "CTRL" and len(ack_values) >= 3 and ack_values[0] == "VECTOR":
                self._set_state(ackVector=f"{ack_values[1]},{ack_values[2]}")
            elif ack_kind == "CTRL" and len(ack_values) >= 2 and ack_values[0] == "HOLD":
                self._set_state(holdPosition=(ack_values[1] == "ON"))

        status_update = parse_status_update(response_text)
        if status_update is not None:
            status_key, status_values = status_update
            if status_key == "MODE" and status_values:
                self._set_state(controllerMode=status_values[0].lower())
            elif status_key == "POS":
                self._set_state(currentLocation=_format_location(status_values))
            elif status_key == "TARGET":
                self._set_state(targetLocation=_format_location(status_values))
            elif status_key == "HOLD" and status_values:
                self._set_state(holdPosition=(status_values[0] == "ON"))
            elif status_key == "BATTERY":
                self._set_state(batteryStatus=_parse_battery_status(" ".join(status_values)))
            elif status_key == "CURRENT":
                self._set_state(currentDraw=_parse_battery_status(" ".join(status_values)))

        science_update = parse_science_update(response_text)
        if science_update is not None:
            science_key, science_values = science_update
            if science_key == "WATER_TEMP":
                self._set_state(waterTemperature=_parse_science_value(science_values, "C"))
            elif science_key == "AIR_TEMP":
                self._set_state(airTemperature=_parse_science_value(science_values, "C"))
            elif science_key == "DEPTH":
                self._set_state(currentDepth=_parse_science_value(science_values, "m"))
            elif science_key == "IMU_ACCEL":
                self._set_state(imuAccel=_parse_vector3_text(" ".join(science_values), "g"))
            elif science_key == "IMU_GYRO":
                self._set_state(imuGyro=_parse_vector3_text(" ".join(science_values), "dps"))
            elif science_key == "IMU_TEMP":
                self._set_state(imuTemperature=_parse_science_value(science_values, "C"))

        if response_text.startswith("TEL MOTOR CAL "):
            payload = response_text[14:].strip()
            parts = payload.split()
            if len(parts) >= 8:
                motor_name = parts[0].strip().lower()
                direction = parts[1].strip().lower()
                try:
                    start_boost_pwm = int(parts[2])
                    start_boost_ms = int(parts[3])
                    sustain_min_pwm = int(parts[4])
                    max_pwm = int(parts[5])
                    curve = float(parts[6])
                    ramp_seconds = float(parts[7])
                    default_reversed = bool(int(parts[8])) if len(parts) >= 9 else False
                except ValueError:
                    start_boost_pwm = 0
                    start_boost_ms = 0
                    sustain_min_pwm = 0
                    max_pwm = 0
                    curve = 0.0
                    ramp_seconds = 0.0
                    default_reversed = False
                self._set_state(
                    motorConfigFetchId=int(self._state.get("motorConfigFetchId", 0)) + 1,
                    motorConfigMotor=motor_name,
                    motorConfigDirection=direction,
                    motorConfigStartBoostPwm=start_boost_pwm,
                    motorConfigStartBoostMs=start_boost_ms,
                    motorConfigSustainMinPwm=sustain_min_pwm,
                    motorConfigMaxPwm=max_pwm,
                    motorConfigCurveTimes100=int(round(curve * 100.0)),
                    motorConfigRampTenths=int(round(ramp_seconds * 10.0)),
                    motorConfigDefaultReversed=default_reversed,
                )

        if is_protocol_message(response_text):
            self._last_protocol_response_time = time.time()
            self._awaiting_protocol_response = False
            serial_status = "Connected"
            if self._connected_device_name:
                serial_status = f"Connected ({self._connected_device_name})"
            self._set_state(serialStatus=serial_status)
            self._ping_sent = False

        error_text = parse_error(response_text)
        if error_text is not None:
            self._set_state(lastSendResult=f"ERR {error_text}")

    def _process_binary_telemetry(self) -> None:
        packets = read_telemetry_packets(self._transport)
        if not packets:
            return

        self._last_protocol_response_time = time.time()
        serial_status = "Connected"
        if self._connected_device_name:
            serial_status = f"Connected ({self._connected_device_name})"
        self._set_state(serialStatus=serial_status)

        for packet in packets:
            if packet.packet_type == "IMU":
                if self._imu_last_sequence is not None:
                    expected_next = (self._imu_last_sequence + 1) & 0xFFFF
                    if packet.sequence != expected_next:
                        gap = (packet.sequence - expected_next) & 0xFFFF
                        self._imu_missing_count += gap
                self._imu_last_sequence = packet.sequence
                self._imu_received_count += 1
                total_imu = self._imu_received_count + self._imu_missing_count
                loss_pct = (self._imu_missing_count / total_imu * 100.0) if total_imu > 0 else 0.0
                ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps, temperature_c = packet.values
                self._set_state(
                    imuAccel=f"{ax_g:.3f}, {ay_g:.3f}, {az_g:.3f} g",
                    imuGyro=f"{gx_dps:.1f}, {gy_dps:.1f}, {gz_dps:.1f} dps",
                    imuTemperature=f"{temperature_c:.2f} C",
                    imuUdpLoss=f"{loss_pct:.1f}% ({self._imu_missing_count}/{total_imu})",
                )
            elif packet.packet_type == "POWER":
                battery_volts, current_amps, battery_pct = packet.values
                self._set_state(
                    batteryStatus=_format_udp_battery_status(battery_volts, current_amps, battery_pct),
                    currentDraw=f"{current_amps:.3f} A",
                )
            elif packet.packet_type == "STATE":
                mode_value, hold_enabled, gps_valid, _motors_enabled = packet.values
                current_location = str(self._state.get("currentLocation", "Unknown")) if gps_valid else "Unknown"
                self._set_state(
                    controllerMode=_decode_control_mode(mode_value),
                    holdPosition=bool(hold_enabled),
                    currentLocation=current_location,
                )
            elif packet.packet_type == "GPS":
                latitude, longitude = packet.values
                self._set_state(currentLocation=_format_udp_position(latitude, longitude))
            elif packet.packet_type == "RANGE":
                distance_mm, valid = packet.values
                self._set_state(currentDepth=(f"{distance_mm / 1000.0:.3f} m" if valid else "N/A"))
            elif packet.packet_type == "AUDIO":
                pcm_bytes, _sample_count, channels = packet.values
                samples = _decode_pcm_samples(pcm_bytes)
                peak = _update_audio_waveform_buffer(self._waveform_buffer, samples, channels)
                self._audio_waveform = list(self._waveform_buffer)
                self._set_state(
                    audioStream=(
                        f"16.0 kHz stereo" if channels == 2 else "16.0 kHz mono"
                    ),
                    audioLevel=f"{peak * 100.0:.1f}%",
                )
                if not self._audio_muted and self._audio_channel is not None:
                    try:
                        if not self._audio_channel.get_busy():
                            sound = pygame.mixer.Sound(buffer=pcm_bytes)
                            self._audio_channel.play(sound)
                    except Exception:
                        pass

        self._maybe_record_science_sample()

    def _maybe_record_science_sample(self) -> None:
        if time.time() - self._last_science_sample_time < 0.5:
            return
        sample = _parse_science_sample(self._state, time.time())
        if sample is None:
            return
        self._science_history = (self._science_history + [asdict(sample)])[-180:]
        self._last_science_sample_time = time.time()
        self.stateChanged.emit()

    def _tick(self) -> None:
        if self._joystick is None:
            self._joystick = get_controller()
            if self._joystick is not None:
                self._set_state(controllerStatus="Connected", controllerName=self._joystick.get_name())
        else:
            self._set_state(controllerStatus="Connected", controllerName=self._joystick.get_name())

        if (
            self._transport is None
            and self._pending_connection is None
            and not self._connection_in_flight
            and (time.time() - self._last_reconnect_attempt) >= self._reconnect_interval
        ):
            self._last_reconnect_attempt = time.time()
            self._queue_transport_connection()

        self._drain_pending_connection()

        if self._transport is not None:
            if self._transport.is_open:
                self._send_current_command()
                incoming_bytes = read_available_bytes(self._transport)
                if incoming_bytes:
                    self._rx_buffer.extend(incoming_bytes)
                    while True:
                        newline_index = self._rx_buffer.find(b"\n")
                        if newline_index == -1:
                            break
                        line_bytes = bytes(self._rx_buffer[:newline_index]).rstrip(b"\r")
                        del self._rx_buffer[:newline_index + 1]
                        if not line_bytes:
                            continue
                        response_text = line_bytes.decode("ascii", errors="replace")
                        self._process_text_line(response_text)
                self._process_binary_telemetry()
            else:
                self._set_transport_disconnected("Reconnecting")
                try:
                    self._transport.close()
                except Exception:
                    pass
                self._transport = None

        if self._transport is not None and self._awaiting_protocol_response:
            if (time.time() - self._last_protocol_send_time) > self._link_timeout:
                self._append_comm_log("LINK TIMEOUT")
                try:
                    self._transport.close()
                except Exception:
                    pass
                self._set_transport_disconnected("Reconnecting")
                self._awaiting_protocol_response = False
                self._ping_sent = False
                self._transport = None

        if self._ping_sent and (time.time() - self._last_ping_time) > 3.0:
            self._append_comm_log("PING timeout")
            try:
                if self._transport is not None:
                    self._transport.close()
            except Exception:
                pass
            self._set_transport_disconnected("Reconnecting")
            self._ping_sent = False
            self._awaiting_protocol_response = False
            self._transport = None

        self.stateChanged.emit()

    @Slot()
    def startHelloPing(self) -> None:
        pass


def _format_location(values: tuple[str, ...]) -> str:
    if len(values) < 2:
        return "Unknown"
    if values[0] == "UNKNOWN" or values[1] == "UNKNOWN":
        return "Unknown"
    return f"{values[0]}, {values[1]}"


def _parse_science_value(values: tuple[str, ...], unit: str) -> str:
    if not values or values[0] == "UNKNOWN":
        return "N/A"
    return f"{values[0]} {unit}".strip()


def _parse_battery_status(values_text: str) -> str:
    if not values_text or values_text.strip() == "UNKNOWN":
        return "N/A"
    return values_text


def run(args: argparse.Namespace) -> None:
    app = QGuiApplication(sys.argv)

    app_dir = Path(__file__).resolve().parent.parent
    log_dir = app_dir / "logs"
    log_dir.mkdir(parents=True, exist_ok=True)
    timestamp = time.strftime("%Y%m%d-%H%M%S")
    log_file = Path(args.comm_log_file) if args.comm_log_file else (log_dir / f"comm-{timestamp}.log")
    prefs_path = log_dir / CONNECTION_PREFS_FILENAME
    apply_connection_preferences(args, prefs_path)
    with log_file.open("a", encoding="utf-8") as handle:
        handle.write(f"{time.time()}: COMM LOG START file={log_file}\n")

    engine = QQmlApplicationEngine()
    backend = ControlStationBackend(args, log_file, prefs_path)
    engine.rootContext().setContextProperty("backend", backend)

    qml_path = app_dir / "qml" / "Main.qml"
    engine.load(QUrl.fromLocalFile(str(qml_path)))
    if not engine.rootObjects():
        raise RuntimeError(f"Failed to load QML UI: {qml_path}")

    app.aboutToQuit.connect(backend.shutdown)
    QTimer.singleShot(0, backend.start)
    exit_code = app.exec()
    with log_file.open("a", encoding="utf-8") as handle:
        handle.write(f"{time.time()}: COMM LOG END\n")
    raise SystemExit(exit_code)

