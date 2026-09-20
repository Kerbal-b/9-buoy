from __future__ import annotations

import argparse
from array import array
from collections import deque
from dataclasses import asdict
import math
from pathlib import Path
import sys
import time
import threading

try:
    import pygame
except ModuleNotFoundError:
    pygame = None
from PySide6.QtCore import QEvent, QObject, Property, QTimer, Qt, Signal, Slot, QUrl
from PySide6.QtGui import QGuiApplication
from PySide6.QtQml import QQmlApplicationEngine

from .controller import get_controller, read_motion_axes
from .geometry import build_manual_command
from .models import ScienceSample
from .navigation import GpsSpeedTracker, NavigationAssist
from .log_categories import (
    NAVIGATION_LOG,
    SCIENTIFIC_LOG,
    classify_log_entry,
    display_timestamp,
    interpret_log_entry,
)
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
from .test_session import PersistentTestSession
from .telemetry_quality import ImuStreamQuality

DEFAULT_MIXER_POWER_LIMIT = 0.50


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


def _select_manual_input(
    input_mode: str,
    joystick_turn: float,
    joystick_thrust: float,
    keyboard_turn: float,
    keyboard_thrust: float,
    keyboard_yaw: float,
    joystick_yaw: float = 0.0,
) -> tuple[float, float, float]:
    if input_mode == "controller":
        return (
            _clamp_axis(joystick_turn),
            _clamp_axis(joystick_thrust),
            _clamp_axis(joystick_yaw),
        )
    if input_mode == "keyboard":
        return _clamp_axis(keyboard_turn), _clamp_axis(keyboard_thrust), _clamp_axis(keyboard_yaw)
    return 0.0, 0.0, 0.0


def _limit_motion_to_motor_output(
    lateral: float,
    thrust: float,
    yaw: float,
    maximum: float = DEFAULT_MIXER_POWER_LIMIT,
) -> tuple[float, float, float]:
    """Scale a motion vector so no mixed motor exceeds the safety limit."""
    lateral = _clamp_axis(lateral)
    thrust = _clamp_axis(thrust)
    yaw = _clamp_axis(yaw)
    translation_scale = (2.0 / 3.0) * 2.0
    front_x = math.sqrt(3.0) / 2.0
    mixed = (
        (translation_scale * thrust) + yaw,
        (translation_scale * ((lateral * front_x) - (thrust * 0.5))) + yaw,
        (translation_scale * ((-lateral * front_x) - (thrust * 0.5))) + yaw,
    )
    peak = max(abs(value) for value in mixed)
    bounded_maximum = max(0.0, min(1.0, float(maximum)))
    if peak <= bounded_maximum or peak == 0.0:
        return lateral, thrust, yaw
    scale = bounded_maximum / peak
    return lateral * scale, thrust * scale, yaw * scale


def _motor_percent_command(motor: str, power_percent: int) -> str:
    """Build an explicit percentage command for firmware-side calibration."""
    normalized_motor = (motor or "").strip().lower().replace("-", "_")
    bounded_percent = max(-100, min(100, int(power_percent)))
    return f"CTRL MOTOR PERCENT {normalized_motor} {bounded_percent:+d}"


def _calibrated_motor_pwm(power_percent: int, minimum_pwm: int, maximum_pwm: int, curve: float) -> int:
    """Calculate steady PWM from a signed logical motor percentage."""
    magnitude = max(0.0, min(1.0, abs(float(power_percent)) / 100.0))
    if magnitude == 0.0:
        return 0
    minimum = max(0, min(255, int(minimum_pwm)))
    maximum = max(minimum, min(255, int(maximum_pwm)))
    shaped = math.pow(magnitude, max(0.1, float(curve)))
    return int(math.floor(minimum + ((maximum - minimum) * shaped) + 0.5))


def _parse_motor_output_telemetry(
    response_text: str,
) -> tuple[int, int, int, int, int, int] | None:
    prefix = "TEL MOTOR OUT "
    if not response_text.startswith(prefix):
        return None
    parts = response_text[len(prefix):].strip().split()
    if len(parts) < 6:
        return None
    try:
        return tuple(int(value) for value in parts[:6])
    except ValueError:
        return None


class KeyboardDriveFilter(QObject):
    KEY_TO_DIRECTION = {
        Qt.Key.Key_A: "left",
        Qt.Key.Key_D: "right",
        Qt.Key.Key_W: "up",
        Qt.Key.Key_S: "down",
        Qt.Key.Key_Left: "yaw_left",
        Qt.Key.Key_Right: "yaw_right",
    }

    def __init__(self, backend: "ControlStationBackend") -> None:
        super().__init__(backend)
        self._backend = backend

    def eventFilter(self, watched: QObject, event: QEvent) -> bool:
        del watched
        if not self._backend.keyboard_drive_enabled:
            return False
        if event.type() not in {QEvent.Type.KeyPress, QEvent.Type.KeyRelease}:
            return False

        key = event.key()
        if key == Qt.Key.Key_Space:
            if event.type() == QEvent.Type.KeyPress and not event.isAutoRepeat():
                self._backend.stopNavigationTest()
            return True

        direction = self.KEY_TO_DIRECTION.get(key)
        if direction is None:
            # Consume vertical arrows while Keyboard Drive is active so Qt
            # controls cannot unexpectedly navigate or change values.
            return key in {Qt.Key.Key_Up, Qt.Key.Key_Down}
        if not event.isAutoRepeat():
            self._backend.setKeyboardInput(direction, event.type() == QEvent.Type.KeyPress)
        return True


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
            "controlInputMode": "controller",
            "controlInputModeDetail": "Left stick movement + right stick yaw",
            "mixerPowerPercent": int(round(DEFAULT_MIXER_POWER_LIMIT * 100.0)),
            "serialStatus": "Disconnected",
            "serialTarget": "No port selected",
            "lastSendResult": "Waiting for first command",
            "lastSentLine": "Nothing sent yet",
            "lastResponse": "No response yet",
            "ackVector": "N/A",
            "turn": 0,
            "thrust": 0,
            "yaw": 0,
            "rearMotor": 0,
            "frontLeftMotor": 0,
            "frontRightMotor": 0,
            "rearMotorActualDrive": 0,
            "frontLeftMotorActualDrive": 0,
            "frontRightMotorActualDrive": 0,
            "rearMotorPwm": -1,
            "frontLeftMotorPwm": -1,
            "frontRightMotorPwm": -1,
            "rearMotorTargetPwm": 0,
            "frontLeftMotorTargetPwm": 0,
            "frontRightMotorTargetPwm": 0,
            "motorOutputTelemetry": False,
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
            "navigationAssistEnabled": False,
            "navigationAssistActive": False,
            "navigationImuReady": False,
            "navigationStatus": "Zero the IMU while the buoy is stationary",
            "navigationRequestedTurn": 0,
            "navigationRequestedThrust": 0,
            "navigationRequestedYaw": 0,
            "navigationCorrectedTurn": 0,
            "navigationCorrectedThrust": 0,
            "navigationCorrectedYaw": 0,
            "navigationMeasuredXG": 0.0,
            "navigationMeasuredYG": 0.0,
            "navigationMeasuredMagnitudeG": 0.0,
            "navigationMeasuredYawDps": 0.0,
            "navigationRollDeg": 0.0,
            "navigationPitchDeg": 0.0,
            "navigationEstimatedVelocityXMps": 0.0,
            "navigationEstimatedVelocityYMps": 0.0,
            "navigationEstimatedSpeedMps": 0.0,
            "navigationSpeedConfidencePercent": 0,
            "navigationTiltCompensating": False,
            "navigationTiltRateDps": 0.0,
            "navigationGpsSpeedAvailable": False,
            "navigationGpsSpeedMps": 0.0,
            "navigationBestSpeedMps": 0.0,
            "navigationSpeedSource": "IMU estimate",
            "navigationTestEnvironment": "dry",
            "testSessionActive": False,
            "testSessionStatus": "Not recording",
            "testSessionFile": "",
            "testSessionSampleCount": 0,
            "testSessionImuRateHz": 0.0,
            "testSessionImuLossPercent": 0.0,
            "navigationDirectionErrorDeg": 0.0,
            "navigationCorrectionDeg": 0.0,
            "navigationGainPercent": 35,
            "navigationMaxCorrectionDeg": 20,
            "navigationInvertX": True,
            "navigationInvertY": False,
            "navigationSwapXY": True,
            "navigationReleaseRequired": False,
            "keyboardDriveEnabled": False,
            "keyboardInputSummary": "Idle",
            "navigationRequestedRearMotor": 0,
            "navigationRequestedFrontLeftMotor": 0,
            "navigationRequestedFrontRightMotor": 0,
            "navigationCorrectedRearMotor": 0,
            "navigationCorrectedFrontLeftMotor": 0,
            "navigationCorrectedFrontRightMotor": 0,
        }
        self._comm_log: list[str] = []
        self._categorized_log: list[dict[str, str]] = []
        self._system_log: list[dict[str, str]] = []
        self._navigation_log: list[dict[str, str]] = []
        self._scientific_log: list[dict[str, str]] = []
        self._science_history: list[dict[str, object]] = []
        self._audio_waveform: list[float] = [0.0] * 480
        default_profile = {"minimum": 130, "maximum": 255, "curve": 2.0}
        self._motor_calibrations: dict[str, dict[str, object]] = {
            "rear": {
                "forward": dict(default_profile),
                "reverse": dict(default_profile),
                "default_reversed": False,
            },
            "front_left": {
                "forward": dict(default_profile),
                "reverse": dict(default_profile),
                "default_reversed": False,
            },
            "front_right": {
                "forward": dict(default_profile),
                "reverse": dict(default_profile),
                "default_reversed": True,
            },
        }
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
        self._keyboard_yaw = 0.0
        self._keyboard_keys: set[str] = set()
        self._mixer_power_limit = DEFAULT_MIXER_POWER_LIMIT
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
        self._imu_quality = ImuStreamQuality(expected_interval_ms=50)
        self._last_imu_gyro = (0.0, 0.0, 0.0)
        self._last_science_sample_time = 0.0
        self._navigation = NavigationAssist()
        self._gps_speed = GpsSpeedTracker()
        # The installed MPU6050 is rotated 90 degrees clockwise: body-right is
        # sensor -Y and body-forward is sensor +X.
        self._navigation.set_axis_signs(True, False, True)
        self._last_navigation_log_time = 0.0
        self._navigation_requires_center = False
        self._test_session: PersistentTestSession | None = None
        self._test_imu_quality: ImuStreamQuality | None = None
        self._latest_power: dict[str, float | int | None] = {
            "battery_voltage_v": None,
            "current_a": None,
            "battery_percent": None,
        }
        self._last_recorded_imu_arrival_at: float | None = None
        self._last_motor_output_at: float | None = None
        self._last_power_at: float | None = None

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
    def categorizedLog(self) -> list[dict[str, str]]:
        return self._categorized_log

    @Property(list, notify=stateChanged)
    def systemLog(self) -> list[dict[str, str]]:
        return self._system_log

    @Property(list, notify=stateChanged)
    def navigationLog(self) -> list[dict[str, str]]:
        return self._navigation_log

    @Property(list, notify=stateChanged)
    def scientificLog(self) -> list[dict[str, str]]:
        return self._scientific_log

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

    @property
    def keyboard_drive_enabled(self) -> bool:
        return self._state.get("controlInputMode") == "keyboard"

    @Slot(str)
    def setDashboardTab(self, tab: str) -> None:
        if tab not in {"science", "logs", "map", "analysis", "motors", "navigation"}:
            return
        if self._dashboard_tab != tab:
            leaving_motor_debug = self._dashboard_tab == "motors" and tab != "motors"
            leaving_navigation_test = self._dashboard_tab == "navigation" and tab != "navigation"
            self._dashboard_tab = tab
            self.dashboardTabChanged.emit()
            if leaving_motor_debug:
                self.stopAllMotors()
            if leaving_navigation_test:
                was_enabled = self._navigation.enabled
                self._navigation.set_enabled(False, time.monotonic())
                navigation_was_active = was_enabled
                self._navigation_requires_center = navigation_was_active
                self._set_state(
                    navigationAssistEnabled=False,
                    navigationAssistActive=False,
                    navigationReleaseRequired=navigation_was_active,
                    navigationStatus=("Center controller to resume" if navigation_was_active else "Assist off"),
                    navigationCorrectionDeg=0.0,
                )
                if navigation_was_active:
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
            self._connection_request_id += 1
            self._set_transport_disconnected("Disconnected")
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

    @Slot()
    def zeroNavigationImu(self) -> None:
        if self._navigation.zero():
            self._set_state(
                navigationImuReady=self._navigation.measurement_is_fresh(time.monotonic()),
                navigationStatus="IMU zero captured; assist ready",
                navigationMeasuredXG=0.0,
                navigationMeasuredYG=0.0,
                navigationMeasuredMagnitudeG=0.0,
            )
            self._append_comm_log("NAV IMU ZERO")
        else:
            self._set_state(navigationStatus="Cannot zero: waiting for IMU telemetry")

    @Slot(bool)
    def setNavigationAssist(self, enabled: bool) -> None:
        if enabled and self._transport is None:
            self._set_state(navigationStatus="Cannot enable: buoy is disconnected")
            return

        was_enabled = self._navigation.enabled
        is_enabled = self._navigation.set_enabled(bool(enabled), time.monotonic())
        if enabled and not is_enabled:
            self._set_state(
                navigationAssistEnabled=False,
                navigationAssistActive=False,
                navigationStatus="Cannot enable: zero the IMU and wait for fresh telemetry",
            )
            return

        self._set_state(
            navigationAssistEnabled=is_enabled,
            navigationAssistActive=False,
            navigationStatus=("Assist armed; waiting for controller input" if is_enabled else "Assist off"),
            navigationCorrectionDeg=0.0,
        )
        self._append_comm_log(f"NAV ASSIST {'ON' if is_enabled else 'OFF'}")
        if was_enabled and not is_enabled:
            self.stopAllMotors()

    @Slot()
    def stopNavigationTest(self) -> None:
        self._navigation.set_enabled(False, time.monotonic())
        self._navigation_requires_center = True
        self._keyboard_keys.clear()
        self._keyboard_turn = 0.0
        self._keyboard_thrust = 0.0
        self._keyboard_yaw = 0.0
        self._set_state(
            navigationAssistEnabled=False,
            navigationAssistActive=False,
            navigationReleaseRequired=True,
            navigationStatus="Stopped; center controller to resume",
            navigationCorrectionDeg=0.0,
            keyboardInputSummary="Idle",
        )
        self._append_comm_log("NAV STOP")
        self.stopAllMotors()

    @Slot(bool)
    def setKeyboardDriveEnabled(self, enabled: bool) -> None:
        self.setControlInputMode("keyboard" if enabled else "controller")

    @Slot(str)
    def setControlInputMode(self, mode: str) -> None:
        normalized_mode = (mode or "").strip().lower()
        if normalized_mode not in {"keyboard", "controller", "auto"}:
            return
        if normalized_mode == self._state.get("controlInputMode"):
            return

        self._keyboard_keys.clear()
        self._keyboard_turn = 0.0
        self._keyboard_thrust = 0.0
        self._keyboard_yaw = 0.0
        self._navigation_requires_center = normalized_mode == "controller"
        detail = {
            "keyboard": "WASD movement + arrow-key yaw",
            "controller": "Left stick movement + right stick yaw",
            "auto": "Program execution coming later",
        }[normalized_mode]
        self._set_state(
            controlInputMode=normalized_mode,
            controlInputModeDetail=detail,
            keyboardDriveEnabled=(normalized_mode == "keyboard"),
            keyboardInputSummary=(
                "Ready: WASD translation + arrows yaw" if normalized_mode == "keyboard" else "Idle"
            ),
            navigationReleaseRequired=(normalized_mode == "controller"),
        )
        self._append_comm_log(f"CONTROL SOURCE {normalized_mode.upper()}")
        self.stopAllMotors()

    @Slot(float)
    def setMixerPowerLimit(self, power_percent: float) -> None:
        power_percent = max(10.0, min(100.0, float(power_percent)))
        self._mixer_power_limit = power_percent / 100.0
        self._set_state(mixerPowerPercent=int(round(power_percent)))
        self._append_comm_log(f"MIXER POWER LIMIT {int(round(power_percent))}%")

    @Slot(float)
    def setNavigationGain(self, gain_percent: float) -> None:
        self._navigation.set_gain(float(gain_percent) / 100.0)
        self._set_state(navigationGainPercent=int(round(self._navigation.gain * 100.0)))

    @Slot(float)
    def setNavigationMaxCorrection(self, degrees: float) -> None:
        self._navigation.set_max_correction(degrees)
        self._set_state(navigationMaxCorrectionDeg=int(round(self._navigation.max_correction_deg)))

    @Slot(bool, bool, bool)
    def setNavigationAxisSigns(self, invert_x: bool, invert_y: bool, swap_xy: bool) -> None:
        was_enabled = self._navigation.enabled
        self._navigation.set_axis_signs(invert_x, invert_y, swap_xy)
        self._set_state(
            navigationAssistEnabled=False,
            navigationAssistActive=False,
            navigationImuReady=False,
            navigationInvertX=bool(invert_x),
            navigationInvertY=bool(invert_y),
            navigationSwapXY=bool(swap_xy),
            navigationStatus="Axis mapping changed; zero the IMU again",
            navigationCorrectionDeg=0.0,
        )
        self._append_comm_log(
            f"NAV AXES invert_x={int(bool(invert_x))} invert_y={int(bool(invert_y))} "
            f"swap_xy={int(bool(swap_xy))}"
        )
        if was_enabled:
            self.stopAllMotors()

    @Slot(str)
    def setNavigationTestEnvironment(self, environment: str) -> None:
        if self._test_session is not None and self._test_session.active:
            self._set_state(testSessionStatus="Stop recording before changing test environment")
            return
        normalized = str(environment).strip().lower()
        if normalized not in {"dry", "water"}:
            return
        if normalized == self._state.get("navigationTestEnvironment"):
            return
        self._set_state(navigationTestEnvironment=normalized)
        self._append_comm_log(f"NAV ENVIRONMENT {normalized.upper()}")

    @Slot()
    def startTestSession(self) -> None:
        if self._test_session is not None and self._test_session.active:
            return
        if self._transport is None:
            self._set_state(testSessionStatus="Connect to the buoy before recording")
            return

        environment = str(self._state.get("navigationTestEnvironment", "water"))
        metadata = {
            "source": "buoy-control-station",
            "test_kind": "water_imu_calibration" if environment == "water" else "dry_land_baseline",
            "serial_target": str(self._state.get("serialTarget", "")),
            "control_input_mode": str(self._state.get("controlInputMode", "controller")),
            "controller_name": str(self._state.get("controllerName", "")),
            "send_rate_hz": float(self._args.send_rate),
            "controller_deadzone": float(self._args.deadzone),
            "mixer_power_limit_percent": int(round(self._mixer_power_limit * 100.0)),
            "navigation_axis_mapping": {
                "invert_x": bool(self._state.get("navigationInvertX", False)),
                "invert_y": bool(self._state.get("navigationInvertY", False)),
                "swap_xy": bool(self._state.get("navigationSwapXY", False)),
            },
            "motor_calibration": self._motor_calibrations,
            "notes": "Motor PWM is commanded duty, not measured propeller RPM.",
        }
        try:
            self._test_session = PersistentTestSession(
                self._log_file.parent / "tests",
                environment,
                metadata,
            )
        except OSError as exc:
            self._test_session = None
            self._set_state(testSessionStatus=f"Could not start recorder: {exc}")
            return

        self._test_imu_quality = ImuStreamQuality(expected_interval_ms=50)
        self._last_recorded_imu_arrival_at = None
        self._set_state(
            testSessionActive=True,
            testSessionStatus="Recording synchronized telemetry",
            testSessionFile=str(self._test_session.path),
            testSessionSampleCount=0,
            testSessionImuRateHz=0.0,
            testSessionImuLossPercent=0.0,
        )
        self._append_comm_log(f"TEST RECORD START file={self._test_session.path}")

    def _close_test_session(self, reason: str) -> None:
        session = self._test_session
        if session is None:
            return
        path = session.path
        sample_count = session.sample_count
        close_error: OSError | None = None
        try:
            session.close(reason, self._test_session_summary())
        except OSError as exc:
            close_error = exc
        finally:
            self._test_session = None
            self._test_imu_quality = None
        self._set_state(
            testSessionActive=False,
            testSessionStatus=(
                f"Saved {sample_count} samples" if close_error is None else f"Recorder close error: {close_error}"
            ),
            testSessionFile=str(path),
            testSessionSampleCount=sample_count,
        )
        self._append_comm_log(
            f"TEST RECORD STOP samples={sample_count} file={path}"
            + (f" error={close_error}" if close_error is not None else "")
        )

    @Slot()
    def stopTestSession(self) -> None:
        self._close_test_session("operator_stop")

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
        # All calibration belongs to firmware. Send the requested percentage
        # without converting it to a raw drive or applying the curve here.
        del curve
        if normalized_direction == "reverse":
            throttle_value = -throttle_value

        command = _motor_percent_command(normalized_motor, throttle_value)
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
        command = _motor_percent_command(normalized_motor, power_value)
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
        if not self.keyboard_drive_enabled or self._transport is None:
            return
        if key not in {"left", "right", "up", "down", "yaw_left", "yaw_right"}:
            return
        if pressed:
            self._keyboard_keys.add(key)
        else:
            self._keyboard_keys.discard(key)

        self._keyboard_turn = float(
            ("right" in self._keyboard_keys) - ("left" in self._keyboard_keys)
        )
        self._keyboard_thrust = float(
            ("up" in self._keyboard_keys) - ("down" in self._keyboard_keys)
        )
        self._keyboard_yaw = float(
            ("yaw_right" in self._keyboard_keys) - ("yaw_left" in self._keyboard_keys)
        )
        labels = {
            "up": "FORWARD",
            "down": "REVERSE",
            "left": "MOVE LEFT",
            "right": "MOVE RIGHT",
            "yaw_left": "YAW LEFT",
            "yaw_right": "YAW RIGHT",
        }
        active_keys = [labels[name] for name in labels if name in self._keyboard_keys]
        self._set_state(
            keyboardInputSummary=(" + ".join(active_keys) if active_keys else "Ready: WASD translation + arrows yaw")
        )

    def _append_comm_log(self, entry: str) -> None:
        timestamp = time.time()
        category = classify_log_entry(entry)
        display_entry = {
            "timestamp": display_timestamp(timestamp),
            "message": entry,
            "category": category,
            "interpretation": interpret_log_entry(entry),
        }
        self._comm_log = (self._comm_log + [entry])[-120:]
        self._categorized_log = (self._categorized_log + [display_entry])[-360:]
        if category == NAVIGATION_LOG:
            self._navigation_log = (self._navigation_log + [display_entry])[-120:]
        elif category == SCIENTIFIC_LOG:
            self._scientific_log = (self._scientific_log + [display_entry])[-120:]
        else:
            self._system_log = (self._system_log + [display_entry])[-120:]
        with self._log_file.open("a", encoding="utf-8") as handle:
            handle.write(f"{timestamp}: {entry}\n")
        self.stateChanged.emit()

    def _set_state(self, **updates: object) -> None:
        self._state = {**self._state, **updates}
        self.stateChanged.emit()

    def _motor_pwm_for_percent(self, motor_name: str, power_percent: int) -> int:
        calibration = self._motor_calibrations[motor_name]
        logical_forward = power_percent > 0
        physical_forward = logical_forward != bool(calibration["default_reversed"])
        profile = calibration["forward" if physical_forward else "reverse"]
        return _calibrated_motor_pwm(
            power_percent,
            int(profile["minimum"]),
            int(profile["maximum"]),
            float(profile["curve"]),
        )

    def _set_transport_disconnected(self, status: str = "Disconnected") -> None:
        self._transport = None
        self._connected_device_name = None
        self._gps_speed.invalidate()
        self._navigation.set_enabled(False, time.monotonic())
        self._navigation_requires_center = False
        self._keyboard_keys.clear()
        self._keyboard_turn = 0.0
        self._keyboard_thrust = 0.0
        self._keyboard_yaw = 0.0
        self._set_state(
            serialStatus=status,
            serialTarget="No port selected",
            currentDraw="N/A",
            navigationAssistEnabled=False,
            navigationAssistActive=False,
            navigationStatus="Assist off: buoy disconnected",
            navigationReleaseRequired=False,
            navigationCorrectionDeg=0.0,
            keyboardDriveEnabled=(self._state.get("controlInputMode") == "keyboard"),
            keyboardInputSummary="Idle",
            rearMotorActualDrive=0,
            frontLeftMotorActualDrive=0,
            frontRightMotorActualDrive=0,
            rearMotorPwm=-1,
            frontLeftMotorPwm=-1,
            frontRightMotorPwm=-1,
            motorOutputTelemetry=False,
            navigationGpsSpeedAvailable=False,
            navigationGpsSpeedMps=0.0,
            navigationBestSpeedMps=0.0,
            navigationSpeedSource="IMU estimate",
            navigationSpeedConfidencePercent=0,
        )

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
        self._close_test_session("application_exit")
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
        input_mode = str(self._state.get("controlInputMode", "controller"))
        if input_mode == "controller":
            joystick_turn, joystick_thrust, joystick_yaw = read_motion_axes(
                self._joystick, self._args.deadzone
            )
        else:
            joystick_turn, joystick_thrust, joystick_yaw = 0.0, 0.0, 0.0
        turn, thrust, yaw = _select_manual_input(
            input_mode,
            joystick_turn,
            joystick_thrust,
            self._keyboard_turn,
            self._keyboard_thrust,
            self._keyboard_yaw,
            joystick_yaw,
        )
        if self._navigation_requires_center:
            if turn == 0.0 and thrust == 0.0 and yaw == 0.0:
                self._navigation_requires_center = False
                self._set_state(navigationReleaseRequired=False)
            else:
                turn = 0.0
                thrust = 0.0
                yaw = 0.0
        requested_command = build_manual_command(turn, thrust, yaw)
        navigation_now = time.monotonic()
        navigation_solution = self._navigation.solve(turn, thrust, navigation_now)
        corrected_turn = navigation_solution.corrected_x
        corrected_thrust = navigation_solution.corrected_y
        corrected_yaw = yaw
        corrected_turn, corrected_thrust, corrected_yaw = _limit_motion_to_motor_output(
            corrected_turn,
            corrected_thrust,
            corrected_yaw,
            self._mixer_power_limit,
        )
        self._command = build_manual_command(corrected_turn, corrected_thrust, corrected_yaw)

        navigation_status = navigation_solution.status
        if not self._navigation.zeroed:
            navigation_status = "Zero the IMU while the buoy is stationary"
        elif self._navigation_requires_center:
            navigation_status = "Stopped; center controller to resume"
        elif not self._navigation.enabled and not navigation_status.startswith("Assist stopped"):
            navigation_status = "IMU zeroed; assist off"

        self._set_state(
            turn=self._command.turn,
            thrust=self._command.thrust,
            yaw=self._command.yaw,
            rearMotor=self._command.rear_motor,
            frontLeftMotor=self._command.front_left_motor,
            frontRightMotor=self._command.front_right_motor,
            rearMotorTargetPwm=self._motor_pwm_for_percent("rear", self._command.rear_motor),
            frontLeftMotorTargetPwm=self._motor_pwm_for_percent("front_left", self._command.front_left_motor),
            frontRightMotorTargetPwm=self._motor_pwm_for_percent("front_right", self._command.front_right_motor),
            navigationAssistEnabled=self._navigation.enabled,
            navigationAssistActive=navigation_solution.active,
            navigationReleaseRequired=self._navigation_requires_center,
            navigationImuReady=(
                self._navigation.zeroed and self._navigation.measurement_is_fresh(navigation_now)
            ),
            navigationStatus=navigation_status,
            navigationRequestedTurn=requested_command.turn,
            navigationRequestedThrust=requested_command.thrust,
            navigationRequestedYaw=requested_command.yaw,
            navigationCorrectedTurn=self._command.turn,
            navigationCorrectedThrust=self._command.thrust,
            navigationCorrectedYaw=self._command.yaw,
            navigationMeasuredXG=round(self._navigation.measured_x_g, 4),
            navigationMeasuredYG=round(self._navigation.measured_y_g, 4),
            navigationMeasuredMagnitudeG=round(self._navigation.measured_magnitude_g, 4),
            navigationDirectionErrorDeg=round(navigation_solution.direction_error_deg, 1),
            navigationCorrectionDeg=round(navigation_solution.correction_deg, 1),
            navigationRequestedRearMotor=requested_command.rear_motor,
            navigationRequestedFrontLeftMotor=requested_command.front_left_motor,
            navigationRequestedFrontRightMotor=requested_command.front_right_motor,
            navigationCorrectedRearMotor=self._command.rear_motor,
            navigationCorrectedFrontLeftMotor=self._command.front_left_motor,
            navigationCorrectedFrontRightMotor=self._command.front_right_motor,
        )

        if (
            navigation_solution.active
            and navigation_now - self._last_navigation_log_time >= 0.25
        ):
            self._last_navigation_log_time = navigation_now
            self._append_comm_log(
                "NAV "
                f"requested={requested_command.turn:+d},{requested_command.thrust:+d},yaw={requested_command.yaw:+d} "
                f"corrected={self._command.turn:+d},{self._command.thrust:+d},yaw={self._command.yaw:+d} "
                f"measured_g={navigation_solution.measured_x_g:+.3f},{navigation_solution.measured_y_g:+.3f} "
                f"yaw_dps={float(self._state.get('navigationMeasuredYawDps', 0.0)):+.1f} "
                f"error_deg={navigation_solution.direction_error_deg:+.1f} "
                f"correction_deg={navigation_solution.correction_deg:+.1f}"
            )

        now = time.monotonic()
        min_send_interval = 1.0 / max(self._args.send_rate, 0.1)
        should_send_keepalive_ping = (
            self._transport is not None
            and not self._args.hello_ping
            and not self._args.send_text
            and self._command.turn == 0
            and self._command.thrust == 0
            and self._command.yaw == 0
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
            elif (now - self._last_send_time) >= min_send_interval and not (
                self._command.turn == 0 and self._command.thrust == 0 and self._command.yaw == 0
            ):
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

    def _update_navigation_imu(
        self,
        accel_x_g: float,
        accel_y_g: float,
        accel_z_g: float = 1.0,
        gyro_x_dps: float | None = None,
        gyro_y_dps: float | None = None,
        gyro_z_dps: float | None = None,
    ) -> None:
        if gyro_x_dps is not None and gyro_y_dps is not None and gyro_z_dps is not None:
            self._last_imu_gyro = (float(gyro_x_dps), float(gyro_y_dps), float(gyro_z_dps))
        active_gyro_x, active_gyro_y, active_gyro_z = self._last_imu_gyro
        requested_turn = int(self._state.get("navigationRequestedTurn", 0))
        requested_thrust = int(self._state.get("navigationRequestedThrust", 0))
        requested_yaw = int(self._state.get("navigationRequestedYaw", 0))
        stationary = abs(requested_turn) < 5 and abs(requested_thrust) < 5 and abs(requested_yaw) < 5
        now = time.monotonic()
        self._navigation.update_measurement(
            accel_x_g,
            accel_y_g,
            now,
            accel_z_g=accel_z_g,
            gyro_x_dps=active_gyro_x,
            gyro_y_dps=active_gyro_y,
            stationary=stationary,
        )
        updates: dict[str, object] = {
            "navigationImuReady": self._navigation.zeroed and self._navigation.measurement_is_fresh(now),
            "navigationMeasuredXG": round(self._navigation.measured_x_g, 4),
            "navigationMeasuredYG": round(self._navigation.measured_y_g, 4),
            "navigationMeasuredMagnitudeG": round(self._navigation.measured_magnitude_g, 4),
            "navigationRollDeg": round(self._navigation.roll_deg, 1),
            "navigationPitchDeg": round(self._navigation.pitch_deg, 1),
            "navigationEstimatedVelocityXMps": round(self._navigation.estimated_velocity_x_mps, 3),
            "navigationEstimatedVelocityYMps": round(self._navigation.estimated_velocity_y_mps, 3),
            "navigationEstimatedSpeedMps": round(self._navigation.estimated_speed_mps, 3),
            "navigationTiltCompensating": self._navigation.tilt_compensating,
            "navigationTiltRateDps": round(self._navigation.tilt_rate_dps, 1),
        }
        if gyro_z_dps is not None:
            updates["navigationMeasuredYawDps"] = round(active_gyro_z, 1)
        self._set_state(**updates)
        self._refresh_speed_source()

    def _update_gps_fix(self, latitude: float, longitude: float) -> None:
        self._gps_speed.update(latitude, longitude, time.time())
        self._refresh_speed_source()

    def _refresh_speed_source(self) -> None:
        gps_speed = self._gps_speed.speed_mps(time.time())
        if gps_speed is not None:
            updates: dict[str, object] = {
                "navigationGpsSpeedAvailable": True,
                "navigationGpsSpeedMps": round(gps_speed, 3),
                "navigationBestSpeedMps": round(gps_speed, 3),
                "navigationSpeedSource": "GPS ground speed",
                "navigationSpeedConfidencePercent": 90,
            }
        else:
            imu_confidence = self._navigation.velocity_confidence_percent(time.monotonic())
            if self._state.get("navigationTestEnvironment") == "dry":
                imu_confidence = min(imu_confidence, 25)
            updates = {
                "navigationGpsSpeedAvailable": False,
                "navigationGpsSpeedMps": 0.0,
                "navigationBestSpeedMps": round(self._navigation.estimated_speed_mps, 3),
                "navigationSpeedSource": "IMU estimate",
                "navigationSpeedConfidencePercent": imu_confidence,
            }
        changed = {key: value for key, value in updates.items() if self._state.get(key) != value}
        if changed:
            self._set_state(**changed)

    def _process_text_line(self, response_text: str) -> None:
        self._set_state(lastResponse=response_text)
        if response_text.startswith("TEL MOTOR OUT "):
            output = _parse_motor_output_telemetry(response_text)
            if output is not None:
                self._last_motor_output_at = time.time()
                rear_drive, rear_pwm, left_drive, left_pwm, right_drive, right_pwm = output
                self._set_state(
                    rearMotorActualDrive=rear_drive,
                    frontLeftMotorActualDrive=left_drive,
                    frontRightMotorActualDrive=right_drive,
                    rearMotorPwm=rear_pwm,
                    frontLeftMotorPwm=left_pwm,
                    frontRightMotorPwm=right_pwm,
                    motorOutputTelemetry=True,
                )
                self._last_protocol_response_time = time.time()
            return
        self._append_comm_log(f"RX {response_text}")

        ack = parse_acknowledgement(response_text)
        if ack is not None:
            ack_kind, ack_values = ack
            if ack_kind == "CTRL" and len(ack_values) >= 3 and ack_values[0] == "VECTOR":
                self._set_state(ackVector=f"{ack_values[1]},{ack_values[2]}")
            elif ack_kind == "CTRL" and len(ack_values) >= 4 and ack_values[0] == "MOTION":
                self._set_state(ackVector=f"{ack_values[1]},{ack_values[2]}, yaw {ack_values[3]}")
            elif ack_kind == "CTRL" and len(ack_values) >= 2 and ack_values[0] == "HOLD":
                self._set_state(holdPosition=(ack_values[1] == "ON"))

        status_update = parse_status_update(response_text)
        if status_update is not None:
            status_key, status_values = status_update
            if status_key == "MODE" and status_values:
                self._set_state(controllerMode=status_values[0].lower())
            elif status_key == "POS":
                self._set_state(currentLocation=_format_location(status_values))
                if len(status_values) >= 2 and "UNKNOWN" not in {value.upper() for value in status_values[:2]}:
                    try:
                        self._update_gps_fix(float(status_values[0]), float(status_values[1]))
                    except ValueError:
                        self._gps_speed.invalidate()
                else:
                    self._gps_speed.invalidate()
                    self._refresh_speed_source()
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
                if len(science_values) >= 2:
                    try:
                        self._update_navigation_imu(
                            float(science_values[0]),
                            float(science_values[1]),
                            float(science_values[2]) if len(science_values) >= 3 else 1.0,
                        )
                    except ValueError:
                        pass
            elif science_key == "IMU_GYRO":
                self._set_state(imuGyro=_parse_vector3_text(" ".join(science_values), "dps"))
                if len(science_values) >= 3:
                    try:
                        self._last_imu_gyro = tuple(float(value) for value in science_values[:3])
                        self._set_state(navigationMeasuredYawDps=round(self._last_imu_gyro[2], 1))
                    except ValueError:
                        pass
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
                if motor_name in self._motor_calibrations and direction in {"forward", "reverse"}:
                    calibration = self._motor_calibrations[motor_name]
                    calibration[direction] = {
                        "minimum": sustain_min_pwm,
                        "maximum": max_pwm,
                        "curve": curve,
                    }
                    calibration["default_reversed"] = default_reversed
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
                    rearMotorTargetPwm=self._motor_pwm_for_percent("rear", int(self._state.get("rearMotor", 0))),
                    frontLeftMotorTargetPwm=self._motor_pwm_for_percent(
                        "front_left", int(self._state.get("frontLeftMotor", 0))
                    ),
                    frontRightMotorTargetPwm=self._motor_pwm_for_percent(
                        "front_right", int(self._state.get("frontRightMotor", 0))
                    ),
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

    def _record_test_sample(
        self,
        sequence: int,
        firmware_timestamp_ms: int,
        accel_x_g: float,
        accel_y_g: float,
        accel_z_g: float,
        gyro_x_dps: float,
        gyro_y_dps: float,
        gyro_z_dps: float,
        temperature_c: float,
    ) -> None:
        session = self._test_session
        if session is None or not session.active:
            return

        captured_at = time.time()
        quality = self._test_imu_quality
        if quality is None:
            return
        session_received = quality.received_count
        session_missing = quality.missing_count
        session_duplicates = quality.duplicate_count
        session_out_of_order = quality.out_of_order_count
        session_restarts = quality.restart_count
        session_elapsed_ms = quality.elapsed_firmware_ms
        session_total = session_received + session_missing
        session_loss_pct = (session_missing / session_total * 100.0) if session_total > 0 else 0.0
        session_rate_hz = (
            (session_received - 1 - session_restarts) / (session_elapsed_ms / 1000.0)
            if session_received - 1 - session_restarts > 0 and session_elapsed_ms > 0
            else 0.0
        )
        arrival_interval_ms = (
            None
            if self._last_recorded_imu_arrival_at is None
            else round(max(0.0, captured_at - self._last_recorded_imu_arrival_at) * 1000.0, 1)
        )
        self._last_recorded_imu_arrival_at = captured_at

        def age_ms(last_update: float | None) -> float | None:
            if last_update is None:
                return None
            return round(max(0.0, captured_at - last_update) * 1000.0, 1)

        sample = {
            "environment": str(self._state.get("navigationTestEnvironment", "")),
            "link_status": str(self._state.get("serialStatus", "")),
            "imu": {
                "sequence": sequence,
                "firmware_timestamp_ms": firmware_timestamp_ms,
                "firmware_interval_ms": quality.last_interval_ms,
                "arrival_interval_ms": arrival_interval_ms,
                "accel_g": {"x": accel_x_g, "y": accel_y_g, "z": accel_z_g},
                "gyro_dps": {"x": gyro_x_dps, "y": gyro_y_dps, "z": gyro_z_dps},
                "temperature_c": temperature_c,
                "session_received": session_received,
                "session_missing": session_missing,
                "session_duplicates": session_duplicates,
                "session_out_of_order": session_out_of_order,
                "session_restarts": session_restarts,
                "session_loss_percent": round(session_loss_pct, 3),
                "session_effective_hz": round(session_rate_hz, 3),
            },
            "motion": {
                "input_mode": str(self._state.get("controlInputMode", "")),
                "requested_percent": {
                    "lateral": int(self._state.get("navigationRequestedTurn", 0)),
                    "thrust": int(self._state.get("navigationRequestedThrust", 0)),
                    "yaw": int(self._state.get("navigationRequestedYaw", 0)),
                },
                "output_percent": {
                    "lateral": int(self._state.get("turn", 0)),
                    "thrust": int(self._state.get("thrust", 0)),
                    "yaw": int(self._state.get("yaw", 0)),
                },
            },
            "motors": {
                "telemetry_age_ms": age_ms(self._last_motor_output_at),
                "rear": {
                    "mixed_percent": int(self._state.get("rearMotor", 0)),
                    "target_pwm": int(self._state.get("rearMotorTargetPwm", 0)),
                    "actual_drive": int(self._state.get("rearMotorActualDrive", 0)),
                    "applied_pwm": int(self._state.get("rearMotorPwm", -1)),
                },
                "front_left": {
                    "mixed_percent": int(self._state.get("frontLeftMotor", 0)),
                    "target_pwm": int(self._state.get("frontLeftMotorTargetPwm", 0)),
                    "actual_drive": int(self._state.get("frontLeftMotorActualDrive", 0)),
                    "applied_pwm": int(self._state.get("frontLeftMotorPwm", -1)),
                },
                "front_right": {
                    "mixed_percent": int(self._state.get("frontRightMotor", 0)),
                    "target_pwm": int(self._state.get("frontRightMotorTargetPwm", 0)),
                    "actual_drive": int(self._state.get("frontRightMotorActualDrive", 0)),
                    "applied_pwm": int(self._state.get("frontRightMotorPwm", -1)),
                },
            },
            "power": {
                **self._latest_power,
                "telemetry_age_ms": age_ms(self._last_power_at),
            },
            "navigation": {
                "measured_linear_accel_g": {
                    "x": float(self._state.get("navigationMeasuredXG", 0.0)),
                    "y": float(self._state.get("navigationMeasuredYG", 0.0)),
                    "magnitude": float(self._state.get("navigationMeasuredMagnitudeG", 0.0)),
                },
                "roll_deg": float(self._state.get("navigationRollDeg", 0.0)),
                "pitch_deg": float(self._state.get("navigationPitchDeg", 0.0)),
                "tilt_rate_dps": float(self._state.get("navigationTiltRateDps", 0.0)),
                "tilt_compensating": bool(self._state.get("navigationTiltCompensating", False)),
                "estimated_speed_mps": float(self._state.get("navigationEstimatedSpeedMps", 0.0)),
                "direction_error_deg": float(self._state.get("navigationDirectionErrorDeg", 0.0)),
                "correction_deg": float(self._state.get("navigationCorrectionDeg", 0.0)),
                "assist_active": bool(self._state.get("navigationAssistActive", False)),
            },
        }
        try:
            session.write_sample(sample, captured_at)
        except (OSError, ValueError) as exc:
            try:
                session.close("write_error")
            except OSError:
                pass
            self._test_session = None
            self._test_imu_quality = None
            self._set_state(
                testSessionActive=False,
                testSessionStatus=f"Recorder stopped: {exc}",
            )
            return
        self._set_state(
            testSessionSampleCount=session.sample_count,
            testSessionImuRateHz=session_rate_hz,
            testSessionImuLossPercent=session_loss_pct,
        )

    def _test_session_summary(self) -> dict[str, object]:
        quality = self._test_imu_quality
        if quality is None:
            return {"imu_stream": {"target_hz": 20.0, "received": 0}}
        received = quality.received_count
        missing = quality.missing_count
        elapsed_ms = quality.elapsed_firmware_ms
        expected = received + missing
        effective_hz = (
            (received - 1 - quality.restart_count) / (elapsed_ms / 1000.0)
            if received - 1 - quality.restart_count > 0 and elapsed_ms > 0
            else 0.0
        )
        return {
            "imu_stream": {
                "target_hz": 20.0,
                "received": received,
                "missing": missing,
                "duplicates": quality.duplicate_count,
                "out_of_order": quality.out_of_order_count,
                "firmware_restarts": quality.restart_count,
                "loss_percent": round((missing / expected * 100.0) if expected else 0.0, 3),
                "effective_hz": round(effective_hz, 3),
                "elapsed_firmware_ms": elapsed_ms,
            }
        }

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
                if not self._imu_quality.observe(packet.sequence, packet.timestamp_ms):
                    continue
                quality = self._imu_quality
                if self._test_imu_quality is not None:
                    self._test_imu_quality.observe(packet.sequence, packet.timestamp_ms)
                ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps, temperature_c = packet.values
                self._update_navigation_imu(ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps)
                self._set_state(
                    imuAccel=f"{ax_g:.3f}, {ay_g:.3f}, {az_g:.3f} g",
                    imuGyro=f"{gx_dps:.1f}, {gy_dps:.1f}, {gz_dps:.1f} dps",
                    imuTemperature=f"{temperature_c:.2f} C",
                    imuUdpLoss=(
                        f"{quality.loss_percent:.1f}% "
                        f"({quality.missing_count} missing / {quality.expected_count} expected; "
                        f"{quality.effective_hz:.1f} Hz)"
                    ),
                )
                self._record_test_sample(
                    packet.sequence,
                    packet.timestamp_ms,
                    ax_g,
                    ay_g,
                    az_g,
                    gx_dps,
                    gy_dps,
                    gz_dps,
                    temperature_c,
                )
            elif packet.packet_type == "POWER":
                battery_volts, current_amps, battery_pct = packet.values
                self._last_power_at = time.time()
                self._latest_power = {
                    "battery_voltage_v": battery_volts,
                    "current_a": current_amps,
                    "battery_percent": battery_pct,
                }
                self._set_state(
                    batteryStatus=_format_udp_battery_status(battery_volts, current_amps, battery_pct),
                    currentDraw=f"{current_amps:.3f} A",
                )
            elif packet.packet_type == "STATE":
                mode_value, hold_enabled, gps_valid, _motors_enabled = packet.values
                current_location = str(self._state.get("currentLocation", "Unknown")) if gps_valid else "Unknown"
                if not gps_valid:
                    self._gps_speed.invalidate()
                    self._refresh_speed_source()
                self._set_state(
                    controllerMode=_decode_control_mode(mode_value),
                    holdPosition=bool(hold_enabled),
                    currentLocation=current_location,
                )
            elif packet.packet_type == "GPS":
                latitude, longitude = packet.values
                self._set_state(currentLocation=_format_udp_position(latitude, longitude))
                self._update_gps_fix(latitude, longitude)
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
        self._refresh_speed_source()
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
    keyboard_drive_filter = KeyboardDriveFilter(backend)
    app.installEventFilter(keyboard_drive_filter)
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

