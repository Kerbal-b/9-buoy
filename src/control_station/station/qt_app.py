from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
from array import array
from collections import deque
from dataclasses import asdict
from datetime import datetime, timezone
import math
import json
import ipaddress
from pathlib import Path
import queue
import shutil
import socket
import sys
import time
import threading
import zlib

try:
    import pygame
except ModuleNotFoundError:
    pygame = None
from PySide6.QtCore import QCoreApplication, QEvent, QObject, Property, QStandardPaths, QThreadPool, QRunnable, QTimer, Qt, Signal, Slot, QUrl
from PySide6.QtGui import QGuiApplication
from PySide6.QtQml import QQmlApplicationEngine
from PySide6.QtWebEngineQuick import QtWebEngineQuick

from .geometry import ACTIVE_MOTOR_MIX, MOTOR_OUTPUT_BOOST, build_manual_command
from .models import ScienceSample
from .mission_plans import (MissionStore, boundary_point_near, generate_automatic_waypoints,
                            mission_estimate, new_plan, point_inside_boundary, rebuild_mission_routes, untangle_boundary,
                            uuid4_from_string, valid_point, validate_plan)
from .power_history import POWER_HISTORY_WINDOWS, add_power_sample, load_power_histories, save_power_histories
from .mavlink_telemetry import decode_datagram, gcs_heartbeat
from .navigation import GpsSpeedTracker, NavigationAssist
from .log_categories import (
    NAVIGATION_LOG,
    SCIENTIFIC_LOG,
    classify_log_entry,
    communication_kind,
    communication_severity,
    communication_source,
    display_timestamp,
    interpret_log_entry,
)
from .protocol import format_firmware_version, is_protocol_message, parse_acknowledgement, parse_error, parse_science_update, parse_status_update
from .radio_wifi import discover_tx_backpack
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
from .sd_card_client import delete_sd_session, download_sd_file, download_sd_logs, download_sd_session, list_sd_files, list_sd_session
from .test_session import PersistentTestSession
from .telemetry_quality import ImuStreamQuality
from .mixer_calibration_csv import (
    analyze_calibration_csv,
    compare_calibration_trials,
    load_candidate_json,
    make_candidate_json,
)
from .tx12_mapping import DEFAULT_TX12_MAPPING_PATH, load_tx12_mapping

DEFAULT_MIXER_POWER_LIMIT = 0.50
ELRS_MAVLINK_UDP_PORT = 14550
BUOY_ROUTER_NETWORK = ipaddress.IPv4Network("192.168.8.0/24")
MISSION_START_BLOCK_REASONS = {
    0: "", 1: "Cycle the arm switch low, then high",
    2: "Set power to minimum and center the sticks",
    3: "Wi-Fi control currently owns the buoy",
    4: "Mission setup is not ready",
    5: "GPS fix must be fresh, with 5 satellites and HDOP 3.0 or lower",
    6: "Buoy GPS position is outside the mission boundary",
    7: "Move within 30 m of mission Home",
    8: "Magnetometer calibration or heading is unavailable",
    9: "Cycle SE low before retrying Mission",
    10: "Load a V4 mission with Home and a boundary",
    11: "Buoy SD card is unavailable",
    12: "Stop or save the existing science session first",
    13: "Mission route point or speed is invalid",
    14: "Move the arm switch fully high",
    15: "Buoy service mode or SD transfer is active",
    16: "Could not create the mission's SD session; check the card and retry",
}


class _MissionTilesSignals(QObject):
    finished = Signal(str, str)


class _MissionTilesTask(QRunnable):
    def __init__(self, store: MissionStore, plan_id: str) -> None:
        super().__init__()
        self.setAutoDelete(False)
        self.store = store
        self.plan_id = plan_id
        self.signals = _MissionTilesSignals()

    def run(self) -> None:
        try:
            count = self.store.prepare_offline_tiles(self.plan_id)
            message = f"Offline map ready ({count} new USGS tiles; zoom 10–16)"
        except Exception as exc:
            message = f"Mission saved; offline map unavailable: {exc}"
        self.signals.finished.emit(self.plan_id, message)


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
    parser.add_argument("--send-rate", type=float, default=10.0, help="How many command updates to send per second")
    parser.add_argument("--hello-ping", action="store_true", help="Send hello world text messages over the active transport instead of CTRL commands")
    parser.add_argument("--send-text", help="Send this text over the active transport instead of CTRL commands")
    parser.add_argument("--send-interval", type=float, default=1.0, help="Seconds between repeated text messages when --send-text or --hello-ping is enabled")
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


def _update_channel_waveforms(
    left_buffer: deque[float], right_buffer: deque[float], samples: array, channels: int
) -> tuple[float, float | None]:
    if channels <= 0:
        return 0.0, None
    left_values: list[int] = []
    right_values: list[int] = []
    for index in range(0, len(samples) - channels + 1, channels):
        left_values.append(int(samples[index]))
        if channels > 1:
            right_values.append(int(samples[index + 1]))
    if not left_values:
        return 0.0, (0.0 if channels > 1 else None)

    def append_centered(values: list[int], buffer: deque[float]) -> float:
        offset = sum(values) / len(values)
        centered = [value - offset for value in values]
        peak = max(abs(value) for value in centered) / 32768.0
        gain = min(100.0, 0.75 / peak) if peak > 0.0 else 1.0
        for value in centered:
            buffer.append(max(-1.0, min(1.0, (value / 32768.0) * gain)))
        return peak * 100.0

    left_peak = append_centered(left_values, left_buffer)
    right_peak = append_centered(right_values, right_buffer) if channels > 1 else None
    return left_peak, right_peak


def _audio_channel_rms_percent(samples: array, channels: int) -> tuple[float, float | None]:
    if channels <= 0:
        return 0.0, None
    values = ([], [])
    for index in range(0, len(samples) - channels + 1, channels):
        values[0].append(int(samples[index]))
        if channels > 1:
            values[1].append(int(samples[index + 1]))

    def rms_percent(channel_samples: list[int]) -> float:
        if not channel_samples:
            return 0.0
        mean = sum(channel_samples) / len(channel_samples)
        mean_square = sum(value * value for value in channel_samples) / len(channel_samples)
        centered_rms = math.sqrt(max(0.0, mean_square - mean * mean))
        return centered_rms / 32768.0 * 100.0

    return rms_percent(values[0]), (rms_percent(values[1]) if channels > 1 else None)


def _clamp_axis(value: float) -> float:
    return max(-1.0, min(1.0, value))


def _select_manual_input(
    input_mode: str,
    keyboard_turn: float,
    keyboard_thrust: float,
    keyboard_yaw: float,
) -> tuple[float, float, float]:
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
    translation_scale = (2.0 / 3.0) * MOTOR_OUTPUT_BOOST
    mixed = tuple(
        translation_scale * ((lateral * axis[0]) + (thrust * axis[1])) + (yaw * yaw_gain)
        for axis, yaw_gain in zip(ACTIVE_MOTOR_MIX.axes, ACTIVE_MOTOR_MIX.yaw_gains)
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
    logChanged = Signal()
    audioWaveformChanged = Signal()
    dashboardTabChanged = Signal()
    audioMutedChanged = Signal()

    def __init__(self, args: argparse.Namespace, log_file: Path, prefs_path: Path,
                 mission_folder: Path | None = None) -> None:
        super().__init__()
        self._args = args
        self._log_file = log_file
        self._compass_load_baseline_path = log_file.parent / "system" / "compass_load_baseline.json"
        self._compass_load_baseline: dict[str, object] | None = None
        try:
            loaded_baseline = json.loads(self._compass_load_baseline_path.read_text(encoding="utf-8"))
            if isinstance(loaded_baseline, dict) and isinstance(loaded_baseline.get("means"), dict):
                self._compass_load_baseline = loaded_baseline
        except (OSError, ValueError):
            pass
        self._link_log_file = log_file.with_name(log_file.stem + "-link-events.jsonl")
        self._prefs_path = prefs_path
        self._mission_store = MissionStore(mission_folder or prefs_path.parent / "missions")
        self._mission_tile_tasks: list[_MissionTilesTask] = []
        self._mission_draft = new_plan()
        self._mission_upload_queue: list[str] = []
        self._mission_upload_payload = b""
        self._mission_upload_active = False
        self._mission_upload_waiting_for_readback = False
        self._mission_upload_timeout_timer = QTimer(self)
        self._mission_upload_timeout_timer.setSingleShot(True)
        self._mission_upload_timeout_timer.setInterval(10000)
        self._mission_upload_timeout_timer.timeout.connect(self._on_mission_upload_timeout)
        self._mission_revision = 0
        self._tx12_mapping_path = DEFAULT_TX12_MAPPING_PATH
        self._state: dict[str, object] = {
            "radioWifiStatus": "Not scanned",
            "radioWifiIp": "",
            "radioWifiScanning": False,
            "linkLogExportStatus": "",
            "networkBuoyDevices": [],
            "networkScanStatus": "Looking for the 192.168.8.0/24 buoy network",
            "networkScanning": False,
            "wifiReceivedMessages": 0,
            "wifiReceivedBytes": 0,
            "wifiMessageRate": 0.0,
            "wifiThroughputKib": 0.0,
            "wifiTrafficStatus": "No active buoy control link",
            "wifiSafetyTelemetryLive": False,
            "wifiDetailTelemetryLive": False,
            "tcpPort": int(args.tcp_port),
            "udpPort": int(args.udp_port),
            "audioPort": int(args.audio_port),
            "radioTelemetryStatus": "Not listening",
            "radioTelemetryPackets": 0,
            "elrsPacketLogging": False,
            "radioSystemId": "Unknown",
            "radioLinkStatus": "No receiver RADIO_STATUS relayed",
            "radioRssiStatus": "Unknown",
            "radioLinkQuality": "Waiting for ELRS receiver status",
            "radioSnrStatus": "Unknown",
            "radioBatteryStatus": "N/A",
            "radioCurrentDraw": "N/A",
            "radioLocation": "Unknown",
            "radioGroundMotion": "Unknown",
            "radioBuoyMode": "Unknown",
            "radioMotorOutput": "Unknown",
            "radioRecordingStatus": "Unknown",
            "radioAudioRecording": None,
            "radioAssistMode": "Unknown",
            "radioCalibrationStatus": "Waiting for calibration telemetry",
            "radioCalibrationStep": 0,
            "radioCalibrationPower": 0,
            "radioCalibrationMaxPower": 0,
            "radioCalibrationDuration": 0,
            "radioMissionStatus": "No mission mode yet",
            "missionStartBlockReason": "",
            "missionDraft": self._mission_draft,
            "missionPreviewPlan": self._mission_draft,
            "missionPreviewSelectedId": self._mission_draft["id"],
            "missionPreviewIsDraft": True,
            "missionPreviewMessage": "Previewing the current draft",
            "missionProgress": None,
            "radioMissionProgress": None,
            "missionUploadPending": False,
            "missionUploadSucceeded": False,
            "buoyMissionExpectedPointCount": 0,
            "buoyMissionPointCount": 0,
            "buoyMissionPoints": [],
            "buoyMissionPointsId": "NONE",
            "buoyMissionHome": None,
            "buoyMissionEnd": None,
            "buoyMissionBoundary": [],
            "buoyMissionReadId": "",
            "buoyMissionList": [],
            "buoyMissionScanComplete": False,
            "buoyMissionInventoryStatus": "",
            "buoyMissionLoadedId": "NONE",
            "buoyMissionRunningId": "NONE",
            "transportMode": args.transport,
            "missionRevision": 0,
            "missionList": self._mission_store.list_plans(),
            "missionIssues": validate_plan(self._mission_draft),
            "missionDirty": False,
            "missionMessage": "Upload a mission, then select Mission on SB/CH7 and arm with SE/CH5. Propulsion stops when Mission is deselected or disarmed.",
            "radioBuoyState": None,
            "radioChannels": [],
            "manualHomeStatus": "Not available",
            "manualHomeStorageStatus": "Unknown",
            "activeHomeStatus": "No active Home",
            "activeHomeCoordinates": "Unknown",
            "homeDistanceMeters": None,
            "tx12Controls": [],
            "tx12MapPath": str(self._tx12_mapping_path),
            "tx12MapStatus": "Mapping not loaded",
            "telemetrySource": "No live telemetry",
            "telemetryMode": "WIFI",
            "buoyMode": "idle",
            "controlInputMode": "tx12",
            "controlInputModeDetail": "TX12 controls the buoy through the ELRS receiver",
            "controlSource": "NONE",
            "radioArmStatus": "DISARMED",
            "recoveryStatus": "READY",
            "mixerPowerPercent": int(round(DEFAULT_MIXER_POWER_LIMIT * 100.0)),
            "serialStatus": "Disconnected",
            "serialTarget": "No port selected",
            "firmwareVersion": "Unknown",
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
            "motorEnergyWh": {"rear": 0.0, "left": 0.0, "right": 0.0},
            "motorEnergyEstimatorStatus": "Waiting for motor and battery telemetry",
            "batteryStatus": "N/A",
            "currentLocation": "Unknown",
            "targetLocation": "Not set",
            "holdPosition": False,
            "currentDraw": "N/A",
            "powerHistory": [],
            "powerHistoryTitle": "Live buoy power",
            "powerHistoryStatus": "Waiting for buoy Wi-Fi power telemetry",
            "powerHistorySampleCount": 0,
            "powerHistoryIntervalSeconds": 300,
            "powerHistoryWindowSeconds": 86400,
            "currentDepth": "N/A",
            "waterTemperature": "N/A",
            "conductivityVoltage": "N/A",
            "turbidityVoltage": "N/A",
            "airTemperature": "N/A",
            "imuAccel": "N/A",
            "imuGyro": "N/A",
            "imuTemperature": "N/A",
            "magnetometerRaw": "N/A",
            "magnetometerRawX": None,
            "magnetometerRawY": None,
            "magnetometerRawZ": None,
            "magnetometerHeadingDeg": None,
            "magnetometerTiltHeadingDeg": None,
            "magnetometerTiltStatus": "Waiting for calibrated level",
            "navigationSeaAccelerationRmsG": None,
            "navigationSeaAngularRateRmsDps": None,
            "magnetometerStatus": "Unknown",
            "magnetometerCalibrationActive": False,
            "magnetometerCalibrationSaved": False,
            "magnetometerCalibrationSamples": 0,
            "magnetometerCalibrationStatus": "No saved calibration",
            "magnetometerBuoyConfigStatus": "Not checked",
            "imuUdpLoss": "N/A",
            "audioStream": "N/A",
            "audioLevel": "N/A",
            "audioChannelCount": 0,
            "audioCaptureChannel": "BOTH",
            "audioRecordingEnabled": False,
            "audioSdStatus": "Unknown",
            "audioSampleRateHz": 16000,
            "audioQuality": "DEFAULT",
            "audioQualityLocked": False,
            "sdFiles": [],
            "sdCardCapacity": {},
            "sdSessions": [],
            "scienceExperimentState": "IDLE",
            "scienceSessionPath": "",
            "serviceMode": False,
            "sdDownloadActive": False,
            "sdDownloadSessionPath": "",
            "audioStreamEnabled": False,
            "imuStreamEnabled": False,
            "sdCardStatus": "Connect over Wi-Fi, then refresh to browse the card",
            "sdTransferStatus": "Idle",
            "sdTransferProgress": 0.0,
            "sdTransferBytes": 0,
            "sdTransferTotalBytes": 0,
            "sdTransferSpeedBytesPerSecond": 0.0,
            "sdTransferElapsedSeconds": 0.0,
            "sdTransferEtaSeconds": -1.0,
            "sdTransferFileName": "",
            "sdBusy": False,
            "sdSelectedPath": "",
            "audioPacketLossStatus": "Packet loss unavailable until the audio firmware is updated",
            "motorConfigFetchId": 0,
            "motorConfigMotor": "",
            "motorConfigDirection": "",
            "motorConfigurations": {},
            "motorCalibrationStatus": "",
            "motorCalibrationAckId": 0,
            "motorCalibrationAckMotor": "",
            "mixerCalibrationRows": [],
            "mixerCalibrationStatus": "Upload an AI-processed mixer profile to populate the table and update the buoy",
            "mixerCalibrationProfileLoaded": False,
            "mixerCalibrationProfileComplete": False,
            "mixerCalibrationProfileActive": False,
            "mixerCalibrationStoredCount": 0,
            "mixerCalibrationCrc": "",
            "mixerCalibrationSourceLog": "",
            "mixerCalibrationAnalysisJson": "",
            "mixerCalibrationCandidateJsonPath": "",
            "mixerCalibrationComparisonStatus": "",
            "compassLoadCapturePhase": "",
            "compassLoadCaptureStatus": "Select motors-off baseline or motor-load capture in Navigation",
            "compassLoadBaselineStatus": "Loaded previous motors-off water baseline" if self._compass_load_baseline else "No water motors-off baseline saved",
            "compassLoadLastSummary": {},
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
            "buoyMixerInput": None,
            "navigationCorrectedTurn": 0,
            "navigationCorrectedThrust": 0,
            "navigationCorrectedYaw": 0,
            "navigationMeasuredXG": 0.0,
            "navigationMeasuredYG": 0.0,
            "navigationMeasuredMagnitudeG": 0.0,
            "navigationMeasuredYawDps": 0.0,
            "navigationRollDeg": 0.0,
            "navigationPitchDeg": 0.0,
            "navigationLevelCalibrated": False,
            "navigationLevelCalibrationStatus": "Calibrate while the buoy is floating level",
            "navigationEstimatedVelocityXMps": 0.0,
            "navigationEstimatedVelocityYMps": 0.0,
            "navigationEstimatedSpeedMps": 0.0,
            "navigationSpeedConfidencePercent": 0,
            "navigationTiltCompensating": False,
            "navigationTiltRateDps": 0.0,
            "navigationGpsSpeedAvailable": False,
            "navigationGpsSpeedMps": 0.0,
            "navigationGpsCourseAvailable": False,
            "navigationGpsCourseDeg": 0.0,
            "navigationHeadingCourseOffsetDeg": 0.0,
            "navigationGpsTrack": [],
            "gpsSatelliteCount": None,
            "gpsHdop": None,
            "gpsFixValid": False,
            "gpsMinimumSatellites": 6,
            "gpsQualityStatus": "Waiting for GPS data",
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
        self._state["mixerCalibrationRows"] = self._empty_mixer_calibration_rows()
        self.reloadTx12Mapping()
        self._comm_log: list[str] = []
        self._command_trace_pending: list[str] = []
        self._link_log_pending: list[str] = []
        self._link_log_sequence = 0
        self._link_log: list[dict[str, object]] = []
        self._last_command_trace_flush = time.monotonic()
        self._categorized_log: list[dict[str, str]] = []
        self._system_log: list[dict[str, str]] = []
        self._navigation_log: list[dict[str, str]] = []
        self._scientific_log: list[dict[str, str]] = []
        self._science_history: list[dict[str, object]] = []
        self._audio_waveform: list[float] = [0.0] * 480
        self._audio_left_waveform: list[float] = [0.0] * 480
        self._audio_right_waveform: list[float] = [0.0] * 480
        self._audio_left_level = "N/A"
        self._audio_right_level = "N/A"
        self._audio_left_rms_percent = 0.0
        self._audio_right_rms_percent = 0.0
        self._audio_sample_rate = 0
        self._audio_channels = 0
        self._audio_packet_version = 0
        self._audio_last_sequence: int | None = None
        self._audio_packets_received = 0
        self._audio_packets_lost = 0
        self._audio_packets_out_of_order = 0
        self._audio_playback_drops = 0
        self._last_audio_ui_update = 0.0
        self._audio_high_pass = True
        self._audio_high_pass_hz = 80.0
        self._audio_low_pass = True
        self._audio_low_pass_hz = 6000.0
        self._audio_noise_gate = False
        self._audio_noise_gate_percent = 2.5
        self._audio_filter_previous_x = [0.0, 0.0]
        self._audio_filter_previous_y = [0.0, 0.0]
        self._audio_filter_low_pass_y = [0.0, 0.0]
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
                "default_reversed": True,
            },
            "front_right": {
                "forward": dict(default_profile),
                "reverse": dict(default_profile),
                "default_reversed": True,
            },
        }
        self._pending_motor_calibration: dict[str, set[str]] = {}
        self._dashboard_tab = "overview"
        self._audio_muted = False
        self._network_interfaces = list_wifi_interfaces()
        self._wifi_local_ip = getattr(args, "wifi_local_ip", "auto") or "auto"
        self._waveform_buffer: deque[float] = deque([0.0] * 480, maxlen=480)
        self._audio_left_buffer: deque[float] = deque([0.0] * 480, maxlen=480)
        self._audio_right_buffer: deque[float] = deque([0.0] * 480, maxlen=480)
        self._audio_debug_channel = "both"
        self._audio_debug_gain = 100.0
        self._audio_channel = None
        self._mixer_ready = False
        self._audio_playback_queue: queue.Queue[object] = queue.Queue(maxsize=8)
        self._audio_playback_pending = bytearray()
        self._audio_playback_wake = threading.Event()
        self._audio_playback_running = False
        self._audio_playback_thread: threading.Thread | None = None
        self._audio_output_status = "Audio output has not started"
        self._audio_output_device = "System default"
        self._audio_output_devices = ["System default"]
        self._audio_output_device = "System default"
        self._audio_output_devices = ["System default"]
        self._audio_playback_error = ""
        self._audio_sounds_submitted = 0
        self._audio_sounds_scheduled = 0

        self._transport = None
        self._wifi_tcp_messages_total = 0
        self._wifi_traffic_transport = None
        self._wifi_traffic_sample: tuple[int, int, float] | None = None
        self._connection_thread: threading.Thread | None = None
        self._connection_lock = threading.Lock()
        self._connection_in_flight = False
        self._pending_connection: tuple[int, LinkTransport | None, str, str] | None = None
        self._connection_request_id = 0
        self._sd_operation_lock = threading.Lock()
        self._sd_pending_result: tuple[str, object] | None = None
        self._sd_progress_pending: tuple[int, int, str] | None = None
        self._sd_operation_thread: threading.Thread | None = None
        self._sd_operation_busy = False
        self._sd_transfer_started_at = 0.0
        self._sd_cancel_event = threading.Event()
        self._sd_service_requested = False
        self._radio_wifi_lock = threading.Lock()
        self._radio_wifi_pending: tuple[str | None, str] | None = None
        self._radio_wifi_scanning = False
        self._network_scan_lock = threading.Lock()
        self._network_scan_pending: tuple[list[dict[str, object]], str] | None = None
        self._network_scanning = False
        self._radio_telemetry_socket: socket.socket | None = None
        self._radio_telemetry_last_at = 0.0
        self._radio_link_last_at = 0.0
        self._radio_heartbeat_last_at = 0.0
        self._radio_telemetry_packets = 0
        self._radio_system_id: int | None = None
        self._radio_field_last_at: dict[str, float] = {}
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
        self._auto_reconnect_enabled = True
        self._ping_interval = 5.0
        self._link_timeout = 8.0
        self._imu_quality = ImuStreamQuality(expected_interval_ms=50)
        self._last_imu_gyro = (0.0, 0.0, 0.0)
        self._last_science_sample_time = 0.0
        self._mag_calibration_path = prefs_path.parent / "magnetometer_calibration.json"
        self._mag_calibration: dict[str, float] = {}
        self._mag_calibration_active = False
        self._mag_calibration_min: list[int] | None = None
        self._mag_calibration_max: list[int] | None = None
        self._mag_calibration_samples = 0
        self._last_mag_raw: tuple[float, float, float] | None = None
        self._last_mag_raw_at: float | None = None
        self._last_mixer_input_at: float | None = None
        self._last_imu_accel: tuple[float, float, float] | None = None
        self._level_calibration_path = prefs_path.parent / "imu_level_calibration.json"
        self._level_roll_offset = 0.0
        self._level_pitch_offset = 0.0
        self._gps_quality_path = prefs_path.parent / "gps_quality.json"
        try:
            level_data = json.loads(self._level_calibration_path.read_text(encoding="utf-8"))
            self._level_roll_offset = float(level_data["roll_offset_deg"])
            self._level_pitch_offset = float(level_data["pitch_offset_deg"])
            self._state.update(navigationLevelCalibrated=True,
                               navigationLevelCalibrationStatus="Waterline level saved on this station")
        except (OSError, ValueError, TypeError, KeyError, IndexError):
            pass
        try:
            gps_data = json.loads(self._gps_quality_path.read_text(encoding="utf-8"))
            minimum_satellites = max(4, min(12, int(gps_data.get("minimum_satellites", 6))))
            self._state["gpsMinimumSatellites"] = minimum_satellites
        except (OSError, ValueError, TypeError, AttributeError):
            pass
        try:
            calibration_data = json.loads(self._mag_calibration_path.read_text(encoding="utf-8"))
            if isinstance(calibration_data, dict):
                self._mag_calibration = {
                    key: float(calibration_data[key])
                    for key in ("offset_x", "offset_y", "scale_x", "scale_y")
                    if key in calibration_data
                }
        except (OSError, ValueError, TypeError):
            self._mag_calibration = {}
        if {"offset_x", "offset_y", "scale_x", "scale_y"} <= self._mag_calibration.keys():
            self._state.update(magnetometerCalibrationSaved=True,
                               magnetometerCalibrationStatus="Saved calibration loaded")
        self._navigation = NavigationAssist()
        self._gps_speed = GpsSpeedTracker()
        self._gps_track: list[tuple[float, float]] = []
        # The installed MPU6050 is rotated 90 degrees clockwise: body-right is
        # sensor -Y and body-forward is sensor +X.
        self._navigation.set_axis_signs(True, False, True)
        self._last_navigation_log_time = 0.0
        self._navigation_requires_center = False
        self._test_session: PersistentTestSession | None = None
        self._compass_load_capture_phase = ""
        self._compass_load_capture_setup: dict[str, object] = {}
        self._compass_load_capture_values: dict[str, list[float]] = {}
        self._test_imu_quality: ImuStreamQuality | None = None
        self._latest_power: dict[str, float | int | None] = {
            "battery_voltage_v": None,
            "current_a": None,
            "battery_percent": None,
        }
        self._power_history_path = prefs_path.parent / "system" / "power_history.json"
        self._power_histories = load_power_histories(self._power_history_path)
        now_epoch = time.time()
        for interval, points in self._power_histories.items():
            cutoff = now_epoch - POWER_HISTORY_WINDOWS[interval]
            points[:] = [point for point in points if float(point["t"]) >= cutoff]
        self._last_power_history_persist_bucket = 0
        ten_second_points = self._power_histories[10]
        if ten_second_points:
            self._last_power_history_persist_bucket = int(float(ten_second_points[-1]["t"]))
            self._state.update(
                powerHistory=[dict(point) for point in self._power_histories[300]],
                powerHistorySampleCount=sum(int(point["count"]) for point in self._power_histories[300]),
                powerHistoryStatus="Stored local power history loaded; waiting for live telemetry",
            )
        self._power_history_interval_seconds = 300
        self._last_power_history_publish_at = 0.0
        self._motor_energy_last_at = 0.0
        self._motor_energy_wh = {"rear": 0.0, "left": 0.0, "right": 0.0}
        self._last_recorded_imu_arrival_at: float | None = None
        self._last_motor_output_at: float | None = None
        self._last_motor_output_log_at = 0.0
        self._last_motor_test_log_command: str | None = None
        self._active_motor_test: tuple[str, int] | None = None
        self._last_motor_test_send_at = 0.0
        self._last_motor_test_actual_drive = 0
        self._last_power_at: float | None = None
        self._wifi_safety_last_at = 0.0
        self._wifi_detail_last_at = 0.0
        self._last_tick_at = 0.0

        self._timer = QTimer(self)
        self._timer.setInterval(33)
        self._timer.timeout.connect(self._tick)
        # A status snapshot updates many fields in one event-loop turn. Queue
        # one QML refresh for the completed batch instead of refreshing the
        # entire dashboard for every telemetry line.
        self._state_update_timer = QTimer(self)
        self._state_update_timer.setSingleShot(True)
        self._state_update_timer.setInterval(0)
        self._state_update_timer.timeout.connect(self.stateChanged.emit)
        self._log_update_timer = QTimer(self)
        self._log_update_timer.setSingleShot(True)
        self._log_update_timer.setInterval(250)
        self._log_update_timer.timeout.connect(self.logChanged.emit)

    @Property(dict, notify=stateChanged)
    def state(self) -> dict[str, object]:
        return self._state

    @Property(list, notify=logChanged)
    def commLog(self) -> list[str]:
        return self._comm_log

    @Property(list, notify=logChanged)
    def categorizedLog(self) -> list[dict[str, str]]:
        return self._categorized_log

    @Property(list, notify=logChanged)
    def linkLog(self) -> list[dict[str, object]]:
        return self._link_log

    @Property(list, notify=logChanged)
    def systemLog(self) -> list[dict[str, str]]:
        return self._system_log

    @Property(list, notify=logChanged)
    def navigationLog(self) -> list[dict[str, str]]:
        return self._navigation_log

    @Property(list, notify=logChanged)
    def scientificLog(self) -> list[dict[str, str]]:
        return self._scientific_log

    @Property(list, notify=stateChanged)
    def scienceHistory(self) -> list[dict[str, object]]:
        return self._science_history

    @Property(list, notify=audioWaveformChanged)
    def audioWaveform(self) -> list[float]:
        return self._audio_waveform

    @Property(list, notify=audioWaveformChanged)
    def audioLeftWaveform(self) -> list[float]:
        return self._audio_left_waveform

    @Property(list, notify=audioWaveformChanged)
    def audioRightWaveform(self) -> list[float]:
        return self._audio_right_waveform

    @Property(str, notify=stateChanged)
    def audioLeftLevel(self) -> str:
        return self._audio_left_level

    @Property(str, notify=stateChanged)
    def audioRightLevel(self) -> str:
        return self._audio_right_level

    @Property(float, notify=stateChanged)
    def audioLeftRmsPercent(self) -> float:
        return self._audio_left_rms_percent

    @Property(float, notify=stateChanged)
    def audioRightRmsPercent(self) -> float:
        return self._audio_right_rms_percent

    @Property(str, notify=stateChanged)
    def audioDebugChannel(self) -> str:
        return self._audio_debug_channel

    @Property(str, notify=stateChanged)
    def audioCaptureChannel(self) -> str:
        return str(self._state.get("audioCaptureChannel", "BOTH"))

    @Property(bool, notify=stateChanged)
    def audioRecordingEnabled(self) -> bool:
        return bool(self._state.get("audioRecordingEnabled", False))

    @Property(str, notify=stateChanged)
    def audioSdStatus(self) -> str:
        return str(self._state.get("audioSdStatus", "Unknown"))

    @Property(int, notify=stateChanged)
    def audioSampleRateHz(self) -> int:
        return int(self._state.get("audioSampleRateHz", 16000))

    @Property(str, notify=stateChanged)
    def audioQuality(self) -> str:
        return str(self._state.get("audioQuality", "DEFAULT"))

    @Property(bool, notify=stateChanged)
    def audioQualityLocked(self) -> bool:
        return bool(self._state.get("audioQualityLocked", False))

    @Property(list, notify=stateChanged)
    def sdFiles(self) -> list[dict[str, object]]:
        return list(self._state.get("sdFiles", []))

    @Property(str, notify=stateChanged)
    def sdCardStatus(self) -> str:
        return str(self._state.get("sdCardStatus", "Unknown"))

    @Property(str, notify=stateChanged)
    def sdTransferStatus(self) -> str:
        return str(self._state.get("sdTransferStatus", "Idle"))

    @Property(float, notify=stateChanged)
    def sdTransferProgress(self) -> float:
        return float(self._state.get("sdTransferProgress", 0.0))

    @Property(bool, notify=stateChanged)
    def sdBusy(self) -> bool:
        return bool(self._state.get("sdBusy", False))

    @Property(QUrl, notify=stateChanged)
    def sdDownloadSuggestedUrl(self) -> QUrl:
        destination_dir = Path(QStandardPaths.writableLocation(QStandardPaths.StandardLocation.DownloadLocation))
        remote_path = str(self._state.get("sdSelectedPath", ""))
        file_name = Path(remote_path).name or "sd-card-file.bin"
        return QUrl.fromLocalFile(str(destination_dir / file_name))

    @Property(QUrl, notify=stateChanged)
    def sdDownloadFolderUrl(self) -> QUrl:
        return QUrl.fromLocalFile(QStandardPaths.writableLocation(QStandardPaths.StandardLocation.DownloadLocation))

    @Property(float, notify=stateChanged)
    def audioDebugGain(self) -> float:
        return self._audio_debug_gain

    @Property(bool, notify=stateChanged)
    def audioHighPassEnabled(self) -> bool:
        return self._audio_high_pass

    @Property(float, notify=stateChanged)
    def audioHighPassHz(self) -> float:
        return self._audio_high_pass_hz

    @Property(bool, notify=stateChanged)
    def audioLowPassEnabled(self) -> bool:
        return self._audio_low_pass

    @Property(float, notify=stateChanged)
    def audioLowPassHz(self) -> float:
        return self._audio_low_pass_hz

    @Property(bool, notify=stateChanged)
    def audioNoiseGateEnabled(self) -> bool:
        return self._audio_noise_gate

    @Property(float, notify=stateChanged)
    def audioNoiseGatePercent(self) -> float:
        return self._audio_noise_gate_percent

    @Property(int, notify=stateChanged)
    def audioPacketsReceived(self) -> int:
        return self._audio_packets_received

    @Property(int, notify=stateChanged)
    def audioPacketsLost(self) -> int:
        return self._audio_packets_lost

    @Property(int, notify=stateChanged)
    def audioPacketsOutOfOrder(self) -> int:
        return self._audio_packets_out_of_order

    @Property(int, notify=stateChanged)
    def audioPlaybackDrops(self) -> int:
        return self._audio_playback_drops

    @Property(int, notify=stateChanged)
    def audioPlaybackQueuedPackets(self) -> int:
        return self._audio_playback_queue.qsize()

    @Property(str, notify=stateChanged)
    def audioOutputStatus(self) -> str:
        return self._audio_output_status

    @Property(list, notify=stateChanged)
    def audioOutputDevices(self) -> list[str]:
        return list(self._audio_output_devices)

    @Property(str, notify=stateChanged)
    def audioOutputDevice(self) -> str:
        return self._audio_output_device

    @Property(str, notify=stateChanged)
    def audioPacketLossStatus(self) -> str:
        if self._audio_packet_version < 2:
            return "Exact loss tracking requires the updated audio firmware"
        return (
            f"{self._audio_packets_received} packets received | "
            f"{self._audio_packets_lost} missing | "
            f"{self._audio_packets_out_of_order} duplicate/out of order | "
            f"{self._audio_playback_drops} local playback drops | "
            f"{self._audio_playback_queue.qsize()} queued"
        )

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
    def stationNetworkIp(self) -> str:
        if self._transport is not None and str(self._state.get("serialTarget", "")).startswith("WIFI:"):
            tcp_socket = getattr(self._transport, "_tcp", None)
            try:
                return str(tcp_socket.getsockname()[0]) if tcp_socket else self._wifi_local_ip
            except OSError:
                pass
        if self._wifi_local_ip and self._wifi_local_ip.lower() != "auto":
            return self._wifi_local_ip
        addresses = [entry["value"] for entry in self._network_interfaces if entry.get("value") != "auto"]
        for address in addresses:
            try:
                if ipaddress.IPv4Address(address) in BUOY_ROUTER_NETWORK:
                    return address
            except ipaddress.AddressValueError:
                continue
        return addresses[0] if addresses else "No IPv4 address detected"

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

    def _publish_mission(self, message: str = "Unsaved changes", dirty: bool = True) -> None:
        route_error = rebuild_mission_routes(self._mission_draft)
        if route_error and message in ("Unsaved changes", "Mission loaded"):
            message = route_error
        self._mission_revision += 1
        updates = dict(
            missionDraft=self._mission_draft,
            missionEstimate=mission_estimate(self._mission_draft),
            missionRevision=self._mission_revision,
            missionIssues=validate_plan(self._mission_draft),
            missionDirty=dirty,
            missionMessage=message,
        )
        if self._state.get("missionPreviewIsDraft", True):
            updates.update(missionPreviewPlan=self._mission_draft,
                           missionPreviewSelectedId=self._mission_draft["id"])
        self._set_state(**updates)

    @Slot(str)
    def selectMissionPreview(self, plan_id: str) -> None:
        try:
            plan = self._mission_store.load(plan_id)
        except (OSError, ValueError, KeyError, TypeError) as exc:
            self._set_state(missionPreviewMessage=f"Could not load mission preview: {exc}")
            return
        self._set_state(missionPreviewPlan=plan, missionPreviewSelectedId=plan["id"],
                        missionPreviewIsDraft=False,
                        missionPreviewMessage="Saved mission preview; this does not start or upload it")

    @Slot()
    def previewMissionDraft(self) -> None:
        self._set_state(missionPreviewPlan=self._mission_draft,
                        missionPreviewSelectedId=self._mission_draft["id"],
                        missionPreviewIsDraft=True,
                        missionPreviewMessage="Previewing the current draft; save it before future mission upload")

    @Slot()
    def uploadMission(self) -> None:
        """Upload one validated mission file to the buoy's SD card."""
        if self._mission_upload_active or self._mission_upload_waiting_for_readback:
            self._set_state(missionMessage="A mission upload is already in progress")
            return
        plan_id = str(self._state.get("missionPreviewSelectedId", ""))
        try:
            plan = self._mission_store.load(plan_id)
        except (OSError, ValueError, KeyError) as exc:
            self._set_state(missionMessage=f"Could not load mission for upload: {exc}")
            return
        issues = validate_plan(plan)
        points = plan.get("planned_route", [])
        if issues:
            self._set_state(missionMessage="Mission is not ready to upload: " + " ".join(issues))
            return
        if len(points) > 64:
            self._set_state(missionMessage="The safe planned route exceeds the buoy's 64-point limit; simplify or space out the mission")
            return
        if self._transport is None:
            self._set_state(missionMessage="Connect to the buoy over Wi-Fi before uploading")
            return
        if self._args.transport != "wifi":
            self._set_state(missionMessage="Mission upload requires the buoy Wi-Fi TCP connection")
            return
        boundary = plan.get("boundary", [])
        if len(boundary) > 64:
            self._set_state(missionMessage="This mission has more than the buoy limit of 64 boundary points")
            return
        measurement_bits = {"water_temperature": 1, "depth": 2, "conductivity": 4,
                            "turbidity": 8, "air_temperature": 16}
        failsafes = plan.get("failsafes", {})
        action_code = lambda name: 1 if failsafes.get(name, "return_home") == "return_home" else 0
        rows = [f"BUOY_MISSION_V4,{plan_id.upper()},{len(points)},{len(boundary)},"
                f"{int(plan.get('home') is not None)},{int(plan.get('end') is not None)},"
                f"{action_code('link_loss')},{action_code('boundary_breach')},"
                f"{action_code('low_battery')},{int(failsafes.get('battery_threshold_percent', 20))}"]
        for tag in ("HOME", "END"):
            coordinate = plan.get(tag.lower())
            if coordinate is not None:
                rows.append(f"{tag},{float(coordinate['lat']):.7f},{float(coordinate['lon']):.7f}")
        rows.extend(f"BOUNDARY,{index},{float(point['lat']):.7f},{float(point['lon']):.7f}"
                    for index, point in enumerate(boundary))
        rows.extend(
            f"POINT,{float(point['lat']):.7f},{float(point['lon']):.7f},"
            f"{sum(measurement_bits.get(name, 0) for name in point.get('measurements', []))},"
            f"{int(bool(point.get('collect_data', bool(point.get('measurements') or point.get('hold_seconds', 0)))))},"
            f"{int(point.get('hold_seconds', 0))},{int(point.get('speed_percent', 50))}"
            for point in points)
        self._mission_upload_payload = ("\n".join(rows) + "\n").encode("ascii")
        checksum = zlib.crc32(self._mission_upload_payload) & 0xFFFFFFFF
        self._mission_upload_queue = [
            f"CTRL MISSION FILE DATA {offset} {self._mission_upload_payload[offset:offset + 200].hex()}"
            for offset in range(0, len(self._mission_upload_payload), 200)
        ]
        self._mission_upload_queue.append("CTRL MISSION FILE COMMIT")
        self._mission_upload_active = True
        self._mission_upload_waiting_for_readback = False
        self._set_state(missionMessage=f"Uploading {len(points)} planned route points to the buoy SD card…",
                        missionUploadPending=True, missionUploadSucceeded=False)
        result = send_text(self._transport,
                           f"CTRL MISSION FILE BEGIN {len(self._mission_upload_payload)} {checksum:08X}")
        if result != "OK":
            self._mission_upload_queue.clear()
            self._mission_upload_active = False
            self._mission_upload_timeout_timer.stop()
            self._set_state(missionUploadPending=False, missionUploadSucceeded=False,
                            missionMessage=f"Mission upload failed: {result}")
            return
        self._mission_upload_timeout_timer.start()
        self._append_comm_log(f"TX mission file: {len(self._mission_upload_payload)} bytes, {len(self._mission_upload_queue) - 1} chunks")

    def _send_next_mission_upload_command(self) -> None:
        if not self._mission_upload_active or not self._mission_upload_queue:
            return
        command = self._mission_upload_queue.pop(0)
        result = send_text(self._transport, command)
        if result != "OK":
            self._mission_upload_queue.clear()
            self._mission_upload_active = False
            self._mission_upload_timeout_timer.stop()
            self._set_state(missionUploadPending=False,
                            missionMessage=f"Mission file upload stopped: {result}")
            return
        self._mission_upload_timeout_timer.start()

    @Slot()
    def _on_mission_upload_timeout(self) -> None:
        if self._mission_upload_active:
            self._mission_upload_active = False
            self._mission_upload_queue.clear()
            self._mission_upload_waiting_for_readback = False
            firmware_version = str(self._state.get("firmwareVersion", "Unknown"))
            self._set_state(missionUploadPending=False, missionUploadSucceeded=False,
                            missionMessage=f"No acknowledgement from the buoy during mission upload (firmware {firmware_version}). Check the Wi-Fi connection and flash the latest mission firmware if needed.")
        elif self._mission_upload_waiting_for_readback:
            self._mission_upload_waiting_for_readback = False
            self._set_state(missionUploadPending=False, missionUploadSucceeded=False,
                            missionMessage="The buoy accepted the mission upload but did not return the stored mission for map readback. Scan the SD card and check the firmware connection.")

    @Slot()
    def refreshBuoyMissionList(self) -> None:
        if self._args.transport != "wifi" or self._transport is None:
            self._set_state(missionMessage="Connect to the buoy over Wi-Fi to scan its SD card")
            return
        self._set_state(buoyMissionList=[], buoyMissionPoints=[], buoyMissionPointsId="NONE",
                        buoyMissionHome=None, buoyMissionEnd=None, buoyMissionBoundary=[],
                        buoyMissionReadId="", buoyMissionScanComplete=False,
                        buoyMissionInventoryStatus="Scanning SD card…",
                        missionMessage="Scanning missions on the buoy SD card…")
        result = send_text(self._transport, "CTRL MISSION LIST")
        if result != "OK":
            self._set_state(missionMessage=f"Could not scan buoy SD card: {result}")

    @Slot(str)
    def readBuoyMission(self, mission_id: str) -> None:
        if self._args.transport != "wifi" or self._transport is None:
            self._set_state(missionMessage="Connect to the buoy over Wi-Fi to read a stored mission")
            return
        self._set_state(buoyMissionPoints=[], buoyMissionPointsId=mission_id.lower(),
                        buoyMissionHome=None, buoyMissionEnd=None, buoyMissionBoundary=[],
                        buoyMissionReadId=mission_id.lower(),
                        missionMessage="Reading mission points from the buoy SD card…")
        result = send_text(self._transport, f"CTRL MISSION READ {mission_id.upper()}")
        if result != "OK":
            self._set_state(missionMessage=f"Could not read buoy mission: {result}")

    @Slot(str)
    def deleteBuoyMission(self, mission_id: str) -> None:
        if self._args.transport != "wifi" or self._transport is None:
            self._set_state(missionMessage="Connect to the buoy over Wi-Fi to delete an SD card mission")
            return
        if len(mission_id) != 36 or any(char not in "0123456789abcdefABCDEF-" for char in mission_id):
            self._set_state(missionMessage="Cannot delete mission: invalid mission ID")
            return
        self._set_state(missionMessage="Deleting mission from the buoy SD card…")
        result = send_text(self._transport, f"CTRL MISSION DELETE {mission_id.upper()}")
        if result != "OK":
            self._set_state(missionMessage=f"Could not delete buoy mission: {result}")

    @Slot()
    def newMission(self) -> None:
        self._mission_draft = new_plan()
        self._publish_mission("New local mission draft", False)

    @Slot(str)
    def loadMission(self, plan_id: str) -> None:
        try:
            plan = self._mission_store.load(plan_id)
        except (OSError, ValueError, KeyError, TypeError) as exc:
            self._set_state(missionMessage=f"Could not open mission: {exc}")
            return
        if plan.get("movement_mode") == "automatic":
            previous_route = plan["waypoints"]
            previous_clearance = plan.get("auto_clearance_m")
            for key, default in (("auto_clearance_m", 15), ("arrival_radius_m", 5),
                                 ("auto_collection_seconds", 30), ("audio_capture", "checkpoint_segments")):
                plan.setdefault(key, default)
            if isinstance(plan["auto_clearance_m"], int):
                plan["auto_clearance_m"] = max(5, plan["auto_clearance_m"])
            message = self._regenerate_automatic_route(plan)
            if message.startswith("Automatic route unavailable"):
                plan["waypoints"] = previous_route
                self._mission_draft = plan
                self._publish_mission(f"Mission loaded; {message}. Adjust clearance or boundary", previous_clearance is None)
                return
            changed = plan["waypoints"] != previous_route or plan["auto_clearance_m"] != previous_clearance
            self._mission_draft = plan
            self._publish_mission((f"Mission loaded; {message}. Save to keep the updated route" if changed else "Mission loaded"), changed)
        else:
            self._mission_draft = plan
            self._publish_mission("Mission loaded", False)

    @Slot()
    def saveMission(self) -> None:
        name = str(self._mission_draft.get("name", "")).strip()
        if not name:
            self._set_state(missionMessage="Enter a mission name before saving")
            return
        self._mission_draft = {**self._mission_draft, "name": name}
        try:
            self._mission_store.save(self._mission_draft)
            self._mission_draft["offline_tiles_available"] = False
            self._mission_draft["offline_tile_template"] = ""
            self._set_state(missionList=self._mission_store.list_plans())
            self._publish_mission("Mission saved locally; preparing offline map tiles", False)
            plan_id = str(self._mission_draft["id"])
            if not any(task.plan_id == plan_id for task in self._mission_tile_tasks):
                task = _MissionTilesTask(self._mission_store, plan_id)
                task.signals.finished.connect(self._mission_tiles_finished)
                self._mission_tile_tasks.append(task)
                QThreadPool.globalInstance().start(task)
        except (OSError, ValueError, TypeError) as exc:
            self._set_state(missionMessage=f"Could not save mission: {exc}")

    @Slot(str, str)
    def _mission_tiles_finished(self, plan_id: str, message: str) -> None:
        self._mission_tile_tasks = [task for task in self._mission_tile_tasks
                                    if task.plan_id != plan_id]
        self._set_state(missionList=self._mission_store.list_plans())
        if str(self._mission_draft.get("id")) == plan_id:
            try:
                self._mission_draft = self._mission_store.load(plan_id)
                self._publish_mission(message, False)
            except (OSError, ValueError, KeyError, TypeError):
                self._set_state(missionMessage=message)
        else:
            self._set_state(missionMessage=message)

    @Slot(str, result=str)
    def missionOfflineTileTemplate(self, mission_id: str) -> str:
        try:
            normalized_id = str(uuid4_from_string(mission_id))
            return str(self._mission_store.load(normalized_id).get("offline_tile_template", ""))
        except (OSError, ValueError, KeyError, TypeError):
            return ""

    @Slot()
    def deleteMission(self) -> None:
        try:
            self._mission_store.delete(str(self._mission_draft["id"]))
            self._mission_draft = new_plan()
            self._set_state(missionList=self._mission_store.list_plans())
            self._publish_mission("Mission deleted", False)
        except (OSError, ValueError) as exc:
            self._set_state(missionMessage=f"Could not delete mission: {exc}")

    @Slot(str)
    def setMissionName(self, name: str) -> None:
        self._mission_draft = {**self._mission_draft, "name": name[:100]}
        self._publish_mission()

    def _regenerate_automatic_route(self, plan: dict) -> str:
        if plan.get("movement_mode", "manual") != "automatic":
            return "Unsaved changes"
        try:
            plan["waypoints"] = generate_automatic_waypoints(
                plan["boundary"], plan.get("home"), int(plan.get("auto_spacing_m", 25)),
                int(plan.get("auto_speed_percent", 50)), int(plan.get("auto_clearance_m", 15)),
                collection_seconds=int(plan.get("auto_collection_seconds", 30)),
            )
        except (ValueError, TypeError) as exc:
            plan["waypoints"] = []
            return f"Automatic route unavailable: {exc}"
        checkpoints = sum(point.get("collect_data", False) for point in plan["waypoints"])
        return f"Generated {checkpoints} collection checkpoints and {len(plan['waypoints']) - checkpoints} transit points with 5 m arrival circles"

    @Slot(str)
    def setMissionMovementMode(self, mode: str) -> None:
        if mode not in ("manual", "automatic"):
            return
        plan = {**self._mission_draft, "movement_mode": mode}
        message = self._regenerate_automatic_route(plan) if mode == "automatic" else "Manual route: edit movement points on the map"
        self._mission_draft = plan
        self._publish_mission(message)

    @Slot(int)
    def setMissionAutoSpacing(self, spacing_m: int) -> None:
        plan = {**self._mission_draft, "auto_spacing_m": max(5, min(500, spacing_m))}
        message = self._regenerate_automatic_route(plan)
        self._mission_draft = plan
        self._publish_mission(message)

    @Slot(int)
    def setMissionAutoSpeed(self, speed_percent: int) -> None:
        plan = {**self._mission_draft, "auto_speed_percent": max(1, min(100, speed_percent))}
        message = self._regenerate_automatic_route(plan)
        self._mission_draft = plan
        self._publish_mission(message)

    @Slot(int)
    def setMissionAutoClearance(self, clearance_m: int) -> None:
        plan = {**self._mission_draft, "auto_clearance_m": max(5, min(100, clearance_m))}
        message = self._regenerate_automatic_route(plan)
        self._mission_draft = plan
        self._publish_mission(message)

    @Slot(int)
    def setMissionAutoCollectionDuration(self, seconds: int) -> None:
        plan = {**self._mission_draft, "auto_collection_seconds": max(1, min(3600, seconds))}
        message = self._regenerate_automatic_route(plan)
        self._mission_draft = plan
        self._publish_mission(message)

    @Slot(str, float, float)
    def addMissionPoint(self, kind: str, lat: float, lon: float) -> None:
        point = {"lat": round(lat, 7), "lon": round(lon, 7)}
        if not valid_point(point):
            self._set_state(missionMessage="Position must be valid latitude and longitude")
            return
        plan = {**self._mission_draft}
        boundary_reordered = False
        if kind in ("home", "end"):
            if len(plan["boundary"]) >= 3 and not point_inside_boundary(point, plan["boundary"]):
                self._set_state(missionMessage=f"{kind.title()} must be placed inside the mission boundary")
                return
            plan[kind] = point
        elif kind in ("boundary", "waypoints"):
            if kind == "waypoints" and plan.get("movement_mode") == "automatic":
                return
            if kind == "boundary" and boundary_point_near(plan["boundary"], point):
                self._set_state(missionMessage="Boundary point is within 2 m of another point; choose a different location")
                return
            if kind == "waypoints" and (len(plan["boundary"]) < 3 or not point_inside_boundary(point, plan["boundary"])):
                self._set_state(missionMessage="Set a complete water boundary first; movement points must be inside it")
                return
            points = [*plan[kind], {**point, **({"speed_percent": 50, "hold_seconds": 30, "collect_data": True,
                                                  "measurements": ["water_temperature", "depth", "conductivity", "turbidity", "air_temperature"]} if kind == "waypoints" else {})}]
            plan[kind] = untangle_boundary(points) if kind == "boundary" else points
            boundary_reordered = kind == "boundary" and plan[kind] != points
        else:
            return
        self._mission_draft = plan
        message = self._regenerate_automatic_route(plan) if kind in ("home", "boundary") else "Unsaved changes"
        self._publish_mission(f"Boundary edges uncrossed. {message}" if boundary_reordered else message)

    @Slot(str, int, float, float)
    def updateMissionPoint(self, kind: str, index: int, lat: float, lon: float) -> None:
        point = {"lat": round(lat, 7), "lon": round(lon, 7)}
        if not valid_point(point):
            self._set_state(missionMessage="Position must be valid latitude and longitude")
            return
        plan = {**self._mission_draft}
        boundary_reordered = False
        if kind in ("home", "end"):
            if len(plan["boundary"]) >= 3 and not point_inside_boundary(point, plan["boundary"]):
                self._set_state(missionMessage=f"{kind.title()} must stay inside the mission boundary")
                return
            plan[kind] = point
        elif kind in ("boundary", "waypoints") and 0 <= index < len(plan[kind]):
            if kind == "waypoints" and plan.get("movement_mode") == "automatic":
                return
            if kind == "boundary" and boundary_point_near(plan["boundary"], point, skip_index=index):
                self._set_state(missionMessage="Boundary point is within 2 m of another point; choose a different location")
                return
            if kind == "waypoints" and (len(plan["boundary"]) < 3 or not point_inside_boundary(point, plan["boundary"])):
                self._set_state(missionMessage="Movement points must stay inside the complete water boundary")
                return
            values = [dict(item) for item in plan[kind]]
            values[index].update(point)
            if kind == "boundary":
                ordered = untangle_boundary(values)
                boundary_reordered = ordered != values
                # Preserve the selected point's index after edge reversals.
                shift = (ordered.index(values[index]) - index) % len(ordered)
                plan[kind] = ordered[shift:] + ordered[:shift]
            else:
                plan[kind] = values
        else:
            return
        self._mission_draft = plan
        message = self._regenerate_automatic_route(plan) if kind in ("home", "boundary") else "Unsaved changes"
        self._publish_mission(f"Boundary edges uncrossed. {message}" if boundary_reordered else message)

    @Slot(str, int)
    def removeMissionPoint(self, kind: str, index: int) -> None:
        plan = {**self._mission_draft}
        if kind in ("home", "end"):
            plan[kind] = None
        elif kind in ("boundary", "waypoints") and 0 <= index < len(plan[kind]):
            if kind == "waypoints" and plan.get("movement_mode") == "automatic":
                return
            points = [item for i, item in enumerate(plan[kind]) if i != index]
            plan[kind] = untangle_boundary(points) if kind == "boundary" else points
        else:
            return
        self._mission_draft = plan
        self._publish_mission(self._regenerate_automatic_route(plan) if kind in ("home", "boundary") else "Unsaved changes")

    @Slot(int, int)
    def moveMissionWaypoint(self, index: int, direction: int) -> None:
        if self._mission_draft.get("movement_mode") == "automatic":
            return
        points = [dict(item) for item in self._mission_draft["waypoints"]]
        target = index + direction
        if not (0 <= index < len(points) and 0 <= target < len(points)):
            return
        points[index], points[target] = points[target], points[index]
        self._mission_draft = {**self._mission_draft, "waypoints": points}
        self._publish_mission()

    @Slot(int, int, int)
    def setMissionWaypointOptions(self, index: int, speed: int, hold: int) -> None:
        points = [dict(item) for item in self._mission_draft["waypoints"]]
        if not 0 <= index < len(points):
            return
        duration = max(0, min(3600, hold))
        points[index].update(speed_percent=max(1, min(100, speed)), hold_seconds=duration,
                             collect_data=duration > 0)
        self._mission_draft = {**self._mission_draft, "waypoints": points}
        self._publish_mission()

    @Slot(int, "QVariantList")
    def setMissionWaypointMeasurements(self, index: int, measurements: list) -> None:
        points = [dict(item) for item in self._mission_draft["waypoints"]]
        if not 0 <= index < len(points):
            return
        allowed = {"water_temperature", "depth", "conductivity", "turbidity", "air_temperature"}
        selected = list(dict.fromkeys(str(item) for item in measurements if str(item) in allowed))
        points[index]["measurements"] = selected
        self._mission_draft = {**self._mission_draft, "waypoints": points}
        self._publish_mission("Point measurements updated")

    @Slot(str, str)
    def setMissionFailsafe(self, key: str, action: str) -> None:
        if key not in ("link_loss", "boundary_breach", "low_battery") or action not in ("stop", "return_home"):
            return
        self._mission_draft = {**self._mission_draft, "failsafes": {**self._mission_draft["failsafes"], key: action}}
        self._publish_mission()

    @Slot(int)
    def setMissionBatteryThreshold(self, percent: int) -> None:
        self._mission_draft = {**self._mission_draft, "failsafes": {**self._mission_draft["failsafes"], "battery_threshold_percent": max(5, min(80, percent))}}
        self._publish_mission()

    @Slot(str, int, float)
    def setMissionCalibration(self, profile: str, percent: int, value: float) -> None:
        if profile not in ("speed_mps", "current_a") or percent not in (25, 50, 75, 100):
            return
        if not math.isfinite(value) or value < 0:
            return
        calibration = dict(self._mission_draft.get("calibration", {}))
        values = dict(calibration.get(profile, {}))
        values[str(percent)] = round(value, 3) if value > 0 else None
        calibration[profile] = values
        self._mission_draft = {**self._mission_draft, "calibration": calibration}
        self._publish_mission("Calibration updated; estimates refreshed")

    @Slot(float)
    def setMissionBatteryCapacity(self, value: float) -> None:
        if not math.isfinite(value) or value < 0:
            return
        calibration = dict(self._mission_draft.get("calibration", {}))
        calibration["battery_capacity_ah"] = round(value, 2) if value > 0 else None
        self._mission_draft = {**self._mission_draft, "calibration": calibration}
        self._publish_mission("Battery capacity updated; estimates refreshed")

    @Slot(float)
    def setMissionIdleCurrent(self, value: float) -> None:
        if not math.isfinite(value) or value < 0:
            return
        calibration = dict(self._mission_draft.get("calibration", {}))
        calibration["idle_current_a"] = round(value, 3) if value > 0 else None
        self._mission_draft = {**self._mission_draft, "calibration": calibration}
        self._publish_mission("Idle current updated; estimates refreshed")

    @Slot(str)
    def setDashboardTab(self, tab: str) -> None:
        if tab not in {"overview", "communications", "power", "motors", "navigation", "missions", "instrument_debug", "sd_debug"}:
            return
        if self._dashboard_tab != tab:
            leaving_motor_debug = self._dashboard_tab == "motors" and tab != "motors"
            leaving_navigation_test = self._dashboard_tab == "navigation" and tab != "navigation"
            self._dashboard_tab = tab
            self.dashboardTabChanged.emit()
            if leaving_motor_debug:
                if self._active_motor_test is not None:
                    stamp = datetime.now(timezone.utc).isoformat(timespec="milliseconds")
                    self._command_trace_pending.append(f"{stamp} MOTOR TEST TAB LEFT\n")
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
                navigationStatus=("Release keyboard controls to resume" if navigation_was_active else "Assist off"),
                    navigationCorrectionDeg=0.0,
                )
                if navigation_was_active and self.keyboard_drive_enabled:
                    self.stopAllMotors()

    def _record_power_sample(self, voltage: float, current: float) -> None:
        self._integrate_motor_energy_estimate(voltage, current)
        now = time.time()
        for interval, points in self._power_histories.items():
            add_power_sample(points, now, voltage, current, interval)
            while points and points[0]["t"] < now - POWER_HISTORY_WINDOWS[interval]:
                points.pop(0)
        current_bucket = int(now // 10) * 10
        if current_bucket != self._last_power_history_persist_bucket:
            self._last_power_history_persist_bucket = current_bucket
            self._persist_power_histories()
        if now - self._last_power_history_publish_at >= 2.0:
            self._last_power_history_publish_at = now
            self._publish_power_history()

    def _integrate_motor_energy_estimate(self, voltage: float, current: float) -> None:
        """Allocate measured pack load to observed motors using a PWM-cubed model."""
        now = time.monotonic()
        previous = self._motor_energy_last_at
        self._motor_energy_last_at = now
        if not previous:
            return
        elapsed = now - previous
        if elapsed <= 0 or elapsed > 5.0:
            return

        outputs = None
        radio_outputs = False
        if self._state.get("motorOutputTelemetry") and time.time() - self._last_motor_output_at <= 2.5:
            outputs = [self._state.get("rearMotorPwm", -1),
                       self._state.get("frontLeftMotorPwm", -1),
                       self._state.get("frontRightMotorPwm", -1)]
        else:
            radio_state = self._state.get("radioBuoyState")
            last_state = self._radio_field_last_at.get("buoy_state", 0.0)
            values = radio_state.get("motor_pwm") if isinstance(radio_state, dict) else None
            if values and now - last_state <= 2.5:
                outputs = values[:3]
                radio_outputs = True
        if not outputs or voltage <= 0 or current <= 0:
            return
        try:
            numeric_outputs = [float(value) for value in outputs]
        except (TypeError, ValueError):
            return
        if not radio_outputs and any(value < 0 for value in numeric_outputs):
            return

        # A propeller's electrical load rises faster than linearly with speed.
        # This estimate uses PWM^3 weights, the measured pack watts, and a
        # small electronics baseline; calibration can replace these weights.
        weights = [abs(value) ** 3 for value in numeric_outputs]
        weight_sum = sum(weights)
        if weight_sum <= 0:
            return
        propulsion_watts = max(0.0, voltage * (current - 0.21))
        if propulsion_watts <= 0:
            return
        scale = propulsion_watts * elapsed / 3600.0 / weight_sum
        for key, weight in zip(("rear", "left", "right"), weights):
            self._motor_energy_wh[key] += weight * scale
        self._set_state(motorEnergyWh=dict(self._motor_energy_wh),
                        motorEnergyEstimatorStatus="Estimated from pack current and PWM³ · since station telemetry started")

    def _publish_power_history(self) -> None:
        interval = self._power_history_interval_seconds
        points = self._power_histories[interval]
        label = {10: "10-second bars / last 30 minutes", 30: "30-second bars / last 2 hours",
                 300: "5-minute bars / last 24 hours"}[interval]
        self._set_state(powerHistory=[dict(point) for point in points],
                        powerHistorySampleCount=sum(int(point["count"]) for point in points),
                        powerHistoryStatus=f"Local rolling history · {label}; positive current = discharge")

    def _persist_power_histories(self) -> None:
        try:
            save_power_histories(self._power_history_path, self._power_histories)
        except OSError as exc:
            self._set_state(powerHistoryStatus=f"Could not save local power history: {exc}")

    @Slot(int)
    def setPowerHistoryInterval(self, interval_seconds: int) -> None:
        if interval_seconds not in self._power_histories:
            return
        self._power_history_interval_seconds = interval_seconds
        window = {10: 1800, 30: 7200, 300: 86400}[interval_seconds]
        self._set_state(powerHistoryIntervalSeconds=interval_seconds,
                        powerHistoryWindowSeconds=window)
        self._publish_power_history()

    @Slot()
    def refreshSdFiles(self) -> None:
        self._start_sd_card_operation("list")

    @Slot(str)
    def loadSdSession(self, session_path: str) -> None:
        self._start_sd_card_operation("session", remote_path=session_path)

    @Slot(str)
    def deleteSdSession(self, session_path: str) -> None:
        if self._state.get("scienceExperimentState") != "IDLE":
            self._set_state(sdTransferStatus="Save and close the mission before deleting a session")
            return
        self._start_sd_card_operation("delete", remote_path=session_path)

    @Slot()
    def startScienceExperiment(self) -> None:
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        self._send_audio_firmware_command(f"CTRL SCIENCE START {stamp}")

    @Slot()
    def stopScienceExperiment(self) -> None:
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        self._send_audio_firmware_command(f"CTRL SCIENCE STOP {stamp}")

    @Slot()
    def saveScienceExperiment(self) -> None:
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        self._send_audio_firmware_command(f"CTRL SCIENCE SAVE {stamp}")

    @Slot(bool)
    def setAudioStreamEnabled(self, enabled: bool) -> None:
        self._send_audio_firmware_command(f"CTRL STREAM AUDIO {'START' if enabled else 'STOP'}")

    @Slot(bool)
    def setImuStreamEnabled(self, enabled: bool) -> None:
        self._send_audio_firmware_command(f"CTRL STREAM IMU {'START' if enabled else 'STOP'}")

    @Slot()
    def startMagnetometerCalibration(self) -> None:
        if self._test_session is not None and self._test_session.active:
            self._set_state(magnetometerCalibrationStatus="Stop the active telemetry capture before base calibration")
            return
        if self._last_motor_output_at is None or time.time() - self._last_motor_output_at > 2.0:
            self._set_state(magnetometerCalibrationStatus="Waiting for fresh motor-output telemetry; base calibration requires motors off")
            return
        if any(abs(int(self._state.get(key, 0))) > 2 for key in ("rearMotorActualDrive", "frontLeftMotorActualDrive", "frontRightMotorActualDrive")):
            self._set_state(magnetometerCalibrationStatus="Stop all motors before base compass calibration")
            return
        self._mag_calibration_active = True
        self._mag_calibration_min = None
        self._mag_calibration_max = None
        self._mag_calibration_samples = 0
        self._set_state(magnetometerCalibrationActive=True,
                        magnetometerCalibrationSamples=0,
                        magnetometerCalibrationStatus="Rotate the buoy slowly through full turns, keeping it level")

    @Slot()
    def calibrateNavigationLevel(self) -> None:
        now = time.monotonic()
        if not self._navigation.measurement_is_fresh(now):
            self._set_state(navigationLevelCalibrationStatus="No fresh IMU reading; start the IMU stream first")
            return
        self._level_roll_offset = self._navigation.roll_deg
        self._level_pitch_offset = self._navigation.pitch_deg
        try:
            self._level_calibration_path.parent.mkdir(parents=True, exist_ok=True)
            self._level_calibration_path.write_text(json.dumps({
                "roll_offset_deg": self._level_roll_offset,
                "pitch_offset_deg": self._level_pitch_offset,
            }, indent=2) + "\n", encoding="utf-8")
        except OSError as exc:
            self._set_state(navigationLevelCalibrationStatus=f"Could not save level: {exc}")
            return
        self._set_state(navigationLevelCalibrated=True,
                        navigationLevelCalibrationStatus="Waterline level saved; heading tilt compensation active")
        self._refresh_tilt_heading()

    @Slot(int)
    def setGpsMinimumSatellites(self, value: int) -> None:
        minimum = max(4, min(12, int(value)))
        try:
            self._gps_quality_path.parent.mkdir(parents=True, exist_ok=True)
            self._gps_quality_path.write_text(json.dumps({"minimum_satellites": minimum}, indent=2) + "\n", encoding="utf-8")
        except OSError as exc:
            self._set_state(gpsQualityStatus=f"Could not save satellite threshold: {exc}")
            return
        self._set_state(gpsMinimumSatellites=minimum)
        self._refresh_gps_quality()

    def _refresh_gps_quality(self) -> None:
        satellites = self._state.get("gpsSatelliteCount")
        if satellites is None:
            status = "Waiting for GPS satellite data"
        elif not self._state.get("gpsFixValid"):
            status = f"No GPS position fix · {satellites} satellites visible"
        elif int(satellites) < int(self._state.get("gpsMinimumSatellites", 6)):
            status = f"Low satellite count · {satellites} / {self._state.get('gpsMinimumSatellites', 6)} minimum"
        elif self._state.get("gpsHdop") is not None and float(self._state["gpsHdop"]) > 2.5:
            status = f"Fix available · {satellites} satellites · weak geometry"
        else:
            status = f"GPS quality acceptable · {satellites} satellites"
        self._set_state(gpsQualityStatus=status)

    def _refresh_tilt_heading(self) -> None:
        mag = self._last_mag_raw
        now = time.monotonic()
        if mag is None:
            self._set_state(magnetometerTiltHeadingDeg=None, magnetometerTiltStatus="Waiting for magnetometer")
            return
        if not self._state.get("navigationLevelCalibrated"):
            self._set_state(magnetometerTiltHeadingDeg=None,
                            magnetometerTiltStatus="Calibrate waterline level to enable tilt compensation")
            return
        if not self._navigation.measurement_is_fresh(now):
            self._set_state(magnetometerTiltHeadingDeg=None, magnetometerTiltStatus="Waiting for fresh IMU attitude")
            return
        mx, my, mz = mag
        if {"offset_x", "offset_y", "scale_x", "scale_y"} <= self._mag_calibration.keys():
            mx = (mx - self._mag_calibration["offset_x"]) * self._mag_calibration["scale_x"]
            my = (my - self._mag_calibration["offset_y"]) * self._mag_calibration["scale_y"]
        roll = math.radians(self._navigation.roll_deg - self._level_roll_offset)
        pitch = math.radians(self._navigation.pitch_deg - self._level_pitch_offset)
        horizontal_x = mx * math.cos(pitch) + mz * math.sin(pitch)
        horizontal_y = (mx * math.sin(roll) * math.sin(pitch)
                        + my * math.cos(roll) - mz * math.sin(roll) * math.cos(pitch))
        heading = math.degrees(math.atan2(horizontal_y, horizontal_x)) % 360.0
        self._set_state(magnetometerTiltHeadingDeg=round(heading, 1),
                        magnetometerTiltStatus=("Tilt corrected · 2-axis calibration · verify sensor alignment" if self._mag_calibration else
                                                "Tilt corrected · 3-axis calibration recommended"))

    @Slot()
    def finishMagnetometerCalibration(self) -> None:
        minimum, maximum = self._mag_calibration_min, self._mag_calibration_max
        self._mag_calibration_active = False
        if minimum is None or maximum is None or self._mag_calibration_samples < 20:
            self._set_state(magnetometerCalibrationActive=False,
                            magnetometerCalibrationStatus="Not enough compass samples; start again and rotate through full turns")
            return
        span_x, span_y = maximum[0] - minimum[0], maximum[1] - minimum[1]
        if span_x < 100 or span_y < 100:
            self._set_state(magnetometerCalibrationActive=False,
                            magnetometerCalibrationStatus="Rotation coverage was too small; rotate through full turns and retry")
            return
        radius = (span_x + span_y) / 4.0
        self._mag_calibration = {
            "offset_x": (maximum[0] + minimum[0]) / 2.0,
            "offset_y": (maximum[1] + minimum[1]) / 2.0,
            "scale_x": (2.0 * radius) / span_x,
            "scale_y": (2.0 * radius) / span_y,
        }
        try:
            self._mag_calibration_path.parent.mkdir(parents=True, exist_ok=True)
            self._mag_calibration_path.write_text(json.dumps(self._mag_calibration, indent=2) + "\n", encoding="utf-8")
        except OSError as exc:
            self._set_state(magnetometerCalibrationActive=False,
                            magnetometerCalibrationStatus=f"Could not save calibration: {exc}")
            return
        self._set_state(magnetometerCalibrationActive=False,
                        magnetometerCalibrationSaved=True,
                        magnetometerCalibrationStatus="Compass calibration saved on this control station")
        self._refresh_tilt_heading()

    @Slot()
    def cancelMagnetometerCalibration(self) -> None:
        self._mag_calibration_active = False
        self._set_state(magnetometerCalibrationActive=False,
                        magnetometerCalibrationStatus="Calibration cancelled; previous saved calibration retained")

    @Slot()
    def resetMagnetometerCalibration(self) -> None:
        self._mag_calibration = {}
        try:
            self._mag_calibration_path.unlink(missing_ok=True)
        except OSError as exc:
            self._set_state(magnetometerCalibrationStatus=f"Could not remove calibration: {exc}")
            return
        self._set_state(magnetometerCalibrationSaved=False,
                        magnetometerCalibrationStatus="Calibration cleared; heading is uncalibrated")
        self._refresh_tilt_heading()

    @Slot()
    def uploadMagnetometerCalibration(self) -> None:
        required = {"offset_x", "offset_y", "scale_x", "scale_y"}
        if not required <= self._mag_calibration.keys():
            self._set_state(magnetometerBuoyConfigStatus="No saved calibration to upload")
            return
        if self._transport is None:
            self._set_state(magnetometerBuoyConfigStatus="Connect to the buoy over Wi-Fi to upload calibration")
            return
        command = "CTRL CONFIG MAGCAL " + " ".join(
            f"{self._mag_calibration[key]:.6f}"
            for key in ("offset_x", "offset_y", "scale_x", "scale_y")
        )
        result = send_text(self._transport, command)
        self._set_state(lastSendResult=result, lastSentLine=command,
                        magnetometerBuoyConfigStatus="Sending calibration to buoy SD card")
        if "failed" not in result.lower():
            self._append_comm_log(f"TX {command}")

    @Slot(str)
    def prepareSdDownload(self, remote_path: str) -> None:
        self._set_state(sdSelectedPath=remote_path)

    @Slot(str, str)
    def downloadSdFile(self, remote_path: str, destination: str) -> None:
        if not destination:
            return
        self._set_state(sdSelectedPath=remote_path)
        self._start_sd_card_operation("download", remote_path=remote_path, destination=destination)

    @Slot(str, QUrl)
    def downloadSdFileFromUrl(self, remote_path: str, destination_url: QUrl) -> None:
        self.downloadSdFile(remote_path, destination_url.toLocalFile())

    @Slot(str, QUrl)
    def downloadSdSessionFromUrl(self, session_path: str, destination_url: QUrl) -> None:
        destination = destination_url.toLocalFile()
        if destination:
            self._start_sd_card_operation("session_download", remote_path=session_path, destination=destination)

    @Slot(QUrl)
    def downloadSdLogsFromUrl(self, destination_url: QUrl) -> None:
        destination = destination_url.toLocalFile()
        if destination:
            self._start_sd_card_operation("logs_download", remote_path="/logs", destination=destination)

    @Slot()
    def cancelSdDownload(self) -> None:
        if self._state.get("sdDownloadActive"):
            self._sd_cancel_event.set()
            self._set_state(sdTransferStatus="Cancelling download...")

    def _start_sd_card_operation(
        self,
        operation: str,
        *,
        remote_path: str = "",
        destination: str = "",
    ) -> None:
        if self._args.transport != "wifi" or self._transport is None or not self._transport.is_open:
            self._set_state(sdCardStatus="SD browsing requires an active Wi-Fi connection")
            return
        is_download = operation in {"download", "session_download", "logs_download"}
        requires_service_mode = is_download or operation == "delete"
        if requires_service_mode and self._state.get("scienceExperimentState") != "IDLE":
            action = "deleting" if operation == "delete" else "downloading"
            self._set_state(sdTransferStatus=f"Save and close the mission before {action}")
            return
        with self._sd_operation_lock:
            if self._sd_operation_busy:
                return
            self._sd_operation_busy = True
            self._sd_pending_result = None
            self._sd_progress_pending = None
        host = getattr(self._transport, "host", "")
        local_ip = self._wifi_local_ip
        self._sd_cancel_event.clear()
        self._sd_service_requested = requires_service_mode
        if requires_service_mode:
            self._navigation_requires_center = True
            self._set_state(navigationReleaseRequired=True)
            self._send_audio_firmware_command("CTRL SERVICE START")
            if "failed" in str(self._state.get("lastSendResult", "")).lower():
                with self._sd_operation_lock:
                    self._sd_operation_busy = False
                self._sd_service_requested = False
                self._set_state(sdTransferStatus="Could not enter service mode")
                return
        self._sd_transfer_started_at = time.monotonic()
        self._set_state(
            sdBusy=True,
            sdTransferProgress=0.0,
            sdTransferBytes=0,
            sdTransferTotalBytes=0,
            sdTransferSpeedBytesPerSecond=0.0,
            sdTransferElapsedSeconds=0.0,
            sdTransferEtaSeconds=-1.0,
            sdTransferFileName="",
            sdDownloadActive=is_download,
            sdDownloadSessionPath=(remote_path if operation in {"session_download", "logs_download"} else str(Path(remote_path).parent).replace("\\", "/")) if is_download else "",
            sdTransferStatus=(
                f"Downloading {remote_path}" if operation in {"download", "session_download"}
                else (f"Deleting {remote_path}" if operation == "delete" else f"Reading {remote_path or 'SD sessions'}")
            ),
        )

        def worker() -> None:
            try:
                if operation == "list":
                    result: object = list_sd_files(host, local_ip)
                elif operation == "session":
                    result = {"path": remote_path, **list_sd_session(host, remote_path, local_ip)}
                elif operation == "delete":
                    deleted_path = delete_sd_session(host, remote_path, local_ip)
                    result = {"deletedPath": deleted_path, **list_sd_files(host, local_ip)}
                elif operation == "session_download":
                    def update_session_progress(received: int, total: int, file_name: str) -> None:
                        with self._sd_operation_lock:
                            self._sd_progress_pending = (received, total, file_name)

                    result = str(download_sd_session(host, remote_path, destination, local_ip, update_session_progress, self._sd_cancel_event.is_set))
                elif operation == "logs_download":
                    def update_logs_progress(received: int, total: int, file_name: str) -> None:
                        with self._sd_operation_lock:
                            self._sd_progress_pending = (received, total, file_name)

                    result = str(download_sd_logs(host, destination, local_ip, update_logs_progress, self._sd_cancel_event.is_set))
                else:
                    def update_progress(received: int, total: int) -> None:
                        with self._sd_operation_lock:
                            self._sd_progress_pending = (received, total, Path(remote_path).name)

                    result = str(download_sd_file(host, remote_path, destination, local_ip, update_progress, self._sd_cancel_event.is_set))
                payload: tuple[str, object] = (operation, result)
            except Exception as exc:
                payload = ("error", str(exc))
            with self._sd_operation_lock:
                self._sd_pending_result = payload

        self._sd_operation_thread = threading.Thread(target=worker, name="sd-card-transfer", daemon=True)
        self._sd_operation_thread.start()

    def _drain_sd_card_operation(self) -> None:
        with self._sd_operation_lock:
            progress = self._sd_progress_pending
            self._sd_progress_pending = None
            pending = self._sd_pending_result
            self._sd_pending_result = None
        if progress is not None:
            received, total, file_name = progress
            percent = 100.0 if total <= 0 else 100.0 * received / total
            elapsed = max(0.001, time.monotonic() - self._sd_transfer_started_at)
            speed = received / elapsed
            eta = (total - received) / speed if speed > 0 and total > received else (0.0 if total <= received else -1.0)
            self._set_state(
                sdTransferProgress=percent,
                sdTransferBytes=received,
                sdTransferTotalBytes=total,
                sdTransferSpeedBytesPerSecond=speed,
                sdTransferElapsedSeconds=elapsed,
                sdTransferEtaSeconds=eta,
                sdTransferFileName=file_name,
                sdTransferStatus=f"Downloading {file_name}: {received:,} / {total:,} bytes ({percent:.0f}%)",
            )
        if pending is None:
            return
        with self._sd_operation_lock:
            self._sd_operation_busy = False
        result_kind, result = pending
        if self._sd_service_requested:
            self._send_audio_firmware_command("CTRL SERVICE STOP")
            self._sd_service_requested = False
        self._set_state(sdDownloadActive=False)
        if result_kind == "error":
            status = "Download cancelled" if str(result) == "Download cancelled" else f"SD operation failed: {result}"
            self._set_state(sdBusy=False, sdTransferStatus=status)
            return
        if result_kind in {"list", "delete"} and isinstance(result, dict):
            files = result["files"]
            capacity = result.get("capacity", {})
            sessions: dict[str, dict[str, object]] = {}
            for entry in files:
                path = str(entry.get("path", ""))
                parts = path.strip("/").split("/")
                if entry.get("isDirectory") and len(parts) == 1 and parts[0].lower().startswith("session"):
                    sessions[parts[0]] = {
                        "name": parts[0],
                        "path": path,
                        "size": int(entry.get("size", 0)),
                        "files": [],
                        "loaded": False,
                        "sessionOrigin": str(entry.get("sessionOrigin", "unknown")),
                        "missionId": str(entry.get("missionId", "NONE")),
                        "missionRunId": str(entry.get("missionRunId", "NONE")),
                    }
            session_rows = [
                sessions[name] for name in sorted(sessions, reverse=True)
            ]
            self._set_state(
                sdFiles=files,
                sdSessions=session_rows,
                sdCardCapacity=capacity,
                sdCardStatus=f"{len(session_rows)} science sessions",
                sdBusy=False,
                sdTransferStatus=(
                    f"Deleted {result['deletedPath'].rsplit('/', 1)[-1]}"
                    if result_kind == "delete" else "SD listing refreshed"
                ),
                sdTransferProgress=0.0,
            )
        elif result_kind == "session" and isinstance(result, dict):
            categories = {
                "audio.wav": ("AUDIO", "Stereo microphone recording"),
                "audio.idx": ("AUDIO INDEX", "Recording index"),
                "science.csv": ("SCIENCE", "Sensor measurements"),
                "telemetry.csv": ("TELEMETRY", "GPS, control, and system readings"),
                "manifest.txt": ("SESSION INFO", "Mission/run reference and route measurement snapshot"),
            }
            session_files = []
            for entry in result["files"]:
                name = str(entry.get("name", ""))
                if name.lower() in categories and not entry.get("isDirectory"):
                    category, description = categories[name.lower()]
                    session_files.append({**entry, "category": category, "description": description})
            rows = []
            for row in self._state.get("sdSessions", []):
                if row["path"] == result["path"]:
                    rows.append({**row, "files": sorted(session_files, key=lambda item: str(item["name"]).lower()), "loaded": True,
                                 "size": sum(int(item["size"]) for item in result["files"] if not item.get("isDirectory"))})
                else:
                    rows.append(row)
            self._set_state(sdSessions=rows, sdBusy=False, sdTransferStatus=f"Loaded {result['path']}")
        elif result_kind in {"download", "session_download", "logs_download"}:
            self._set_state(
                sdBusy=False,
                sdTransferStatus=f"Saved locally: {result}",
                sdTransferProgress=100.0,
                sdTransferElapsedSeconds=time.monotonic() - self._sd_transfer_started_at,
                sdTransferEtaSeconds=0.0,
            )

    @Slot()
    def toggleAudioMute(self) -> None:
        self._audio_muted = not self._audio_muted
        if self._audio_muted:
            self._clear_audio_playback_queue()
        if self._audio_muted and self._audio_channel is not None:
            try:
                self._audio_channel.stop()
            except Exception:
                pass
        self.audioMutedChanged.emit()
        self.stateChanged.emit()

    def _clear_audio_playback_queue(self) -> None:
        self._audio_playback_pending.clear()
        while True:
            try:
                self._audio_playback_queue.get_nowait()
            except queue.Empty:
                break

    def _queue_audio_playback(self, sound: object) -> None:
        try:
            self._audio_playback_queue.put_nowait(sound)
        except queue.Full:
            try:
                self._audio_playback_queue.get_nowait()
            except queue.Empty:
                pass
            self._audio_playback_queue.put_nowait(sound)
            self._audio_playback_drops += 1
            self.stateChanged.emit()
        self._audio_sounds_submitted += 1
        self._audio_playback_wake.set()

    def _audio_playback_loop(self) -> None:
        while self._audio_playback_running:
            if self._audio_muted or self._audio_channel is None:
                self._clear_audio_playback_queue()
                self._audio_playback_wake.wait(0.02)
                self._audio_playback_wake.clear()
                continue
            try:
                if self._audio_channel.get_busy():
                    if self._audio_channel.get_queue() is None:
                        self._audio_channel.queue(self._audio_playback_queue.get_nowait())
                        self._audio_sounds_scheduled += 1
                    else:
                        self._audio_playback_wake.wait(0.005)
                        self._audio_playback_wake.clear()
                else:
                    self._audio_channel.play(self._audio_playback_queue.get_nowait())
                    self._audio_sounds_scheduled += 1
                self._audio_output_status = (
                    f"{self._audio_output_device} | "
                    f"{self._audio_sounds_scheduled} chunks scheduled"
                )
            except queue.Empty:
                self._audio_playback_wake.wait(0.005)
                self._audio_playback_wake.clear()
            except Exception as exc:
                self._audio_playback_error = str(exc)
                self._audio_output_status = f"Playback error: {exc}"
                self.stateChanged.emit()
                self._audio_playback_wake.wait(0.01)
                self._audio_playback_wake.clear()

    def _stop_audio_playback_thread(self) -> None:
        self._audio_playback_running = False
        self._audio_playback_wake.set()
        if self._audio_playback_thread is not None:
            if self._audio_playback_thread is not threading.current_thread():
                self._audio_playback_thread.join(timeout=0.5)
            self._audio_playback_thread = None
        self._clear_audio_playback_queue()

    def _start_audio_playback_thread(self) -> None:
        if self._audio_channel is None:
            return
        self._audio_playback_running = True
        self._audio_playback_thread = threading.Thread(
            target=self._audio_playback_loop,
            name="audio-playback-buffer",
            daemon=True,
        )
        self._audio_playback_thread.start()

    def _initialize_audio_output(self, device_name: str | None) -> bool:
        self._audio_playback_running = False
        self._audio_playback_wake.set()
        if self._audio_playback_thread is not None:
            self._audio_playback_thread.join(timeout=0.5)
            self._audio_playback_thread = None
        self._clear_audio_playback_queue()
        if self._audio_channel is not None:
            try:
                self._audio_channel.stop()
            except Exception:
                pass
        self._audio_channel = None
        self._mixer_ready = False
        if pygame is None:
            self._audio_output_status = "pygame is unavailable"
            return False
        try:
            if pygame.mixer.get_init() is not None:
                pygame.mixer.quit()
            pygame.mixer.init(
                frequency=int(self._audio_sample_rate or 16000),
                size=-16,
                channels=2,
                buffer=1024,
                devicename=None if device_name in (None, "", "System default") else device_name,
                allowedchanges=0,
            )
            mixer_format = pygame.mixer.get_init()
            if mixer_format is None:
                raise pygame.error("mixer did not initialize")
            self._mixer_ready = True
            self._audio_channel = pygame.mixer.Channel(0)
            self._audio_output_device = device_name or "System default"
            self._audio_output_status = f"{self._audio_output_device} | mixer {mixer_format}"
            self._start_audio_playback_thread()
            return True
        except pygame.error as exc:
            self._audio_playback_error = str(exc)
            self._audio_output_status = f"Cannot open {device_name or 'system default'}: {exc}"
            return False

    @Slot(str)
    def setAudioOutputDevice(self, device_name: str) -> None:
        requested = (device_name or "System default").strip()
        if requested == self._audio_output_device:
            return
        selected = None if requested == "System default" else requested
        if not self._initialize_audio_output(selected):
            failed_status = self._audio_output_status
            self._audio_output_device = "System default"
            if self._initialize_audio_output(None):
                self._audio_output_status = f"{failed_status} | using system default"
            else:
                self._audio_output_status = failed_status
        else:
            self._audio_output_device = requested
        self.stateChanged.emit()

    @Slot(str)
    def setAudioDebugChannel(self, channel: str) -> None:
        normalized = channel.lower().strip()
        if normalized in {"both", "left", "right"} and normalized != self._audio_debug_channel:
            self._audio_debug_channel = normalized
            self.stateChanged.emit()

    @Slot(str)
    def setAudioCaptureChannel(self, channel: str) -> None:
        normalized = channel.upper().strip()
        if normalized not in {"BOTH", "LEFT", "RIGHT"}:
            return
        self._send_audio_firmware_command(f"CTRL AUDIO CHANNEL {normalized}")

    @Slot(str)
    def setAudioQuality(self, quality: str) -> None:
        normalized = quality.upper().strip()
        if normalized not in {"LOW", "DEFAULT", "MAX"}:
            return
        if self.audioRecordingEnabled or self.audioQualityLocked:
            self._set_state(lastSendResult="Stop and restart the SD audio session before changing quality")
            return
        self._send_audio_firmware_command(f"CTRL AUDIO QUALITY {normalized}")

    @Slot()
    def toggleAudioRecording(self) -> None:
        command = "START" if not self.audioRecordingEnabled else "STOP"
        self._send_audio_firmware_command(f"CTRL AUDIO RECORD {command}")

    def _send_audio_firmware_command(self, command: str) -> None:
        if self._transport is None:
            self._set_state(lastSendResult="No serial link", lastSentLine=command)
            return
        result = send_text(self._transport, command)
        self._set_state(lastSendResult=result, lastSentLine=command)
        if "failed" not in result.lower():
            self._append_comm_log(f"TX {command}")

    @Slot(float)
    def setAudioDebugGain(self, gain_percent: float) -> None:
        normalized = max(0.0, min(200.0, float(gain_percent)))
        if abs(normalized - self._audio_debug_gain) >= 0.5:
            self._audio_debug_gain = normalized
            self.stateChanged.emit()

    @Slot(bool)
    def setAudioHighPassEnabled(self, enabled: bool) -> None:
        self._audio_high_pass = bool(enabled)
        self._reset_audio_filter_state()
        self.stateChanged.emit()

    @Slot(float)
    def setAudioHighPassHz(self, frequency_hz: float) -> None:
        self._audio_high_pass_hz = max(20.0, min(300.0, float(frequency_hz)))
        self.stateChanged.emit()

    @Slot(bool)
    def setAudioLowPassEnabled(self, enabled: bool) -> None:
        self._audio_low_pass = bool(enabled)
        self._reset_audio_filter_state()
        self.stateChanged.emit()

    @Slot(float)
    def setAudioLowPassHz(self, frequency_hz: float) -> None:
        self._audio_low_pass_hz = max(500.0, min(7900.0, (self._audio_sample_rate or 16000) * 0.45, float(frequency_hz)))
        self.stateChanged.emit()

    @Slot(bool)
    def setAudioNoiseGateEnabled(self, enabled: bool) -> None:
        self._audio_noise_gate = bool(enabled)
        self.stateChanged.emit()

    @Slot(float)
    def setAudioNoiseGatePercent(self, threshold_percent: float) -> None:
        self._audio_noise_gate_percent = max(0.0, min(10.0, float(threshold_percent)))
        self.stateChanged.emit()

    def _reset_audio_filter_state(self) -> None:
        self._audio_filter_previous_x = [0.0, 0.0]
        self._audio_filter_previous_y = [0.0, 0.0]
        self._audio_filter_low_pass_y = [0.0, 0.0]

    def _filter_audio_samples(self, samples: array, channels: int) -> array:
        filtered = array("h")
        if channels <= 0:
            return filtered
        sample_rate = float(self._audio_sample_rate or 16000)
        dt = 1.0 / sample_rate
        hp_rc = 1.0 / (2.0 * math.pi * self._audio_high_pass_hz)
        hp_alpha = hp_rc / (hp_rc + dt)
        lp_alpha = 1.0 - math.exp(-2.0 * math.pi * self._audio_low_pass_hz / sample_rate)
        gate_threshold = self._audio_noise_gate_percent / 100.0
        for frame_start in range(0, len(samples) - channels + 1, channels):
            frame_out: list[int] = []
            for channel in range(min(channels, 2)):
                value = int(samples[frame_start + channel]) / 32768.0
                if self._audio_high_pass:
                    value = hp_alpha * (
                        self._audio_filter_previous_y[channel]
                        + value - self._audio_filter_previous_x[channel]
                    )
                    self._audio_filter_previous_x[channel] = int(samples[frame_start + channel]) / 32768.0
                    self._audio_filter_previous_y[channel] = value
                if self._audio_low_pass:
                    self._audio_filter_low_pass_y[channel] += lp_alpha * (
                        value - self._audio_filter_low_pass_y[channel]
                    )
                    value = self._audio_filter_low_pass_y[channel]
                if self._audio_noise_gate and abs(value) < gate_threshold:
                    value = 0.0
                frame_out.append(max(-32768, min(32767, int(value * 32767.0))))
            if channels == 1:
                filtered.extend((frame_out[0], frame_out[0]))
            else:
                filtered.extend((frame_out[0], frame_out[1]))
        return filtered

    @Slot(bool)
    def setAudioHighPassEnabled(self, enabled: bool) -> None:
        self._audio_high_pass = bool(enabled)
        self._reset_audio_filter_state()
        self.stateChanged.emit()

    @Slot(float)
    def setAudioHighPassHz(self, frequency_hz: float) -> None:
        self._audio_high_pass_hz = max(20.0, min(300.0, float(frequency_hz)))
        self.stateChanged.emit()

    @Slot(bool)
    def setAudioLowPassEnabled(self, enabled: bool) -> None:
        self._audio_low_pass = bool(enabled)
        self._reset_audio_filter_state()
        self.stateChanged.emit()

    @Slot(float)
    def setAudioLowPassHz(self, frequency_hz: float) -> None:
        self._audio_low_pass_hz = max(500.0, min(7900.0, (self._audio_sample_rate or 16000) * 0.45, float(frequency_hz)))
        self.stateChanged.emit()

    @Slot(bool)
    def setAudioNoiseGateEnabled(self, enabled: bool) -> None:
        self._audio_noise_gate = bool(enabled)
        self.stateChanged.emit()

    @Slot(float)
    def setAudioNoiseGatePercent(self, threshold_percent: float) -> None:
        self._audio_noise_gate_percent = max(0.0, min(10.0, float(threshold_percent)))
        self.stateChanged.emit()

    def _reset_audio_filter_state(self) -> None:
        self._audio_filter_previous_x = [0.0, 0.0]
        self._audio_filter_previous_y = [0.0, 0.0]
        self._audio_filter_low_pass_y = [0.0, 0.0]

    def _filter_audio_samples(self, samples: array, channels: int) -> array:
        filtered = array("h")
        if channels <= 0:
            return filtered
        sample_rate = float(self._audio_sample_rate or 16000)
        dt = 1.0 / sample_rate
        hp_rc = 1.0 / (2.0 * math.pi * self._audio_high_pass_hz)
        hp_alpha = hp_rc / (hp_rc + dt)
        lp_alpha = 1.0 - math.exp(-2.0 * math.pi * self._audio_low_pass_hz / sample_rate)
        gate_threshold = self._audio_noise_gate_percent / 100.0
        for frame_start in range(0, len(samples) - channels + 1, channels):
            frame_out: list[int] = []
            for channel in range(min(channels, 2)):
                value = int(samples[frame_start + channel]) / 32768.0
                if self._audio_high_pass:
                    value = hp_alpha * (
                        self._audio_filter_previous_y[channel]
                        + value - self._audio_filter_previous_x[channel]
                    )
                    self._audio_filter_previous_x[channel] = int(samples[frame_start + channel]) / 32768.0
                    self._audio_filter_previous_y[channel] = value
                if self._audio_low_pass:
                    self._audio_filter_low_pass_y[channel] += lp_alpha * (
                        value - self._audio_filter_low_pass_y[channel]
                    )
                    value = self._audio_filter_low_pass_y[channel]
                if self._audio_noise_gate and abs(value) < gate_threshold:
                    value = 0.0
                frame_out.append(max(-32768, min(32767, int(value * 32767.0))))
            if channels == 1:
                filtered.extend((frame_out[0], frame_out[0]))
            else:
                filtered.extend((frame_out[0], frame_out[1]))
        return filtered

    @Slot(str)
    def setWifiLocalIp(self, local_ip: str) -> None:
        normalized = local_ip or "auto"
        if normalized == self._wifi_local_ip:
            return
        self._wifi_local_ip = normalized
        self._args.wifi_local_ip = normalized
        self._auto_reconnect_enabled = True
        self._reconnect_interval = 2.0
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
            self._auto_reconnect_enabled = False
            try:
                self._transport.close()
            except Exception:
                pass
            self._connection_request_id += 1
            self._set_transport_disconnected("Disconnected")
            return

        self._auto_reconnect_enabled = True
        self._reconnect_interval = 2.0
        self._queue_transport_connection(initial=False)
        self.stateChanged.emit()

    @Slot()
    def reloadTx12Mapping(self) -> None:
        try:
            controls = load_tx12_mapping(self._tx12_mapping_path)
        except (OSError, ValueError, TypeError) as exc:
            self._set_state(tx12MapStatus=f"Mapping error: {exc}")
            return
        assignable = [item for item in controls if item["kind"] != "radio_ui"]
        assigned = sum(item["channel"] is not None for item in assignable)
        defined = sum(
            item["channel"] is not None and (
                bool(item["purpose"]) and item["purpose"] != "Unassigned"
                or any(position["meaning"] for position in item["positions"])
            )
            for item in assignable
        )
        self._set_state(
            tx12Controls=controls,
            tx12MapStatus=f"{defined} functions defined; {assigned}/{len(assignable)} channels assigned",
        )

    @Slot()
    def findRadioBackpack(self) -> None:
        if self._radio_wifi_scanning:
            return
        self._network_interfaces = list_wifi_interfaces()
        self._radio_wifi_scanning = True
        self._set_state(radioWifiStatus="Scanning local Wi-Fi...", radioWifiScanning=True)
        local_ips = [entry["value"] for entry in self._network_interfaces if entry["value"] != "auto"]
        selected_ip = self._wifi_local_ip

        def worker() -> None:
            try:
                result = discover_tx_backpack(local_ips, selected_ip)
            except Exception as exc:
                result = (None, f"TX Backpack scan failed: {exc}")
            with self._radio_wifi_lock:
                self._radio_wifi_pending = result

        threading.Thread(target=worker, daemon=True, name="tx-backpack-discovery").start()

    @Slot(bool)
    def setElrsPacketLogging(self, enabled: bool) -> None:
        self._set_state(elrsPacketLogging=bool(enabled))

    @Slot()
    def exportLinkLog(self) -> None:
        try:
            self._flush_command_traces()
            if not self._link_log_file.exists():
                self._set_state(linkLogExportStatus="No saved link events to export")
                return
            export_folder = self._log_file.parent / "exports"
            export_folder.mkdir(parents=True, exist_ok=True)
            stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
            destination = export_folder / f"link-events-{stamp}.jsonl"
            shutil.copyfile(self._link_log_file, destination)
        except OSError as exc:
            self._set_state(linkLogExportStatus=f"Export failed: {exc}")
            return
        self._set_state(linkLogExportStatus=f"Exported {destination}")

    @Slot()
    def scanBuoyNetwork(self) -> None:
        target = str(self._state.get("serialTarget", ""))
        if self._network_scanning:
            return
        self._network_interfaces = list_wifi_interfaces()
        local_ip = self.stationNetworkIp
        try:
            address = ipaddress.IPv4Address(local_ip)
        except ValueError:
            self._set_state(networkScanStatus="No station IPv4 address is available for a network scan")
            return
        if address not in BUOY_ROUTER_NETWORK:
            self._set_state(
                networkScanStatus="Connect the station to 192.168.8.0/24 to scan for buoy devices",
                networkBuoyDevices=[],
            )
            return
        subnet = BUOY_ROUTER_NETWORK
        local_ip = str(address)
        port = int(self._args.tcp_port)
        control_host = (
            extract_wifi_host_from_target(target)
            if target.startswith("WIFI:")
            else None
        )
        transport_connected = self._transport is not None and self._transport.is_open
        # The target may already have accepted a pending control connection even
        # before the main thread installs its transport. Never probe its port.
        reserved_host = control_host
        connected_host = control_host if transport_connected else None
        self._network_scanning = True
        self._set_state(networkScanning=True, networkScanStatus=f"Scanning {subnet} for buoy services...")

        def worker() -> None:
            found: list[str] = [connected_host] if connected_host else []
            def probe(address: str) -> str | None:
                try:
                    with socket.create_connection((address, port), timeout=0.22, source_address=(local_ip, 0)) as connection:
                        connection.close()
                    return address
                except OSError:
                    return None
            addresses = [
                str(candidate)
                for candidate in subnet.hosts()
                if str(candidate) not in {local_ip, reserved_host}
            ]
            with ThreadPoolExecutor(max_workers=64) as executor:
                futures = [executor.submit(probe, address) for address in addresses]
                for future in as_completed(futures):
                    result = future.result()
                    if result:
                        found.append(result)
            if control_host and control_host not in found and self._transport is not None and self._transport.is_open:
                found.append(control_host)
            found.sort(key=lambda value: tuple(int(part) for part in value.split(".")))
            devices = [
                {"ip": address, "label": f"Buoy {index + 1}", "status": "Connected" if target.startswith("WIFI:") and address == extract_wifi_host_from_target(target) else "Reachable"}
                for index, address in enumerate(found)
            ]
            with self._network_scan_lock:
                self._network_scan_pending = (devices, f"{len(devices)} buoy device(s) found on {subnet}")

        threading.Thread(target=worker, daemon=True, name="buoy-network-scan").start()

    def _drain_network_scan(self) -> None:
        with self._network_scan_lock:
            result = self._network_scan_pending
            self._network_scan_pending = None
        if result is None:
            return
        devices, status = result
        self._network_scanning = False
        self._set_state(networkBuoyDevices=devices, networkScanStatus=status, networkScanning=False)

    def _drain_radio_wifi_discovery(self) -> None:
        with self._radio_wifi_lock:
            result = self._radio_wifi_pending
            self._radio_wifi_pending = None
        if result is None:
            return
        address, status = result
        self._radio_wifi_scanning = False
        if address != self._state.get("radioWifiIp"):
            self._radio_system_id = None
            self._radio_telemetry_last_at = 0.0
            self._radio_link_last_at = 0.0
            self._radio_heartbeat_last_at = 0.0
            self._radio_telemetry_packets = 0
            self._radio_field_last_at.clear()
            reset: dict[str, object] = dict(
                radioTelemetryPackets=0,
                radioSystemId="Unknown",
                radioLinkStatus="No receiver RADIO_STATUS relayed",
                radioRssiStatus="Unknown",
                radioLinkQuality="Waiting for ELRS receiver status",
                radioSnrStatus="Unknown",
                radioBatteryStatus="N/A",
                radioCurrentDraw="N/A",
                radioLocation="Unknown",
                radioChannels=[],
            )
            if self._radio_telemetry_socket is not None:
                reset["radioTelemetryStatus"] = f"Listening on UDP {ELRS_MAVLINK_UDP_PORT}; waiting for buoy telemetry"
            if self._state.get("telemetrySource") == "ELRS":
                reset.update(
                    telemetrySource="No live telemetry",
                    batteryStatus="N/A",
                    currentDraw="N/A",
                    currentLocation="Unknown",
                )
            self._set_state(**reset)
        self._set_state(radioWifiStatus=status, radioWifiIp=address or "", radioWifiScanning=False)
        if "MAVLink forwarding disabled" in status:
            self._set_state(radioTelemetryStatus="Backpack MAVLink forwarding disabled; set Backpack > Telemetry to WiFi in ELRS Lua")

    def _start_radio_telemetry(self) -> None:
        receiver = None
        try:
            receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            receiver.bind(("0.0.0.0", ELRS_MAVLINK_UDP_PORT))
            receiver.setblocking(False)
        except OSError as exc:
            if receiver is not None:
                receiver.close()
            self._set_state(radioTelemetryStatus=f"UDP {ELRS_MAVLINK_UDP_PORT} unavailable: {exc}")
            return
        self._radio_telemetry_socket = receiver
        self._set_state(radioTelemetryStatus=f"Listening on UDP {ELRS_MAVLINK_UDP_PORT}; waiting for buoy telemetry")

    def _radio_is_primary(self) -> bool:
        return (self._state.get("telemetryMode") == "MAVLINK"
                or self._transport is None or not self._transport.is_open)

    def _poll_radio_telemetry(self) -> None:
        receiver = self._radio_telemetry_socket
        backpack_ip = str(self._state.get("radioWifiIp", ""))
        if receiver is None:
            return
        now = time.monotonic()
        if (backpack_ip and "MAVLink forwarding active" in str(self._state.get("radioWifiStatus", ""))
                and now - self._radio_heartbeat_last_at >= 3.0):
            try:
                receiver.sendto(gcs_heartbeat(), (backpack_ip, 14555))
                self._radio_heartbeat_last_at = now
            except OSError as exc:
                self._set_state(radioTelemetryStatus=f"ELRS UDP send error: {exc}")
        for _ in range(32):
            try:
                datagram, sender = receiver.recvfrom(4096)
            except BlockingIOError:
                break
            except OSError as exc:
                self._set_state(radioTelemetryStatus=f"ELRS UDP receive error: {exc}")
                break
            if not backpack_ip or sender[0] != backpack_ip:
                continue
            for message in decode_datagram(datagram):
                if self._state.get("elrsPacketLogging"):
                    packet_json = json.dumps(message, sort_keys=True, separators=(",", ":"))
                    self._append_comm_log(f"ELRS MAVLink RX {packet_json}")
                if message["kind"] == "radio_status":
                    self._radio_link_last_at = time.monotonic()
                    local_rssi = message["rssi_raw"]
                    remote_rssi = message["remote_rssi_raw"]
                    if int(message["system_id"]) == 1 and int(message["component_id"]) == 68:
                        # The buoy firmware relays the ExpressLRS receiver's
                        # RADIO_STATUS: rssi is uplink LQ on a 0..255 scale,
                        # remrssi is the positive magnitude of RSSI in dBm.
                        self._set_state(
                            radioLinkStatus="ELRS receiver status live",
                            radioRssiStatus=f"-{remote_rssi} dBm (RX uplink)" if remote_rssi is not None and remote_rssi > 0 else "Unknown",
                            radioLinkQuality=f"{round(local_rssi * 100 / 255)}% (RX uplink)" if local_rssi is not None else "Unknown",
                            radioSnrStatus=(f"{message['noise_raw'] if message['noise_raw'] < 128 else message['noise_raw'] - 256} dB (RX uplink)"
                                            if message["noise_raw"] is not None else "Unknown"),
                        )
                    else:
                        self._set_state(
                            radioLinkStatus="MAVLink RADIO_STATUS received",
                            radioRssiStatus=f"{local_rssi} (device units)" if local_rssi is not None else "Unknown",
                            radioLinkQuality="Unavailable from this radio status",
                            radioSnrStatus="Unknown",
                        )
                    continue
                system_id = int(message["system_id"])
                if self._radio_system_id is None:
                    self._radio_system_id = system_id
                if system_id != self._radio_system_id:
                    continue
                self._radio_telemetry_last_at = time.monotonic()
                self._radio_telemetry_packets += 1
                updates: dict[str, object] = {
                    "radioTelemetryStatus": "Receiving buoy telemetry over ELRS",
                    "radioTelemetryPackets": self._radio_telemetry_packets,
                    "radioSystemId": str(system_id),
                }
                kind = message["kind"]
                if kind == "compact_status":
                    self._radio_field_last_at["buoy_state"] = self._radio_telemetry_last_at
                    self._radio_field_last_at["mission_progress"] = self._radio_telemetry_last_at
                    mission_status = {0: "IDLE", 1: "STARTING", 2: "MOVING", 3: "COLLECTING",
                                      4: "RETURNING_HOME", 5: "STOPPED", 6: "COMPLETE",
                                      7: "RETURNED_HOME", 8: "BLOCKED"}.get(message["mission_state"], "UNKNOWN")
                    progress = dict(self._state.get("radioMissionProgress") or {})
                    progress.update(status=mission_status, percent_complete=message["percent"],
                                    current_waypoint_index=max(0, message["waypoint"] - 1),
                                    waypoint_count=message["waypoint_count"])
                    block_reason = MISSION_START_BLOCK_REASONS.get(message["mission_block"], "Unknown start interlock")
                    if block_reason:
                        progress["block_reason"] = block_reason
                    else:
                        progress.pop("block_reason", None)
                    if message["home_distance_m"] >= 0:
                        progress["distance_from_home_m"] = message["home_distance_m"]
                    else:
                        progress.pop("distance_from_home_m", None)
                    updates.update(radioArmStatus="ARMED" if message["armed"] else "DISARMED",
                                   radioAssistMode={0: "Direct", 1: "Assist", 2: "Assist, no heading",
                                                    3: "Mission"}.get(message["assist"], "Unknown"),
                                   radioRecordingStatus="Recording" if message["recording"] else "Not recording",
                                   radioAudioRecording=message["audio_recording"],
                                   radioMissionProgress=progress, radioMissionStatus=mission_status.replace("_", " "),
                                   missionStartBlockReason=block_reason,
                                   activeHomeStatus={0: "No active Home", 1: "Manual Home", 2: "Mission Home"}.get(message["home_kind"], "Unknown"),
                                   gpsFixValid=message["gps_fix"],
                                   homeDistanceMeters=message["home_distance_m"] if message["home_distance_m"] >= 0 else None)
                    buoy_state = dict(self._state.get("radioBuoyState") or {})
                    buoy_state.update(armed=message["armed"], recording=message["recording"])
                    updates["radioBuoyState"] = buoy_state
                    if self._radio_is_primary():
                        updates["scienceExperimentState"] = "RECORDING" if message["recording"] else "IDLE"
                        updates["audioRecordingEnabled"] = message["audio_recording"]
                        if message["heading_deg"] >= 0:
                            updates["magnetometerTiltHeadingDeg"] = message["heading_deg"]
                elif kind == "calibration_status":
                    updates.update(radioCalibrationStatus={0: "Idle", 1: "Active", 2: "Stopped",
                                                           3: "Complete"}.get(message["state"], "Unknown"),
                                   radioCalibrationStep=message["step"], radioCalibrationPower=message["power"],
                                   radioCalibrationMaxPower=message["max_power"],
                                   radioCalibrationDuration=message["duration_s"])
                elif kind == "heartbeat":
                    self._radio_field_last_at["arm"] = self._radio_telemetry_last_at
                    updates["radioArmStatus"] = "ARMED" if message["armed"] else "DISARMED"
                elif kind == "power":
                    self._radio_field_last_at["power"] = self._radio_telemetry_last_at
                    voltage = message["voltage_v"]
                    current = message["current_a"]
                    percent = message["remaining_pct"]
                    battery_parts = []
                    if voltage is not None:
                        battery_parts.append(f"{voltage:.3f} V")
                    if current is not None:
                        battery_parts.append(f"{current:.3f} A")
                    if percent is not None:
                        battery_parts.append(f"{percent}%")
                    battery = " / ".join(battery_parts) or "N/A"
                    draw = f"{current:.3f} A" if current is not None else "N/A"
                    updates.update(radioBatteryStatus=battery, radioCurrentDraw=draw)
                    if self._radio_is_primary():
                        updates.update(batteryStatus=battery, currentDraw=draw, telemetrySource="ELRS")
                        if voltage is not None and current is not None:
                            self._last_power_at = time.time()
                            self._record_power_sample(voltage, current)
                            self._latest_power = {
                                "battery_voltage_v": voltage,
                                "current_a": current,
                                "battery_percent": percent,
                            }
                elif kind == "position":
                    self._radio_field_last_at["position"] = self._radio_telemetry_last_at
                    location = _format_udp_position(message["latitude"], message["longitude"])
                    updates["radioLocation"] = location
                    speed = message["ground_speed_mps"]
                    track = message["ground_track_deg"]
                    updates["radioGroundMotion"] = (
                        f"{speed:.2f} m/s, GPS track {track:.0f}°" if speed is not None and track is not None
                        else "GPS motion unknown"
                    )
                    if self._radio_is_primary():
                        updates.update(currentLocation=location, telemetrySource="ELRS",
                                       navigationGpsSpeedAvailable=speed is not None,
                                       navigationGpsSpeedMps=round(speed, 3) if speed is not None else 0.0,
                                       navigationGpsCourseAvailable=track is not None,
                                       navigationGpsCourseDeg=round(track, 1) if track is not None else 0.0)
                elif kind == "gps_quality":
                    self._radio_field_last_at["gps_quality"] = self._radio_telemetry_last_at
                    updates.update(gpsSatelliteCount=message["satellites"], gpsHdop=message["hdop"], gpsFixValid=message["fix_valid"])
                    if self._radio_is_primary():
                        updates["telemetrySource"] = "ELRS"
                elif kind == "buoy_state":
                    self._radio_field_last_at["buoy_state"] = self._radio_telemetry_last_at
                    motor_pwm = message["motor_pwm"]
                    signed_motor_drive = [
                        int(round(value * 100.0 / 255.0)) for value in motor_pwm
                    ]
                    motor_text = " | ".join(
                        f"{name} {value:+d}/255" if value else f"{name} stopped"
                        for name, value in zip(("Rear", "Left", "Right"), motor_pwm)
                    )
                    updates.update(
                        radioBuoyMode=message["mode"],
                        radioMotorOutput=motor_text,
                        radioRecordingStatus="Recording" if message["recording"] else "Not recording",
                        radioBuoyState=message,
                    )
                    if self._radio_is_primary():
                        self._last_motor_output_at = time.time()
                        updates.update(
                            buoyMode=message["mode"].lower(),
                            controlSource=message["control_source"],
                            radioArmStatus="ARMED" if message["armed"] else "DISARMED",
                            recoveryStatus="MANUAL_STOP" if message["recovery_stop"] else "READY",
                            scienceExperimentState="RECORDING" if message["recording"] else "UNKNOWN",
                            telemetrySource="ELRS",
                            rearMotorActualDrive=signed_motor_drive[0],
                            frontLeftMotorActualDrive=signed_motor_drive[1],
                            frontRightMotorActualDrive=signed_motor_drive[2],
                            rearMotorPwm=motor_pwm[0],
                            frontLeftMotorPwm=motor_pwm[1],
                            frontRightMotorPwm=motor_pwm[2],
                            motorOutputTelemetry=True,
                        )
                elif kind == "buoy_audio_state":
                    self._radio_field_last_at["buoy_audio_state"] = self._radio_telemetry_last_at
                    updates["radioAudioRecording"] = bool(message["audio_recording"])
                    if self._radio_is_primary():
                        updates["telemetrySource"] = "ELRS"
                elif kind == "home_position":
                    coordinates = f"{message['latitude']:.7f}, {message['longitude']:.7f}"
                    updates.update(activeHomeCoordinates=coordinates, telemetrySource="ELRS")
                elif kind == "home_source":
                    self._radio_field_last_at["home_source"] = self._radio_telemetry_last_at
                    source = {0: "No active Home", 1: "Manual Home", 2: "Mission Home"}.get(message["source"], "Unknown Home source")
                    updates.update(activeHomeStatus=source, telemetrySource="ELRS")
                    if message["source"] == 0:
                        updates.update(activeHomeCoordinates="Unknown", homeDistanceMeters=None)
                elif kind == "science":
                    science_name = str(message["name"])
                    science_value = float(message["value"])
                    self._radio_field_last_at[f"science:{science_name}"] = self._radio_telemetry_last_at
                    if science_name == "MAN_AST":
                        updates["radioAssistMode"] = {0: "Direct", 1: "Assist", 2: "Assist, no heading",
                                                      3: "Mission"}.get(int(round(science_value)), "Unknown")
                    elif science_name == "CAL_STAT":
                        updates["radioCalibrationStatus"] = {0: "Idle", 1: "Active", 2: "Stopped",
                                                             3: "Complete", 4: "Blocked", 5: "SD error"}.get(int(round(science_value)), "Unknown")
                    elif science_name in {"CAL_STEP", "CAL_PWR", "CAL_MAX", "CAL_DUR"}:
                        state_key = {"CAL_STEP": "radioCalibrationStep", "CAL_PWR": "radioCalibrationPower",
                                     "CAL_MAX": "radioCalibrationMaxPower", "CAL_DUR": "radioCalibrationDuration"}[science_name]
                        updates[state_key] = int(round(science_value))
                    if self._radio_is_primary():
                        if science_name in {"MIS_STATE", "MIS_BLOCK", "MIS_PCT", "MIS_COUNT", "MIS_WP", "DIST_HOME", "DIST_WP"}:
                            self._radio_field_last_at["mission_progress"] = self._radio_telemetry_last_at
                        progress = dict(self._state.get("radioMissionProgress") or {})
                        if science_name == "MIS_STATE":
                            mission_status = {0: "IDLE", 1: "STARTING", 2: "MOVING", 3: "COLLECTING",
                                              4: "RETURNING_HOME", 5: "STOPPED", 6: "COMPLETE",
                                              7: "RETURNED_HOME", 8: "BLOCKED"}.get(int(round(science_value)), "UNKNOWN")
                            if mission_status in {"IDLE", "STARTING"}:
                                progress = {"status": mission_status}
                            else:
                                progress.update(status=mission_status)
                            updates["radioMissionProgress"] = progress
                            updates["radioMissionStatus"] = mission_status.replace("_", " ")
                        elif science_name == "MIS_BLOCK":
                            reason = MISSION_START_BLOCK_REASONS.get(int(round(science_value)), "Unknown start interlock")
                            if reason:
                                progress["block_reason"] = reason
                            else:
                                progress.pop("block_reason", None)
                            updates["radioMissionProgress"] = progress
                            updates["missionStartBlockReason"] = reason
                        elif science_name == "MIS_PCT":
                            progress["percent_complete"] = int(round(science_value))
                            updates["radioMissionProgress"] = progress
                        elif science_name == "MIS_COUNT":
                            progress["waypoint_count"] = int(round(science_value))
                            updates["radioMissionProgress"] = progress
                        elif science_name == "MIS_WP":
                            progress["current_waypoint_index"] = max(0, int(round(science_value)) - 1)
                            updates["radioMissionProgress"] = progress
                        elif science_name == "DIST_HOME":
                            progress["distance_from_home_m"] = science_value
                            updates["radioMissionProgress"] = progress
                        elif science_name == "DIST_WP":
                            progress["distance_to_waypoint_m"] = science_value
                            updates["radioMissionProgress"] = progress
                        if science_name in ("HOME_DIST", "DIST_HOME"):
                            updates["homeDistanceMeters"] = round(science_value, 1)
                            updates["telemetrySource"] = "ELRS"
                        if science_name == "MAG_HEAD":
                            updates["magnetometerHeadingDeg"] = round(science_value, 1)
                            updates["telemetrySource"] = "ELRS"
                        elif science_name == "TILT_HEAD":
                            updates["magnetometerTiltHeadingDeg"] = round(science_value, 1)
                            updates["magnetometerTiltStatus"] = "Low-rate accelerometer tilt correction via Backpack"
                            updates["telemetrySource"] = "ELRS"
                        elif science_name == "SEA_ACC":
                            updates["navigationSeaAccelerationRmsG"] = round(science_value, 4)
                            updates["telemetrySource"] = "ELRS"
                        elif science_name == "SEA_GYR":
                            updates["navigationSeaAngularRateRmsDps"] = round(science_value, 1)
                            updates["telemetrySource"] = "ELRS"
                        formatted = {
                            "WATER_TEMP": ("waterTemperature", f"{science_value:.2f} °C"),
                            "DEPTH_M": ("currentDepth", f"{science_value:.3f} m"),
                            "TDS_V": ("conductivityVoltage", f"{science_value:.3f} V"),
                            "TURBIDITY": ("turbidityVoltage", f"{science_value:.3f} V"),
                        }.get(science_name)
                        if formatted:
                            updates[formatted[0]] = formatted[1]
                            updates["telemetrySource"] = "ELRS"
                elif kind == "channels":
                    self._radio_field_last_at["channels"] = self._radio_telemetry_last_at
                    updates["radioChannels"] = message["channels"]
                elif kind == "channels_raw":
                    self._radio_field_last_at["channels"] = self._radio_telemetry_last_at
                    channels = list(self._state.get("radioChannels", []))
                    channels.extend([None] * (16 - len(channels)))
                    offset = int(message["port"]) * 8
                    channels[offset:offset + 8] = message["channels"]
                    updates["radioChannels"] = channels
                    sf_channel = channels[5]
                    if sf_channel is not None and 900 <= sf_channel <= 2100:
                        updates["telemetryMode"] = "MAVLINK" if sf_channel >= 1700 else "WIFI"
                self._set_state(**updates)
                if kind == "gps_quality":
                    self._refresh_gps_quality()

        now = time.monotonic()
        if self._radio_link_last_at and now - self._radio_link_last_at > 5.0:
            self._radio_link_last_at = 0.0
            self._set_state(radioLinkStatus="RADIO_STATUS stale", radioRssiStatus="Unknown", radioLinkQuality="Unknown", radioSnrStatus="Unknown")
        if self._radio_field_last_at.get("channels", 0.0) and now - self._radio_field_last_at["channels"] > 10.0:
            self._radio_field_last_at.pop("channels")
            self._set_state(radioChannels=[])
        if self._radio_field_last_at.get("arm", 0.0) and now - self._radio_field_last_at["arm"] > 5.0:
            self._radio_field_last_at.pop("arm")
            if self._radio_is_primary():
                self._set_state(radioArmStatus="Unknown")
        if self._radio_field_last_at.get("power", 0.0) and now - self._radio_field_last_at["power"] > 15.0:
            self._radio_field_last_at.pop("power")
            updates = {"radioBatteryStatus": "N/A", "radioCurrentDraw": "N/A"}
            if self._state.get("telemetrySource") == "ELRS":
                updates.update(batteryStatus="N/A", currentDraw="N/A")
            self._set_state(**updates)
        if self._radio_field_last_at.get("position", 0.0) and now - self._radio_field_last_at["position"] > 15.0:
            self._radio_field_last_at.pop("position")
            updates = {"radioLocation": "Unknown", "radioGroundMotion": "Unknown"}
            if self._state.get("telemetrySource") == "ELRS":
                updates.update(currentLocation="Unknown", navigationGpsSpeedAvailable=False,
                               navigationGpsCourseAvailable=False)
            self._set_state(**updates)
        if self._radio_field_last_at.get("buoy_state", 0.0) and now - self._radio_field_last_at["buoy_state"] > 5.0:
            self._radio_field_last_at.pop("buoy_state")
            updates = {"radioBuoyMode": "Unknown", "radioMotorOutput": "Unknown",
                       "radioRecordingStatus": "Unknown", "radioBuoyState": None,
                       "radioAudioRecording": None}
            if self._radio_is_primary():
                updates.update(buoyMode="unknown", controlSource="UNKNOWN", radioArmStatus="Unknown",
                               recoveryStatus="Unknown", scienceExperimentState="UNKNOWN",
                               rearMotorActualDrive=0, frontLeftMotorActualDrive=0,
                               frontRightMotorActualDrive=0, rearMotorPwm=-1,
                               frontLeftMotorPwm=-1, frontRightMotorPwm=-1,
                               motorOutputTelemetry=False)
            self._set_state(**updates)
        for science_name, state_key in (("WATER_TEMP", "waterTemperature"), ("DEPTH_M", "currentDepth"),
                                        ("TDS_V", "conductivityVoltage"), ("TURBIDITY", "turbidityVoltage")):
            field_key = f"science:{science_name}"
            if self._radio_field_last_at.get(field_key, 0.0) and now - self._radio_field_last_at[field_key] > 12.0:
                self._radio_field_last_at.pop(field_key, None)
                if self._radio_is_primary():
                    self._set_state(**{state_key: "N/A"})
        for science_name in ("HOME_DIST", "DIST_HOME"):
            field_key = f"science:{science_name}"
            if self._radio_field_last_at.get(field_key, 0.0) and now - self._radio_field_last_at[field_key] > 12.0:
                self._radio_field_last_at.pop(field_key, None)
                if self._radio_is_primary():
                    self._set_state(homeDistanceMeters=None)
        for science_name, state_key in (("MAG_HEAD", "magnetometerHeadingDeg"),
                                        ("TILT_HEAD", "magnetometerTiltHeadingDeg"),
                                        ("SEA_ACC", "navigationSeaAccelerationRmsG"),
                                        ("SEA_GYR", "navigationSeaAngularRateRmsDps")):
            field_key = f"science:{science_name}"
            if self._radio_field_last_at.get(field_key, 0.0) and now - self._radio_field_last_at[field_key] > 12.0:
                self._radio_field_last_at.pop(field_key, None)
                if self._radio_is_primary():
                    self._set_state(**{state_key: None})
        if (self._radio_field_last_at.get("mission_progress", 0.0)
                and now - self._radio_field_last_at["mission_progress"] > 12.0):
            self._radio_field_last_at.pop("mission_progress", None)
            if self._radio_is_primary():
                self._set_state(radioMissionProgress=None, radioMissionStatus="Mission telemetry stale")
        if self._radio_telemetry_last_at and time.monotonic() - self._radio_telemetry_last_at > 5.0:
            self._radio_telemetry_last_at = 0.0
            self._radio_field_last_at.clear()
            updates = {
                "radioTelemetryStatus": "ELRS telemetry stale (no packets for 5 s)",
                "radioBatteryStatus": "N/A",
                "radioCurrentDraw": "N/A",
                "radioLocation": "Unknown",
                "radioGroundMotion": "Unknown",
                "radioChannels": [],
                "radioBuoyMode": "Unknown",
                "radioMotorOutput": "Unknown",
                "radioRecordingStatus": "Unknown",
                "radioBuoyState": None,
                "radioAudioRecording": None,
                "radioAssistMode": "Unknown",
                "radioCalibrationStatus": "Telemetry stale",
            }
            if self._radio_is_primary():
                updates.update(radioArmStatus="Unknown", waterTemperature="N/A", currentDepth="N/A",
                               conductivityVoltage="N/A", turbidityVoltage="N/A",
                               magnetometerHeadingDeg=None, magnetometerTiltHeadingDeg=None,
                               navigationSeaAccelerationRmsG=None, navigationSeaAngularRateRmsDps=None)
            if self._state.get("telemetrySource") == "ELRS":
                updates.update(
                    telemetrySource="No live telemetry",
                    batteryStatus="N/A",
                    currentDraw="N/A",
                    currentLocation="Unknown",
                    rearMotorActualDrive=0,
                    frontLeftMotorActualDrive=0,
                    frontRightMotorActualDrive=0,
                    rearMotorPwm=-1,
                    frontLeftMotorPwm=-1,
                    frontRightMotorPwm=-1,
                    motorOutputTelemetry=False,
                )
            self._set_state(**updates)

    @Slot()
    def stopAllMotors(self) -> None:
        if self._active_motor_test is not None:
            stamp = datetime.now(timezone.utc).isoformat(timespec="milliseconds")
            age_ms = int((time.monotonic() - self._last_motor_test_send_at) * 1000)
            self._command_trace_pending.append(f"{stamp} MOTOR TEST STOP REQUEST last_keepalive_age_ms={age_ms}\n")
        self._last_motor_test_log_command = None
        self._active_motor_test = None
        self._last_motor_test_actual_drive = 0
        if self._transport is None:
            self._set_state(lastSendResult="No serial link", lastSentLine="")
            return

        command = "CTRL STOP"
        result = send_text(self._transport, command)
        self._set_state(lastSendResult=result, lastSentLine=command)
        if "failed" not in result.lower():
            self._append_comm_log(f"TX {command}")

    @Slot(str)
    def stopMotorTest(self, reason: str) -> None:
        if self._active_motor_test is not None:
            stamp = datetime.now(timezone.utc).isoformat(timespec="milliseconds")
            self._command_trace_pending.append(f"{stamp} MOTOR TEST HOLD ENDED reason={reason}\n")
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
            navigationStatus=("Assist armed; waiting for Wi-Fi keyboard input" if is_enabled else "Assist off"),
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
            navigationStatus="Stopped; release keyboard controls to resume",
            navigationCorrectionDeg=0.0,
            keyboardInputSummary="Idle",
        )
        self._append_comm_log("NAV STOP")
        self.stopAllMotors()

    @Slot(bool)
    def setKeyboardDriveEnabled(self, enabled: bool) -> None:
        self.setControlInputMode("keyboard" if enabled else "tx12")

    @Slot(str)
    def setControlInputMode(self, mode: str) -> None:
        normalized_mode = (mode or "").strip().lower()
        if normalized_mode not in {"keyboard", "tx12"}:
            return
        if normalized_mode == self._state.get("controlInputMode"):
            return

        self._keyboard_keys.clear()
        self._keyboard_turn = 0.0
        self._keyboard_thrust = 0.0
        self._keyboard_yaw = 0.0
        self._navigation_requires_center = False
        detail = {
            "keyboard": "Wi-Fi keyboard drive: WASD movement + arrow-key yaw",
            "tx12": "TX12 controls the buoy through the ELRS receiver",
        }[normalized_mode]
        self._set_state(
            controlInputMode=normalized_mode,
            controlInputModeDetail=detail,
            keyboardDriveEnabled=(normalized_mode == "keyboard"),
            keyboardInputSummary=(
                "Ready: WASD translation + arrows yaw" if normalized_mode == "keyboard" else "Idle"
            ),
            navigationReleaseRequired=False,
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
            "test_kind": (
                "water_compass_motor_load" if self._compass_load_capture_phase
                else "water_imu_calibration" if environment == "water"
                else "dry_land_baseline"
            ),
            "serial_target": str(self._state.get("serialTarget", "")),
            "control_input_mode": str(self._state.get("controlInputMode", "tx12")),
            "send_rate_hz": float(self._args.send_rate),
            "mixer_power_limit_percent": int(round(self._mixer_power_limit * 100.0)),
            "navigation_axis_mapping": {
                "invert_x": bool(self._state.get("navigationInvertX", False)),
                "invert_y": bool(self._state.get("navigationInvertY", False)),
                "swap_xy": bool(self._state.get("navigationSwapXY", False)),
            },
            "motor_calibration": self._motor_calibrations,
            "compass_load_capture_phase": self._compass_load_capture_phase or None,
            "compass_load_capture_setup": dict(self._compass_load_capture_setup),
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
        self._compass_load_capture_values = {key: [] for key in ("mag_x", "mag_y", "mag_z", "heading_deg", "current_a", "gyro_x_dps", "gyro_y_dps", "gyro_z_dps")}
        self._set_state(
            testSessionActive=True,
            testSessionStatus=("Recording compass " + self._compass_load_capture_phase if self._compass_load_capture_phase else "Recording synchronized telemetry"),
            testSessionFile=str(self._test_session.path),
            testSessionSampleCount=0,
            testSessionImuRateHz=0.0,
            testSessionImuLossPercent=0.0,
        )
        self._append_comm_log(f"TEST RECORD START file={self._test_session.path}")

    @Slot(str, str, str, int)
    def startCompassLoadCapture(self, phase: str, motor: str, direction: str, power_percent: int) -> None:
        normalized = str(phase or "").strip().lower()
        if normalized not in {"motors_off_baseline", "motor_load"}:
            return
        if self._state.get("navigationTestEnvironment") != "water":
            self._set_state(compassLoadCaptureStatus="Select Tethered water before compass load calibration")
            return
        if self._test_session is not None and self._test_session.active:
            self._set_state(compassLoadCaptureStatus="Stop the active log before starting a compass capture")
            return
        if not self._state.get("imuStreamEnabled"):
            self._set_state(compassLoadCaptureStatus="Start the IMU stream before capturing compass load data")
            return
        if normalized == "motors_off_baseline":
            if self._last_motor_output_at is None or time.time() - self._last_motor_output_at > 2.0:
                self._set_state(compassLoadCaptureStatus="Waiting for fresh motor-output telemetry before baseline capture")
                return
            actual = [
                int(self._state.get("rearMotorActualDrive", 0)),
                int(self._state.get("frontLeftMotorActualDrive", 0)),
                int(self._state.get("frontRightMotorActualDrive", 0)),
            ]
            if any(abs(value) > 2 for value in actual):
                self._set_state(compassLoadCaptureStatus="Stop all motors before capturing the water baseline")
                return
        self._compass_load_capture_phase = normalized
        self._compass_load_capture_setup = {
            "motor": str(motor or "none"),
            "direction": str(direction or "none"),
            "requested_power_percent": max(0, min(100, int(power_percent))),
        }
        self._set_state(compassLoadCapturePhase=normalized, compassLoadCaptureStatus="Starting water capture…")
        self.startTestSession()
        if self._test_session is None:
            self._compass_load_capture_phase = ""
            self._compass_load_capture_setup = {}
            self._set_state(compassLoadCapturePhase="")
        else:
            self._set_state(compassLoadCaptureStatus="Capturing raw field, current, heading, gyro, and motor outputs")

    def _close_test_session(self, reason: str) -> None:
        session = self._test_session
        if session is None:
            return
        path = session.path
        sample_count = session.sample_count
        capture_summary = self._test_session_summary().get("compass_motor_load", {})
        close_error: OSError | None = None
        try:
            session.close(reason, self._test_session_summary())
        except OSError as exc:
            close_error = exc
        finally:
            self._test_session = None
            self._test_imu_quality = None
            phase = self._compass_load_capture_phase
            self._compass_load_capture_phase = ""
            self._compass_load_capture_setup = {}
        compass_summary_state: dict[str, object] = {}
        compass_status = self._state.get("compassLoadCaptureStatus", "")
        if phase and close_error is None and capture_summary:
            compass_summary_state = {
                "phase": phase,
                "setup": capture_summary.get("setup", {}),
                "means": capture_summary.get("means", {}),
                "valid_mag_samples": capture_summary.get("valid_mag_samples", 0),
            }
            means = capture_summary.get("means", {})
            if phase == "motors_off_baseline" and int(capture_summary.get("valid_mag_samples", 0)) > 0:
                self._compass_load_baseline = compass_summary_state
                try:
                    self._compass_load_baseline_path.parent.mkdir(parents=True, exist_ok=True)
                    temporary_path = self._compass_load_baseline_path.with_suffix(".tmp")
                    temporary_path.write_text(json.dumps(self._compass_load_baseline, indent=2), encoding="utf-8")
                    temporary_path.replace(self._compass_load_baseline_path)
                    compass_status = f"Saved water baseline: {capture_summary.get('valid_mag_samples')} magnetometer samples"
                except OSError as exc:
                    compass_status = f"Baseline captured in log but could not persist locally: {exc}"
            elif phase == "motor_load":
                baseline_means = (self._compass_load_baseline or {}).get("means", {})
                if isinstance(means, dict) and isinstance(baseline_means, dict) and all(means.get(key) is not None and baseline_means.get(key) is not None for key in ("mag_x", "mag_y", "mag_z")):
                    delta = {key: round(float(means[key]) - float(baseline_means[key]), 2) for key in ("mag_x", "mag_y", "mag_z")}
                    base_heading, load_heading = baseline_means.get("heading_deg"), means.get("heading_deg")
                    if base_heading is not None and load_heading is not None:
                        delta["heading_deg"] = round((float(load_heading) - float(base_heading) + 180.0) % 360.0 - 180.0, 2)
                    compass_summary_state["delta_from_baseline"] = delta
                    compass_status = f"Load capture: Δfield X {delta['mag_x']:+.1f}, Y {delta['mag_y']:+.1f}, Z {delta['mag_z']:+.1f}; see log {path.name}"
                else:
                    compass_status = f"Load log saved: {path.name}; capture a same-heading motors-off baseline to calculate field delta"
        self._set_state(
            testSessionActive=False,
            testSessionStatus=(
                f"Saved {sample_count} samples" if close_error is None else f"Recorder close error: {close_error}"
            ),
            testSessionFile=str(path),
            testSessionSampleCount=sample_count,
            compassLoadCapturePhase="",
            compassLoadCaptureStatus=compass_status,
            compassLoadBaselineStatus=(
                "Water baseline ready · " + self._compass_load_baseline_path.name
                if self._compass_load_baseline else "No water motors-off baseline saved"
            ),
            compassLoadLastSummary=compass_summary_state or self._state.get("compassLoadLastSummary", {}),
        )
        self._append_comm_log(
            f"TEST RECORD STOP samples={sample_count} file={path}"
            + (f" error={close_error}" if close_error is not None else "")
        )

    @Slot()
    def stopTestSession(self) -> None:
        self._close_test_session("operator_stop")

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
        if "failed" in result.lower():
            self._active_motor_test = None
            self._last_motor_test_log_command = None
            self._append_comm_log(f"TX failed {command}: {result}")
            return
        next_test = (normalized_motor, power_value)
        if self._active_motor_test != next_test:
            self._last_motor_test_actual_drive = 0
        self._active_motor_test = next_test
        self._last_motor_test_send_at = time.monotonic()
        if command != self._last_motor_test_log_command:
            self._append_comm_log(f"TX {command}")
            self._last_motor_test_log_command = command
        else:
            stamp = datetime.now(timezone.utc).isoformat(timespec="milliseconds")
            self._command_trace_pending.append(f"{stamp} TX {command} keepalive\n")

    def _send_motor_test_keepalive(self) -> None:
        active = self._active_motor_test
        if active is None or self._transport is None or not self._transport.is_open:
            return
        if time.monotonic() - self._last_motor_test_send_at >= 0.2:
            self.sendMotorTestPower(*active)

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
            self._append_comm_log(f"TX failed CTRL MOTOR CAL {motor} {direction}: no serial link")
            self._set_state(motorCalibrationStatus=f"Apply failed for {motor}: no serial link")
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
        else:
            self._append_comm_log(f"TX failed {command}: {result}")
            self._set_state(motorCalibrationStatus=f"Apply failed for {normalized_motor}: {result}")

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
            self._append_comm_log(f"TX failed CTRL MOTOR CAL {motor}: no serial link")
            self._set_state(motorCalibrationStatus=f"Apply failed for {motor}: no serial link")
            return

        normalized_motor = (motor or "").strip().lower().replace("-", "_")
        self._pending_motor_calibration[normalized_motor] = set()
        self._set_state(motorCalibrationStatus=f"Sending {normalized_motor} settings to buoy…")
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
        if not str(self._state.get("motorCalibrationStatus", "")).startswith("Apply failed"):
            self._set_state(motorCalibrationStatus=f"Sent {normalized_motor} settings; waiting for buoy confirmation")

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

    @staticmethod
    def _empty_mixer_calibration_rows() -> list[dict[str, object]]:
        axes = ("forward", "reverse", "strafe_right", "strafe_left", "yaw_clockwise", "yaw_counterclockwise")
        return [
            {
                "power": power, "axis": axis, "axis_index": axis_index,
                "rear": 0, "front_left": 0, "front_right": 0, "set": False,
                "samples": 0, "accel_x_g": None, "accel_y_g": None,
                "yaw_rate_dps": None, "gyro_x_dps": None, "gyro_y_dps": None,
                "current_a": None, "battery_voltage_v": None, "heading_deg": None,
                "actual_drive_rear": None, "actual_drive_front_left": None, "actual_drive_front_right": None,
                "applied_pwm_rear": None, "applied_pwm_front_left": None, "applied_pwm_front_right": None,
                "gps_speed_mps": None, "gps_course_deg": None,
                "mag_x": None, "mag_y": None, "mag_z": None,
            }
            for axis_index, axis in enumerate(axes)
            for power in range(10, 101, 10)
        ]

    @Slot()
    def requestMixerCalibrationProfile(self) -> None:
        if self._transport is None:
            self._set_state(mixerCalibrationStatus="Connect to the buoy to read its SD profile")
            return
        command = "REQ MIX PROFILE"
        rows = self._empty_mixer_calibration_rows()
        self._set_state(
            mixerCalibrationRows=rows,
            mixerCalibrationStatus="Reading the buoy SD mixer profile…",
            mixerCalibrationSourceLog="",
        )
        result = send_text(self._transport, command)
        if "failed" in result.lower():
            self._set_state(mixerCalibrationStatus=f"Profile request failed: {result}")
        else:
            self._append_comm_log(f"TX {command}")

    @Slot()
    def importLatestMixerCalibrationLog(self) -> None:
        log_dir = self._log_file.parent / "tests"
        try:
            candidates = sorted(log_dir.glob("*.jsonl"), key=lambda item: item.stat().st_mtime, reverse=True)
        except OSError as exc:
            self._set_state(mixerCalibrationStatus=f"Cannot list test logs: {exc}")
            return
        rows = self._empty_mixer_calibration_rows()
        axis_lookup = {"thrust": (0, 1), "lateral": (2, 3), "yaw": (4, 5)}
        accum: dict[tuple[int, int], dict[str, list[float]]] = {}
        source_paths: list[str] = []
        for candidate in candidates:
            try:
                with candidate.open("r", encoding="utf-8") as handle:
                    records = [json.loads(line) for line in handle if line.strip()]
            except (OSError, ValueError):
                continue
            metadata = next((record for record in records if record.get("record_type") == "metadata"), {})
            if metadata.get("test_kind") != "water_imu_calibration":
                continue
            groups: dict[tuple[int, int], dict[str, list[float]]] = {}
            for record in records:
                if record.get("record_type") != "sample":
                    continue
                motion = record.get("motion", {}).get("requested_percent", {})
                values = {name: float(motion.get(name, 0) or 0) for name in ("lateral", "thrust", "yaw")}
                active = [(name, value) for name, value in values.items() if abs(value) >= 5.0]
                if len(active) != 1:
                    continue
                name, value = active[0]
                bucket = int(round(abs(value) / 10.0) * 10)
                if bucket < 10 or bucket > 100:
                    continue
                positive_axis, negative_axis = axis_lookup[name]
                axis_index = positive_axis if value > 0 else negative_axis
                motors = record.get("motors", {})
                vector = [motors.get(key, {}).get("mixed_percent") for key in ("rear", "front_left", "front_right")]
                if any(item is None for item in vector):
                    continue
                response = record.get("navigation", {}).get("measured_linear_accel_g", {})
                gyro = record.get("imu", {}).get("gyro_dps", {})
                current = record.get("power", {}).get("current_a")
                key = (axis_index, bucket)
                group = groups.setdefault(key, {name: [] for name in (
                    "rear", "front_left", "front_right", "accel_x_g", "accel_y_g", "yaw_rate_dps", "gyro_x_dps", "gyro_y_dps",
                    "current_a", "battery_voltage_v", "heading_deg", "actual_drive_rear", "actual_drive_front_left", "actual_drive_front_right",
                    "applied_pwm_rear", "applied_pwm_front_left", "applied_pwm_front_right",
                    "gps_speed_mps", "gps_course_deg", "mag_x", "mag_y", "mag_z",
                )})
                for field, measured in zip(("rear", "front_left", "front_right"), vector):
                    group[field].append(float(measured))
                for field, measured in (("accel_x_g", response.get("x")), ("accel_y_g", response.get("y")),
                                        ("yaw_rate_dps", gyro.get("z")), ("gyro_x_dps", gyro.get("x")),
                                        ("gyro_y_dps", gyro.get("y")), ("current_a", current),
                                        ("battery_voltage_v", record.get("power", {}).get("battery_voltage_v")),
                                        ("heading_deg", record.get("navigation", {}).get("magnetometer_raw", {}).get("heading_deg")),
                                        ("gps_speed_mps", record.get("navigation", {}).get("gps", {}).get("speed_mps")),
                                        ("gps_course_deg", record.get("navigation", {}).get("gps", {}).get("course_deg")),
                                        ("mag_x", record.get("navigation", {}).get("magnetometer_raw", {}).get("x")),
                                        ("mag_y", record.get("navigation", {}).get("magnetometer_raw", {}).get("y")),
                                        ("mag_z", record.get("navigation", {}).get("magnetometer_raw", {}).get("z"))):
                    if measured is not None:
                        group[field].append(float(measured))
                for motor_name, log_key in (("rear", "rear"), ("front_left", "front_left"), ("front_right", "front_right")):
                    motor_record = motors.get(log_key, {})
                    for field in ("actual_drive", "applied_pwm"):
                        measured = motor_record.get(field)
                        if measured is not None and int(measured) >= 0:
                            group[f"{field}_{motor_name}"].append(float(measured))
            if groups:
                source_paths.append(str(candidate))
                for key, group in groups.items():
                    target = accum.setdefault(key, {name: [] for name in group})
                    for field, values in group.items():
                        target[field].extend(values)
        if not accum:
            self._set_state(mixerCalibrationStatus=f"No directional samples found in {log_dir}")
            return
        axis_order = ("forward", "reverse", "strafe_right", "strafe_left", "yaw_clockwise", "yaw_counterclockwise")
        for row in rows:
            group = accum.get((int(row["axis_index"]), int(row["power"])))
            if not group:
                continue
            row["set"] = True
            row["samples"] = len(group["rear"])
            for field in ("rear", "front_left", "front_right"):
                row[field] = int(round(sum(group[field]) / len(group[field])))
            for field in group:
                if group[field]:
                    row[field] = round(sum(group[field]) / len(group[field]), 4)
        filled = sum(bool(row["set"]) for row in rows)
        self._set_state(
            mixerCalibrationRows=rows,
            mixerCalibrationSourceLog=f"{len(source_paths)} water test logs",
            mixerCalibrationStatus=f"Imported {filled}/60 power-direction points from {len(source_paths)} water test logs; inspect and edit the candidate before saving",
        )

    @Slot(str)
    def analyzeMixerCalibrationCsv(self, file_url: str) -> None:
        """Analyze a buoy SD calibration CSV and write a portable candidate JSON beside it."""
        source_path = QUrl(file_url).toLocalFile() if file_url.startswith("file:") else file_url
        try:
            bundle = analyze_calibration_csv(source_path)
            destination = Path(source_path).with_suffix(".mixer-candidate.json")
            destination.write_text(json.dumps(bundle, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        except (OSError, ValueError, TypeError) as exc:
            self._set_state(mixerCalibrationStatus=f"CSV analysis failed: {exc}")
            return
        analysis = bundle["analysis"]
        profile_rows = bundle["profile"]
        self._set_state(
            mixerCalibrationRows=profile_rows,
            mixerCalibrationAnalysisJson=json.dumps(analysis, ensure_ascii=False),
            mixerCalibrationSourceLog=analysis.get("source_file", ""),
            mixerCalibrationCandidateJsonPath=str(destination),
            mixerCalibrationStatus=(
                f"Analyzed {analysis['active_sample_rows']} active samples across "
                f"{analysis['measured_groups']} movement/power points. Candidate JSON: {destination.name}. "
                "It preserves the tested baseline; review and edit before upload."
            ),
        )

    @Slot(str)
    def importMixerCalibrationCandidateJson(self, file_url: str) -> None:
        source_path = QUrl(file_url).toLocalFile() if file_url.startswith("file:") else file_url
        try:
            bundle = load_candidate_json(source_path)
            rows = bundle["profile"]
            self._validateMixerRows(rows)
        except (OSError, ValueError, TypeError) as exc:
            self._set_state(mixerCalibrationStatus=f"Candidate JSON import failed: {exc}")
            return
        analysis = bundle.get("analysis", {})
        self._set_state(
            mixerCalibrationRows=rows,
            mixerCalibrationAnalysisJson=json.dumps(analysis, ensure_ascii=False),
            mixerCalibrationSourceLog=analysis.get("source_file", "Candidate JSON") if isinstance(analysis, dict) else "Candidate JSON",
            mixerCalibrationCandidateJsonPath=str(source_path),
            mixerCalibrationStatus=f"Loaded {sum(bool(row.get('set')) for row in rows)}/60 candidate points from {Path(source_path).name}; review before upload",
        )

    @Slot(str)
    def applyMixerCalibrationCandidateJson(self, file_url: str) -> None:
        """Load a completed, processed candidate and upload its full profile in one action."""
        if self._transport is None:
            self._set_state(mixerCalibrationStatus="Connect to the buoy before uploading a processed profile")
            return
        if str(self._state.get("radioArmStatus", "")).upper() != "DISARMED" or str(self._state.get("buoyMode", "")).lower() == "mission":
            self._set_state(mixerCalibrationStatus="Disarm the buoy and leave Mission mode before uploading")
            return
        source_path = QUrl(file_url).toLocalFile() if file_url.startswith("file:") else file_url
        try:
            bundle = load_candidate_json(source_path)
            rows = bundle["profile"]
            self._validateMixerRows(rows)
            analysis = bundle.get("analysis")
            if not isinstance(analysis, dict) or analysis.get("completed") is not True:
                raise ValueError("Processed profile must retain completed-run analysis metadata")
            baseline = analysis.get("baseline_profile")
            if not isinstance(baseline, list) or len(baseline) != 60:
                raise ValueError("Processed profile must retain all 60 tested baseline points")
            if any(not bool(row.get("set")) for row in rows):
                raise ValueError("Processed profile must define all 60 movement and power points")
        except (OSError, ValueError, TypeError) as exc:
            self._set_state(mixerCalibrationStatus=f"Processed mixer file rejected: {exc}")
            return

        self._set_state(
            mixerCalibrationRows=rows,
            mixerCalibrationAnalysisJson=json.dumps(analysis, ensure_ascii=False),
            mixerCalibrationSourceLog=analysis.get("source_file", Path(source_path).name),
            mixerCalibrationCandidateJsonPath=str(source_path),
            mixerCalibrationStatus=f"Loaded all 60 values from {Path(source_path).name}; uploading to buoy SD",
        )
        self.saveMixerCalibrationProfile(json.dumps(rows, ensure_ascii=False))

    @Slot(str, str)
    def exportMixerCalibrationCandidateJson(self, rows_json: str, file_url: str) -> None:
        destination_path = QUrl(file_url).toLocalFile() if file_url.startswith("file:") else file_url
        try:
            bundle = make_candidate_json(rows_json, str(self._state.get("mixerCalibrationAnalysisJson", "")))
            self._validateMixerRows(bundle["profile"])
            destination = Path(destination_path)
            destination.write_text(json.dumps(bundle, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        except (OSError, ValueError, TypeError) as exc:
            self._set_state(mixerCalibrationStatus=f"Candidate JSON export failed: {exc}")
            return
        self._set_state(mixerCalibrationCandidateJsonPath=str(destination), mixerCalibrationStatus=f"Exported candidate JSON to {destination.name}")

    @Slot(str)
    def compareMixerCalibrationTrials(self, file_urls_json: str) -> None:
        try:
            urls = json.loads(file_urls_json)
            if not isinstance(urls, list):
                raise ValueError("Select multiple calibration CSV files")
            paths = [QUrl(str(url)).toLocalFile() if str(url).startswith("file:") else str(url) for url in urls]
            comparison = compare_calibration_trials(paths)
            destination = Path(paths[0]).with_suffix(".trial-comparison.json")
            destination.write_text(json.dumps(comparison, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        except (OSError, ValueError, TypeError) as exc:
            self._set_state(mixerCalibrationComparisonStatus=f"Trial comparison failed: {exc}")
            return
        config_labels = ", ".join(
            f"{item['mixer_profile_crc32']} ({item['trial_count']} runs)"
            for item in comparison["configurations"]
        )
        self._set_state(
            mixerCalibrationComparisonStatus=(
                f"Compared {len(comparison['runs'])} completed runs across {len(comparison['configurations'])} mixer profile(s): "
                f"{config_labels}. Saved {destination.name}; use 3+ runs per profile and compare changes with run-to-run spread."
            )
        )

    @staticmethod
    def _validateMixerRows(rows: object) -> None:
        if not isinstance(rows, list) or len(rows) != 60:
            raise ValueError("A configuration needs exactly 60 profile rows")
        seen: set[tuple[int, int]] = set()
        for row in rows:
            if not isinstance(row, dict):
                raise ValueError("Profile row is invalid")
            power = int(row.get("power", 0))
            axis = int(row.get("axis_index", -1))
            motors = [int(row.get(field, 0)) for field in ("rear", "front_left", "front_right")]
            if power not in range(10, 101, 10) or axis not in range(6) or any(value < -100 or value > 100 for value in motors):
                raise ValueError("Motor values must be -100..100, at power steps 10..100, on axes 0..5")
            point = (axis, power)
            if point in seen:
                raise ValueError("Configuration contains a duplicate movement/power point")
            seen.add(point)
        if len(seen) != 60:
            raise ValueError("Configuration must define every movement at every power step")

    @Slot(str)
    def saveMixerCalibrationProfile(self, rows_json: str) -> None:
        if self._transport is None:
            self._set_state(mixerCalibrationStatus="Connect to the buoy before saving")
            return
        if str(self._state.get("radioArmStatus", "")).upper() != "DISARMED" or str(self._state.get("buoyMode", "")).lower() == "mission":
            self._set_state(mixerCalibrationStatus="Disarm the buoy and leave Mission mode before saving")
            return
        try:
            rows = json.loads(rows_json)
        except (TypeError, ValueError):
            self._set_state(mixerCalibrationStatus="Candidate profile data is invalid")
            return
        try:
            self._validateMixerRows(rows)
        except (TypeError, ValueError) as exc:
            self._set_state(mixerCalibrationStatus=f"Candidate profile is invalid: {exc}")
            return
        commands: list[str] = []
        for row in rows:
            if not isinstance(row, dict):
                self._set_state(mixerCalibrationStatus="Candidate profile row is invalid")
                return
            if not row.get("set"):
                continue
            try:
                power = int(row["power"])
                axis = int(row["axis_index"])
                motors = [int(row[field]) for field in ("rear", "front_left", "front_right")]
            except (KeyError, TypeError, ValueError):
                self._set_state(mixerCalibrationStatus="Candidate profile has an invalid motor value")
                return
            if power not in range(10, 101, 10) or axis not in range(6) or any(value < -100 or value > 100 for value in motors):
                self._set_state(mixerCalibrationStatus="Motor values must be between -100% and +100%")
                return
            commands.append(f"CTRL MIX POINT {power} {axis} {motors[0]} {motors[1]} {motors[2]}")
        if not commands:
            self._set_state(mixerCalibrationStatus="Enter or import at least one calibrated point before saving a draft")
            return
        payload = "\n".join(commands + ["CTRL MIX SAVE"])
        result = send_text(self._transport, payload)
        if "failed" in result.lower():
            self._set_state(mixerCalibrationStatus=f"Profile upload failed: {result}")
            return
        self._set_state(mixerCalibrationStatus=f"Sent {len(commands)} calibrated mixer points; waiting for SD save confirmation")
        self._append_comm_log(f"TX CTRL MIX POINT [{len(commands)} calibrated rows] + CTRL MIX SAVE")

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
        kind = communication_kind(entry)
        if kind is not None:
            self._link_log_sequence += 1
            link_entry = {
                **display_entry,
                "sequence": self._link_log_sequence,
                "utc": datetime.fromtimestamp(timestamp, timezone.utc).isoformat(timespec="milliseconds"),
                "epochMs": int(timestamp * 1000),
                "kind": kind,
                "source": communication_source(entry),
                "severity": communication_severity(entry),
            }
            self._link_log = (self._link_log + [link_entry])[-360:]
            self._link_log_pending.append(json.dumps(link_entry, ensure_ascii=False) + "\n")
        if category == NAVIGATION_LOG:
            self._navigation_log = (self._navigation_log + [display_entry])[-120:]
        elif category == SCIENTIFIC_LOG:
            self._scientific_log = (self._scientific_log + [display_entry])[-120:]
        else:
            self._system_log = (self._system_log + [display_entry])[-120:]
        if entry.startswith("TX ") or (
            entry.startswith("RX ") and str(self._state.get("serialTarget", "")).startswith("WIFI:")
        ):
            stamp = datetime.fromtimestamp(timestamp, timezone.utc).isoformat(timespec="milliseconds")
            self._command_trace_pending.append(f"{stamp} {entry}\n")
        if QCoreApplication.instance() is None:
            self.logChanged.emit()
        elif not self._log_update_timer.isActive():
            self._log_update_timer.start()

    def _flush_command_traces(self) -> None:
        if not self._command_trace_pending and not self._link_log_pending:
            return
        if self._command_trace_pending:
            lines = self._command_trace_pending[:]
            with self._log_file.open("a", encoding="utf-8") as handle:
                handle.writelines(lines)
            del self._command_trace_pending[:len(lines)]
        if self._link_log_pending:
            link_lines = self._link_log_pending[:]
            with self._link_log_file.open("a", encoding="utf-8") as handle:
                handle.writelines(link_lines)
            del self._link_log_pending[:len(link_lines)]
        self._last_command_trace_flush = time.monotonic()

    def _set_state(self, **updates: object) -> None:
        updates = {key: value for key, value in updates.items() if self._state.get(key) != value}
        if not updates:
            return
        self._state = {**self._state, **updates}
        timer = getattr(self, "_state_update_timer", None)
        if timer is None or QCoreApplication.instance() is None:
            self.stateChanged.emit()
        elif not timer.isActive():
            timer.start()

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
        if self._active_motor_test is not None:
            stamp = datetime.now(timezone.utc).isoformat(timespec="milliseconds")
            self._command_trace_pending.append(f"{stamp} MOTOR TEST LINK LOST status={status}\n")
        self._transport = None
        self._wifi_safety_last_at = 0.0
        self._wifi_detail_last_at = 0.0
        self._active_motor_test = None
        self._last_motor_test_log_command = None
        self._last_motor_test_actual_drive = 0
        self._connected_device_name = None
        self._gps_speed.invalidate()
        self._navigation.set_enabled(False, time.monotonic())
        self._navigation_requires_center = False
        self._keyboard_keys.clear()
        self._keyboard_turn = 0.0
        self._keyboard_thrust = 0.0
        self._keyboard_yaw = 0.0
        radio_fresh = bool(self._radio_telemetry_last_at and time.monotonic() - self._radio_telemetry_last_at <= 5.0)
        radio_buoy_state = self._state.get("radioBuoyState") if radio_fresh else None
        self._set_state(
            serialStatus=status,
            serialTarget="No port selected",
            wifiSafetyTelemetryLive=False,
            wifiDetailTelemetryLive=False,
            batteryStatus=self._state["radioBatteryStatus"] if radio_fresh else "N/A",
            currentDraw=self._state["radioCurrentDraw"] if radio_fresh else "N/A",
            currentLocation=self._state["radioLocation"] if radio_fresh else "Unknown",
            telemetrySource="ELRS" if radio_fresh else "No live telemetry",
            buoyMode=str(radio_buoy_state["mode"]).lower() if radio_buoy_state else "unknown",
            controlSource=str(radio_buoy_state["control_source"]) if radio_buoy_state else "UNKNOWN",
            recoveryStatus=("MANUAL_STOP" if radio_buoy_state["recovery_stop"] else "READY")
                           if radio_buoy_state else "Unknown",
            scienceExperimentState=("RECORDING" if radio_buoy_state["recording"] else "UNKNOWN")
                                   if radio_buoy_state else "UNKNOWN",
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
            try:
                from pygame._sdl2.audio import get_audio_device_names

                self._audio_output_devices = ["System default"] + list(get_audio_device_names(False))
            except Exception:
                self._audio_output_devices = ["System default"]
            self._initialize_audio_output(None)
        else:
            self._audio_output_status = "pygame is unavailable; audio cannot play"
        self.stateChanged.emit()
        self._start_radio_telemetry()
        self._queue_transport_connection(initial=True)
        self._timer.start()
        self.findRadioBackpack()
        self.scanBuoyNetwork()

    def shutdown(self) -> None:
        self._timer.stop()
        self._persist_power_histories()
        if self._radio_telemetry_socket is not None:
            self._radio_telemetry_socket.close()
            self._radio_telemetry_socket = None
        self._flush_command_traces()
        self._stop_audio_playback_thread()
        self._close_test_session("application_exit")
        if self._transport is not None:
            try:
                self._transport.close()
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
        self._last_reconnect_attempt = time.time()
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
            self._last_reconnect_attempt = time.time()
            self._reconnect_interval = min(10.0, max(2.0, self._reconnect_interval * 1.7))
            self._set_state(serialStatus="Reconnecting", serialTarget=target)
            if not initial:
                self._append_comm_log(f"RECONNECT failed target={target}: {status}")
            else:
                self._append_comm_log(f"CONNECT failed target={target}: {status}")
            return

        self._transport = transport
        self._reconnect_interval = 2.0
        self._last_protocol_response_time = time.time()
        self._last_protocol_send_time = 0.0
        self._last_ping_time = time.time()
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
            self.scanBuoyNetwork()
            if self._state.get("controlInputMode") == "keyboard":
                stop_result = send_text(self._transport, "CTRL STOP")
                self._set_state(lastSendResult=stop_result, lastSentLine="CTRL STOP")
                self._append_comm_log(f"TX CTRL STOP target={target}")
        send_text(self._transport, "REQ STATUS ALL")
        self._append_comm_log(f"TX REQ STATUS ALL target={target}")
        if initial:
            self._append_comm_log(f"CONNECT success target={target}")
        else:
            self._append_comm_log(f"RECONNECTED target={target}")

    def _send_current_command(self) -> None:
        if self._state.get("sdDownloadActive") or self._state.get("serviceMode"):
            return
        input_mode = str(self._state.get("controlInputMode", "tx12"))
        turn, thrust, yaw = _select_manual_input(
            input_mode,
            self._keyboard_turn,
            self._keyboard_thrust,
            self._keyboard_yaw,
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
            navigation_status = "Stopped; release keyboard controls to resume"
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

        if input_mode == "keyboard" and self._dashboard_tab != "motors":
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
        self._last_imu_accel = (float(accel_x_g), float(accel_y_g), float(accel_z_g))
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
            "navigationRollDeg": round(self._navigation.roll_deg - self._level_roll_offset, 1),
            "navigationPitchDeg": round(self._navigation.pitch_deg - self._level_pitch_offset, 1),
            "navigationEstimatedVelocityXMps": round(self._navigation.estimated_velocity_x_mps, 3),
            "navigationEstimatedVelocityYMps": round(self._navigation.estimated_velocity_y_mps, 3),
            "navigationEstimatedSpeedMps": round(self._navigation.estimated_speed_mps, 3),
            "navigationTiltCompensating": self._navigation.tilt_compensating,
            "navigationTiltRateDps": round(self._navigation.tilt_rate_dps, 1),
        }
        if gyro_z_dps is not None:
            updates["navigationMeasuredYawDps"] = round(active_gyro_z, 1)
        self._set_state(**updates)
        self._refresh_tilt_heading()
        self._refresh_speed_source()

    def _update_gps_fix(self, latitude: float, longitude: float) -> None:
        self._gps_speed.update(latitude, longitude, time.time())
        self._gps_track.append((float(latitude), float(longitude)))
        self._gps_track = self._gps_track[-30:]
        track = [{"latitude": lat, "longitude": lon} for lat, lon in self._gps_track]
        self._set_state(navigationGpsTrack=track)
        self._refresh_speed_source()

    def _refresh_speed_source(self) -> None:
        now = time.time()
        gps_speed = self._gps_speed.speed_mps(now)
        gps_course = self._gps_speed.course_deg(now)
        heading = self._state.get("magnetometerTiltHeadingDeg")
        if heading is None:
            heading = self._state.get("magnetometerHeadingDeg")
        gps_updates: dict[str, object] = {
            "navigationGpsCourseAvailable": gps_course is not None,
        }
        if gps_course is not None:
            gps_updates["navigationGpsCourseDeg"] = round(gps_course, 1)
            if heading is not None:
                gps_updates["navigationHeadingCourseOffsetDeg"] = round((gps_course - float(heading) + 180.0) % 360.0 - 180.0, 1)
        self._set_state(**gps_updates)
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
        if response_text.startswith("TEL MIX INPUT "):
            fields = response_text.split()
            if len(fields) == 7:
                try:
                    self._last_mixer_input_at = time.time()
                    self._set_state(buoyMixerInput={
                        "lateral": int(fields[3]), "thrust": int(fields[4]),
                        "yaw": int(fields[5]), "tx12": fields[6] == "1",
                    })
                except ValueError:
                    self._set_state(buoyMixerInput=None)
                    self._last_mixer_input_at = None
            return
        if response_text.startswith("TEL MIX POINT "):
            fields = response_text.split()
            if len(fields) == 8:
                try:
                    power, axis = int(fields[3]), int(fields[4])
                    values = [int(fields[index]) for index in (5, 6, 7)]
                    if power in range(10, 101, 10) and axis in range(6) and all(-100 <= value <= 100 for value in values):
                        rows = list(self._state.get("mixerCalibrationRows", self._empty_mixer_calibration_rows()))
                        row_index = axis * 10 + power // 10 - 1
                        row = dict(rows[row_index])
                        row.update(rear=values[0], front_left=values[1], front_right=values[2], set=True)
                        rows[row_index] = row
                        self._set_state(mixerCalibrationRows=rows)
                except (IndexError, TypeError, ValueError):
                    pass
            return
        if response_text.startswith("TEL MIX PROFILE "):
            fields = response_text.split()
            if len(fields) >= 7:
                try:
                    count, complete, loaded = int(fields[3]), fields[4] == "1", fields[5] == "1"
                    self._set_state(
                        mixerCalibrationStoredCount=count,
                        mixerCalibrationProfileComplete=complete,
                        mixerCalibrationProfileLoaded=loaded,
                        mixerCalibrationProfileActive=loaded and complete,
                        mixerCalibrationCrc=fields[6],
                        mixerCalibrationStatus=(
                            f"Buoy SD profile read back: {count}/60 points, {'active' if loaded and complete else 'not active until complete'}"
                        ),
                    )
                except (TypeError, ValueError):
                    self._set_state(mixerCalibrationStatus="Buoy returned an invalid mixer profile summary")
            return
        if response_text.startswith("ACK CTRL MIX SAVE "):
            fields = response_text.split()
            if len(fields) >= 7:
                self._set_state(
                    mixerCalibrationStoredCount=int(fields[4]),
                    mixerCalibrationProfileComplete=fields[5] == "1",
                    mixerCalibrationProfileLoaded=True,
                    mixerCalibrationProfileActive=fields[5] == "1",
                    mixerCalibrationCrc=fields[6],
                    mixerCalibrationStatus=f"Saved and verified on buoy SD: {fields[4]}/60 points; {'profile active' if fields[5] == '1' else 'incomplete profile saved but safe default mix remains active'}",
                )
            return
        if response_text.startswith("TEL MOTOR OUT "):
            output = _parse_motor_output_telemetry(response_text)
            if output is not None:
                self._last_motor_output_at = time.time()
                rear_drive, rear_pwm, left_drive, left_pwm, right_drive, right_pwm = output
                if self._active_motor_test is not None:
                    selected_motor = self._active_motor_test[0]
                    selected_drive = {
                        "rear": rear_drive,
                        "front_left": left_drive,
                        "front_right": right_drive,
                    }.get(selected_motor, 0)
                    if self._last_motor_test_actual_drive != 0 and selected_drive == 0:
                        age_ms = int((time.monotonic() - self._last_motor_test_send_at) * 1000)
                        stamp = datetime.now(timezone.utc).isoformat(timespec="milliseconds")
                        self._command_trace_pending.append(
                            f"{stamp} MOTOR OUTPUT DROPPED {selected_motor} last_keepalive_age_ms={age_ms}\n"
                        )
                    self._last_motor_test_actual_drive = selected_drive
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
                now = time.monotonic()
                if now - self._last_motor_output_log_at >= 1.0:
                    self._append_comm_log(f"RX {response_text}")
                    self._last_motor_output_log_at = now
            return
        self._append_comm_log(f"RX {response_text}")
        if response_text.startswith("TEL EVENT TELEMETRY_MODE "):
            self._set_state(telemetryMode=response_text.rsplit(" ", 1)[-1])
        if response_text.startswith("TEL EVENT MISSION_START BLOCKED "):
            fields = response_text.split()
            try:
                block_code = int(fields[5])
            except (IndexError, ValueError):
                block_code = -1
            reason = MISSION_START_BLOCK_REASONS.get(block_code, "Mission start interlock is active")
            self._set_state(missionStartBlockReason=reason,
                            radioMissionStatus="Mission start blocked: " + reason)
        elif response_text.startswith("TEL EVENT MISSION_START ") or response_text == "TEL EVENT MISSION_MODE MANUAL":
            self._set_state(missionStartBlockReason="")

        ack = parse_acknowledgement(response_text)
        if ack is not None:
            ack_kind, ack_values = ack
            if ack_kind == "MISSION" and len(ack_values) >= 2 and ack_values[0] == "COMMIT":
                self._mission_upload_timeout_timer.stop()
                self._mission_upload_active = False
                self._mission_upload_queue.clear()
                self._mission_upload_waiting_for_readback = True
                try:
                    point_count = int(ack_values[1])
                except ValueError:
                    point_count = 0
                self._set_state(missionUploadPending=True, missionUploadSucceeded=False, buoyMissionPointCount=point_count,
                                missionMessage=f"Buoy validated and loaded {point_count} mission points from its SD card.")
                self._mission_upload_timeout_timer.start(20000)
                self.refreshBuoyMissionList()
            elif ack_kind == "MISSION" and len(ack_values) >= 2 and ack_values[0] == "FILE" and ack_values[1] == "BEGIN":
                self._mission_upload_timeout_timer.stop()
                self._send_next_mission_upload_command()
            elif ack_kind == "MISSION" and len(ack_values) >= 3 and ack_values[0] == "FILE" and ack_values[1] == "DATA":
                self._mission_upload_timeout_timer.stop()
                self._send_next_mission_upload_command()
            elif ack_kind == "MISSION" and len(ack_values) >= 2 and ack_values[0] == "POINT":
                self._mission_upload_timeout_timer.stop()
                self._send_next_mission_upload_command()
            elif ack_kind == "MISSION" and ack_values and ack_values[0] in ("HOME", "END", "BOUNDARY"):
                self._mission_upload_timeout_timer.stop()
                self._send_next_mission_upload_command()
            elif ack_kind == "MISSION" and len(ack_values) >= 2 and ack_values[0] == "READ":
                point_count = ack_values[2] if len(ack_values) > 2 else "?"
                try:
                    expected_point_count = int(point_count)
                except (TypeError, ValueError):
                    expected_point_count = 0
                self._set_state(buoyMissionPoints=[], buoyMissionPointsId=ack_values[1].lower(),
                                buoyMissionExpectedPointCount=expected_point_count,
                                missionMessage=f"Reading {point_count} points from mission {ack_values[1]} on the buoy SD card…")
            elif ack_kind == "MISSION" and len(ack_values) >= 1 and ack_values[0] == "LIST":
                self._set_state(buoyMissionList=[], buoyMissionScanComplete=False,
                                buoyMissionInventoryStatus="Scanning SD card…",
                                missionMessage=f"Reading {ack_values[1] if len(ack_values) > 1 else 0} missions from the buoy SD card…")
            elif ack_kind == "MISSION" and len(ack_values) >= 2 and ack_values[0] == "DELETE":
                self._set_state(missionMessage="Mission deleted from the buoy SD card.")
                self.refreshBuoyMissionList()
            elif ack_kind == "CTRL" and len(ack_values) >= 3 and ack_values[0] == "VECTOR":
                self._set_state(ackVector=f"{ack_values[1]},{ack_values[2]}")
            elif ack_kind == "CTRL" and len(ack_values) >= 4 and ack_values[0] == "MOTION":
                self._set_state(ackVector=f"{ack_values[1]},{ack_values[2]}, yaw {ack_values[3]}")
            elif ack_kind == "CTRL" and len(ack_values) >= 4 and ack_values[:2] == ("MOTOR", "CAL"):
                motor_name = ack_values[2].strip().lower().replace("-", "_")
                direction = ack_values[3].strip().lower()
                acknowledged = self._pending_motor_calibration.get(motor_name)
                if direction in {"forward", "reverse"} and acknowledged is not None:
                    acknowledged.add(direction)
                    if acknowledged == {"forward", "reverse"}:
                        self._pending_motor_calibration.pop(motor_name, None)
                        self._set_state(
                            motorCalibrationStatus=f"Applied {motor_name} settings in buoy RAM (both directions confirmed)",
                            motorCalibrationAckMotor=motor_name,
                            motorCalibrationAckId=int(self._state.get("motorCalibrationAckId", 0)) + 1,
                        )
            elif ack_kind == "CTRL" and len(ack_values) >= 2 and ack_values[0] == "HOLD":
                self._set_state(holdPosition=(ack_values[1] == "ON"))
            elif ack_kind == "CONFIG" and len(ack_values) >= 2 and ack_values[0] == "MAGCAL" and ack_values[1] == "STORED":
                self._set_state(magnetometerBuoyConfigStatus="Calibration saved to buoy SD card")

        if response_text.startswith("TEL MISSION PROGRESS "):
            fields = response_text.split()
            if len(fields) >= 12:
                try:
                    progress = {
                        "mission_id": fields[3].lower(), "status": fields[4],
                        "current_waypoint_index": int(fields[5]), "waypoint_count": int(fields[6]),
                        "completed_waypoints": int(fields[7]),
                        "distance_to_waypoint_m": float(fields[8]), "distance_from_home_m": float(fields[9]),
                        "percent_complete": int(fields[10]), "reason": fields[11],
                    }
                    running = progress["status"] not in ("COMPLETE", "RETURNED_HOME", "STOPPED")
                    self._set_state(missionProgress=progress,
                                    buoyMissionRunningId=(progress["mission_id"] if running else "NONE"))
                except (TypeError, ValueError):
                    self._set_state(missionMessage="Buoy returned invalid mission progress")

        if response_text.startswith("TEL MISSION POINT "):
            fields = response_text.split()
            if len(fields) in (6, 9, 10):
                try:
                    point = {"index": int(fields[3]), "lat": float(fields[4]), "lon": float(fields[5])}
                    if len(fields) == 9:
                        mask = int(fields[6])
                        point.update(measurement_mask=mask, hold_seconds=int(fields[7]), speed_percent=int(fields[8]),
                                     measurements=[name for bit, name in ((1, "water_temperature"), (2, "depth"),
                                         (4, "conductivity"), (8, "turbidity"), (16, "air_temperature")) if mask & bit])
                    elif len(fields) == 10:
                        mask = int(fields[6])
                        point.update(measurement_mask=mask, collect_data=bool(int(fields[7])),
                                     hold_seconds=int(fields[8]), speed_percent=int(fields[9]),
                                     measurements=[name for bit, name in ((1, "water_temperature"), (2, "depth"),
                                         (4, "conductivity"), (8, "turbidity"), (16, "air_temperature")) if mask & bit])
                    points = list(self._state.get("buoyMissionPoints", []))
                    points = [item for item in points if int(item.get("index", -1)) != point["index"]]
                    points.append(point)
                    points.sort(key=lambda item: int(item["index"]))
                    self._set_state(buoyMissionPoints=points,
                                    missionMessage=f"Read {len(points)} point(s) from the buoy")
                    expected_count = int(self._state.get("buoyMissionExpectedPointCount", 0))
                    if (self._mission_upload_waiting_for_readback and expected_count > 0
                            and len(points) >= expected_count):
                        self._mission_upload_timeout_timer.stop()
                        self._mission_upload_waiting_for_readback = False
                        self._set_state(missionUploadPending=False, missionUploadSucceeded=True,
                                        missionMessage=f"Mission verified from the buoy SD card: {len(points)} points read back.")
                except (TypeError, ValueError):
                    self._set_state(missionMessage="Buoy returned an invalid mission point")

        if response_text.startswith("TEL MISSION HOME ") or response_text.startswith("TEL MISSION END "):
            fields = response_text.split()
            if len(fields) == 5:
                try:
                    coordinate = {"lat": float(fields[3]), "lon": float(fields[4])}
                    self._set_state(**({"buoyMissionHome": coordinate} if fields[2] == "HOME" else {"buoyMissionEnd": coordinate}))
                except ValueError:
                    self._set_state(missionMessage="Buoy returned an invalid mission location")

        if response_text.startswith("TEL MISSION BOUNDARY "):
            fields = response_text.split()
            if len(fields) == 6:
                try:
                    point = {"index": int(fields[3]), "lat": float(fields[4]), "lon": float(fields[5])}
                    points = list(self._state.get("buoyMissionBoundary", []))
                    points = [item for item in points if int(item.get("index", -1)) != point["index"]]
                    points.append(point)
                    points.sort(key=lambda item: int(item["index"]))
                    self._set_state(buoyMissionBoundary=points)
                except (TypeError, ValueError):
                    self._set_state(missionMessage="Buoy returned an invalid mission boundary point")

        if response_text.startswith("TEL MISSION ITEM "):
            fields = response_text.split()
            if len(fields) == 5:
                mission_id = fields[3].lower()
                try:
                    point_count = int(fields[4])
                except ValueError:
                    point_count = -1
                if point_count >= 0:
                    known = next((item for item in self._state.get("missionList", [])
                                  if str(item.get("id", "")).lower() == mission_id), None)
                    item = {"id": mission_id, "name": str(known.get("name", mission_id)) if known else mission_id,
                            "point_count": point_count}
                    missions = list(self._state.get("buoyMissionList", []))
                    missions = [entry for entry in missions if str(entry.get("id", "")).lower() != mission_id]
                    missions.append(item)
                    missions.sort(key=lambda entry: str(entry.get("name", "")).lower())
                    self._set_state(buoyMissionList=missions,
                                    missionMessage=f"Found {len(missions)} mission(s) on the buoy SD card")

        if response_text.startswith("TEL MISSION ACTIVE "):
            fields = response_text.split()
            try:
                running_index = fields.index("ACTIVE") + 1
                loaded_index = fields.index("LOADED") + 1
                loaded_id = fields[loaded_index]
                missions = self._state.get("buoyMissionList", [])
                if loaded_id.upper() != "NONE":
                    missions = [item for item in missions
                                if str(item.get("id", "")).lower() == loaded_id.lower()]
                elif len(missions) > 1:
                    missions = []
                self._set_state(buoyMissionRunningId=fields[running_index],
                                buoyMissionLoadedId=fields[loaded_index],
                                buoyMissionList=missions,
                                buoyMissionScanComplete=True,
                                buoyMissionInventoryStatus=(
                                    ("There is a mission on the SD card:" if len(missions) == 1
                                     else "There are missions on the SD card:") if missions
                                    else "No mission on SD card."
                                ))
                if loaded_id.upper() == "NONE":
                    self._set_state(buoyMissionPoints=[], buoyMissionPointsId="NONE",
                                    buoyMissionReadId="")
                elif loaded_id.lower() != str(self._state.get("buoyMissionReadId", "")).lower():
                    self.readBuoyMission(loaded_id)
            except (ValueError, IndexError):
                self._set_state(missionMessage="Buoy returned invalid mission state")

        status_update = parse_status_update(response_text)
        if status_update is not None:
            status_key, status_values = status_update
            if str(self._state.get("serialTarget", "")).startswith("WIFI:"):
                if status_key in {"ELRS_ARM", "RECOVERY"}:
                    self._wifi_safety_last_at = time.monotonic()
                    self._set_state(wifiSafetyTelemetryLive=True)
                if self._state.get("telemetryMode") == "WIFI" and status_key in {"POS", "BATTERY"}:
                    self._wifi_detail_last_at = time.monotonic()
                    self._set_state(wifiDetailTelemetryLive=True)
            if status_key == "TELEMETRY_MODE" and status_values:
                self._set_state(telemetryMode=status_values[0])
            elif status_key == "MISSION" and status_values:
                try:
                    count = int(status_values[0])
                except ValueError:
                    count = 0
                updates = {"buoyMissionPointCount": count}
                if "LOADED" in status_values:
                    updates["buoyMissionLoadedId"] = status_values[status_values.index("LOADED") + 1]
                if "RUNNING" in status_values:
                    updates["buoyMissionRunningId"] = status_values[status_values.index("RUNNING") + 1]
                self._set_state(**updates)
            elif status_key == "SD" and status_values:
                available = status_values[0] == "READY"
                self._set_state(sdCardStatus=("SD card ready" if available else "SD card unavailable"))
            elif status_key == "ELRS_CHANNELS" and self._state.get("telemetryMode") == "WIFI":
                try:
                    channels = [int(value) for value in status_values[:16]]
                except ValueError:
                    channels = []
                if channels:
                    self._radio_field_last_at["channels"] = time.monotonic()
                    self._set_state(radioChannels=[value if 900 <= value <= 2100 else None for value in channels] + [None] * max(0, 16 - len(channels)))
            elif status_key == "ELRS_LINK" and self._state.get("telemetryMode") == "WIFI":
                values = list(status_values)
                try:
                    link_quality = int(values[values.index("LQ") + 1])
                    age_text = values[values.index("AGE_MS") + 1]
                    rssi = int(values[values.index("RSSI_DBM") + 1])
                    snr = int(values[values.index("SNR_DB") + 1])
                    fresh = age_text != "UNKNOWN" and int(age_text) <= 2000
                except (ValueError, IndexError):
                    fresh = False
                if fresh:
                    self._radio_link_last_at = time.monotonic()
                    self._set_state(
                        radioLinkStatus="ELRS receiver status via buoy Wi-Fi",
                        radioLinkQuality=f"{link_quality}% (RX uplink)",
                        radioRssiStatus=f"{rssi} dBm (RX uplink)" if rssi < 0 else "Unknown",
                        radioSnrStatus=f"{snr} dB (RX uplink)",
                    )
            elif status_key == "MODE" and status_values:
                self._set_state(buoyMode=status_values[0].lower())
            elif status_key == "MISSION_CONTROL" and status_values:
                mission_control = status_values[0]
                mission_status_label = mission_control.replace("_", " ").title()
                if mission_control == "MANUAL":
                    self._set_state(radioMissionStatus="Manual · Direct")
                elif mission_control == "MANUAL_ASSIST":
                    self._set_state(radioMissionStatus="Manual Assist · Heading hold")
                elif mission_control == "MANUAL_ASSIST_NO_HEADING":
                    self._set_state(radioMissionStatus="Manual Assist · Heading unavailable")
                else:
                    self._set_state(radioMissionStatus="Mission mode · " + mission_status_label)
            elif status_key == "CONTROL_SOURCE" and status_values:
                self._set_state(controlSource=status_values[0])
            elif status_key == "ELRS_ARM" and status_values:
                self._set_state(radioArmStatus=status_values[0])
            elif status_key == "MANUAL_HOME" and status_values:
                if status_values[0] == "SET":
                    self._set_state(manualHomeStatus=("Set · " + " ".join(status_values[1:3]) if len(status_values) >= 3 else "Set"))
                elif status_values[0] == "RETURNING":
                    self._set_state(manualHomeStatus=("Returning · " + status_values[1] if len(status_values) > 1 else "Returning"))
                else:
                    self._set_state(manualHomeStatus="Not available")
            elif status_key == "MANUAL_HOME_STORAGE" and status_values:
                storage_status = " ".join(status_values)
                if storage_status == "NVS_READY":
                    storage_status = "Persistent storage ready"
                elif storage_status == "LOADED NVS":
                    storage_status = "Home loaded from persistent storage"
                elif storage_status == "SAVED NVS":
                    storage_status = "Saved across power cycles"
                elif storage_status == "SAVE FAILED":
                    storage_status = "Save failed · recapture Home"
                elif storage_status == "NVS_UNAVAILABLE":
                    storage_status = "Persistent storage unavailable"
                self._set_state(manualHomeStorageStatus=storage_status)
            elif status_key == "ACTIVE_HOME" and status_values:
                source = status_values[0]
                if source in ("MANUAL", "MISSION") and len(status_values) >= 3:
                    try:
                        coordinates = f"{float(status_values[1]):.7f}, {float(status_values[2]):.7f}"
                    except ValueError:
                        coordinates = "Unknown"
                    self._set_state(activeHomeStatus=("Manual Home" if source == "MANUAL" else "Mission Home"),
                                    activeHomeCoordinates=coordinates)
                else:
                    self._set_state(activeHomeStatus="No active Home", activeHomeCoordinates="Unknown", homeDistanceMeters=None)
            elif status_key == "HOME_DISTANCE" and status_values:
                try:
                    self._set_state(homeDistanceMeters=round(float(status_values[0]), 1))
                except ValueError:
                    self._set_state(homeDistanceMeters=None)
            elif status_key == "RECOVERY" and status_values:
                self._set_state(recoveryStatus=status_values[0])
            elif status_key == "FIRMWARE" and status_values:
                self._set_state(firmwareVersion=format_firmware_version(status_values))
            elif status_key == "MAGNETOMETER" and status_values:
                self._set_state(magnetometerStatus=" ".join(status_values))
            elif status_key == "MAGCAL" and status_values:
                loaded = status_values[0] == "LOADED"
                self._set_state(magnetometerBuoyConfigStatus=("Calibration loaded from buoy SD" if loaded else "No calibration file on buoy SD"))
            elif status_key == "SCIENCE" and status_values:
                self._set_state(scienceExperimentState=status_values[0],
                                scienceSessionPath=status_values[1] if len(status_values) > 1 and status_values[1] != "NONE" else "")
            elif status_key == "SERVICE" and status_values:
                self._set_state(serviceMode=status_values[0] == "ON")
            elif status_key == "STREAM" and len(status_values) >= 2:
                if status_values[0] == "AUDIO":
                    self._set_state(audioStreamEnabled=status_values[1] == "ON")
                elif status_values[0] == "IMU":
                    self._set_state(imuStreamEnabled=status_values[1] == "ON")
            elif status_key == "AUDIO" and len(status_values) >= 5:
                values = list(status_values)
                sample_rate = int(values[values.index("RATE") + 1]) if "RATE" in values else 16000
                quality_locked = "QUALITY_LOCKED" in values and values[values.index("QUALITY_LOCKED") + 1] == "ON"
                previous_rate = self._audio_sample_rate
                self._audio_sample_rate = sample_rate
                if previous_rate != sample_rate:
                    device_name = self._audio_output_device
                    selected_device = None if device_name in (None, "", "System default") else device_name
                    if not self._initialize_audio_output(selected_device) and selected_device is not None:
                        self._initialize_audio_output(None)
                self._audio_low_pass_hz = min(self._audio_low_pass_hz, sample_rate * 0.45)
                quality_name = {8000: "LOW", 16000: "DEFAULT", 48000: "MAX"}.get(sample_rate, "CUSTOM")
                self._set_state(
                    audioCaptureChannel=values[0],
                    audioRecordingEnabled=(values[2] == "ON"),
                    audioSdStatus=values[4],
                    audioSampleRateHz=sample_rate,
                    audioQuality=quality_name,
                    audioQualityLocked=quality_locked,
                    audioLowPassHz=self._audio_low_pass_hz,
                    audioOutputStatus=self._audio_output_status,
                )
            elif status_key == "POS":
                if self._state.get("telemetryMode") == "WIFI":
                    self._set_state(currentLocation=_format_location(status_values), telemetrySource="Buoy Wi-Fi")
                if len(status_values) >= 2 and "UNKNOWN" not in {value.upper() for value in status_values[:2]}:
                    try:
                        self._update_gps_fix(float(status_values[0]), float(status_values[1]))
                    except ValueError:
                        self._gps_speed.invalidate()
                else:
                    self._gps_speed.invalidate()
                    self._refresh_speed_source()
            elif status_key == "GPS":
                satellites = None
                hdop = None
                try:
                    if status_values and status_values[0] != "UNKNOWN":
                        satellites = int(status_values[0])
                    if len(status_values) > 1 and status_values[1] != "UNKNOWN":
                        hdop = float(status_values[1])
                except ValueError:
                    satellites = None
                    hdop = None
                fix_valid = len(status_values) > 2 and status_values[2] == "FIX"
                self._set_state(gpsSatelliteCount=satellites, gpsHdop=hdop, gpsFixValid=fix_valid)
                self._refresh_gps_quality()
            elif status_key == "TARGET":
                self._set_state(targetLocation=_format_location(status_values))
            elif status_key == "HOLD" and status_values:
                self._set_state(holdPosition=(status_values[0] == "ON"))
            elif status_key == "BATTERY":
                if self._state.get("telemetryMode") == "WIFI":
                    self._set_state(batteryStatus=_parse_battery_status(" ".join(status_values)), telemetrySource="Buoy Wi-Fi")
                if len(status_values) >= 2 and (self._last_power_at is None or time.time() - self._last_power_at > 2.0):
                    try:
                        self._record_power_sample(float(status_values[0]), float(status_values[1]))
                    except ValueError:
                        pass
            elif status_key == "CURRENT":
                if self._state.get("telemetryMode") == "WIFI":
                    self._set_state(currentDraw=_parse_battery_status(" ".join(status_values)), telemetrySource="Buoy Wi-Fi")

        science_update = parse_science_update(response_text)
        if science_update is not None:
            science_key, science_values = science_update
            if science_key == "WATER_TEMP":
                self._set_state(waterTemperature=_parse_science_value(science_values, "C"))
            elif science_key == "TDS_V":
                self._set_state(conductivityVoltage=_parse_science_value(science_values, "V"))
            elif science_key == "TURBIDITY_V":
                self._set_state(turbidityVoltage=_parse_science_value(science_values, "V"))
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
            elif science_key == "MAG_RAW":
                if len(science_values) >= 3 and all(value.upper() != "UNKNOWN" for value in science_values[:3]):
                    try:
                        x, y, z = (int(value) for value in science_values[:3])
                    except ValueError:
                        self._set_state(magnetometerRaw="N/A", magnetometerHeadingDeg=None)
                    else:
                        self._last_mag_raw = (float(x), float(y), float(z))
                        self._last_mag_raw_at = time.time()
                        if self._mag_calibration_active:
                            if self._mag_calibration_min is None:
                                self._mag_calibration_min = [x, y, z]
                                self._mag_calibration_max = [x, y, z]
                            else:
                                self._mag_calibration_min = [min(a, b) for a, b in zip(self._mag_calibration_min, (x, y, z))]
                                self._mag_calibration_max = [max(a, b) for a, b in zip(self._mag_calibration_max or (x, y, z), (x, y, z))]
                            self._mag_calibration_samples += 1
                        if {"offset_x", "offset_y", "scale_x", "scale_y"} <= self._mag_calibration.keys():
                            calibrated_x = (x - self._mag_calibration["offset_x"]) * self._mag_calibration["scale_x"]
                            calibrated_y = (y - self._mag_calibration["offset_y"]) * self._mag_calibration["scale_y"]
                        else:
                            calibrated_x, calibrated_y = float(x), float(y)
                        heading_deg = (math.degrees(math.atan2(calibrated_y, calibrated_x)) + 360.0) % 360.0
                        updates = {
                            "magnetometerRaw": f"X {x}  |  Y {y}  |  Z {z}",
                            "magnetometerRawX": x,
                            "magnetometerRawY": y,
                            "magnetometerRawZ": z,
                            "magnetometerHeadingDeg": round(heading_deg, 1),
                            "magnetometerCalibrationSamples": self._mag_calibration_samples,
                        }
                        if self._mag_calibration_active:
                            updates["magnetometerCalibrationStatus"] = (
                                f"Rotate the buoy slowly through full turns · {self._mag_calibration_samples} samples"
                            )
                        self._set_state(**updates)
                        self._refresh_tilt_heading()
                        self._refresh_speed_source()
                else:
                    self._last_mag_raw = None
                    self._last_mag_raw_at = None
                    self._set_state(
                        magnetometerRaw="N/A",
                        magnetometerRawX=None,
                        magnetometerRawY=None,
                        magnetometerRawZ=None,
                        magnetometerHeadingDeg=None,
                    )
            elif science_key == "MAG_HEADING" and science_values:
                try:
                    self._set_state(magnetometerHeadingDeg=round(float(science_values[0]), 1),
                                    magnetometerBuoyConfigStatus="Buoy is applying SD calibration")
                    self._refresh_speed_source()
                except ValueError:
                    pass

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
                normalized_motor = motor_name.replace("-", "_")
                configurations = dict(self._state.get("motorConfigurations", {}))
                configurations[normalized_motor] = {
                    "direction": direction,
                    "start_boost_pwm": start_boost_pwm,
                    "start_boost_ms": start_boost_ms,
                    "sustain_min_pwm": sustain_min_pwm,
                    "max_pwm": max_pwm,
                    "curve": curve,
                    "ramp_seconds": ramp_seconds,
                    "default_reversed": default_reversed,
                }
                self._set_state(
                    motorConfigFetchId=int(self._state.get("motorConfigFetchId", 0)) + 1,
                    motorConfigMotor=motor_name,
                    motorConfigDirection=direction,
                    motorConfigurations=configurations,
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
            updates = {"lastSendResult": f"ERR {error_text}"}
            if "MIX" in error_text.upper():
                updates["mixerCalibrationStatus"] = f"Buoy rejected mixer profile: {error_text}"
            if "MOTOR CAL" in error_text.upper():
                updates["motorCalibrationStatus"] = f"Buoy rejected motor settings: {error_text}"
            if error_text.startswith("CONFIG "):
                updates["magnetometerBuoyConfigStatus"] = f"Buoy rejected calibration config: {error_text}"
            if error_text.startswith("MISSION ") and self._mission_upload_active:
                self._mission_upload_active = False
                self._mission_upload_queue.clear()
                self._mission_upload_timeout_timer.stop()
                updates["missionUploadPending"] = False
                updates["missionUploadSucceeded"] = False
            if error_text.startswith("MISSION ") and self._mission_upload_waiting_for_readback:
                self._mission_upload_waiting_for_readback = False
                self._mission_upload_timeout_timer.stop()
                updates["missionUploadPending"] = False
                updates["missionUploadSucceeded"] = False
            if error_text == "MISSION SD UNAVAILABLE":
                updates.update(sdCardStatus="SD card unavailable",
                               buoyMissionList=[], buoyMissionScanComplete=True,
                               buoyMissionInventoryStatus="No SD card.")
            if error_text.startswith("MISSION ") or self._state.get("missionUploadPending"):
                updates.update(missionUploadPending=False,
                               missionMessage=f"Buoy rejected mission request: {error_text}")
            self._set_state(**updates)

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

        requested_mixer = {
            "lateral": int(self._state.get("navigationRequestedTurn", 0)),
            "thrust": int(self._state.get("navigationRequestedThrust", 0)),
            "yaw": int(self._state.get("navigationRequestedYaw", 0)),
        }
        buoy_mixer_input = self._state.get("buoyMixerInput")
        if isinstance(buoy_mixer_input, dict) and self._last_mixer_input_at is not None and captured_at - self._last_mixer_input_at <= 1.0:
            requested_mixer = {key: int(buoy_mixer_input.get(key, 0)) for key in ("lateral", "thrust", "yaw")}
        elif str(self._state.get("controlInputMode", "tx12")) == "tx12":
            channels = self._state.get("radioChannels", [])
            if isinstance(channels, list) and len(channels) >= 4:
                try:
                    def channel_axis(index: int) -> float:
                        raw = int(channels[index])
                        offset = max(-500, min(500, raw - 1500))
                        return 0.0 if abs(offset) <= 40 else max(-100.0, min(100.0, offset / 5.0))
                    power = max(0.0, min(100.0, (int(channels[2]) - 1000) / 10.0))
                    requested_mixer = {
                        "lateral": int(round(channel_axis(0) * power / 100.0)),
                        "thrust": int(round(channel_axis(1) * power / 100.0)),
                        "yaw": int(round(channel_axis(3) * power / 100.0)),
                    }
                except (TypeError, ValueError):
                    pass
        gps_location = _parse_location_pair(str(self._state.get("currentLocation", "")))

        sample = {
            "environment": str(self._state.get("navigationTestEnvironment", "")),
            "compass_load_capture_phase": self._compass_load_capture_phase or None,
            "compass_load_capture_setup": dict(self._compass_load_capture_setup),
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
                    **requested_mixer,
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
                "magnetometer_raw": {
                    "x": self._state.get("magnetometerRawX"),
                    "y": self._state.get("magnetometerRawY"),
                    "z": self._state.get("magnetometerRawZ"),
                    "heading_deg": self._state.get("magnetometerHeadingDeg"),
                    "telemetry_age_ms": age_ms(self._last_mag_raw_at),
                },
                "gps": {
                    "latitude": gps_location[0] if gps_location else None,
                    "longitude": gps_location[1] if gps_location else None,
                    "speed_mps": self._state.get("navigationGpsSpeedMps") if self._state.get("navigationGpsSpeedAvailable") else None,
                    "course_deg": self._state.get("navigationGpsCourseDeg") if self._state.get("navigationGpsCourseAvailable") else None,
                },
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
            "propulsion_load": {
                "battery_voltage_v": self._latest_power.get("battery_voltage_v"),
                "total_current_a": self._latest_power.get("current_a"),
                "power_telemetry_age_ms": age_ms(self._last_power_at),
            },
        }
        if self._compass_load_capture_phase:
            captured_fields = {
                "mag_x": self._state.get("magnetometerRawX"),
                "mag_y": self._state.get("magnetometerRawY"),
                "mag_z": self._state.get("magnetometerRawZ"),
                "heading_deg": self._state.get("magnetometerHeadingDeg"),
                "current_a": self._latest_power.get("current_a"),
                "gyro_x_dps": gx_dps,
                "gyro_y_dps": gy_dps,
                "gyro_z_dps": gz_dps,
            }
            for key, value in captured_fields.items():
                if value is not None and (not key.startswith("mag_") and key != "heading_deg" or age_ms(self._last_mag_raw_at) is not None and age_ms(self._last_mag_raw_at) <= 1000.0):
                    self._compass_load_capture_values.setdefault(key, []).append(float(value))
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
        summary: dict[str, object] = {
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
        if self._compass_load_capture_phase:
            fields = self._compass_load_capture_values
            means = {
                key: round(sum(values) / len(values), 4) if values else None
                for key, values in fields.items()
            }
            summary["compass_motor_load"] = {
                "phase": self._compass_load_capture_phase,
                "setup": dict(self._compass_load_capture_setup),
                "valid_mag_samples": min(len(fields.get("mag_x", [])), len(fields.get("mag_y", [])), len(fields.get("mag_z", []))),
                "means": means,
                "mean_current_a": means["current_a"],
            }
        return summary

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
                if self._state.get("telemetryMode") == "MAVLINK":
                    continue
                battery_volts, current_amps, battery_pct = packet.values
                self._last_power_at = time.time()
                self._record_power_sample(battery_volts, current_amps)
                self._latest_power = {
                    "battery_voltage_v": battery_volts,
                    "current_a": current_amps,
                    "battery_percent": battery_pct,
                }
                self._set_state(
                    batteryStatus=_format_udp_battery_status(battery_volts, current_amps, battery_pct),
                    currentDraw=f"{current_amps:.3f} A",
                    telemetrySource="Buoy Wi-Fi",
                )
            elif packet.packet_type == "STATE":
                if self._state.get("telemetryMode") == "MAVLINK":
                    continue
                mode_value, hold_enabled, gps_valid, _motors_enabled = packet.values
                current_location = str(self._state.get("currentLocation", "Unknown")) if gps_valid else "Unknown"
                if not gps_valid:
                    self._gps_speed.invalidate()
                    self._refresh_speed_source()
                self._set_state(
                    buoyMode=_decode_control_mode(mode_value),
                    holdPosition=bool(hold_enabled),
                    currentLocation=current_location,
                )
            elif packet.packet_type == "ELRS_CHANNELS":
                if self._state.get("telemetryMode") == "MAVLINK":
                    continue
                channels = [value if 900 <= value <= 2100 else None for value in packet.values]
                if any(value is not None for value in channels):
                    self._radio_field_last_at["channels"] = time.monotonic()
                else:
                    self._radio_field_last_at.pop("channels", None)
                self._set_state(radioChannels=channels)
            elif packet.packet_type == "GPS":
                if self._state.get("telemetryMode") == "MAVLINK":
                    continue
                latitude, longitude = packet.values
                self._set_state(currentLocation=_format_udp_position(latitude, longitude), telemetrySource="Buoy Wi-Fi")
                self._update_gps_fix(latitude, longitude)
            elif packet.packet_type == "RANGE":
                distance_mm, valid = packet.values
                self._set_state(currentDepth=(f"{distance_mm / 1000.0:.3f} m" if valid else "N/A"))
            elif packet.packet_type == "AUDIO":
                pcm_bytes, _sample_count, channels, packet_version = packet.values
                samples = _decode_pcm_samples(pcm_bytes)
                filtered_samples = self._filter_audio_samples(samples, channels)
                self._audio_channels = channels
                if self._audio_sample_rate <= 0:
                    self._audio_sample_rate = 16000
                if packet_version >= 2:
                    if self._audio_packet_version < 2:
                        self._audio_last_sequence = None
                        self._audio_packets_received = 0
                        self._audio_packets_lost = 0
                        self._audio_packets_out_of_order = 0
                        self._audio_playback_drops = 0
                    self._audio_packets_received += 1
                    sequence = packet.sequence & 0xFFFF
                    if self._audio_last_sequence is not None:
                        delta = (sequence - self._audio_last_sequence) & 0xFFFF
                        if delta == 0 or delta >= 0x8000:
                            self._audio_packets_out_of_order += 1
                        elif delta > 1:
                            self._audio_packets_lost += delta - 1
                            self._audio_last_sequence = sequence
                        else:
                            self._audio_last_sequence = sequence
                    else:
                        self._audio_last_sequence = sequence
                self._audio_packet_version = packet_version
                # Audio packets arrive much faster than the UI can usefully redraw.
                # Keep the real-time filter and playback path per packet, but do
                # waveform analysis, list copies, and QML notifications at 10 Hz.
                now = time.monotonic()
                if now - self._last_audio_ui_update >= 0.1:
                    self._last_audio_ui_update = now
                    peak = _update_audio_waveform_buffer(self._waveform_buffer, samples, channels)
                    left_rms, right_rms = _audio_channel_rms_percent(samples, channels)
                    self._audio_left_rms_percent = left_rms
                    self._audio_right_rms_percent = right_rms or 0.0
                    left_peak, right_peak = _update_channel_waveforms(
                        deque(maxlen=1), deque(maxlen=1), samples, channels
                    )
                    _update_channel_waveforms(
                        self._audio_left_buffer, self._audio_right_buffer, filtered_samples, channels
                    )
                    self._audio_waveform = list(self._waveform_buffer)
                    self._audio_left_waveform = list(self._audio_left_buffer)
                    self._audio_right_waveform = list(self._audio_right_buffer)
                    self.audioWaveformChanged.emit()
                    self._audio_left_level = f"{left_peak:.1f}% peak"
                    self._audio_right_level = f"{right_peak:.1f}% peak" if right_peak is not None else "Unavailable"
                    self._set_state(
                        audioStream=(
                            f"{self._audio_sample_rate / 1000.0:.1f} kHz | {channels} channel{'s' if channels != 1 else ''}"
                        ),
                        audioLevel=f"{peak * 100.0:.1f}%",
                        audioChannelCount=channels,
                        audioLeftLevel=self._audio_left_level,
                        audioRightLevel=self._audio_right_level,
                        audioLeftRmsPercent=self._audio_left_rms_percent,
                        audioRightRmsPercent=self._audio_right_rms_percent,
                        audioPacketLossStatus=self.audioPacketLossStatus,
                        audioPacketsReceived=self._audio_packets_received,
                        audioPacketsLost=self._audio_packets_lost,
                        audioPacketsOutOfOrder=self._audio_packets_out_of_order,
                        audioPlaybackDrops=self._audio_playback_drops,
                    )
                if not self._audio_muted and self._audio_channel is not None:
                    try:
                        monitor_samples = array("h")
                        gain = self._audio_debug_gain / 100.0
                        if channels == 1:
                            for sample in filtered_samples[::2]:
                                value = max(-32768, min(32767, int(sample * gain)))
                                monitor_samples.extend((value, value))
                        else:
                            for index in range(0, len(filtered_samples) - 1, 2):
                                left = int(filtered_samples[index])
                                right = int(filtered_samples[index + 1])
                                if self._audio_debug_channel == "left":
                                    right = left
                                elif self._audio_debug_channel == "right":
                                    left = right
                                left = max(-32768, min(32767, int(left * gain)))
                                right = max(-32768, min(32767, int(right * gain)))
                                monitor_samples.extend((left, right))
                        if sys.byteorder != "little":
                            monitor_samples.byteswap()
                        # ESP32 packets are only 5.3 ms at 48 kHz. Creating and
                        # scheduling one SDL Sound per datagram overwhelms the UI
                        # and mixer; combine ~40 ms of PCM into each playback item.
                        self._audio_playback_pending.extend(monitor_samples.tobytes())
                        bytes_per_second = int((self._audio_sample_rate or 16000) * 2 * 2)
                        batch_bytes = max(1024, bytes_per_second // 25)
                        if len(self._audio_playback_pending) >= batch_bytes:
                            aligned_length = len(self._audio_playback_pending) & ~3
                            pcm_batch = bytes(self._audio_playback_pending[:aligned_length])
                            del self._audio_playback_pending[:aligned_length]
                            sound = pygame.mixer.Sound(buffer=pcm_batch)
                            self._queue_audio_playback(sound)
                    except Exception as exc:
                        self._audio_playback_error = str(exc)
                        self._audio_output_status = f"Audio chunk error: {exc}"
                        self.stateChanged.emit()

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

    def _refresh_wifi_traffic(self) -> None:
        transport = self._transport
        target = str(self._state.get("serialTarget", ""))
        if not target.startswith("WIFI:") or transport is None or not transport.is_open:
            status = "Buoy control link connecting" if str(self._state.get("serialStatus", "")).startswith(("Connecting", "Reconnecting")) else "No active buoy control link"
            self._set_state(wifiTrafficStatus=status)
            self._wifi_traffic_transport = None
            self._wifi_traffic_sample = None
            return

        tcp_bytes = int(getattr(transport, "rx_tcp_bytes", 0))
        udp_bytes = int(getattr(transport, "rx_udp_bytes", 0))
        udp_messages = int(getattr(transport, "rx_udp_datagrams", 0))
        total_bytes = tcp_bytes + udp_bytes
        total_messages = self._wifi_tcp_messages_total + udp_messages
        now = time.monotonic()
        if transport is not self._wifi_traffic_transport:
            self._wifi_traffic_transport = transport
            self._wifi_traffic_sample = (total_bytes, total_messages, now)
            self._set_state(
                wifiReceivedMessages=total_messages,
                wifiReceivedBytes=total_bytes,
                wifiMessageRate=0.0,
                wifiThroughputKib=0.0,
                wifiTrafficStatus="Connected; measuring received traffic",
            )
            return

        sample = self._wifi_traffic_sample
        if sample is None:
            self._wifi_traffic_sample = (total_bytes, total_messages, now)
            return
        elapsed = now - sample[2]
        if elapsed < 1.0:
            return
        bytes_per_second = max(0, total_bytes - sample[0]) / elapsed
        messages_per_second = max(0, total_messages - sample[1]) / elapsed
        self._wifi_traffic_sample = (total_bytes, total_messages, now)
        self._set_state(
            wifiReceivedMessages=total_messages,
            wifiReceivedBytes=total_bytes,
            wifiMessageRate=round(messages_per_second, 1),
            wifiThroughputKib=round(bytes_per_second / 1024.0, 1),
            wifiTrafficStatus="Receiving buoy Wi-Fi data" if total_bytes > sample[0] else "Connected; no received data in the last second",
        )

    def _tick(self) -> None:
        tick_at = time.monotonic()
        wifi_connected = (self._transport is not None and self._transport.is_open
                          and str(self._state.get("serialTarget", "")).startswith("WIFI:"))
        self._set_state(
            wifiSafetyTelemetryLive=bool(wifi_connected and self._wifi_safety_last_at > 0
                                         and tick_at - self._wifi_safety_last_at <= 5.0),
            wifiDetailTelemetryLive=bool(wifi_connected and self._wifi_detail_last_at > 0
                                         and self._state.get("telemetryMode") == "WIFI"
                                         and tick_at - self._wifi_detail_last_at <= 5.0),
        )
        if self._last_tick_at and tick_at - self._last_tick_at > 0.25:
            stamp = datetime.now(timezone.utc).isoformat(timespec="milliseconds")
            gap_ms = int((tick_at - self._last_tick_at) * 1000)
            self._command_trace_pending.append(f"{stamp} CONTROL LOOP GAP {gap_ms} ms\n")
        self._last_tick_at = tick_at
        if time.monotonic() - self._last_command_trace_flush >= 1.0:
            self._flush_command_traces()
        self._refresh_speed_source()
        if (
            self._auto_reconnect_enabled
            and self._transport is None
            and self._pending_connection is None
            and not self._connection_in_flight
            and (time.time() - self._last_reconnect_attempt) >= self._reconnect_interval
        ):
            self._queue_transport_connection()

        self._drain_pending_connection()
        self._drain_radio_wifi_discovery()
        self._drain_network_scan()
        self._poll_radio_telemetry()
        self._drain_sd_card_operation()

        if self._transport is not None:
            if self._transport.is_open:
                self._send_motor_test_keepalive()
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
                        if str(self._state.get("serialTarget", "")).startswith("WIFI:"):
                            self._wifi_tcp_messages_total += 1
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

        self._refresh_wifi_traffic()

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
    QtWebEngineQuick.initialize()
    app = QGuiApplication(sys.argv)

    app_dir = (Path(sys.executable).resolve().parent if getattr(sys, "frozen", False)
               else Path(__file__).resolve().parent.parent)
    log_dir = app_dir / "logs"
    log_dir.mkdir(parents=True, exist_ok=True)
    mission_dir = app_dir / "missions"
    MissionStore(mission_dir).copy_legacy_plans(log_dir / "missions")
    mission_dir.mkdir(parents=True, exist_ok=True)
    system_log_dir = log_dir / "system"
    system_log_dir.mkdir(parents=True, exist_ok=True)
    timestamp = time.strftime("%Y%m%d-%H%M%S")
    log_file = Path(args.comm_log_file) if args.comm_log_file else (system_log_dir / f"commands-{timestamp}.log")
    prefs_path = log_dir / CONNECTION_PREFS_FILENAME
    apply_connection_preferences(args, prefs_path)
    with log_file.open("a", encoding="utf-8") as handle:
        handle.write(f"{datetime.now(timezone.utc).isoformat(timespec='seconds')} COMMAND TRACE START\n")

    engine = QQmlApplicationEngine()
    backend = ControlStationBackend(args, log_file, prefs_path, mission_dir)
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
        handle.write(f"{datetime.now(timezone.utc).isoformat(timespec='seconds')} COMMAND TRACE END\n")
    raise SystemExit(exit_code)
