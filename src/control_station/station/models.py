from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class ManualCommand:
    turn: int
    thrust: int
    rear_motor: int
    front_left_motor: int
    front_right_motor: int

    def to_line(self) -> str:
        return f"CTRL VECTOR {self.turn:+d} {self.thrust:+d}\n"


@dataclass(frozen=True)
class AnalogInput:
    name: str
    value: float


@dataclass(frozen=True)
class DigitalInput:
    name: str
    active: bool


@dataclass(frozen=True)
class ControllerSnapshot:
    analog_inputs: tuple[AnalogInput, ...]
    digital_inputs: tuple[DigitalInput, ...]


@dataclass(frozen=True)
class TelemetryState:
    control_mode: str
    current_location: str
    target_location: str
    hold_position: bool
    battery_status: str
    current_draw: str
    current_depth: str
    water_temperature: str
    air_temperature: str
    imu_accel: str = "N/A"
    imu_gyro: str = "N/A"
    imu_temperature: str = "N/A"
    imu_udp_loss: str = "N/A"
    audio_stream: str = "N/A"
    audio_level: str = "N/A"


@dataclass(frozen=True)
class ScienceSample:
    timestamp: float
    latitude: float
    longitude: float
    depth_m: float
    audio_level_percent: float
    water_temperature_c: float | None = None


@dataclass(frozen=True)
class RuntimeState:
    command: ManualCommand
    controller_status: str
    controller_name: str
    controller_snapshot: ControllerSnapshot
    serial_status: str
    serial_target: str
    last_send_result: str
    last_sent_line: str
    command_input: str
    last_response: str
    telemetry: TelemetryState
    ack_vector: str
    comm_log: list[str]
    audio_muted: bool = True
    audio_waveform: tuple[float, ...] = ()
