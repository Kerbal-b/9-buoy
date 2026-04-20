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
        return (
            f"CTRL {self.turn:+d} {self.thrust:+d} "
            f"{self.rear_motor:+d} {self.front_left_motor:+d} {self.front_right_motor:+d}\n"
        )


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
class RuntimeState:
    command: ManualCommand
    controller_status: str
    controller_name: str
    controller_snapshot: ControllerSnapshot
    serial_status: str
    serial_target: str
    last_send_result: str
    last_sent_line: str
