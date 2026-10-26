from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class ManualCommand:
    turn: int
    thrust: int
    rear_motor: int
    front_left_motor: int
    front_right_motor: int
    yaw: int = 0

    def to_line(self) -> str:
        if self.yaw != 0:
            return f"CTRL MOTION {self.turn:+d} {self.thrust:+d} {self.yaw:+d}\n"
        return f"CTRL VECTOR {self.turn:+d} {self.thrust:+d}\n"


@dataclass(frozen=True)
class ScienceSample:
    timestamp: float
    latitude: float
    longitude: float
    depth_m: float
    audio_level_percent: float
    water_temperature_c: float | None = None
