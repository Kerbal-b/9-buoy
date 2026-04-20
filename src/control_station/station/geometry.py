from __future__ import annotations

import math

from .controller import clamp_unit
from .models import ManualCommand


def build_manual_command(turn: float, thrust: float) -> ManualCommand:
    desired_x = clamp_unit(turn)
    desired_y = clamp_unit(thrust)

    rear_axis = (0.0, 1.0)
    front_left_axis = (-math.sqrt(3) / 2, -0.5)
    front_right_axis = (math.sqrt(3) / 2, -0.5)

    rear_motor = (2.0 / 3.0) * (desired_x * rear_axis[0] + desired_y * rear_axis[1])
    front_left_motor = (2.0 / 3.0) * (
        desired_x * front_left_axis[0] + desired_y * front_left_axis[1]
    )
    front_right_motor = (2.0 / 3.0) * (
        desired_x * front_right_axis[0] + desired_y * front_right_axis[1]
    )

    return ManualCommand(
        turn=int(round(clamp_unit(turn) * 100)),
        thrust=int(round(clamp_unit(thrust) * 100)),
        rear_motor=int(round(clamp_unit(rear_motor) * 100)),
        front_left_motor=int(round(clamp_unit(front_left_motor) * 100)),
        front_right_motor=int(round(clamp_unit(front_right_motor) * 100)),
    )


def vector_magnitude(turn: float, thrust: float) -> float:
    return min((turn * turn + thrust * thrust) ** 0.5, 1.0)


def get_motor_positions(center: tuple[int, int], radius: int) -> dict[str, tuple[int, int]]:
    center_x, center_y = center
    buoy_radius = int(radius * 0.52)
    return {
        "Rear": (center_x, center_y + buoy_radius),
        "Front L": (
            center_x - int(math.sin(math.radians(60)) * buoy_radius),
            center_y - int(math.cos(math.radians(60)) * buoy_radius),
        ),
        "Front R": (
            center_x + int(math.sin(math.radians(60)) * buoy_radius),
            center_y - int(math.cos(math.radians(60)) * buoy_radius),
        ),
    }
