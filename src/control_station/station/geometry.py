from __future__ import annotations

import math

from .controller import clamp_unit
from .models import ManualCommand


MOTOR_OUTPUT_BOOST = 2.0


def build_manual_command(turn: float, thrust: float) -> ManualCommand:
    # Inputs are already -1 to 1 from controller
    raw_x = turn
    raw_y = thrust
    
    # Clamp vector magnitude to 1 (100% speed)
    mag = math.sqrt(raw_x**2 + raw_y**2)
    if mag > 1.0:
        raw_x /= mag
        raw_y /= mag
    
    desired_x = raw_x
    desired_y = raw_y

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
        turn=int(round(desired_x * 100)),
        thrust=int(round(desired_y * 100)),
        rear_motor=int(round(clamp_unit(rear_motor * MOTOR_OUTPUT_BOOST) * 100)),
        front_left_motor=int(round(clamp_unit(front_left_motor * MOTOR_OUTPUT_BOOST) * 100)),
        front_right_motor=int(round(clamp_unit(front_right_motor * MOTOR_OUTPUT_BOOST) * 100)),
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
