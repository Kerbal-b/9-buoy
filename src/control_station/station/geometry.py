from __future__ import annotations

import math

from .controller import clamp_unit
from .models import ManualCommand


MOTOR_OUTPUT_BOOST = 2.0


def build_manual_command(turn: float, thrust: float, yaw: float = 0.0) -> ManualCommand:
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
    desired_yaw = clamp_unit(yaw)

    rear_axis = (0.0, 1.0)
    # Positive propeller drive pushes the hull away from the motor's radial
    # vector. The front motors therefore use the opposite lateral sign from
    # their physical left/right positions. Rear has no lateral component.
    front_left_axis = (math.sqrt(3) / 2, -0.5)
    front_right_axis = (-math.sqrt(3) / 2, -0.5)

    rear_motor = ((2.0 / 3.0) * (desired_x * rear_axis[0] + desired_y * rear_axis[1]) * MOTOR_OUTPUT_BOOST) + desired_yaw
    front_left_motor = ((2.0 / 3.0) * (
        desired_x * front_left_axis[0] + desired_y * front_left_axis[1]
    ) * MOTOR_OUTPUT_BOOST) + desired_yaw
    front_right_motor = ((2.0 / 3.0) * (
        desired_x * front_right_axis[0] + desired_y * front_right_axis[1]
    ) * MOTOR_OUTPUT_BOOST) + desired_yaw

    max_motor_magnitude = max(abs(rear_motor), abs(front_left_motor), abs(front_right_motor))
    motor_scale = 1.0 / max_motor_magnitude if desired_yaw != 0.0 and max_motor_magnitude > 1.0 else 1.0

    return ManualCommand(
        turn=int(round(desired_x * 100)),
        thrust=int(round(desired_y * 100)),
        rear_motor=int(round(clamp_unit(rear_motor * motor_scale) * 100)),
        front_left_motor=int(round(clamp_unit(front_left_motor * motor_scale) * 100)),
        front_right_motor=int(round(clamp_unit(front_right_motor * motor_scale) * 100)),
        yaw=int(round(desired_yaw * 100)),
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
