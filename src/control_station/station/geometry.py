from __future__ import annotations

import math
from dataclasses import dataclass

from .controller import clamp_unit
from .models import ManualCommand


MOTOR_OUTPUT_BOOST = 2.0


@dataclass(frozen=True)
class MotorMixGeometry:
    name: str
    axes: tuple[tuple[float, float], tuple[float, float], tuple[float, float]]
    yaw_gains: tuple[float, float, float]


LEGACY_RADIAL_MOTOR_MIX = MotorMixGeometry(
    name="legacy_radial",
    axes=(
        (0.0, 1.0),
        (math.sqrt(3) / 2, -0.5),
        (-math.sqrt(3) / 2, -0.5),
    ),
    yaw_gains=(1.0, 1.0, 1.0),
)

# Active external-motor hull. The front-axis values describe force on the
# buoy; the corresponding water jets point back-left/back-right. The rear
# motor points its water jet left, so positive rear force pushes the buoy right.
EXTERNAL_TANGENTIAL_MOTOR_MIX = MotorMixGeometry(
    name="external_tangential",
    axes=(
        (1.0, 0.0),
        (0.5, math.sqrt(3) / 2),
        (-0.5, math.sqrt(3) / 2),
    ),
    yaw_gains=(1.0, -1.0, 1.0),
)

ACTIVE_MOTOR_MIX = EXTERNAL_TANGENTIAL_MOTOR_MIX


def build_manual_command(
    turn: float,
    thrust: float,
    yaw: float = 0.0,
    motor_mix: MotorMixGeometry = ACTIVE_MOTOR_MIX,
) -> ManualCommand:
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

    rear_axis, front_left_axis, front_right_axis = motor_mix.axes
    rear_yaw_gain, front_left_yaw_gain, front_right_yaw_gain = motor_mix.yaw_gains

    rear_motor = ((2.0 / 3.0) * (desired_x * rear_axis[0] + desired_y * rear_axis[1]) * MOTOR_OUTPUT_BOOST) + (desired_yaw * rear_yaw_gain)
    front_left_motor = ((2.0 / 3.0) * (
        desired_x * front_left_axis[0] + desired_y * front_left_axis[1]
    ) * MOTOR_OUTPUT_BOOST) + (desired_yaw * front_left_yaw_gain)
    front_right_motor = ((2.0 / 3.0) * (
        desired_x * front_right_axis[0] + desired_y * front_right_axis[1]
    ) * MOTOR_OUTPUT_BOOST) + (desired_yaw * front_right_yaw_gain)

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
