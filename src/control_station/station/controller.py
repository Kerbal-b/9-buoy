from __future__ import annotations

try:
    import pygame
except ModuleNotFoundError:
    pygame = None

from .models import AnalogInput, ControllerSnapshot, DigitalInput


def get_controller() -> pygame.joystick.Joystick | None:
    if pygame is None:
        return None
    if pygame.joystick.get_count() == 0:
        return None

    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    return joystick


def clamp_unit(value: float) -> float:
    return max(-1.0, min(1.0, value))


def apply_deadzone(value: float, deadzone: float) -> float:
    if abs(value) < deadzone:
        return 0.0
    return clamp_unit(value)


def _map_controller_motion(
    raw_left_x: float,
    raw_left_y: float,
    raw_right_x: float,
    deadzone: float,
) -> tuple[float, float, float]:
    """Map standard SDL Xbox axes to lateral, thrust, and yaw commands."""
    lateral = apply_deadzone(raw_left_x, deadzone)
    thrust = apply_deadzone(-raw_left_y, deadzone)
    yaw = apply_deadzone(raw_right_x, deadzone)
    return lateral, thrust, yaw


def read_motion_axes(
    joystick: pygame.joystick.Joystick | None, deadzone: float
) -> tuple[float, float, float]:
    if joystick is None or pygame is None:
        return 0.0, 0.0, 0.0

    pygame.event.pump()
    return _map_controller_motion(
        _get_axis(joystick, 0),
        _get_axis(joystick, 1),
        _get_axis(joystick, 2),
        deadzone,
    )


def read_axes(
    joystick: pygame.joystick.Joystick | None, deadzone: float
) -> tuple[float, float]:
    lateral, thrust, _yaw = read_motion_axes(joystick, deadzone)
    return lateral, thrust


def _get_axis(joystick: pygame.joystick.Joystick, index: int) -> float:
    if index >= joystick.get_numaxes():
        return 0.0
    return joystick.get_axis(index)


def _get_button(joystick: pygame.joystick.Joystick, index: int) -> bool:
    if index >= joystick.get_numbuttons():
        return False
    return bool(joystick.get_button(index))


def read_controller_snapshot(
    joystick: pygame.joystick.Joystick | None, deadzone: float
) -> ControllerSnapshot:
    if joystick is None or pygame is None:
        return ControllerSnapshot(
            analog_inputs=(),
            digital_inputs=(),
        )

    pygame.event.pump()
    analog_inputs: list[AnalogInput] = []
    digital_inputs: list[DigitalInput] = []

    for axis_index in range(joystick.get_numaxes()):
        raw_value = _get_axis(joystick, axis_index)
        analog_inputs.append(
            AnalogInput(
                name=f"Axis {axis_index}",
                value=apply_deadzone(raw_value, deadzone),
            )
        )

    for hat_index in range(joystick.get_numhats()):
        hat_x, hat_y = joystick.get_hat(hat_index)
        analog_inputs.append(AnalogInput(name=f"Hat {hat_index} X", value=float(hat_x)))
        analog_inputs.append(AnalogInput(name=f"Hat {hat_index} Y", value=float(hat_y)))

    for button_index in range(joystick.get_numbuttons()):
        digital_inputs.append(
            DigitalInput(
                name=f"Button {button_index}",
                active=_get_button(joystick, button_index),
            )
        )

    return ControllerSnapshot(
        analog_inputs=tuple(analog_inputs),
        digital_inputs=tuple(digital_inputs),
    )
