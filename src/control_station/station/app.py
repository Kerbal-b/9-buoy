from __future__ import annotations

import argparse
import time

import pygame

from .controller import get_controller, read_axes, read_controller_snapshot
from .geometry import build_manual_command
from .models import ManualCommand, RuntimeState
from .serial_link import open_serial_connection, send_command
from .settings import DEFAULT_BAUDRATE, DEFAULT_DEADZONE, DEFAULT_SEND_RATE, WINDOW_HEIGHT, WINDOW_WIDTH
from .ui import render_debug_interface, render_main_interface


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Laptop control station for manual buoy control."
    )
    parser.add_argument(
        "--port",
        help="Serial port for the Bluetooth link, for example /dev/tty.HC-05-DevB",
    )
    parser.add_argument(
        "--baudrate",
        type=int,
        default=DEFAULT_BAUDRATE,
        help="Serial baudrate used by the Bluetooth serial link",
    )
    parser.add_argument(
        "--deadzone",
        type=float,
        default=DEFAULT_DEADZONE,
        help="Ignore small joystick movement around center",
    )
    parser.add_argument(
        "--send-rate",
        type=float,
        default=DEFAULT_SEND_RATE,
        help="How many command updates to send per second",
    )
    parser.add_argument(
        "--debug-controller",
        action="store_true",
        help="Open the controller diagnostics window instead of the main interface",
    )
    return parser.parse_args()


def build_runtime_state(
    joystick: pygame.joystick.Joystick | None,
    controller_snapshot,
    serial_status: str,
    serial_target: str,
    last_send_result: str,
    last_sent_line: str,
    command: ManualCommand,
) -> RuntimeState:
    controller_status = "Connected" if joystick is not None else "Not connected"
    controller_name = joystick.get_name() if joystick is not None else "Connect Xbox controller"
    return RuntimeState(
        command=command,
        controller_status=controller_status,
        controller_name=controller_name,
        controller_snapshot=controller_snapshot,
        serial_status=serial_status,
        serial_target=serial_target,
        last_send_result=last_send_result,
        last_sent_line=last_sent_line,
    )


def run() -> None:
    args = parse_args()

    pygame.init()
    pygame.joystick.init()
    pygame.font.init()

    screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
    pygame.display.set_caption("Controller Debug" if args.debug_controller else "Buoy Control Station")
    clock = pygame.time.Clock()

    title_font = pygame.font.SysFont("arial", 30, bold=True)
    body_font = pygame.font.SysFont("arial", 22)
    small_font = pygame.font.SysFont("arial", 18)

    joystick = get_controller()
    serial_link, serial_status = open_serial_connection(args.port, args.baudrate)
    serial_target = args.port if args.port else "No port selected"
    last_send_time = 0.0
    last_sent_line = "Nothing sent yet"
    last_send_result = "Waiting for first command"
    last_command = build_manual_command(0.0, 0.0)
    running = True

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if event.type == pygame.JOYDEVICEADDED and joystick is None:
                joystick = get_controller()
            if event.type == pygame.JOYDEVICEREMOVED and joystick is not None:
                joystick.quit()
                joystick = get_controller()

        turn, thrust = read_axes(joystick, args.deadzone)
        controller_snapshot = read_controller_snapshot(joystick, args.deadzone)
        command = build_manual_command(turn, thrust)
        now = time.monotonic()
        min_send_interval = 1.0 / max(args.send_rate, 0.1)

        if command != last_command or (now - last_send_time) >= min_send_interval:
            last_send_result = send_command(serial_link, command)
            last_send_time = now
            last_command = command
            last_sent_line = command.to_line().strip()

        state = build_runtime_state(
            joystick=joystick,
            controller_snapshot=controller_snapshot,
            serial_status=serial_status,
            serial_target=serial_target,
            last_send_result=last_send_result,
            last_sent_line=last_sent_line,
            command=command,
        )

        if args.debug_controller:
            render_debug_interface(
                screen=screen,
                title_font=title_font,
                body_font=body_font,
                small_font=small_font,
                state=state,
                turn=turn,
                thrust=thrust,
                baudrate=args.baudrate,
            )
        else:
            render_main_interface(
                screen=screen,
                title_font=title_font,
                body_font=body_font,
                state=state,
                turn=turn,
                thrust=thrust,
            )

        pygame.display.flip()
        clock.tick(30)

    if joystick is not None:
        joystick.quit()
    if serial_link is not None:
        serial_link.close()
    pygame.quit()
