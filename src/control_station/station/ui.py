from __future__ import annotations

import math
import pygame

from .geometry import get_motor_positions
from .models import AnalogInput, ControllerSnapshot, DigitalInput, ManualCommand, RuntimeState
from .settings import ACCENT, BACKGROUND, GRID, PANEL, TEXT, VECTOR_COLOR, WARNING


def draw_text(
    surface: pygame.Surface,
    font: pygame.font.Font,
    text: str,
    color: tuple[int, int, int],
    x: int,
    y: int,
) -> None:
    surface.blit(font.render(text, True, color), (x, y))


def draw_vector_panel(
    surface: pygame.Surface,
    center: tuple[int, int],
    radius: int,
    x_value: float,
    y_value: float,
) -> None:
    center_x, center_y = center

    pygame.draw.circle(surface, GRID, center, radius, 2)
    pygame.draw.line(surface, GRID, (center_x - radius, center_y), (center_x + radius, center_y), 1)
    pygame.draw.line(surface, GRID, (center_x, center_y - radius), (center_x, center_y + radius), 1)

    end_x = center_x + int(x_value * radius)
    end_y = center_y - int(y_value * radius)

    pygame.draw.line(surface, VECTOR_COLOR, center, (end_x, end_y), 6)
    pygame.draw.circle(surface, VECTOR_COLOR, (end_x, end_y), 10)
    pygame.draw.circle(surface, ACCENT, center, 7)


def draw_buoy_overlay(
    surface: pygame.Surface,
    center: tuple[int, int],
    radius: int,
    command: ManualCommand,
    show_labels: bool,
    font: pygame.font.Font | None = None,
) -> None:
    motor_positions = get_motor_positions(center, radius)
    motor_outputs = {
        "Rear": command.rear_motor,
        "Front L": command.front_left_motor,
        "Front R": command.front_right_motor,
    }

    pygame.draw.lines(
        surface,
        TEXT,
        True,
        [
            motor_positions["Rear"],
            motor_positions["Front L"],
            motor_positions["Front R"],
        ],
        2,
    )
    pygame.draw.circle(surface, TEXT, center, 6)

    for name, position in motor_positions.items():
        output = motor_outputs[name]
        color = ACCENT if output >= 0 else WARNING
        ring_radius = 19 + max(3, int(abs(output) * 0.1))
        pygame.draw.circle(surface, color, position, ring_radius, 2)
        pygame.draw.circle(surface, PANEL, position, 16)
        pygame.draw.circle(surface, TEXT, position, 16, 2)
        if show_labels and font is not None:
            label_surface = font.render(f"{name} {output:+d}%", True, TEXT)
            label_rect = label_surface.get_rect(center=(position[0], position[1] + 34))
            surface.blit(label_surface, label_rect)


def render_main_interface(
    screen: pygame.Surface,
    title_font: pygame.font.Font,
    body_font: pygame.font.Font,
    small_font: pygame.font.Font,
    state: RuntimeState,
    turn: float,
    thrust: float,
) -> None:
    screen.fill(BACKGROUND)

    # Top left: Visual buoy representation
    pygame.draw.rect(screen, PANEL, (36, 28, 500, 400), border_radius=18)

    draw_vector_panel(
        screen,
        center=(290, 280),
        radius=120,
        x_value=turn,
        y_value=thrust,
    )
    draw_buoy_overlay(
        screen,
        center=(290, 280),
        radius=120,
        command=state.command,
        show_labels=True,
        font=small_font,
    )

    # Left bottom: Buoy status and environmental readings
    pygame.draw.rect(screen, PANEL, (36, 450, 500, 300), border_radius=18)
    draw_text(screen, title_font, "Buoy Status", TEXT, 70, 470)

    # Status section
    draw_text(screen, small_font, f"Connection: {state.serial_status}", TEXT, 70, 500)
    draw_text(screen, small_font, f"Current Location: {state.current_location}", TEXT, 70, 520)
    draw_text(screen, small_font, f"Target Location: {state.target_location}", TEXT, 70, 540)
    if state.hold_position:
        draw_text(screen, small_font, "Hold Position: ON", ACCENT, 70, 560)
    draw_text(screen, small_font, f"Controller Vector: {state.command.turn}, {state.command.thrust}", TEXT, 70, 580)
    draw_text(screen, small_font, f"Ack Vector: {state.ack_vector}", TEXT, 70, 600)

    # Environmental section
    draw_text(screen, small_font, "Environmental Readings:", TEXT, 70, 630)
    draw_text(screen, small_font, f"Depth: {state.current_depth}", TEXT, 70, 650)
    draw_text(screen, small_font, f"Water Temp: {state.water_temperature}", TEXT, 70, 670)
    draw_text(screen, small_font, f"Air Temp: {state.air_temperature}", TEXT, 70, 690)

    # Right side: Communication log
    pygame.draw.rect(screen, PANEL, (560, 28, 440, 722), border_radius=18)
    draw_text(screen, title_font, "Buoy Communication", TEXT, 580, 60)

    y_offset = 100
    for msg in state.comm_log[-20:]:  # Show last 20 messages
        if y_offset > 720:
            break
        draw_text(screen, small_font, msg, TEXT, 580, y_offset)
        y_offset += 20


def render_debug_interface(
    screen: pygame.Surface,
    title_font: pygame.font.Font,
    body_font: pygame.font.Font,
    small_font: pygame.font.Font,
    state: RuntimeState,
    turn: float,
    thrust: float,
    baudrate: int,
) -> None:
    screen.fill(BACKGROUND)
    pygame.draw.rect(screen, PANEL, (36, 28, 908, 630), border_radius=18)
    draw_text(screen, title_font, "Controller Debug", TEXT, 70, 54)
    draw_controller_debug(
        screen,
        snapshot=state.controller_snapshot,
        title_font=title_font,
        body_font=body_font,
        small_font=small_font,
    )


def _draw_analog_bar(
    surface: pygame.Surface,
    body_font: pygame.font.Font,
    small_font: pygame.font.Font,
    rect: pygame.Rect,
    analog_input: AnalogInput,
) -> None:
    pygame.draw.rect(surface, GRID, rect, border_radius=12)
    center_x = rect.x + rect.width // 2
    pygame.draw.line(surface, TEXT, (center_x, rect.y + 8), (center_x, rect.bottom - 8), 2)

    clamped = max(-1.0, min(1.0, analog_input.value))
    fill_width = int((rect.width // 2 - 8) * abs(clamped))
    if fill_width > 0:
        if clamped >= 0:
            fill_rect = pygame.Rect(center_x, rect.y + 8, fill_width, rect.height - 16)
        else:
            fill_rect = pygame.Rect(center_x - fill_width, rect.y + 8, fill_width, rect.height - 16)
        pygame.draw.rect(surface, ACCENT, fill_rect, border_radius=8)

    pygame.draw.rect(surface, TEXT, rect, 2, border_radius=12)
    draw_text(surface, small_font, analog_input.name, TEXT, rect.x + 14, rect.y + 10)
    value_text = f"{analog_input.value:+0.2f}"
    value_surface = body_font.render(value_text, True, TEXT)
    value_rect = value_surface.get_rect(midright=(rect.right - 12, rect.centery))
    surface.blit(value_surface, value_rect)


def _draw_digital_tile(
    surface: pygame.Surface,
    font: pygame.font.Font,
    rect: pygame.Rect,
    digital_input: DigitalInput,
) -> None:
    fill_color = ACCENT if digital_input.active else GRID
    pygame.draw.rect(surface, fill_color, rect, border_radius=10)
    pygame.draw.rect(surface, TEXT, rect, 2, border_radius=10)
    draw_text(surface, font, digital_input.name, TEXT, rect.x + 10, rect.y + 8)
    state_label = "ON" if digital_input.active else "OFF"
    state_surface = font.render(state_label, True, TEXT)
    state_rect = state_surface.get_rect(topright=(rect.right - 10, rect.y + 8))
    surface.blit(state_surface, state_rect)


def draw_controller_debug(
    surface: pygame.Surface,
    snapshot: ControllerSnapshot,
    title_font: pygame.font.Font,
    body_font: pygame.font.Font,
    small_font: pygame.font.Font,
) -> None:
    draw_text(surface, body_font, "Analog Inputs", TEXT, 70, 110)
    draw_text(surface, body_font, "Digital Inputs", TEXT, 510, 110)

    analog_area = pygame.Rect(70, 145, 360, 480)
    digital_area = pygame.Rect(510, 145, 400, 480)
    pygame.draw.rect(surface, GRID, analog_area, 2, border_radius=14)
    pygame.draw.rect(surface, GRID, digital_area, 2, border_radius=14)

    analog_row_height = 52
    for index, analog_input in enumerate(snapshot.analog_inputs):
        row_top = analog_area.y + 12 + index * (analog_row_height + 8)
        if row_top + analog_row_height > analog_area.bottom - 12:
            break
        row_rect = pygame.Rect(84, row_top, 332, analog_row_height)
        _draw_analog_bar(surface, body_font, small_font, row_rect, analog_input)

    digital_columns = 2
    tile_width = 182
    tile_height = 48
    for index, digital_input in enumerate(snapshot.digital_inputs):
        col = index % digital_columns
        row = index // digital_columns
        tile_x = 524 + col * (tile_width + 12)
        tile_y = digital_area.y + 12 + row * (tile_height + 10)
        if tile_y + tile_height > digital_area.bottom - 12:
            break
        tile_rect = pygame.Rect(tile_x, tile_y, tile_width, tile_height)
        _draw_digital_tile(surface, small_font, tile_rect, digital_input)
