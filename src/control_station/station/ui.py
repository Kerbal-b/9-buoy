from __future__ import annotations

import math

import pygame
from .models import AnalogInput, ControllerSnapshot, DigitalInput, ManualCommand, RuntimeState, ScienceSample
from .settings import ACCENT, BACKGROUND, ERROR, GRID, PANEL, TEXT, VECTOR_COLOR, WARNING


def get_connection_color(status: str) -> tuple[int, int, int]:
    if status.startswith("Connected"):
        return ACCENT
    if status.startswith("Reconnecting"):
        return WARNING
    if status.startswith("Disconnected"):
        return ERROR
    return TEXT


def draw_text(
    surface: pygame.Surface,
    font: pygame.font.Font,
    text: str,
    color: tuple[int, int, int],
    x: int,
    y: int,
) -> None:
    surface.blit(font.render(text, True, color), (x, y))


def get_main_layout_rects() -> dict[str, pygame.Rect]:
    return {
        "buoy": pygame.Rect(36, 28, 470, 400),
        "status": pygame.Rect(36, 448, 470, 516),
        "science": pygame.Rect(532, 28, 1032, 360),
        "dashboard": pygame.Rect(532, 408, 1032, 556),
    }


def get_audio_mute_button_rect(science_rect: pygame.Rect) -> pygame.Rect:
    return pygame.Rect(science_rect.right - 54, science_rect.y + 18, 34, 28)


def get_dashboard_tab_rects(dashboard_rect: pygame.Rect) -> tuple[pygame.Rect, pygame.Rect]:
    tab_y = dashboard_rect.y + 44
    logs_rect = pygame.Rect(dashboard_rect.x + 16, tab_y, 92, 30)
    analysis_rect = pygame.Rect(dashboard_rect.x + 114, tab_y, 152, 30)
    return logs_rect, analysis_rect


def parse_battery_metrics(battery_status: str) -> tuple[str, str, str, int | None]:
    parts = [part.strip() for part in battery_status.split("/")]
    if len(parts) != 3:
        return battery_status, "N/A", "N/A", None

    voltage = parts[0]
    current = parts[1]
    percent_text = parts[2]

    percent_value: int | None = None
    if percent_text.endswith("%"):
        try:
            percent_value = max(0, min(100, int(float(percent_text[:-1]))))
        except ValueError:
            percent_value = None

    return voltage, current, percent_text, percent_value


def get_battery_color(percent: int | None) -> tuple[int, int, int]:
    if percent is None:
        return TEXT
    if percent <= 20:
        return ERROR
    if percent <= 50:
        return WARNING
    return ACCENT


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


def draw_status_table(
    screen: pygame.Surface,
    table_caption_font: pygame.font.Font,
    table_value_font: pygame.font.Font,
    rows: list[tuple[str, str]],
    *,
    caption_x: int,
    value_x: int,
    y_start: int,
    row_height: int,
) -> None:
    for i, (caption, value) in enumerate(rows):
        if not caption:
            continue
        y = y_start + i * row_height
        caption_surface = table_caption_font.render(caption, True, TEXT)
        caption_rect = caption_surface.get_rect(left=caption_x, centery=y)
        screen.blit(caption_surface, caption_rect)
        color = get_connection_color(value) if caption == "Connection:" else TEXT
        value_surface = table_value_font.render(value, True, color)
        value_rect = value_surface.get_rect(right=value_x, centery=y)
        screen.blit(value_surface, value_rect)
        pygame.draw.line(screen, GRID, (caption_x - 10, y + 15), (value_x + 10, y + 15), 1)


def draw_battery_card(
    surface: pygame.Surface,
    title_font: pygame.font.Font,
    caption_font: pygame.font.Font,
    value_font: pygame.font.Font,
    *,
    rect: pygame.Rect,
    battery_status: str,
) -> None:
    """Compact single-line battery display showing Voltage, Current, and Percentage.

    This replaces the larger multi-line card to save space in the UI.
    """
    voltage, current, percent_text, percent_value = parse_battery_metrics(battery_status)
    battery_color = get_battery_color(percent_value)

    # Draw a simple bordered container
    pygame.draw.rect(surface, GRID, rect, 1, border_radius=8)

    padding = 12
    x = rect.x + padding
    center_y = rect.y + rect.height // 2

    # Voltage label and value
    draw_text(surface, caption_font, "V:", TEXT, x, center_y - caption_font.get_height() // 2)
    x += 28
    draw_text(surface, value_font, voltage, battery_color, x, center_y - value_font.get_height() // 2)

    # Spacer
    x += 40

    # Current label and value
    draw_text(surface, caption_font, "A:", TEXT, x, center_y - caption_font.get_height() // 2)
    x += 20
    draw_text(surface, value_font, current, TEXT, x, center_y - value_font.get_height() // 2)

    # Percentage on the right
    percent_surface = value_font.render(percent_text, True, battery_color)
    percent_rect = percent_surface.get_rect(midright=(rect.right - padding, center_y))
    surface.blit(percent_surface, percent_rect)


def draw_audio_panel(
    surface: pygame.Surface,
    title_font: pygame.font.Font,
    body_font: pygame.font.Font,
    small_font: pygame.font.Font,
    *,
    rect: pygame.Rect,
    waveform: tuple[float, ...],
    audio_stream: str,
    audio_level: str,
    muted: bool,
) -> None:
    pygame.draw.rect(surface, PANEL, rect, border_radius=18)
    draw_text(surface, title_font, "Audio Monitor", TEXT, rect.x + 20, rect.y + 18)
    draw_text(surface, small_font, f"Stream: {audio_stream}", TEXT, rect.x + 20, rect.y + 56)
    draw_text(surface, small_font, f"Level: {audio_level}", TEXT, rect.x + 20, rect.y + 80)

    button_rect = get_audio_mute_button_rect(rect)
    button_color = WARNING if muted else ACCENT
    pygame.draw.rect(surface, button_color, button_rect, border_radius=10)
    pygame.draw.rect(surface, TEXT, button_rect, 2, border_radius=10)
    _draw_mute_icon(surface, button_rect, muted)

    graph_rect = pygame.Rect(rect.x + 20, rect.y + 120, rect.width - 40, rect.height - 150)
    pygame.draw.rect(surface, BACKGROUND, graph_rect, border_radius=10)
    pygame.draw.rect(surface, GRID, graph_rect, 1, border_radius=10)
    mid_y = graph_rect.y + graph_rect.height // 2
    pygame.draw.line(surface, GRID, (graph_rect.x + 8, mid_y), (graph_rect.right - 8, mid_y), 1)

    if not waveform:
        draw_text(surface, small_font, "Waiting for audio packets...", WARNING, graph_rect.x + 20, graph_rect.y + 20)
        return

    point_count = min(len(waveform), max(2, graph_rect.width - 20))
    step = max(1, len(waveform) // point_count)
    points: list[tuple[int, int]] = []
    draw_index = 0
    for sample_index in range(0, len(waveform), step):
        x = graph_rect.x + 10 + draw_index
        if x >= graph_rect.right - 10:
            break
        value = max(-1.0, min(1.0, waveform[sample_index]))
        y = int(mid_y - value * (graph_rect.height * 0.42))
        points.append((x, y))
        draw_index += 1

    if len(points) >= 2:
        pygame.draw.lines(surface, VECTOR_COLOR, False, points, 2)


def _draw_mute_icon(surface: pygame.Surface, rect: pygame.Rect, muted: bool) -> None:
    icon_center_y = rect.centery
    speaker = [
        (rect.x + 7, icon_center_y - 5),
        (rect.x + 13, icon_center_y - 5),
        (rect.x + 18, icon_center_y - 10),
        (rect.x + 18, icon_center_y + 10),
        (rect.x + 13, icon_center_y + 5),
        (rect.x + 7, icon_center_y + 5),
    ]
    pygame.draw.polygon(surface, BACKGROUND, speaker)
    pygame.draw.polygon(surface, TEXT, speaker, 2)

    if muted:
        pygame.draw.line(surface, ERROR, (rect.x + 22, rect.y + 6), (rect.right - 6, rect.bottom - 6), 3)
        pygame.draw.line(surface, ERROR, (rect.x + 22, rect.bottom - 6), (rect.right - 6, rect.y + 6), 3)
    else:
        wave_one = [
            (rect.x + 20, icon_center_y - 6),
            (rect.x + 25, icon_center_y - 2),
            (rect.x + 25, icon_center_y + 2),
            (rect.x + 20, icon_center_y + 6),
        ]
        wave_two = [
            (rect.x + 26, icon_center_y - 10),
            (rect.x + 32, icon_center_y - 4),
            (rect.x + 32, icon_center_y + 4),
            (rect.x + 26, icon_center_y + 10),
        ]
        pygame.draw.lines(surface, ACCENT, False, wave_one, 2)
        pygame.draw.lines(surface, ACCENT, False, wave_two, 2)


def draw_audio_strip(
    surface: pygame.Surface,
    title_font: pygame.font.Font,
    small_font: pygame.font.Font,
    *,
    rect: pygame.Rect,
    waveform: tuple[float, ...],
    audio_stream: str,
    audio_level: str,
    muted: bool,
) -> None:
    strip_rect = pygame.Rect(rect.x + 16, rect.bottom - 104, rect.width - 32, 84)
    pygame.draw.rect(surface, BACKGROUND, strip_rect, border_radius=12)
    pygame.draw.rect(surface, GRID, strip_rect, 1, border_radius=12)

    button_rect = get_audio_mute_button_rect(rect)
    button_color = WARNING if muted else ACCENT
    pygame.draw.rect(surface, button_color, button_rect, border_radius=8)
    pygame.draw.rect(surface, TEXT, button_rect, 2, border_radius=8)
    _draw_mute_icon(surface, button_rect, muted)

    draw_text(surface, small_font, f"Stream: {audio_stream}", TEXT, strip_rect.x + 14, strip_rect.y + 10)
    draw_text(surface, small_font, f"Level: {audio_level}", TEXT, strip_rect.x + 14, strip_rect.y + 34)

    waveform_rect = pygame.Rect(strip_rect.x + 220, strip_rect.y + 12, strip_rect.width - 280, strip_rect.height - 24)
    pygame.draw.rect(surface, PANEL, waveform_rect, border_radius=10)
    pygame.draw.rect(surface, GRID, waveform_rect, 1, border_radius=10)

    mid_y = waveform_rect.y + waveform_rect.height // 2
    pygame.draw.line(surface, GRID, (waveform_rect.x + 8, mid_y), (waveform_rect.right - 8, mid_y), 1)
    if not waveform:
        draw_text(surface, small_font, "No audio yet", WARNING, waveform_rect.x + 10, waveform_rect.y + 8)
        return

    point_count = min(len(waveform), max(2, waveform_rect.width - 20))
    step = max(1, len(waveform) // point_count)
    points: list[tuple[int, int]] = []
    draw_index = 0
    for sample_index in range(0, len(waveform), step):
        x = waveform_rect.x + 10 + draw_index
        if x >= waveform_rect.right - 10:
            break
        value = max(-1.0, min(1.0, waveform[sample_index]))
        y = int(mid_y - value * (waveform_rect.height * 0.35))
        points.append((x, y))
        draw_index += 1

    if len(points) >= 2:
        pygame.draw.lines(surface, VECTOR_COLOR, False, points, 2)


def _parse_imu_accel_vector(text: str) -> tuple[float, float, float] | None:
    if not text or text == "N/A":
        return None

    cleaned = text.replace(",", " ").replace("g", " ").strip()
    parts = [part for part in cleaned.split() if part]
    if len(parts) < 3:
        return None

    try:
        return float(parts[0]), float(parts[1]), float(parts[2])
    except ValueError:
        return None


def _estimate_roll_pitch_from_accel(accel: tuple[float, float, float]) -> tuple[float, float]:
    ax, ay, az = accel
    roll_rad = math.atan2(ay, az)
    pitch_rad = math.atan2(-ax, math.sqrt((ay * ay) + (az * az)))
    return math.degrees(roll_rad), math.degrees(pitch_rad)


def _rotate_point(point: tuple[float, float, float], pitch_deg: float, roll_deg: float) -> tuple[float, float, float]:
    x, y, z = point
    pitch = math.radians(pitch_deg)
    roll = math.radians(roll_deg)

    y2 = y * math.cos(roll) - z * math.sin(roll)
    z2 = y * math.sin(roll) + z * math.cos(roll)
    x3 = x * math.cos(pitch) + z2 * math.sin(pitch)
    z3 = -x * math.sin(pitch) + z2 * math.cos(pitch)
    return x3, y2, z3


def draw_3d_attitude_panel(
    surface: pygame.Surface,
    title_font: pygame.font.Font,
    body_font: pygame.font.Font,
    small_font: pygame.font.Font,
    *,
    rect: pygame.Rect,
    imu_accel_text: str,
) -> None:
    pygame.draw.rect(surface, PANEL, rect, border_radius=18)
    draw_text(surface, title_font, "3D Buoy View", TEXT, rect.x + 20, rect.y + 18)

    accel = _parse_imu_accel_vector(imu_accel_text)
    if accel is None:
        draw_text(surface, body_font, "Waiting for IMU accel...", WARNING, rect.x + 20, rect.y + 70)
        return

    roll_deg, pitch_deg = _estimate_roll_pitch_from_accel(accel)
    draw_text(surface, small_font, f"Roll: {roll_deg:+0.1f} deg", TEXT, rect.x + 20, rect.y + 70)
    draw_text(surface, small_font, f"Pitch: {pitch_deg:+0.1f} deg", TEXT, rect.x + 220, rect.y + 70)

    center_x = rect.x + rect.width // 2
    center_y = rect.y + rect.height // 2 + 20
    scale = 120.0
    perspective = 260.0

    cube_points = [
        (-1, -0.45, -0.65), (1, -0.45, -0.65), (1, 0.45, -0.65), (-1, 0.45, -0.65),
        (-1, -0.45, 0.65), (1, -0.45, 0.65), (1, 0.45, 0.65), (-1, 0.45, 0.65),
    ]
    edges = [
        (0, 1), (1, 2), (2, 3), (3, 0),
        (4, 5), (5, 6), (6, 7), (7, 4),
        (0, 4), (1, 5), (2, 6), (3, 7),
    ]

    projected: list[tuple[int, int]] = []
    for point in cube_points:
        rx, ry, rz = _rotate_point(point, pitch_deg, roll_deg)
        z_offset = rz + 3.0
        px = int(center_x + (rx * scale * perspective) / z_offset)
        py = int(center_y + (ry * scale * perspective) / z_offset)
        projected.append((px, py))

    for a, b in edges:
        pygame.draw.line(surface, GRID, projected[a], projected[b], 2)

    for index, dot in enumerate(projected):
        color = ACCENT if index < 4 else VECTOR_COLOR
        pygame.draw.circle(surface, color, dot, 4)


def draw_buoy_3d_motor_view(
    surface: pygame.Surface,
    *,
    rect: pygame.Rect,
    command: ManualCommand,
    imu_accel_text: str,
    body_font: pygame.font.Font,
    small_font: pygame.font.Font,
) -> None:
    accel = _parse_imu_accel_vector(imu_accel_text)
    if accel is None:
        roll_deg = 0.0
        pitch_deg = 0.0
    else:
        roll_deg, pitch_deg = _estimate_roll_pitch_from_accel(accel)

    center_x = rect.x + rect.width // 2
    center_y = rect.y + rect.height // 2 + 10
    perspective_scale = 185.0

    # Buoy body as a thin triangular prism (top and bottom faces).
    top_face = [
        (0.0, 0.95, 0.20),      # Rear
        (-0.85, -0.50, 0.20),   # Front left
        (0.85, -0.50, 0.20),    # Front right
    ]
    bottom_face = [(x, y, -0.20) for (x, y, _) in top_face]

    motor_points = {
        "Rear": (0.0, 0.95, 0.22),
        "Front L": (-0.85, -0.50, 0.22),
        "Front R": (0.85, -0.50, 0.22),
    }
    motor_values = {
        "Rear": command.rear_motor,
        "Front L": command.front_left_motor,
        "Front R": command.front_right_motor,
    }

    def project(point: tuple[float, float, float]) -> tuple[float, int, int]:
        rx, ry, rz = _rotate_point(point, pitch_deg, roll_deg)
        z_offset = rz + 3.8
        px = int(center_x + (rx * perspective_scale) / z_offset)
        py = int(center_y + (ry * perspective_scale) / z_offset)
        return rz, px, py

    top_proj = [project(p) for p in top_face]
    bottom_proj = [project(p) for p in bottom_face]

    top_points_2d = [(px, py) for _, px, py in top_proj]
    bottom_points_2d = [(px, py) for _, px, py in bottom_proj]

    # Reference crosshair so render area is obvious even when nearly flat.
    pygame.draw.line(surface, GRID, (rect.x + 24, center_y), (rect.right - 24, center_y), 1)
    pygame.draw.line(surface, GRID, (center_x, rect.y + 56), (center_x, rect.bottom - 20), 1)

    # Draw hull first so motor spheres stay visible on top.
    pygame.draw.polygon(surface, (42, 52, 66), bottom_points_2d)
    pygame.draw.polygon(surface, (63, 78, 98), top_points_2d)
    pygame.draw.polygon(surface, GRID, bottom_points_2d, 2)
    pygame.draw.polygon(surface, GRID, top_points_2d, 2)
    for i in range(3):
        pygame.draw.line(surface, GRID, top_points_2d[i], bottom_points_2d[i], 2)

    projected: list[tuple[float, str, int, int, int]] = []
    for name, point in motor_points.items():
        depth, px, py = project(point)
        magnitude = abs(motor_values[name])
        radius = 11 + int(magnitude * 0.13)
        projected.append((depth, name, px, py, radius))

    projected.sort(key=lambda item: item[0])

    for _, name, px, py, radius in projected:
        output = motor_values[name]
        color = ACCENT if output >= 0 else WARNING
        pygame.draw.circle(surface, color, (px, py), radius)
        pygame.draw.circle(surface, TEXT, (px, py), radius, 2)

    edges = [("Rear", "Front L"), ("Front L", "Front R"), ("Front R", "Rear")]
    point_by_name = {name: (px, py) for _, name, px, py, _ in projected}
    for a, b in edges:
        pygame.draw.line(surface, VECTOR_COLOR, point_by_name[a], point_by_name[b], 3)

    # Fixed label anchors to avoid overlap with rendered spheres.
    front_left_text = small_font.render(f"Front L {motor_values['Front L']:+d}%", True, TEXT)
    front_right_text = small_font.render(f"Front R {motor_values['Front R']:+d}%", True, TEXT)
    rear_text = small_font.render(f"Rear {motor_values['Rear']:+d}%", True, TEXT)

    surface.blit(front_left_text, (rect.x + 14, rect.y + 14))
    front_right_rect = front_right_text.get_rect(topright=(rect.right - 14, rect.y + 14))
    surface.blit(front_right_text, front_right_rect)
    rear_rect = rear_text.get_rect(midbottom=(rect.x + rect.width // 2, rect.bottom - 12))
    surface.blit(rear_text, rear_rect)

    draw_text(surface, small_font, f"Roll {roll_deg:+0.1f} deg", TEXT, rect.x + 14, rect.y + 52)
    draw_text(surface, small_font, f"Pitch {pitch_deg:+0.1f} deg", TEXT, rect.x + 14, rect.y + 72)


def _parse_science_float(value: str) -> float | None:
    if not value or value in {"N/A", "Unknown"}:
        return None
    token = value.split()[0].strip().rstrip(",")
    try:
        return float(token)
    except ValueError:
        return None


def _audio_level_color(level_text: str) -> tuple[int, int, int]:
    level = _parse_science_float(level_text.rstrip("%"))
    if level is None:
        return TEXT
    if level <= 20.0:
        return ACCENT
    if level <= 55.0:
        return WARNING
    return ERROR


def draw_technical_panel(
    surface: pygame.Surface,
    title_font: pygame.font.Font,
    table_caption_font: pygame.font.Font,
    table_value_font: pygame.font.Font,
    *,
    rect: pygame.Rect,
    state: RuntimeState,
) -> None:
    pygame.draw.rect(surface, PANEL, rect, border_radius=18)
    draw_text(surface, title_font, "Buoy Status & Navigation", TEXT, rect.x + 20, rect.y + 18)

    voltage, current, percent_text, _ = parse_battery_metrics(state.telemetry.battery_status)
    battery_value = f"{voltage} / {current} / {percent_text}"

    table_data = [
        ("Battery", battery_value),
        ("Connection", state.serial_status),
        ("Mode", state.telemetry.control_mode),
        ("Location", state.telemetry.current_location),
        ("Target", state.telemetry.target_location),
        ("Hold", "ON" if state.telemetry.hold_position else "OFF"),
        ("Vector", f"{state.command.turn}, {state.command.thrust}"),
        ("Ack", state.ack_vector),
        ("Current", state.telemetry.current_draw),
    ]

    draw_status_table(
        surface,
        table_caption_font,
        table_value_font,
        table_data,
        caption_x=rect.x + 20,
        value_x=rect.right - 18,
        y_start=rect.y + 62,
        row_height=34,
    )


def draw_science_panel(
    surface: pygame.Surface,
    title_font: pygame.font.Font,
    table_caption_font: pygame.font.Font,
    table_value_font: pygame.font.Font,
    small_font: pygame.font.Font,
    *,
    rect: pygame.Rect,
    state: RuntimeState,
) -> None:
    pygame.draw.rect(surface, PANEL, rect, border_radius=18)
    draw_text(surface, title_font, "Science & Audio", TEXT, rect.x + 20, rect.y + 18)

    science_rows = [
        ("Depth", state.telemetry.current_depth),
        ("Water Temp", state.telemetry.water_temperature),
        ("Air Temp", state.telemetry.air_temperature),
        ("IMU Accel", state.telemetry.imu_accel),
        ("IMU Gyro", state.telemetry.imu_gyro),
        ("IMU Temp", state.telemetry.imu_temperature),
        ("IMU UDP Loss", state.telemetry.imu_udp_loss),
    ]

    draw_status_table(
        surface,
        table_caption_font,
        table_value_font,
        science_rows,
        caption_x=rect.x + 20,
        value_x=rect.right - 72,
        y_start=rect.y + 70,
        row_height=24,
    )

    draw_audio_strip(
        surface,
        title_font,
        small_font,
        rect=rect,
        waveform=state.audio_waveform,
        audio_stream=state.telemetry.audio_stream,
        audio_level=state.telemetry.audio_level,
        muted=state.audio_muted,
    )


def draw_log_view(
    surface: pygame.Surface,
    small_font: pygame.font.Font,
    *,
    rect: pygame.Rect,
    comm_log: list[str],
) -> None:
    content_rect = pygame.Rect(rect.x + 16, rect.y + 88, rect.width - 32, rect.height - 104)
    pygame.draw.rect(surface, BACKGROUND, content_rect, border_radius=14)
    pygame.draw.rect(surface, GRID, content_rect, 1, border_radius=14)

    wrapped_lines = _collect_wrapped_log_lines(comm_log, small_font, content_rect.width - 20)
    line_height = small_font.get_height() + 2
    max_lines = max(1, (content_rect.height - 16) // line_height)
    visible = wrapped_lines[-max_lines:]
    y = content_rect.y + 10
    for line in visible:
        draw_text(surface, small_font, line, TEXT, content_rect.x + 10, y)
        y += line_height


def _format_log_entry(entry: str) -> str | None:
    if ": RX RAW " in entry or ": TX RAW " in entry:
        return None

    if ": RX TXT " in entry:
        return entry.split(": RX TXT ", 1)[1]

    if ": COMM LOG START " in entry:
        return "COMM LOG START"

    if ": COMM LOG END" in entry:
        return "COMM LOG END"

    if ": LINK TIMEOUT " in entry:
        return entry.split(": ", 1)[1]

    if ": RECONNECTED " in entry:
        return entry.split(": ", 1)[1]

    if ": " in entry:
        return entry.split(": ", 1)[1]

    return entry


def _wrap_text(font: pygame.font.Font, text: str, max_width: int) -> list[str]:
    words = text.split()
    if not words:
        return [text]

    lines: list[str] = []
    current = words[0]
    for word in words[1:]:
        candidate = f"{current} {word}"
        if font.size(candidate)[0] <= max_width:
            current = candidate
        else:
            lines.append(current)
            current = word
    lines.append(current)
    return lines


def _collect_wrapped_log_lines(comm_log: list[str], font: pygame.font.Font, max_width: int) -> list[str]:
    lines: list[str] = []
    for entry in comm_log:
        display = _format_log_entry(entry)
        if display is None:
            continue
        lines.extend(_wrap_text(font, display, max_width))
    return lines


def _parse_science_samples_bounds(samples: tuple[ScienceSample, ...]) -> tuple[float, float, float, float] | None:
    if not samples:
        return None
    lat_min = min(sample.latitude for sample in samples)
    lat_max = max(sample.latitude for sample in samples)
    lon_min = min(sample.longitude for sample in samples)
    lon_max = max(sample.longitude for sample in samples)
    return lat_min, lat_max, lon_min, lon_max


def _science_sample_color(sample: ScienceSample) -> tuple[int, int, int]:
    level = max(0.0, min(100.0, sample.audio_level_percent)) / 100.0
    if level < 0.5:
        mix = level / 0.5
        return (
            int(ACCENT[0] + (WARNING[0] - ACCENT[0]) * mix),
            int(ACCENT[1] + (WARNING[1] - ACCENT[1]) * mix),
            int(ACCENT[2] + (WARNING[2] - ACCENT[2]) * mix),
        )
    mix = (level - 0.5) / 0.5
    return (
        int(WARNING[0] + (ERROR[0] - WARNING[0]) * mix),
        int(WARNING[1] + (ERROR[1] - WARNING[1]) * mix),
        int(WARNING[2] + (ERROR[2] - WARNING[2]) * mix),
    )


def draw_science_analysis_view(
    surface: pygame.Surface,
    title_font: pygame.font.Font,
    small_font: pygame.font.Font,
    *,
    rect: pygame.Rect,
    science_samples: tuple[ScienceSample, ...],
    telemetry: RuntimeState,
) -> None:
    content_rect = pygame.Rect(rect.x + 16, rect.y + 50, rect.width - 32, rect.height - 66)
    pygame.draw.rect(surface, BACKGROUND, content_rect, border_radius=14)
    pygame.draw.rect(surface, GRID, content_rect, 1, border_radius=14)

    draw_text(surface, small_font, "Local science map: GPS path with point size = depth and color = audio level", TEXT, content_rect.x + 12, content_rect.y + 10)

    if not science_samples:
        draw_text(surface, title_font, "Waiting for science samples...", WARNING, content_rect.x + 40, content_rect.y + content_rect.height // 2 - 20)
        return

    map_rect = pygame.Rect(content_rect.x + 12, content_rect.y + 40, content_rect.width - 24, content_rect.height - 58)
    pygame.draw.rect(surface, (20, 28, 40), map_rect, border_radius=10)
    pygame.draw.rect(surface, GRID, map_rect, 1, border_radius=10)

    bounds = _parse_science_samples_bounds(science_samples)
    if bounds is None:
        return

    lat_min, lat_max, lon_min, lon_max = bounds
    center_lat = (lat_min + lat_max) / 2.0
    center_lon = (lon_min + lon_max) / 2.0
    lat_span = max(lat_max - lat_min, 0.00001)
    lon_span = max(lon_max - lon_min, 0.00001)
    scale_x = (map_rect.width - 60) / lon_span
    scale_y = (map_rect.height - 60) / lat_span
    scale = min(scale_x, scale_y, 180000.0)

    pygame.draw.line(surface, GRID, (map_rect.x + map_rect.width // 2, map_rect.y + 12), (map_rect.x + map_rect.width // 2, map_rect.bottom - 12), 1)
    pygame.draw.line(surface, GRID, (map_rect.x + 12, map_rect.y + map_rect.height // 2), (map_rect.right - 12, map_rect.y + map_rect.height // 2), 1)

    projected_points: list[tuple[int, int, ScienceSample]] = []
    for sample in science_samples:
        px = int(map_rect.centerx + (sample.longitude - center_lon) * scale)
        py = int(map_rect.centery - (sample.latitude - center_lat) * scale)
        projected_points.append((px, py, sample))

    trail = [(px, py) for px, py, _sample in projected_points]
    if len(trail) >= 2:
        pygame.draw.lines(surface, VECTOR_COLOR, False, trail, 2)

    for px, py, sample in projected_points:
        color = _science_sample_color(sample)
        radius = 4 + min(10, int(sample.depth_m * 1.5))
        pygame.draw.circle(surface, color, (px, py), radius)
        pygame.draw.circle(surface, TEXT, (px, py), radius, 1)

    latest = science_samples[-1]
    legend_y = map_rect.y + 10
    draw_text(surface, small_font, f"Latest depth: {latest.depth_m:.2f} m", TEXT, map_rect.x + 12, legend_y)
    draw_text(surface, small_font, f"Latest audio: {latest.audio_level_percent:.1f}%", TEXT, map_rect.x + 220, legend_y)
    draw_text(surface, small_font, f"Location: {latest.latitude:.6f}, {latest.longitude:.6f}", TEXT, map_rect.x + 420, legend_y)


def draw_dashboard_panel(
    surface: pygame.Surface,
    title_font: pygame.font.Font,
    small_font: pygame.font.Font,
    *,
    rect: pygame.Rect,
    dashboard_tab: str,
    comm_log: list[str],
    science_samples: tuple[ScienceSample, ...],
    telemetry: RuntimeState,
) -> None:
    pygame.draw.rect(surface, PANEL, rect, border_radius=18)
    draw_text(surface, title_font, "Communications / Science Analysis", TEXT, rect.x + 20, rect.y + 16)

    logs_tab_rect, analysis_tab_rect = get_dashboard_tab_rects(rect)
    active_color = ACCENT
    inactive_color = GRID
    for tab_name, tab_rect in (("logs", logs_tab_rect), ("analysis", analysis_tab_rect)):
        is_active = dashboard_tab == tab_name
        tab_fill = active_color if is_active else inactive_color
        pygame.draw.rect(surface, tab_fill, tab_rect, border_radius=10)
        pygame.draw.rect(surface, TEXT, tab_rect, 2, border_radius=10)
        label = "Logs" if tab_name == "logs" else "Science Analysis"
        text_surface = small_font.render(label, True, BACKGROUND if is_active else TEXT)
        text_rect = text_surface.get_rect(center=tab_rect.center)
        surface.blit(text_surface, text_rect)

    if dashboard_tab == "analysis":
        draw_science_analysis_view(
            surface,
            title_font,
            small_font,
            rect=rect,
            science_samples=science_samples,
            telemetry=telemetry,
        )
    else:
        draw_log_view(
            surface,
            small_font,
            rect=rect,
            comm_log=comm_log,
        )


def render_main_interface(
    screen: pygame.Surface,
    title_font: pygame.font.Font,
    body_font: pygame.font.Font,
    small_font: pygame.font.Font,
    table_title_font: pygame.font.Font,
    table_caption_font: pygame.font.Font,
    table_value_font: pygame.font.Font,
    state: RuntimeState,
    dashboard_tab: str,
    science_history: tuple[ScienceSample, ...],
    turn: float,
    thrust: float,
) -> None:
    screen.fill(BACKGROUND)

    layout = get_main_layout_rects()

    # Top left: 3D buoy motor representation driven by IMU tilt.
    draw_buoy_3d_motor_view(
        screen,
        rect=layout["buoy"],
        command=state.command,
        imu_accel_text=state.telemetry.imu_accel,
        body_font=small_font,
        small_font=small_font,
    )

    draw_technical_panel(
        screen,
        table_title_font,
        table_caption_font,
        table_value_font,
        rect=layout["status"],
        state=state,
    )

    draw_science_panel(
        screen,
        table_title_font,
        table_caption_font,
        table_value_font,
        small_font,
        rect=layout["science"],
        state=state,
    )

    draw_dashboard_panel(
        screen,
        table_title_font,
        small_font,
        rect=layout["dashboard"],
        dashboard_tab=dashboard_tab,
        comm_log=state.comm_log,
        science_samples=science_history,
        telemetry=state,
    )


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
