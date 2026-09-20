from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class NavigationSolution:
    requested_x: float
    requested_y: float
    measured_x_g: float
    measured_y_g: float
    measured_magnitude_g: float
    corrected_x: float
    corrected_y: float
    direction_error_deg: float
    correction_deg: float
    active: bool
    status: str


class NavigationAssist:
    """Bounded direction correction using zeroed horizontal acceleration.

    The MPU6050 cannot measure steady water-relative velocity or motor thrust.
    This controller therefore uses only the direction of measurable transient
    acceleration and leaves command magnitude under direct operator control.
    """

    def __init__(
        self,
        *,
        gain: float = 0.35,
        max_correction_deg: float = 20.0,
        filter_alpha: float = 0.18,
        gravity_time_constant_s: float = 1.5,
        minimum_command: float = 0.12,
        minimum_acceleration_g: float = 0.015,
        telemetry_timeout_s: float = 0.35,
    ) -> None:
        self.gain = gain
        self.max_correction_deg = max_correction_deg
        self.filter_alpha = filter_alpha
        self.gravity_time_constant_s = gravity_time_constant_s
        self.minimum_command = minimum_command
        self.minimum_acceleration_g = minimum_acceleration_g
        self.telemetry_timeout_s = telemetry_timeout_s

        self.enabled = False
        self.zeroed = False
        self.x_sign = 1.0
        self.y_sign = 1.0
        self.swap_xy = False
        self._raw_x_g = 0.0
        self._raw_y_g = 0.0
        self._raw_z_g = 1.0
        self._gravity_x_g = 0.0
        self._gravity_y_g = 0.0
        self._filtered_x_g = 0.0
        self._filtered_y_g = 0.0
        self._estimated_velocity_x_mps = 0.0
        self._estimated_velocity_y_mps = 0.0
        self._velocity_started_time: float | None = None
        self._last_measurement_time: float | None = None
        self._smoothed_correction_deg = 0.0
        self._tilt_rate_dps = 0.0
        self._tilt_compensating = False

    @staticmethod
    def _clamp(value: float, minimum: float, maximum: float) -> float:
        return max(minimum, min(maximum, value))

    @staticmethod
    def _wrap_degrees(value: float) -> float:
        return (value + 180.0) % 360.0 - 180.0

    def set_gain(self, gain: float) -> None:
        self.gain = self._clamp(float(gain), 0.0, 1.0)

    def set_max_correction(self, degrees: float) -> None:
        self.max_correction_deg = self._clamp(float(degrees), 0.0, 45.0)

    def set_axis_signs(self, invert_x: bool, invert_y: bool, swap_xy: bool = False) -> None:
        new_x_sign = -1.0 if invert_x else 1.0
        new_y_sign = -1.0 if invert_y else 1.0
        if (
            new_x_sign == self.x_sign
            and new_y_sign == self.y_sign
            and bool(swap_xy) == self.swap_xy
        ):
            return
        self.x_sign = new_x_sign
        self.y_sign = new_y_sign
        self.swap_xy = bool(swap_xy)
        self.enabled = False
        self.zeroed = False
        self._filtered_x_g = 0.0
        self._filtered_y_g = 0.0
        self._estimated_velocity_x_mps = 0.0
        self._estimated_velocity_y_mps = 0.0
        self._velocity_started_time = None
        self._smoothed_correction_deg = 0.0

    def update_measurement(
        self,
        accel_x_g: float,
        accel_y_g: float,
        now: float,
        *,
        accel_z_g: float = 1.0,
        gyro_x_dps: float = 0.0,
        gyro_y_dps: float = 0.0,
        stationary: bool = False,
    ) -> None:
        mapped_x = float(accel_y_g) if self.swap_xy else float(accel_x_g)
        mapped_y = float(accel_x_g) if self.swap_xy else float(accel_y_g)
        self._raw_x_g = mapped_x * self.x_sign
        self._raw_y_g = mapped_y * self.y_sign
        self._raw_z_g = float(accel_z_g)
        mapped_gyro_x = float(gyro_y_dps) if self.swap_xy else float(gyro_x_dps)
        mapped_gyro_y = float(gyro_x_dps) if self.swap_xy else float(gyro_y_dps)
        mapped_gyro_x *= self.x_sign
        mapped_gyro_y *= self.y_sign
        self._tilt_rate_dps = math.hypot(mapped_gyro_x, mapped_gyro_y)
        self._tilt_compensating = self._tilt_rate_dps >= 2.0
        previous_measurement_time = self._last_measurement_time
        self._last_measurement_time = now
        dt = 0.05 if previous_measurement_time is None else max(0.001, min(0.25, now - previous_measurement_time))
        gravity_alpha = 1.0 - math.exp(-dt / max(0.1, self.gravity_time_constant_s))
        if not self.zeroed:
            if previous_measurement_time is None:
                self._gravity_x_g = self._raw_x_g
                self._gravity_y_g = self._raw_y_g
            gravity_alpha = max(gravity_alpha, 0.08)
        elif stationary and not self.enabled:
            gravity_alpha = max(gravity_alpha, 0.08)
        if self._tilt_compensating:
            # Rotation makes gravity move across body X/Y. Follow it quickly
            # while the gyro confirms tilt so it is not treated as translation.
            gravity_alpha = max(gravity_alpha, min(0.90, 0.65 + self._tilt_rate_dps / 360.0))

        # Track gravity and low-frequency hull tilt separately. Subtracting
        # this estimate leaves the transient horizontal acceleration used for
        # propulsion-response comparison.
        self._gravity_x_g += gravity_alpha * (self._raw_x_g - self._gravity_x_g)
        self._gravity_y_g += gravity_alpha * (self._raw_y_g - self._gravity_y_g)
        sample_x = self._raw_x_g - self._gravity_x_g
        sample_y = self._raw_y_g - self._gravity_y_g
        alpha = self.filter_alpha
        self._filtered_x_g += alpha * (sample_x - self._filtered_x_g)
        self._filtered_y_g += alpha * (sample_y - self._filtered_y_g)

        # The cockpit display may use this automatic baseline immediately,
        # but velocity integration and active correction still require an
        # explicit operator zero at rest.
        if not self.zeroed:
            self._estimated_velocity_x_mps = 0.0
            self._estimated_velocity_y_mps = 0.0
            return

        # A leaky integrator provides a useful short-term speed indication
        # without implying that an accelerometer can measure steady velocity.
        # It intentionally decays quickly when propulsion is idle and is
        # bounded to reject runaway integration after shocks or tilt errors.
        decay_time_s = 1.0 if stationary else 4.0
        decay = math.exp(-dt / decay_time_s)
        gravity_mps2 = 9.80665
        self._estimated_velocity_x_mps = self._clamp(
            (self._estimated_velocity_x_mps + self._filtered_x_g * gravity_mps2 * dt) * decay,
            -3.0,
            3.0,
        )
        self._estimated_velocity_y_mps = self._clamp(
            (self._estimated_velocity_y_mps + self._filtered_y_g * gravity_mps2 * dt) * decay,
            -3.0,
            3.0,
        )
        if self._velocity_started_time is None:
            self._velocity_started_time = now

    def zero(self) -> bool:
        if self._last_measurement_time is None:
            return False
        self._gravity_x_g = self._raw_x_g
        self._gravity_y_g = self._raw_y_g
        self._filtered_x_g = 0.0
        self._filtered_y_g = 0.0
        self._estimated_velocity_x_mps = 0.0
        self._estimated_velocity_y_mps = 0.0
        self._velocity_started_time = self._last_measurement_time
        self._smoothed_correction_deg = 0.0
        self.zeroed = True
        return True

    def measurement_is_fresh(self, now: float) -> bool:
        return (
            self._last_measurement_time is not None
            and now - self._last_measurement_time <= self.telemetry_timeout_s
        )

    @property
    def measured_x_g(self) -> float:
        return self._filtered_x_g

    @property
    def measured_y_g(self) -> float:
        return self._filtered_y_g

    @property
    def measured_magnitude_g(self) -> float:
        return math.hypot(self._filtered_x_g, self._filtered_y_g)

    @property
    def roll_deg(self) -> float:
        return math.degrees(math.atan2(self._raw_x_g, math.hypot(self._raw_y_g, self._raw_z_g)))

    @property
    def pitch_deg(self) -> float:
        return math.degrees(math.atan2(self._raw_y_g, math.hypot(self._raw_x_g, self._raw_z_g)))

    @property
    def estimated_velocity_x_mps(self) -> float:
        return self._estimated_velocity_x_mps

    @property
    def estimated_velocity_y_mps(self) -> float:
        return self._estimated_velocity_y_mps

    @property
    def estimated_speed_mps(self) -> float:
        return math.hypot(self._estimated_velocity_x_mps, self._estimated_velocity_y_mps)

    @property
    def tilt_rate_dps(self) -> float:
        return self._tilt_rate_dps

    @property
    def tilt_compensating(self) -> bool:
        return self._tilt_compensating

    def velocity_confidence_percent(self, now: float) -> int:
        if not self.zeroed or not self.measurement_is_fresh(now) or self._velocity_started_time is None:
            return 0
        integration_age = max(0.0, now - self._velocity_started_time)
        return round(max(10.0, 70.0 - integration_age * 4.0))

    def set_enabled(self, enabled: bool, now: float) -> bool:
        if enabled and (not self.zeroed or not self.measurement_is_fresh(now)):
            self.enabled = False
            return False
        self.enabled = bool(enabled)
        if not self.enabled:
            self._smoothed_correction_deg = 0.0
        return self.enabled

    def solve(self, requested_x: float, requested_y: float, now: float) -> NavigationSolution:
        requested_x = self._clamp(float(requested_x), -1.0, 1.0)
        requested_y = self._clamp(float(requested_y), -1.0, 1.0)
        original_x = requested_x
        original_y = requested_y
        command_magnitude = min(1.0, math.hypot(requested_x, requested_y))
        measured_magnitude = math.hypot(self._filtered_x_g, self._filtered_y_g)

        status = "Assist off"
        active = False
        error_deg = 0.0
        correction_deg = 0.0

        if self.enabled and not self.measurement_is_fresh(now):
            self.enabled = False
            self._smoothed_correction_deg = 0.0
            status = "Assist stopped: IMU telemetry stale"
        elif self.enabled and command_magnitude < self.minimum_command:
            self._smoothed_correction_deg *= 0.75
            status = "Assist armed; waiting for controller input"
        elif self.enabled and measured_magnitude < self.minimum_acceleration_g:
            self._smoothed_correction_deg *= 0.85
            status = "Assist armed; acceleration below threshold"
        elif self.enabled:
            desired_angle = math.atan2(requested_x, requested_y)
            measured_angle = math.atan2(self._filtered_x_g, self._filtered_y_g)
            error_deg = self._wrap_degrees(math.degrees(desired_angle - measured_angle))
            target_correction = self._clamp(
                self.gain * error_deg,
                -self.max_correction_deg,
                self.max_correction_deg,
            )
            self._smoothed_correction_deg = (
                0.75 * self._smoothed_correction_deg + 0.25 * target_correction
            )
            correction_deg = self._smoothed_correction_deg
            corrected_angle = desired_angle + math.radians(correction_deg)
            requested_x = command_magnitude * math.sin(corrected_angle)
            requested_y = command_magnitude * math.cos(corrected_angle)
            active = True
            status = "Correcting acceleration direction"

        return NavigationSolution(
            requested_x=original_x,
            requested_y=original_y,
            measured_x_g=self._filtered_x_g,
            measured_y_g=self._filtered_y_g,
            measured_magnitude_g=measured_magnitude,
            corrected_x=self._clamp(requested_x, -1.0, 1.0),
            corrected_y=self._clamp(requested_y, -1.0, 1.0),
            direction_error_deg=error_deg,
            correction_deg=correction_deg,
            active=active,
            status=status,
        )


class GpsSpeedTracker:
    """Derive smoothed ground speed while treating missing GPS as normal."""

    def __init__(self, *, stale_after_s: float = 3.0, max_speed_mps: float = 8.0) -> None:
        self.stale_after_s = stale_after_s
        self.max_speed_mps = max_speed_mps
        self._last_position: tuple[float, float] | None = None
        self._last_position_time: float | None = None
        self._last_fix_time: float | None = None
        self._speed_mps: float | None = None
        self._valid = False
        self._fixes: list[tuple[float, float, float]] = []

    @staticmethod
    def _distance_m(first: tuple[float, float], second: tuple[float, float]) -> float:
        lat1, lon1 = map(math.radians, first)
        lat2, lon2 = map(math.radians, second)
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        value = math.sin(dlat / 2.0) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2.0) ** 2
        return 6_371_000.0 * 2.0 * math.atan2(math.sqrt(value), math.sqrt(max(0.0, 1.0 - value)))

    def update(self, latitude: float, longitude: float, now: float) -> None:
        latitude = float(latitude)
        longitude = float(longitude)
        now = float(now)
        if not (-90.0 <= latitude <= 90.0 and -180.0 <= longitude <= 180.0):
            self.invalidate()
            return

        current = (latitude, longitude)
        self._fixes.append((now, latitude, longitude))
        self._fixes = [fix for fix in self._fixes[-12:] if now - fix[0] <= 5.0]
        reference = next((fix for fix in self._fixes if now - fix[0] >= 2.0), None)
        if reference is not None:
            elapsed = now - reference[0]
            distance = self._distance_m((reference[1], reference[2]), current)
            # Consumer GPS jitter is often larger than the buoy's displacement
            # between individual fixes. A short window and stationary radius
            # prevent that noise from being displayed as convincing speed.
            candidate = 0.0 if distance < 1.5 else distance / elapsed
            if candidate <= self.max_speed_mps:
                self._speed_mps = candidate if self._speed_mps is None else 0.65 * self._speed_mps + 0.35 * candidate
                self._valid = True
        self._last_position = current
        self._last_position_time = now
        self._last_fix_time = now

    def invalidate(self) -> None:
        self._valid = False

    def speed_mps(self, now: float) -> float | None:
        if (
            not self._valid
            or self._speed_mps is None
            or self._last_fix_time is None
            or now - self._last_fix_time > self.stale_after_s
        ):
            return None
        return self._speed_mps
