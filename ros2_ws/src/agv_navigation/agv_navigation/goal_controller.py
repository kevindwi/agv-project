"""Kontroler navigasi titik-ke-titik planar untuk robot differential drive."""

import math
from typing import Tuple


def normalize_angle(angle: float) -> float:
    """Normalisasikan sudut ke rentang [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(z: float, w: float) -> float:
    """Konversi kuaternion planar 2D (qz, qw) ke sudut yaw dalam radian."""
    return 2.0 * math.atan2(z, w)


class PointToPointController:
    """Menghitung target kecepatan linier & sudut untuk menuju pose sasaran."""

    def __init__(
        self,
        kp_linear: float = 0.5,
        kp_angular: float = 1.2,
        max_linear_speed: float = 0.15,
        min_linear_speed: float = 0.04,
        max_angular_speed: float = 0.60,
        min_angular_speed: float = 0.12,
        max_linear_accel: float = 0.15,
        max_angular_accel: float = 0.80,
        xy_tolerance: float = 0.05,
        yaw_tolerance: float = 0.10,
        heading_tolerance: float = 0.30,
        align_final_yaw: bool = True,
    ):
        self.kp_linear = kp_linear
        self.kp_angular = kp_angular
        self.max_linear_speed = max_linear_speed
        self.min_linear_speed = min_linear_speed
        self.max_angular_speed = max_angular_speed
        self.min_angular_speed = min_angular_speed
        self.max_linear_accel = max_linear_accel
        self.max_angular_accel = max_angular_accel

        self.xy_tolerance = xy_tolerance
        self.yaw_tolerance = yaw_tolerance
        self.heading_tolerance = heading_tolerance
        self.align_final_yaw = align_final_yaw

        self.current_v = 0.0
        self.current_w = 0.0

    def reset(self):
        self.current_v = 0.0
        self.current_w = 0.0

    def calc_angular_vel(self, error: float) -> float:
        """Hitung kecepatan angular proporsional dengan batasan minimum & maksimum."""
        raw = self.kp_angular * error
        mag = max(self.min_angular_speed, min(self.max_angular_speed, abs(raw)))
        return math.copysign(mag, raw)

    def apply_accel_limits(self, target_v: float, target_w: float, dt: float) -> Tuple[float, float]:
        """Slew-rate limiter percepatan linier dan angular untuk mencegah selip."""
        max_dv = self.max_linear_accel * dt
        if self.current_v < target_v:
            self.current_v = min(target_v, self.current_v + max_dv)
        elif self.current_v > target_v:
            self.current_v = max(target_v, self.current_v - max_dv)

        max_dw = self.max_angular_accel * dt
        if self.current_w < target_w:
            self.current_w = min(target_w, self.current_w + max_dw)
        elif self.current_w > target_w:
            self.current_w = max(target_w, self.current_w - max_dw)

        return self.current_v, self.current_w

    def step(
        self,
        current_x: float,
        current_y: float,
        current_yaw: float,
        target_x: float,
        target_y: float,
        target_yaw: float,
        current_state: str,
        dt: float,
    ) -> Tuple[str, float, float, float]:
        """Hitung satu siklus navigasi.

        Returns:
            new_state: str ("ARRIVED", "ROTATING_TO_HEADING", "DRIVING_TO_POINT", "ROTATING_TO_FINAL_HEADING")
            v: float (kecepatan linier yang sudah diramp)
            w: float (kecepatan angular yang sudah diramp)
            dist_remaining: float (sisa jarak ke target)
        """
        dx = target_x - current_x
        dy = target_y - current_y
        rho = math.hypot(dx, dy)
        target_heading = math.atan2(dy, dx)
        heading_error = normalize_angle(target_heading - current_yaw)

        # 1. Posisi XY sudah dalam toleransi
        if rho <= self.xy_tolerance:
            if self.align_final_yaw:
                yaw_error = normalize_angle(target_yaw - current_yaw)
                if abs(yaw_error) <= self.yaw_tolerance:
                    self.reset()
                    return "ARRIVED", 0.0, 0.0, rho
                target_v = 0.0
                target_w = self.calc_angular_vel(yaw_error)
                v, w = self.apply_accel_limits(target_v, target_w, dt)
                return "ROTATING_TO_FINAL_HEADING", v, w, rho
            else:
                self.reset()
                return "ARRIVED", 0.0, 0.0, rho

        # 2. Menuju posisi sasaran
        state = current_state
        if state == "ROTATING_TO_HEADING":
            if abs(heading_error) <= self.heading_tolerance:
                state = "DRIVING_TO_POINT"
                target_v = 0.0
                target_w = 0.0
            else:
                target_v = 0.0
                target_w = self.calc_angular_vel(heading_error)
        elif state == "DRIVING_TO_POINT":
            if abs(heading_error) > 1.05:  # ~60 derajat deviasi
                state = "ROTATING_TO_HEADING"
                target_v = 0.0
                target_w = self.calc_angular_vel(heading_error)
            else:
                facing_scale = max(0.0, math.cos(heading_error))
                raw_linear = max(
                    self.min_linear_speed, min(self.max_linear_speed, self.kp_linear * rho)
                )
                target_v = raw_linear * facing_scale
                target_w = max(
                    -self.max_angular_speed,
                    min(self.max_angular_speed, self.kp_angular * heading_error),
                )
        else:
            state = "ROTATING_TO_HEADING"
            target_v = 0.0
            target_w = self.calc_angular_vel(heading_error)

        v, w = self.apply_accel_limits(target_v, target_w, dt)
        return state, v, w, rho
