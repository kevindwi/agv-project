"""Pengendali manuver belok dan penguncian ulang garis berbasis odometri."""

import math
import time
from typing import Optional, Tuple


def angle_distance(angle: float, reference: float) -> float:
    """Hitung selisih sudut terpendek dalam rentang [-pi, pi]."""
    return math.atan2(math.sin(angle - reference), math.cos(angle - reference))


class TurnManeuverController:
    """Mengelola state manuver belok rotasi di tempat dan pencarian garis cabang."""

    def __init__(
        self,
        turn_angle_rad: float = 1.35,
        turn_speed_radps: float = 0.35,
        min_turn_speed_radps: float = 0.12,
        yaw_kp: float = 1.2,
        seek_kp: float = 1.2,
        turn_timeout_sec: float = 12.0,
        reacquire_frames: int = 3,
        reacquire_error_threshold: float = 0.15,
        reacquire_search_angle_rad: float = 0.60,
    ):
        self.turn_angle = abs(turn_angle_rad)
        self.turn_speed = max(0.05, abs(turn_speed_radps))
        self.min_turn_speed = min(self.turn_speed, max(0.05, abs(min_turn_speed_radps)))
        self.yaw_kp = max(0.1, float(yaw_kp))
        self.seek_kp = max(0.1, float(seek_kp))
        self.turn_timeout = max(1.0, turn_timeout_sec)
        self.reacquire_frames = max(1, reacquire_frames)
        self.reacquire_error_threshold = reacquire_error_threshold
        self.reacquire_search_angle = abs(reacquire_search_angle_rad)

        self.turn_direction = 0
        self.turn_start_yaw: Optional[float] = None
        self.turn_started_at: Optional[float] = None
        self.seek_phase = 0  # 0: searah belok, 1: sweep balik berlawanan arah
        self.seek_phase_yaw: Optional[float] = None
        self.seek_phase_started_at: Optional[float] = None
        self.seek_started_at: Optional[float] = None
        self.was_line_found = False
        self.reacquire_count = 0

    def start_turn(self, direction: int, current_yaw: float):
        """Mulai manuver belok baru (+1: kiri, -1: kanan)."""
        self.turn_direction = direction
        self.turn_start_yaw = current_yaw
        self.turn_started_at = time.monotonic()
        self.seek_phase = 0
        self.seek_phase_yaw = None
        self.seek_phase_started_at = None
        self.seek_started_at = None
        self.was_line_found = False
        self.reacquire_count = 0

    def reset(self):
        """Reset state manuver."""
        self.turn_start_yaw = None
        self.turn_started_at = None
        self.seek_phase = 0
        self.seek_phase_yaw = None
        self.seek_phase_started_at = None
        self.seek_started_at = None
        self.was_line_found = False
        self.reacquire_count = 0

    def update_turn(
        self, current_yaw: float, odom_received: bool, line_found: bool = False
    ) -> Tuple[str, float]:
        """Jalankan fase pemutaran robot hingga sudut target tercapai.

        Jika sudah melewati sudut minimum (70% target) dan garis cabang sudah
        terlihat kamera, langsung masuk ke fase LCTT agar tidak overshoot.

        Returns: (event, angular_z)
          event: "TURNING", "REACHED_TARGET", atau "TIMEOUT"
        """
        now = time.monotonic()
        if self.turn_started_at is None:
            self.turn_started_at = now
        if self.turn_start_yaw is None:
            self.turn_start_yaw = current_yaw

        if now - self.turn_started_at > self.turn_timeout:
            return ("TIMEOUT", 0.0)

        if odom_received:
            turned = abs(angle_distance(current_yaw, self.turn_start_yaw))
        else:
            turned = (now - self.turn_started_at) * self.turn_speed

        min_blind_angle = max(0.95, self.turn_angle * 0.80)
        if turned >= self.turn_angle or (turned >= min_blind_angle and line_found):
            self.seek_phase = 0
            self.seek_phase_yaw = current_yaw
            self.seek_phase_started_at = now
            self.seek_started_at = now
            self.was_line_found = line_found
            self.reacquire_count = 0
            return ("REACHED_TARGET", 0.0)

        if odom_received:
            remaining = max(0.0, self.turn_angle - turned)
            speed = max(
                self.min_turn_speed, min(self.turn_speed, self.yaw_kp * remaining)
            )
        else:
            speed = self.turn_speed

        angular_z = self.turn_direction * speed
        return ("TURNING", angular_z)

    def update_seek(
        self,
        current_yaw: float,
        odom_received: bool,
        line_found: bool,
        normalized_error: float,
        max_angular_speed: float = 1.0,
    ) -> Tuple[str, float]:
        """Jalankan fase penguncian garis cabang setelah sudut coarse tercapai (LCTT).

        - Jika garis belum terlihat: lakukan pencarian 2 arah (maju searah belok
          sebentar untuk kasus kurang putar, lalu sweep balik berlawanan arah
          bila terjadi overshoot).
        - Begitu garis terlihat: gunakan kontrol proporsional terhadap error garis
          agar robot tepat terpusat tanpa overshoot.

        Returns: (event, angular_z)
          event: "SEEKING", "COMPLETE", atau "FAILED"
        """
        now = time.monotonic()
        if self.seek_started_at is None:
            self.seek_started_at = now
        if self.seek_phase_yaw is None:
            self.seek_phase_yaw = current_yaw
        if self.seek_phase_started_at is None:
            self.seek_phase_started_at = now

        if now - self.seek_started_at > self.turn_timeout:
            return ("FAILED", 0.0)

        if not line_found:
            self.reacquire_count = 0
            if self.was_line_found:
                # Garis sempat terlihat lalu lepas -> mulai sweep ulang dari posisi sekarang
                self.seek_phase = 0
                self.seek_phase_yaw = current_yaw
                self.seek_phase_started_at = now
                self.was_line_found = False

            seek_speed = max(self.min_turn_speed, self.turn_speed * 0.40)
            forward_limit = min(0.25, self.reacquire_search_angle * 0.45)

            if odom_received:
                searched = abs(angle_distance(current_yaw, self.seek_phase_yaw))
            else:
                searched = (now - self.seek_phase_started_at) * seek_speed

            if self.seek_phase == 0:
                if searched >= forward_limit:
                    self.seek_phase = 1
                    self.seek_phase_yaw = current_yaw
                    self.seek_phase_started_at = now
                    return ("SEEKING", -self.turn_direction * seek_speed)
                return ("SEEKING", self.turn_direction * seek_speed)

            # Phase 1: sweep balik berlawanan arah untuk menangkap garis jika overshoot
            reverse_limit = forward_limit + self.reacquire_search_angle
            if searched >= reverse_limit:
                return ("FAILED", 0.0)
            return ("SEEKING", -self.turn_direction * seek_speed)

        self.was_line_found = True

        # Garis terlihat -> Closed-loop visual centering (LCTT)
        if abs(normalized_error) <= self.reacquire_error_threshold:
            self.reacquire_count += 1
        else:
            self.reacquire_count = 0

        if self.reacquire_count >= self.reacquire_frames:
            self.reset()
            return ("COMPLETE", 0.0)

        max_seek_speed = min(abs(max_angular_speed) * 0.5, self.turn_speed)
        angular_z = -self.seek_kp * normalized_error
        angular_z = max(-max_seek_speed, min(max_seek_speed, angular_z))
        return ("SEEKING", angular_z)
