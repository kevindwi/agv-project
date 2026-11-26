"""Pengelola rute misi, debouncing deteksi marker, dan pemilihan aksi checkpoint."""

import time
from typing import List, Optional, Set, Tuple

VALID_ACTIONS = ("straight", "left", "right", "stop")


class RouteManager:
    """Mengelola urutan rute ArUco, latch visited IDs, dan debouncing deteksi."""

    def __init__(
        self,
        route_ids: List[int],
        route_actions: List[str],
        confirm_frames: int = 3,
        detection_timeout_sec: float = 0.5,
        trigger_distance_m: float = 0.35,
        slow_distance_m: float = 0.70,
        center_tolerance_m: float = 0.05,
    ):
        if len(route_ids) != len(route_actions):
            raise ValueError(
                f"route_actions ({len(route_actions)}) != route_ids ({len(route_ids)})"
            )
        for idx, act in enumerate(route_actions):
            act_clean = str(act).strip().lower()
            if act_clean not in VALID_ACTIONS:
                raise ValueError(f"Aksi {act_clean} pada index {idx} tidak valid.")

        self.route = [int(i) for i in route_ids]
        self.actions = [str(a).strip().lower() for a in route_actions]
        self.confirm_frames = max(1, confirm_frames)
        self.detection_timeout = max(0.05, detection_timeout_sec)
        self.trigger_distance = trigger_distance_m
        self.slow_distance = slow_distance_m
        self.center_tolerance = center_tolerance_m

        self.route_idx = 0
        self.visited_ids: Set[int] = set()
        self.candidate_id: Optional[int] = None
        self.candidate_count = 0
        self.candidate_last_seen = 0.0

    @property
    def current_waypoint(self) -> int:
        if self.is_completed or not self.route:
            return -1
        return self.route[self.route_idx]

    @property
    def current_action(self) -> str:
        if self.is_completed or not self.route:
            return "stop"
        return self.actions[self.route_idx]

    @property
    def is_completed(self) -> bool:
        return self.route_idx >= len(self.route)

    def advance(self) -> bool:
        """Pindah ke waypoint berikutnya. Mengembalikan True jika masih ada rute, False jika selesai."""
        self.route_idx += 1
        self.reset_candidate()
        return not self.is_completed

    def reset_candidate(self):
        self.candidate_id = None
        self.candidate_count = 0

    def check_timeout(self, now: float):
        """Reset debounce candidate jika waktu jeda melebihi detection_timeout."""
        if (
            self.candidate_id is not None
            and (now - self.candidate_last_seen) > self.detection_timeout
        ):
            self.reset_candidate()

    def process_detection(
        self, marker_id: int, distance_m: float, offset_x: Optional[float] = None
    ) -> Tuple[str, Optional[str]]:
        """Proses satu deteksi marker.

        Alur:
        1. Hanya proses marker yang merupakan target aktif dan belum dikunjungi.
        2. Akumulasi debouncing candidate_count selama marker terlihat dalam timeout window.
        3. Setelah candidate_count >= confirm_frames:
           - Jika distance_m <= trigger_distance_m → TRIGGER (eksekusi aksi).
           - Jika distance_m <= slow_distance_m → SLOW (opsional melambat, tetap pantau).
           - Jika distance_m > slow_distance_m → tetap pantau, candidate tidak di-reset.

        Returns: (event, action)
          event:
            - "IGNORED": bukan target aktif, sudah dikunjungi, atau belum cukup frame konfirmasi
            - "NEED_CENTERING": marker terdeteksi tapi tidak di tengah (hanya jika pose tersedia)
            - "SLOW": marker sudah cukup terkonfirmasi tapi masih terlalu jauh
            - "TRIGGER": jarak mencapai trigger_distance, siap eksekusi aksi
        """
        if self.is_completed:
            return "IGNORED", None

        # Marker sudah pernah dikunjungi (latch)
        if marker_id in self.visited_ids:
            return "IGNORED", None

        # Hanya respons ke target aktif
        if marker_id != self.current_waypoint:
            return "IGNORED", None

        if distance_m <= 0.05:
            return "IGNORED", None

        now = time.monotonic()
        if (
            marker_id == self.candidate_id
            and (now - self.candidate_last_seen) <= self.detection_timeout
        ):
            self.candidate_count += 1
        else:
            self.candidate_id = marker_id
            self.candidate_count = 1
        self.candidate_last_seen = now

        # Belum cukup frame konfirmasi
        if self.candidate_count < self.confirm_frames:
            return "IGNORED", None

        # Pemeriksaan centering jika informasi pose kamera tersedia
        if offset_x is not None and abs(offset_x) > self.center_tolerance:
            return "NEED_CENTERING", None

        # Cukup terkonfirmasi — cek apakah sudah cukup dekat untuk trigger
        if distance_m <= self.trigger_distance:
            # Jarak mencapai pemicu
            self.visited_ids.add(marker_id)
            self.reset_candidate()
            action = self.current_action
            if self.route_idx == len(self.route) - 1:
                action = "stop"
            return "TRIGGER", action

        # Masih terlalu jauh tapi sudah terkonfirmasi → terus pantau, kirim SLOW sebagai sinyal
        # Candidate TIDAK di-reset agar pada frame berikutnya langsung trigger saat jarak cukup
        return "SLOW", None
