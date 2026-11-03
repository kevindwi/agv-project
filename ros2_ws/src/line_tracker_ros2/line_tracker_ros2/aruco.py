"""Deteksi marker ArUco fisik untuk identifikasi waypoint & persimpangan."""

import time
from typing import Optional
import cv2
import cv2.aruco as aruco
import numpy as np


class ArucoDetector:
    """Detektor ArUco dengan filter debouncing & cooldown untuk navigasi AGV."""

    def __init__(
        self,
        dictionary_id: int = aruco.DICT_4X4_50,
        consecutive_frames: int = 3,
        cooldown_seconds: float = 2.5,
    ):
        self.dictionary_id = dictionary_id
        self.consecutive_frames = consecutive_frames
        self.cooldown_seconds = cooldown_seconds

        # Dukungan kompatibilitas OpenCV 4.6 vs 4.7+
        if hasattr(aruco, "getPredefinedDictionary"):
            self.dictionary = aruco.getPredefinedDictionary(dictionary_id)
        else:
            self.dictionary = aruco.Dictionary_get(dictionary_id)

        if hasattr(aruco, "DetectorParameters_create"):
            self.parameters = aruco.DetectorParameters_create()
        else:
            self.parameters = aruco.DetectorParameters()

        # State tracking debouncing
        self.candidate_id: Optional[int] = None
        self.candidate_count: int = 0

        # State tracking cooldown / hysteresis
        self.last_confirmed_id: Optional[int] = None
        self.last_confirmed_time: float = 0.0

    def detect(self, frame: np.ndarray, debug_image: Optional[np.ndarray] = None) -> Optional[int]:
        """Deteksi marker ArUco pada frame citra.

        Args:
            frame: Citra BGR masukan dari kamera.
            debug_image: Citra visualisasi opsional untuk digambar anotasi deteksi.

        Returns:
            ID marker terkonfirmasi (int) jika lolos filter debounce & cooldown, atau None.
        """
        if frame is None:
            return None

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(
            gray, self.dictionary, parameters=self.parameters
        )

        current_detected_id: Optional[int] = None

        if ids is not None and len(ids) > 0:
            # Ambil marker pertama (atau paling dominan)
            current_detected_id = int(ids[0][0])

            # Gambar visualisasi pada debug image jika disediakan
            if debug_image is not None:
                aruco.drawDetectedMarkers(debug_image, corners, ids)
                # Tambahkan label ID yang jelas
                c = corners[0][0]
                center_x = int(c[:, 0].mean())
                center_y = int(c[:, 1].mean())
                cv2.putText(
                    debug_image,
                    f"ArUco ID: {current_detected_id}",
                    (center_x - 30, center_y - 20),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 255),
                    2,
                )

        now = time.time()

        # Logika Debouncing: harus terdeteksi konsisten selama N frame
        if current_detected_id is not None:
            if current_detected_id == self.candidate_id:
                self.candidate_count += 1
            else:
                self.candidate_id = current_detected_id
                self.candidate_count = 1

            # Evaluasi apakah sudah mencapai batas threshold konfirmasi
            if self.candidate_count >= self.consecutive_frames:
                # Periksa cooldown timer agar tidak memicu sinyal berulang saat robot melintas
                is_different_marker = current_detected_id != self.last_confirmed_id
                is_cooldown_expired = (now - self.last_confirmed_time) > self.cooldown_seconds

                if is_different_marker or is_cooldown_expired:
                    self.last_confirmed_id = current_detected_id
                    self.last_confirmed_time = now

                    if debug_image is not None:
                        cv2.putText(
                            debug_image,
                            f"CONFIRMED ID: {current_detected_id}",
                            (20, 40),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.8,
                            (0, 255, 0),
                            2,
                        )

                    return current_detected_id
        else:
            # Tidak ada marker terdeteksi pada frame ini, reset debounce
            self.candidate_id = None
            self.candidate_count = 0

        return None
