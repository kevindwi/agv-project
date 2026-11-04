"""Akuisisi kamera dan deteksi garis (centroid) di dalam ROI.

Selain posisi garis (centroid), modul ini juga mendeteksi persimpangan
(intersection / T-junction / crossbar) berdasarkan lebar relatif kontur
blob putih pada ROI. Hasil deteksi dikemas dalam :class:`LineDetection`.
"""

from dataclasses import dataclass
from dataclasses import field

import cv2
import numpy as np


@dataclass
class LineDetection:
    """Hasil deteksi garis pada satu frame.

    Atribut:
        found: True bila blob garis terdeteksi di dalam ROI.
        normalized_error: Selisih antara centroid garis dan tengah ROI,
            dinormalisasi ke rentang [-1.0, +1.0].
        intersection_detected: True bila ROI menunjukkan pola persimpangan
            (crossbar / T-junction) — blob jauh lebih lebar dari biasanya.
        debug_image: Salinan frame BGR dengan overlay ROI, centroid, dan
            anotasi intersection untuk visualisasi.
    """

    found: bool
    normalized_error: float
    intersection_detected: bool
    debug_image: "np.ndarray"


class LineDetector:
    """Membuka kamera dan menghitung posisi garis pada Region of Interest (ROI).

    Selain menghitung centroid garis, detektor juga mendeteksi persimpangan
    (crossbar / T-junction) secara pasif setiap kali :meth:`detect` dipanggil.
    Tidak ada state internal; setiap frame dievaluasi secara independen.
    """

    # Rasio lebar blob terhadap lebar ROI yang dianggap sebagai persimpangan.
    # Jika lebar bounding-rect kontur terbesar > threshold ini, berarti ada
    # garis horizontal (crossbar) yang memotong ROI.
    INTERSECTION_WIDTH_RATIO: float = 0.55

    # Tinggi minimum blob (piksel) agar tidak dianggap noise saat cek intersection.
    INTERSECTION_MIN_HEIGHT_PX: int = 4

    def __init__(
        self,
        camera_index: int = 2,
        roi_top_ratio: float = 0.70,
        roi_bottom_ratio: float = 0.90,
        roi_left_ratio: float = 0.20,
        roi_right_ratio: float = 0.80,
    ):
        self.camera_index = camera_index
        self.roi_top_ratio = roi_top_ratio
        self.roi_bottom_ratio = roi_bottom_ratio
        self.roi_left_ratio = roi_left_ratio
        self.roi_right_ratio = roi_right_ratio

        self.capture = cv2.VideoCapture(camera_index, cv2.CAP_V4L2)

    def is_opened(self) -> bool:
        """True bila kamera berhasil dibuka."""
        return self.capture.isOpened()

    def read_frame(self):
        """Ambil satu frame BGR. Return None bila gagal."""
        success, frame = self.capture.read()
        if not success:
            return None
        return frame

    def detect(self, frame) -> LineDetection:
        """Deteksi garis di ROI.

        Mengembalikan :class:`LineDetection` yang berisi:
        - Centroid garis (error ternormalisasi untuk PID).
        - Flag ``intersection_detected`` yang disetel True bila kamera melihat
          persimpangan (crossbar, T-junction, atau cross-junction) di ROI.

        **Algoritma deteksi persimpangan (pasif, tanpa state):**
        Setelah threshold Otsu, semua kontur dievaluasi. Bila *bounding rect*
        kontur terbesar memiliki lebar ≥ ``INTERSECTION_WIDTH_RATIO × lebar ROI``,
        maka frame dianggap memperlihatkan persimpangan. Heuristik ini efektif
        untuk lintasan H-track (crossbar jauh lebih lebar dari segmen vertikal).
        """
        height, width, _ = frame.shape

        roi_top = int(height * self.roi_top_ratio)
        roi_bottom = int(height * self.roi_bottom_ratio)
        roi_left = int(width * self.roi_left_ratio)
        roi_right = int(width * self.roi_right_ratio)
        roi_width = roi_right - roi_left

        roi = frame[roi_top:roi_bottom, roi_left:roi_right]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (3, 3), 0)
        _, binary = cv2.threshold(
            blurred, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU
        )

        # ----- Overlay visualisasi -----
        debug_image = frame.copy()
        cv2.rectangle(
            debug_image, (roi_left, roi_top), (roi_right, roi_bottom), (0, 255, 0), 2
        )
        cv2.line(
            debug_image, (int(width / 2), 0), (int(width / 2), height), (255, 0, 0), 1
        )

        # ----- Deteksi persimpangan via kontur -----
        intersection_detected = self._check_intersection(binary, roi_width)

        if intersection_detected:
            # Anotasi persimpangan di debug image (batas biru tebal)
            cv2.rectangle(
                debug_image,
                (roi_left, roi_top),
                (roi_right, roi_bottom),
                (255, 80, 0),
                4,
            )
            cv2.putText(
                debug_image,
                "INTERSECTION",
                (roi_left, roi_top - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 80, 0),
                2,
            )

        # ----- Centroid untuk PID -----
        moments = cv2.moments(binary)
        if moments["m00"] == 0:
            return LineDetection(
                found=False,
                normalized_error=0.0,
                intersection_detected=intersection_detected,
                debug_image=debug_image,
            )

        center_x = int(moments["m10"] / moments["m00"])
        image_center = roi_width / 2.0
        error_pixels = center_x - image_center
        normalized_error = error_pixels / image_center

        centroid_y = roi_top + int((roi_bottom - roi_top) / 2)
        cv2.circle(debug_image, (center_x + roi_left, centroid_y), 5, (100, 0, 255), -1)

        return LineDetection(
            found=True,
            normalized_error=normalized_error,
            intersection_detected=intersection_detected,
            debug_image=debug_image,
        )

    # ------------------------------------------------------------------ #
    # Private helpers                                                       #
    # ------------------------------------------------------------------ #

    def _check_intersection(self, binary: np.ndarray, roi_width: int) -> bool:
        """Kembalikan True jika ada kontur yang sangat lebar di ROI.

        Strategi: temukan kontur terbesar (by bounding-rect area); jika
        lebarnya ≥ ``INTERSECTION_WIDTH_RATIO × roi_width`` dan tingginya
        ≥ ``INTERSECTION_MIN_HEIGHT_PX``, dianggap persimpangan.
        """
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return False

        # Kontur terbesar berdasarkan area bounding rect
        largest = max(contours, key=lambda c: cv2.boundingRect(c)[2] * cv2.boundingRect(c)[3])
        _, _, w, h = cv2.boundingRect(largest)

        return (
            h >= self.INTERSECTION_MIN_HEIGHT_PX
            and w >= roi_width * self.INTERSECTION_WIDTH_RATIO
        )

    def release(self) -> None:
        """Lepas kamera."""
        if self.capture is not None and self.capture.isOpened():
            self.capture.release()
