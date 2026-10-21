"""Akuisisi kamera dan deteksi garis (centroid) di dalam ROI."""

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class LineDetection:
    """Hasil deteksi garis pada satu frame."""

    found: bool
    normalized_error: float
    debug_image: "np.ndarray"


class LineDetector:
    """Membuka kamera dan menghitung posisi garis pada Region of Interest (ROI)."""

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
        """Deteksi garis di ROI dan kembalikan error ternormalisasi + gambar debug."""
        height, width, _ = frame.shape

        roi_top = int(height * self.roi_top_ratio)
        roi_bottom = int(height * self.roi_bottom_ratio)
        roi_left = int(width * self.roi_left_ratio)
        roi_right = int(width * self.roi_right_ratio)

        roi = frame[roi_top:roi_bottom, roi_left:roi_right]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (3, 3), 0)
        _, binary = cv2.threshold(
            blurred, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU
        )

        # Overlay visualisasi
        debug_image = frame.copy()
        cv2.rectangle(
            debug_image, (roi_left, roi_top), (roi_right, roi_bottom), (0, 255, 0), 2
        )
        cv2.line(
            debug_image, (int(width / 2), 0), (int(width / 2), height), (255, 0, 0), 1
        )

        moments = cv2.moments(binary)
        if moments["m00"] == 0:
            # Tidak ada garis terdeteksi
            return LineDetection(
                found=False, normalized_error=0.0, debug_image=debug_image
            )

        center_x = int(moments["m10"] / moments["m00"])
        image_center = (roi_right - roi_left) / 2.0
        error_pixels = center_x - image_center
        normalized_error = error_pixels / image_center

        centroid_y = roi_top + int((roi_bottom - roi_top) / 2)
        cv2.circle(debug_image, (center_x + roi_left, centroid_y), 5, (100, 0, 255), -1)

        return LineDetection(
            found=True, normalized_error=normalized_error, debug_image=debug_image
        )

    def show_debug(self, debug_image) -> int:
        """Tampilkan window debug OpenCV. Return keycode tombol yang ditekan."""
        cv2.imshow("Line Detection Debug", debug_image)
        return cv2.waitKey(1) & 0xFF

    def release(self) -> None:
        """Lepas kamera."""
        if self.capture is not None and self.capture.isOpened():
            self.capture.release()


def close_debug_windows() -> None:
    """Tutup semua window OpenCV."""
    cv2.destroyAllWindows()
