"""Akuisisi kamera dan deteksi garis dengan Multi-ROI & Otsu Thresholding (Adaptif Pencahayaan)."""

from dataclasses import dataclass
import cv2
import numpy as np


@dataclass
class LineDetection:
    """Hasil deteksi garis dari dua ROI."""
    found: bool
    normalized_error: float
    debug_image: "np.ndarray"
    is_fork: bool = False


class LineDetector:
    """Membuka kamera dan menghitung posisi garis pada 2 Region of Interest (ROI)."""

    def __init__(
        self,
        camera_index: int = 2,
        roi_bot_top: float = 0.70,
        roi_bot_bottom: float = 0.90,
        roi_top_top: float = 0.45,
        roi_top_bottom: float = 0.65,
        roi_left_ratio: float = 0.15,
        roi_right_ratio: float = 0.85,
        min_contour_area: float = 120.0,
    ):
        self.camera_index = camera_index
        self.roi_bot_top = roi_bot_top
        self.roi_bot_bottom = roi_bot_bottom
        self.roi_top_top = roi_top_top
        self.roi_top_bottom = roi_top_bottom
        self.roi_left_ratio = roi_left_ratio
        self.roi_right_ratio = roi_right_ratio
        self.min_contour_area = min_contour_area

        self.capture = cv2.VideoCapture(camera_index, cv2.CAP_V4L2)

    def is_opened(self) -> bool:
        return self.capture.isOpened()

    def read_frame(self):
        success, frame = self.capture.read()
        return frame if success else None

    def _process_roi(self, frame, top_r, bot_r, left_r, right_r):
        """Cropping, Grayscale + Otsu Thresholding (Adaptif), dan ekstraksi kontur."""
        h, w, _ = frame.shape
        y1, y2 = int(h * top_r), int(h * bot_r)
        x1, x2 = int(w * left_r), int(w * right_r)

        roi = frame[y1:y2, x1:x2]
        
        # Kembali ke Grayscale + Otsu Thresholding (Otomatis menyesuaikan cahaya)
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (3, 3), 0)
        _, binary = cv2.threshold(
            blurred, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU
        )

        contours, _ = cv2.findContours(
            binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        valid_contours = [c for c in contours if cv2.contourArea(c) > self.min_contour_area]

        return valid_contours, (x1, y1, x2, y2)

    def detect(self, frame) -> LineDetection:
        """Deteksi garis dengan Multi-ROI dan pemilihan jalur percabangan."""
        height, width, _ = frame.shape
        debug_image = frame.copy()

        contours_top, bounds_top = self._process_roi(
            frame, self.roi_top_top, self.roi_top_bottom, self.roi_left_ratio, self.roi_right_ratio
        )
        contours_bot, bounds_bot = self._process_roi(
            frame, self.roi_bot_top, self.roi_bot_bottom, self.roi_left_ratio, self.roi_right_ratio
        )

        # Gambar Bounding Box ROI pada Debug Image
        cv2.rectangle(debug_image, (bounds_top[0], bounds_top[1]), (bounds_top[2], bounds_top[3]), (255, 255, 0), 1)
        cv2.rectangle(debug_image, (bounds_bot[0], bounds_bot[1]), (bounds_bot[2], bounds_bot[3]), (0, 255, 0), 2)
        cv2.line(debug_image, (int(width / 2), 0), (int(width / 2), height), (255, 0, 0), 1)

        is_fork = len(contours_bot) > 1 or len(contours_top) > 1

        if not contours_bot:
            return LineDetection(
                found=False, normalized_error=0.0, debug_image=debug_image, is_fork=False
            )

        image_center = (bounds_bot[2] - bounds_bot[0]) / 2.0

        # Jika ada percabangan, ambil kontur dengan offset paling belok
        if len(contours_bot) > 1:
            def contour_offset(cnt):
                m = cv2.moments(cnt)
                if m["m00"] == 0:
                    return 0
                return abs((m["m10"] / m["m00"]) - image_center)

            selected_contour = max(contours_bot, key=contour_offset)
        else:
            selected_contour = contours_bot[0]

        moments = cv2.moments(selected_contour)
        if moments["m00"] == 0:
            return LineDetection(
                found=False, normalized_error=0.0, debug_image=debug_image, is_fork=False
            )

        center_x = int(moments["m10"] / moments["m00"])
        error_pixels = center_x - image_center
        normalized_error = error_pixels / image_center

        centroid_y = bounds_bot[1] + int((bounds_bot[3] - bounds_bot[1]) / 2)
        color = (0, 0, 255) if is_fork else (100, 0, 255)
        cv2.circle(debug_image, (center_x + bounds_bot[0], centroid_y), 6, color, -1)

        if is_fork:
            cv2.putText(
                debug_image, "PERCABANGAN DETECTED", (bounds_bot[0], bounds_bot[1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2
            )

        return LineDetection(
            found=True, normalized_error=normalized_error, debug_image=debug_image, is_fork=is_fork
        )

    def release(self) -> None:
        if self.capture is not None and self.capture.isOpened():
            self.capture.release()