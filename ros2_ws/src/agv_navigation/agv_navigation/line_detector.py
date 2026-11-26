"""Modul deteksi garis OpenCV untuk line follower."""

from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np


@dataclass
class LineDetectionResult:
    line_found: bool
    normalized_error: float
    cx_global: Optional[int]
    cy_global: Optional[int]
    debug_image: Optional[np.ndarray]


class LineDetector:
    """Deteksi garis hitam/gelap pada permukaan terang menggunakan ROI dan Otsu thresholding."""

    def __init__(
        self,
        roi_top_ratio: float = 0.70,
        roi_bottom_ratio: float = 0.90,
        roi_left_ratio: float = 0.20,
        roi_right_ratio: float = 0.80,
    ):
        self.roi_top = roi_top_ratio
        self.roi_bottom = roi_bottom_ratio
        self.roi_left = roi_left_ratio
        self.roi_right = roi_right_ratio

    def detect(
        self,
        frame: np.ndarray,
        create_debug: bool = False,
        status_text: str = "",
        linear_speed: float = 0.0,
        wide_roi: bool = False,
    ) -> LineDetectionResult:
        """Deteksi garis pada frame dan kembalikan hasil deteksi serta error ternormalisasi."""
        h, w, _ = frame.shape
        r_top = int(h * self.roi_top)
        r_bottom = int(h * self.roi_bottom)
        left_ratio = min(self.roi_left, 0.05) if wide_roi else self.roi_left
        right_ratio = max(self.roi_right, 0.95) if wide_roi else self.roi_right
        r_left = int(w * left_ratio)
        r_right = int(w * right_ratio)

        roi = frame[r_top:r_bottom, r_left:r_right]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # Deteksi garis gelap/hitam di atas permukaan terang
        _, binary = cv2.threshold(
            blurred, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU
        )

        moments = cv2.moments(binary)
        line_found = moments["m00"] > 100
        normalized_error = 0.0
        cx_global = None
        cy_global = int((r_top + r_bottom) / 2)
        roi_width = r_right - r_left

        if line_found:
            cx_local = int(moments["m10"] / moments["m00"])
            cx_global = r_left + cx_local
            # Error ternormalisasi: -1.0 (kiri) s/d +1.0 (kanan)
            normalized_error = (cx_local - (roi_width / 2.0)) / (roi_width / 2.0)

        debug_img = None
        if create_debug:
            debug_img = frame.copy()
            cv2.rectangle(
                debug_img, (r_left, r_top), (r_right, r_bottom), (0, 255, 0), 2
            )
            if line_found and cx_global is not None:
                cv2.circle(debug_img, (cx_global, cy_global), 6, (0, 0, 255), -1)
                text = f"Err: {normalized_error:.2f} | Speed: {linear_speed:.2f}"
                cv2.putText(
                    debug_img,
                    text,
                    (r_left, r_top - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2,
                )
            elif status_text:
                color = (0, 0, 255) if status_text == "LINE LOST" else (0, 255, 255)
                cv2.putText(
                    debug_img,
                    status_text,
                    (r_left, r_top - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    color,
                    2,
                )

        return LineDetectionResult(
            line_found=line_found,
            normalized_error=normalized_error,
            cx_global=cx_global,
            cy_global=cy_global,
            debug_image=debug_img,
        )
