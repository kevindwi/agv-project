#!/usr/bin/env python3
"""Script utilitas untuk menghasilkan file gambar marker ArUco yang siap dicetak."""

import os
import cv2
import cv2.aruco as aruco
import numpy as np


def generate_marker(
    marker_id: int,
    dictionary_id: int = aruco.DICT_4X4_50,
    size_pixels: int = 600,
    border_pixels: int = 50,
    output_dir: str = "markers",
    label: str = "",
) -> str:
    """Generate marker image dengan label teks dan margin putih."""
    os.makedirs(output_dir, exist_ok=True)

    if hasattr(aruco, "getPredefinedDictionary"):
        dictionary = aruco.getPredefinedDictionary(dictionary_id)
    else:
        dictionary = aruco.Dictionary_get(dictionary_id)

    # Buat citra marker ArUco
    marker_img = aruco.drawMarker(dictionary, marker_id, size_pixels)

    # Tambahkan margin putih di sekeliling marker (quiet zone)
    total_h = size_pixels + 2 * border_pixels + 60
    total_w = size_pixels + 2 * border_pixels
    canvas = np.full((total_h, total_w), 255, dtype=np.uint8)

    canvas[border_pixels : border_pixels + size_pixels, border_pixels : border_pixels + size_pixels] = marker_img

    # Tambahkan teks label di bagian bawah marker
    text = f"ID: {marker_id} {label}".strip()
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 1.0
    thickness = 2
    text_size = cv2.getTextSize(text, font, font_scale, thickness)[0]
    text_x = (total_w - text_size[0]) // 2
    text_y = total_h - 20

    cv2.putText(canvas, text, (text_x, text_y), font, font_scale, 0, thickness)

    output_path = os.path.join(output_dir, f"aruco_id_{marker_id}.png")
    cv2.imwrite(output_path, canvas)
    return output_path


def main():
    output_directory = os.path.join(os.path.dirname(__file__), "..", "..", "..", "..", "docs", "markers")
    output_directory = os.path.abspath(output_directory)
    print(f"Menghasilkan marker ArUco ke direktori: {output_directory}")

    markers_to_generate = [
        (1, "(Persimpangan 1)"),
        (2, "(Persimpangan 2)"),
        (3, "(Persimpangan 3)"),
        (10, "(Checkpoint CP1)"),
        (20, "(Checkpoint CP2)"),
        (30, "(Checkpoint CP3)"),
    ]

    for m_id, label in markers_to_generate:
        path = generate_marker(
            marker_id=m_id,
            dictionary_id=aruco.DICT_4X4_50,
            size_pixels=600,
            border_pixels=60,
            output_dir=output_directory,
            label=label,
        )
        print(f" -> {path}")

    print("\nSemua marker berhasil dibuat! Anda dapat mencetak file PNG tersebut (skala 10 cm x 10 cm).")


if __name__ == "__main__":
    main()
