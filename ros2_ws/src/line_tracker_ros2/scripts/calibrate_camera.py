import cv2
import numpy as np

# 1. Masukkan tebakan ukuran grid (jumlah titik sudut dalam)
# Jika tidak yakin, biarkan skrip mencoba beberapa ukuran di bawah
CANDIDATE_GRIDS = [
    (8, 6),  # Paling umum di template generator
    (9, 6),  # Template ROS standar
    (7, 5),
    (7, 7),  # Papan catur biasa 8x8 kotak
]

SQUARE_SIZE_METERS = 0.025  # 2.5 cm (ubah sesuai ukuran 1 kotak Anda)
CAMERA_INDEX = 2  # /dev/video2 (Logitech)

cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# Flag agar OpenCV lebih kuat terhadap bayangan & perubahan cahaya
flags = (
    cv2.CALIB_CB_ADAPTIVE_THRESH
    + cv2.CALIB_CB_FAST_CHECK
    + cv2.CALIB_CB_NORMALIZE_IMAGE
)

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

active_grid = None
objpoints = []
imgpoints = []
sample_count = 0

print("\n--- PENCARIAN PAPAN CATUR AKTIF ---")
print("Arahkan papan catur menghadap tegak lurus ke kamera...")
print("Pastikan SEMUA bagian papan catur terlihat di layar.\n")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Gagal membaca kamera.")
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    display = frame.copy()
    found = False
    corners = None

    # Jika grid belum terkunci, coba cari ukuran grid yang cocok
    if active_grid is None:
        for grid in CANDIDATE_GRIDS:
            found, corners = cv2.findChessboardCorners(gray, grid, flags)
            if found:
                active_grid = grid
                print(f"\n[SUKSES] Grid papan catur Anda terdeteksi: {active_grid}!")
                # Siapkan koordinat 3D sesuai grid yang ditemukan
                objp = np.zeros((active_grid[0] * active_grid[1], 3), np.float32)
                objp[:, :2] = (
                    np.mgrid[0 : active_grid[0], 0 : active_grid[1]].T.reshape(-1, 2)
                    * SQUARE_SIZE_METERS
                )
                break
    else:
        found, corners = cv2.findChessboardCorners(gray, active_grid, flags)

    if found and corners is not None:
        refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        cv2.drawChessboardCorners(display, active_grid, refined, found)
        cv2.putText(
            display,
            f"TERDETEKSI ({active_grid[0]}x{active_grid[1]})! TEKAN 's'",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2,
        )
    else:
        cv2.putText(
            display,
            "MENCARI PAPAN... (Pastikan semua sudut masuk layar)",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 0, 255),
            2,
        )

    cv2.putText(
        display,
        f"Sampel tersimpan: {sample_count} / 15 (Tekan 'c' jika cukup)",
        (20, 80),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (255, 255, 0),
        2,
    )

    cv2.imshow("Kalibrasi Kamera", display)
    key = cv2.waitKey(1) & 0xFF

    if key == ord("s") and found:
        objpoints.append(objp)
        imgpoints.append(refined)
        sample_count += 1
        print(f"[+] Sampel #{sample_count} berhasil disimpan!")

    elif key == ord("c") and sample_count >= 10:
        print("\nMenghitung kalibrasi... Harap tunggu...")
        ret, mtx, dist, _, _ = cv2.calibrateCamera(
            objpoints, imgpoints, gray.shape[::-1], None, None
        )
        print("\n================== HASIL KALIBRASI ==================")
        print(f"camera_matrix: {mtx.flatten().tolist()}")
        print(f"distortion_coefficients: {dist.flatten().tolist()}")
        print("=====================================================")
        break

    elif key == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
