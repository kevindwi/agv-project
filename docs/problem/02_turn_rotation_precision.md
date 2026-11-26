# Masalah 2: Presisi Rotasi Belok 90° Tanpa Overshoot atau Undershoot

Dokumen ini menganalisis penyebab robot sering berputar melebihi 90° (overshoot) atau kurang dari 90° (undershoot) saat bermanuver di persimpangan, serta merinci solusi teknik kendali yang lebih andal dan stabil.

---

## 1. Analisis Masalah: Mengapa Rotasi Sering Tidak Pas 90°?

Implementasi saat ini di `turn_maneuver.py` membagi proses belok menjadi dua fase:
1. **Fase Belok Utama (`update_turn`):** Robot memutar di tempat dengan kecepatan konstan $\omega = 0{,}40\text{ rad/s}$ hingga integrasi yaw odometri mencapai $\theta = 1{,}57\text{ rad}$ (90°).
2. **Fase Pencarian Garis (`update_seek`):** Setelah mencapai 90°, robot masuk ke mode pencarian garis cabang dengan tetap memutar searah pada kecepatan $0{,}18\text{ rad/s}$ ($0{,}45 \times \text{turn\_speed}$) hingga mendeteksi garis dalam `reacquire_error_threshold`.

### 1.1 Penyebab Utama Overshoot (> 90°)
- **Fase Pencarian Terus Memutar:** Begitu sudut 90° tercapai, robot **tidak berhenti**, melainkan terus berputar sambil mengecek frame kamera.
- **Latensi Frame Kamera:** Kamera berjalan pada 15–20 FPS (jeda antar-frame sekitar 50–70 ms). Di kecepatan $0{,}18\text{ rad/s}$, dalam 4 frame konfirmasi robot sudah berputar tambahan sebesar $10^\circ - 18^\circ$.
- **Batas Jendela Pencarian (`reacquire_search_angle`):** Parameter ini mengizinkan robot berputar ekstra hingga $0{,}55\text{ rad}$ (~31,5°). Akibatnya, total sudut belok di lapangan sering kali menjadi:
  $$\text{Sudut Akhir} = 90^\circ + (15^\circ \text{ s/d } 30^\circ) = 105^\circ - 120^\circ$$

### 1.2 Penyebab Utama Undershoot (< 90°)
- **Slip Roda saat Belok di Tempat:** Pada robot diferensial (*skid-steering* pada roda kastor atau gesekan ban), roda sering mengalami slip saat berputar di tempat. Odometri encoder mencatat robot sudah berputar 90°, padahal sudut fisik robot baru mencapai 75°–80°.
- **Garis Cabang Berada di Luar ROI:** Karena sudut fisik belum cukup, garis cabang belum masuk ke Region of Interest (ROI) kamera. Robot menganggap garis hilang dan memicu timeout (`TURN_FAILED`).

### 1.3 Ketergantungan Callback Kamera pada Loop Belok
Saat ini, logika manuver belok di `line_follower_node.py` dipanggil **di dalam `image_callback`**. Jika proses konversi gambar OpenCV tertahan atau mengalami *frame drop*, perintah putar terhenti sejenak, memicu watchdog firmware atau menyebabkan inersia putar yang tidak konsisten.

---

## 2. Solusi Teknis yang Lebih Baik & Presisi

---

### Solusi A: Closed-Loop Line Centering as Turn Termination (LCTT) — *Paling Direkomendasikan*

Alih-alih membiarkan robot memutar buta lalu mencari garis secara open-loop, gunakan garis cabang itu sendiri sebagai umpan balik penutup (*closed-loop termination*).

#### Konsep Kerja
Proses dibagi menjadi 2 tahap:

```
[Tahap 1: Fast Coarse Turn]
Putar cepat (0.4 rad/s) hingga:
  - Yaw mencapai ~75° (sebelum 90° penuh)
  ATAU
  - Garis cabang pertama kali menyentuh tepi ROI kamera
          │
          ▼
[Tahap 2: Closed-Loop Fine Centering]
Hentikan putaran buta!
Aktifkan kendali proporsional terhadap error posisi garis di kamera:
  angular_z = -Kp * normalized_error
Robot secara aktif melambat dan memusatkan garis ke tengah kamera.
          │
          ▼
[Selesai (TURN_COMPLETE)]
Tercapai saat |normalized_error| < 0.10 dan kecepatan putar mendekati nol.
```

#### Mengapa Solusi Ini Menghilangkan Overshoot?
Pada Tahap 2, robot tidak lagi dipaksa memutar searah secara buta. Kendali PID visual langsung menarik posisi robot agar garis berada persis di tengah sensor kamera ($cx = \text{tengah}$). Jika robot sedikit kebablasan, PID visual akan memutar balik ke arah sebaliknya secara otomatis hingga garis terpusat sempurna.

---

### Solusi B: Profil Kecepatan Putar Melandai (Proportional Yaw Deceleration)

Jika pada Tahap 1 robot memutar dengan kecepatan konstan lalu mendadak berhenti, inersia robot akan menyebabkan ban selip dan overshoot.

Gunakan deselerasi proporsional (*P-controller pada yaw*):
$$\text{Sisa Sudut} = \theta_{\text{target}} - \theta_{\text{sekarang}}$$
$$\omega = \text{sign}(\text{arah}) \cdot \min\left(\omega_{\max}, \max\left(\omega_{\min}, K_{\text{yaw}} \cdot |\text{Sisa Sudut}|\right)\right)$$

- Saat sisa sudut masih besar (> 45°): robot memutar dengan kecepatan penuh ($0{,}40\text{ rad/s}$).
- Saat sisa sudut mengecil (< 20°): kecepatan melandai turun secara bertahap hingga $\omega_{\min} = 0{,}10\text{ rad/s}$.
- Efek: Robot berhenti tanpa hentakan inersia, pembacaan encoder roda menjadi sangat akurat tanpa selip.

---

### Solusi C: Kalibrasi Wheel Track ($b$) & Fusi IMU Yaw

Jika odometri roda tetap meleset karena gesekan lantai:
1. **Kalibrasi Ulang Parameter $b$ (*Wheel Separation*):**
   - Di `docs.md`, terdapat perbedaan catatan antara $b = 0{,}48\text{ m}$ (dimensi fisik nominal) dan $b = 0{,}454\text{ m}$ (hasil uji putar).
   - Selisih 2,6 cm pada $b$ menyebabkan error pembacaan sudut yaw sekitar $5\% - 6\%$ (error 5°–6° setiap kali belok 90°).
   - Lakukan pengujian putar 360° di lantai aktual untuk mengunci nilai $b$ yang presisi.
2. **Pemanfaatan Sensor IMU (BNO055):**
   - Sensor IMU mengukur kecepatan sudut (*gyroscope*) secara langsung tanpa terpengaruh oleh selip ban pada lantai. Menggabungkan yaw IMU ke odometri adalah cara paling efektif untuk menjamin sudut 90° absolut.

---

## 3. Perbandingan Metode

| Metode | Akurasi Sudut | Risiko Overshoot | Ketergantungan Slip Roda |
|---|---|---|---|
| **Metode Saat Ini (Turn + Blind Seek)** | Rendah (sering 105°–120°) | Sangat Tinggi | Tinggi |
| **Metode A: LCTT (Visual Centering)** | **Sangat Tinggi (selalu 90° pas garis)** | **Nol (dikoreksi garis)** | Rendah |
| **Metode B: Ramp-Down Yaw Deceleration** | Sedang–Tinggi (±3°) | Rendah | Sedang |
| **Metode C: Fusi IMU + LCTT** | Maksimal (standar industri) | Nol | Nol |

---

## 4. Parameter yang Perlu Disetel (Nanti saat Implementasi)

Saat pengujian diterapkan pada `pid_params.yaml`:

```yaml
line_follower_node:
  ros__parameters:
    turn_angle_rad: 1.30              # Target coarse turn diturunkan ke ~75° agar tidak kebablasan sebelum seek
    turn_speed_radps: 0.35            # Kecepatan putar utama
    seek_kp: 1.2                      # Penguatan PID untuk memusatkan garis saat seek
    reacquire_error_threshold: 0.15   # Toleransi tengah garis (lebih presisi)
    reacquire_frames: 3               # Jumlah frame stabil sebelum konfirmasi selesai
```
