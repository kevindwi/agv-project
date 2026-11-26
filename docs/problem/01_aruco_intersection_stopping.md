# Masalah 1: Berhenti Presisi di Pusat Persimpangan Berbasis ArUco

Dokumen ini menganalisis penyebab robot tidak berhenti tepat di pusat persimpangan cabang saat menggunakan ArUco, serta merinci solusi teknis berbasis ArUco murni tanpa mengandalkan fitur visual garis/lingkungan.

---

## 1. Analisis Masalah: Mengapa ArUco Harus Ditaruh Jauh?

### 1.1 Geometri Fisik Robot & Kamera
Robot differential drive memiliki karakteristik geometris sebagai berikut:
- **Pusat Rotasi (Center of Rotation):** Berada di titik tengah antara roda penggerak kiri dan kanan (sumbu roda).
- **Posisi Kamera:** Berada di bagian depan robot, sejauh $d_{\text{offset}} = 0{,}45\text{ m}$ di depan sumbu roda.
- **Sudut Kemiringan Kamera (Pitch):** Menghadap $45^\circ$ ke bawah ke arah lantai.

```
                  Kamera (menunduk 45°)
                       \
                        \ Garis pandang optik (tvec.z)
                         \
  [ Belakang ] ─── [ Roda / Pusat Rotasi ] ─── [ Depan ]
                           |←────── 0.45 m ──────→|
                                                  \
                                                   \ 
  ──────────────────────────────────────────────────▼─── [ Lantai / Marker ] ───
```

### 1.2 Mengapa Terjadi Ketidaksesuaian Titik Berhenti Saat Ini?

1. **Efek Offset Kamera ($0{,}45\text{ m}$):**
   Saat kamera tepat berada di atas marker persimpangan, sumbu roda robot masih berada $0{,}45\text{ m}$ di belakang persimpangan. Jika robot berputar saat itu, robot akan berputar mengitari titik yang salah dan roda akan keluar dari garis cabang.

2. **Jarak Deteksi Optik Bersifat Fluktuatif:**
   Pada implementasi saat ini di `mission_controller_node.py`:
   - Robot mendeteksi ArUco pada jarak `trigger_distance_m = 0.30 m`.
   - Dibutuhkan `confirm_frames = 3` untuk verifikasi.
   - Selama 3 frame (sekitar 100–150 ms pada 20 FPS), dengan kecepatan $0{,}10\text{ m/s}$, robot sudah melaju maju sejauh $1{,}5 - 2\text{ cm}$.
   - Deteksi pertama bisa terjadi di jarak $0{,}38\text{ m}$, $0{,}32\text{ m}$, atau $0{,}29\text{ m}$ tergantung pencahayaan dan resolusi.

3. **Kelemahan Jarak Creep Statis (`creep_distance_m = 0.66 m`):**
   Saat ini, begitu marker terkonfirmasi, robot merayap maju dengan jarak **tetap/statis** sejauh $0{,}66\text{ m}$ menggunakan odometri.
   $$\text{Titik Berhenti} = \text{Posisi saat Konfirmasi} + 0{,}66\text{ m}$$
   Karena posisi saat konfirmasi selalu bervariasi (tergantung kapan frame kamera menangkap marker), titik berhenti robot ikut bergeser maju/mundur beberapa sentimeter di setiap pengujian.

---

## 2. Solusi Teknis Berbasis ArUco Murni

Ada 2 pendekatan presisi yang dapat digunakan tanpa bergantung pada kondisi garis di persimpangan:

---

### Solusi A: Dynamic Odometry Creep (Kompensasi Jarak Dinamis) — *Paling Direkomendasikan*

Alih-alih merayap dengan jarak statis ($0{,}66\text{ m}$), robot menghitung sisa jarak tempuh secara dinamis dari **pose 3D ArUco yang diukur kamera saat itu juga**.

#### 1. Formulasi Matematika Jarak ke Marker di Lantai
Dari detektor ArUco (`aruco_detector_node.py`), OpenCV menghasilkan vektor translasi $tvec = [X_c, Y_c, Z_c]$ relatif terhadap lensa kamera.
Karena kamera menunduk dengan sudut pitch $\theta = 45^\circ$:

Jarak horizontal lantai dari lensa kamera ke marker ($D_{\text{cam\_to\_marker}}$):
$$D_{\text{cam\_to\_marker}} = Z_c \cdot \cos(\theta) + Y_c \cdot \sin(\theta)$$
*(Jika marker diletakkan di lantai datar)*

Total jarak yang harus ditempuh roda robot agar **pusat sumbu roda** tiba tepat di atas marker persimpangan:
$$D_{\text{target\_creep}} = D_{\text{cam\_to\_marker}} + d_{\text{center\_to\_camera}}$$

Di mana:
- $d_{\text{center\_to\_camera}} = 0{,}45\text{ m}$ (jarak fisik sumbu roda ke kamera).
- $D_{\text{cam\_to\_marker}}$ dihitung dari pembacaan $tvec$ pada frame konfirmasi terakhir.

#### 2. Logika Algoritma
```text
Saat ArUco terkonfirmasi:
1. Ambil jarak optical_distance (Z_c) dan sudut kemiringan kamera.
2. Hitung jarak lantai aktual ke marker: D_floor = Z_c * cos(45°).
3. Target jarak gerak robot = D_floor + 0.45 meter.
4. Kunci posisi odometri awal: (X0, Y0).
5. Robot maju lurus perlahan sampai:
   sqrt((X_now - X0)^2 + (Y_now - Y0)^2) >= Target jarak gerak.
6. Berhenti total -> Sumbu roda tepat di titik persimpangan!
```

#### 3. Keunggulan Solusi A:
- Tidak peduli apakah ArUco baru terdeteksi di jarak $0{,}50\text{ m}$ atau $0{,}30\text{ m}$, kalkulasi jarak tempuh selalu menghasilkan titik henti sumbu roda yang identik di posisi marker.
- ArUco dapat diletakkan tepat di tengah persimpangan.

---

### Solusi B: Optical Disappearance Trigger (Batas Bawah FOV Kamera)

Jika kalibrasi sudut pitch kamera sulit mendapatkan akurasi milimeter, gunakan metode batas bidang pandang (*Field of View / FOV*):

#### 1. Prinsip Kerja
1. Marker ArUco diletakkan di lantai tepat di tengah persimpangan (atau di samping garis persimpangan).
2. Saat robot bergerak maju, marker ArUco akan bergerak dari area atas layar kamera turun ke bawah layar kamera.
3. Begitu titik tengah marker menyentuh batas bawah frame kamera ($Y_{\text{pixel}} \ge Y_{\text{bottom\_threshold}}$) atau marker baru saja hilang dari pandangan bawah:
   - Posisi lensa kamera diketahui secara pasti berada tepat di atas garis batas depan lantai ($D_{\text{blind\_spot}} \approx 0{,}15\text{ m}$ di depan kamera).
4. Dari momen ini, robot hanya perlu maju sejauh jarak tetap:
   $$D_{\text{fixed}} = d_{\text{center\_to\_camera}} + D_{\text{blind\_spot}} = 0{,}45\text{ m} + 0{,}15\text{ m} = 0{,}60\text{ m}$$

#### 2. Keunggulan Solusi B:
- Sangat konsisten karena trigger tidak bergantung pada akurasi estimasi kedalaman $Z$ yang sering fluktuatif akibat resolusi rendah.
- Titik picu adalah posisi piksel pada sensor kamera ($Y_{\text{pixel}}$), yang memiliki ketelitian sangat tinggi.

---

## 3. Rekomendasi Pemasangan Fisik Marker ArUco

Jika lingkungan persimpangan tidak memadai:

1. **Opsi Marker di Lantai (Paling Ideal untuk Presisi Sumbu Roda):**
   - Letakkan marker di lantai tepat di titik pertemuan garis cabang.
   - Ukuran marker ideal: $8 \times 8\text{ cm}$ atau $10 \times 10\text{ cm}$.
   - Gunakan laminasi *doff/matte* agar pantulan lampu plafon tidak membutakan pembacaan kamera.

2. **Opsi Marker di Tiang Samping (Offset Samping):**
   - Jika marker tidak boleh di lantai, pasang di tiang setinggi $20 - 30\text{ cm}$ tepat sejajar dengan sumbu persimpangan.
   - Gunakan nilai translasi lateral ($X_c$) dari pose ArUco untuk memastikan robot berada tepat di samping tiang sebelum mengeksekusi rotasi.

---

## 4. Parameter Konfigurasi yang Perlu Diperbarui (Nanti)

Saat implementasi diterapkan di `route_scenario.yaml`:

```yaml
mission_controller_node:
  ros__parameters:
    camera_offset_m: 0.45       # Jarak fisik sumbu roda ke kamera
    camera_pitch_deg: 45.0      # Kemiringan kamera ke bawah
    use_dynamic_creep: true     # Mengaktifkan perhitungan jarak dinamis
    creep_speed_mps: 0.08       # Kecepatan konstan pelan saat merayap ke titik pusat
```
