# 📋 Checklist Operasional AGV (Capstone Design ARASHI)
> **Pembaruan:** November 2026  
> **Target:** Pameran Capstone Design di Rektorat UNESA (Awal Desember 2026).  
> **Dokumen Rujukan:** [docs/roadmap.md](roadmap.md)  
> *Keterangan: Tanda `[x]` berarti sudah terimplementasi dan tervalidasi pada robot.*

---

## 🏗️ Fondasi Arsitektur Modular ROS 2 Jazzy

- [x] **Paket `agv_interfaces` (Bahasa Pesan Terstruktur):**
  - [x] Definisi pesan `ArucoDetection.msg` (`int32 id`, `float32 distance_m`).
  - [x] Definisi pesan `MissionState.msg` (`string state`, `int32 current_waypoint`).
- [x] **Paket `agv_bringup` (Driver Hardware & Komunikasi):**
  - [x] `camera_node`: Pembaca kamera USB Logitech (/dev/video2) $\to$ publish `/camera/image_raw`.
  - [x] `serial_bridge_node`: Driver komunikasi serial non-blocking (`timeout=0` + byte stream buffer).
  - [x] Failsafe heartbeat IDLE: Kirim `C,0,0` hanya saat robot diam agar firmware tidak timeout.
  - [x] `agv_core.launch.py`: Launch file gabungan kamera + bridge serial.
- [x] **Paket `agv_navigation` (Navigasi Garis & Odometri):**
  - [x] `line_follower_node`: Pemroses citra garis, ROI, OTSU, PID kemudi garis $\to$ `/cmd_vel`.
  - [x] Ramping akselerasi & deselerasi (*Velocity Slew Rate Limiter*).
  - [x] Filter glitch kehilangan garis (*Coasting* 8 frame toleransi).
  - [x] `odometry_node`: Integrator pulsa encoder $\to$ `/odom`, TF `odom -> base_link`, dan `/path`.
- [x] **Paket `agv_mission` (Persepsi Visual & Logika Rute):**
  - [x] `aruco_detector_node`: Deteksi marker ArUco dictionary `DICT_4X4_50`, estimasi pose 3D jarak $Z$.
  - [x] `mission_controller_node`: Finite State Machine rute (`NAVIGATING` $\to$ `APPROACHING` $\to$ `ARRIVED`).

---

## 🟢 FASE 1: Validasi Gerak Motor & Kalibrasi Odometri Fisik

- [x] **Validasi Arah Gerak Motor:**
  - [x] Perintah `/cmd_vel` linier positif memutar kedua roda ke arah depan (*forward*).
  - [x] Perintah putar sudut $\omega$ memutar robot ke arah yang benar (kiri CCW, kanan CW).
- [x] **Kalibrasi Odometri Jalan Lurus (2.00 Meter):**
  - [x] Uji jalan lurus 2.0 meter di lantai keramik.
  - [x] Kalibrasi CPR encoder: disesuaikan dari 900 ke **610 pulsa/putaran** (`encoder_pulses_per_wheel_rev: 610`).
  - [x] Nilai $X$ di topik `/odom` terbaca tepat **2.00 meter** saat robot menempuh 2 meter fisik.
- [ ] **Kalibrasi Odometri Uji Putar 360° (*Wheel Track* $b$):**
  - [ ] Putar robot di tempat 1 putaran penuh ($360^\circ = 6.283 \text{ rad}$).
  - [ ] Pastikan orientasi yaw di `/odom` kembali ke $\approx 0$ atau $\approx 6.28 \text{ rad}$.
  - [ ] Sesuaikan parameter `wheel_separation_m` di `pid_params.yaml` jika ada selisih sudut.

---

## 🟡 FASE 2: Tuning Line Follower Fisik di Lintasan

- [x] **Stabilitas Penglihatan Garis:**
  - [x] Kamera menghadap ke lantai dengan sudut kemiringan tetap.
  - [x] ROI bawah (70% - 90% tinggi frame) berhasil mengisolasi garis tanpa terganggu lampu langit-langit.
  - [x] Glitch filter aktif (robot tidak tersendat saat garis sempat silau/hilang 1-2 frame).
- [x] **Tuning Parameter PID Garis:**
  - [x] Live tuning via `ros2 param set /line_follower_node ...` aktif tanpa perlu restart node.
  - [x] Monitoring grafik respons PID via `rqt_plot` di topik `/line_follower/error/data` dan `/cmd_vel/angular/z`.
  - [x] Nilai tuning stabil di lintasan fisik: $K_p \approx 0.4$, $K_d \approx 0.06$, $K_i = 0.0$.
  - [x] Kecepatan jelajah stabil di $0.20 \text{ m/s}$ tanpa zig-zag berlebih.

---

## 🟠 FASE 3: Integrasi Line Following + ArUco Stop (Approach & Stop)

- [x] **Kalibrasi Kamera & Estimasi Jarak ArUco:**
  - [x] Kalibrasi catur (*checkerboard*) selesai: `camera_matrix` dan `distortion_coefficients` tercatat di `route_scenario.yaml`.
  - [x] Marker ArUco 10 cm terdeteksi stabil dari jarak $1.0 - 1.2 \text{ meter}$.
  - [x] Pembacaan jarak meter $Z$ akurat ($\pm 3 \text{ cm}$) terhadap meteran fisik.
- [x] **Mekanisme Deselerasi Halus (Smooth Approach):**
  - [x] Zona melambat diperlebar: Mulai melambat di jarak $Z \le 0.90 \text{ m}$, berhenti di $Z \le 0.30 \text{ m}$.
  - [x] Velocity Slew Rate Limiter aktif di `line_follower_node` (`max_acceleration: 0.10 \text{ m/s}^2`).
- [x] **Verifikasi Uji Fisik di Lintasan:**
  - [x] Robot melaju normal $0.20 \text{ m/s}$ $\to$ melihat marker Checkpoint 1 $\to$ meluncur melambat (*smooth deceleration*) ke $0.05 \text{ m/s}$.
  - [x] Robot melandai anggun berhenti tepat pada jarak $0.30 \text{ m}$ di depan marker tanpa hentakan rem mendadak.
  - [x] Topik `/mission/state` beralih dari `NAVIGATING` $\to$ `APPROACHING` $\to$ `ARRIVED`.

---

## 🔴 FASE 4: Skenario Multi-Checkpoint & Percabangan

- [ ] **Mekanisme Latch & Debouncing:**
  - [ ] Marker harus terdeteksi minimal 3 frame berturut-turut (`confirm_frames: 3`) sebelum aksi dipicu.
  - [ ] Mekanisme *Latch ID*: Setelah Checkpoint 1 dilewati, ID 1 diabaikan agar tidak terjadi *double trigger*.
- [ ] **Manuver Percabangan Garis (Checkpoint 2 - Belok):**
  - [ ] Penentuan arah cabang rute (kiri / kanan).
  - [ ] Logika manuver belok di persimpangan: putaran sudut terkontrol $\omega$ sampai kamera mengunci garis cabang baru.
  - [ ] Kembali otomatis ke mode pelacak garis (`FOLLOW`).
- [ ] **Uji Skenario Lengkap 3 Titik (CP1 $\to$ CP2 $\to$ CP3):**
  - [ ] Checkpoint 1 (ID 1): Titik transit (melambat sejenak, konfirmasi, lanjut melaju).
  - [ ] Checkpoint 2 (ID 2): Titik persimpangan (belok ke cabang tujuan).
  - [ ] Checkpoint 3 (ID 3): Stasiun akhir (berhenti total dan misi selesai).

---

## 🔵 FASE 5: Monitoring Telemetri & Kontrol IoT

- [ ] **Infrastruktur Jaringan & ROSBridge:**
  - [ ] Paket `ros-jazzy-rosbridge-suite` terpasang.
  - [ ] WebSocket server aktif di port 9090 (`rosbridge_websocket`).
  - [ ] Laptop/Tablet operator terhubung dalam 1 jaringan Wi-Fi lokal (*hotspot*).
- [ ] **Dashboard Web Operator (HTML + roslibjs):**
  - [ ] Indikator status misi terkini (`IDLE / NAVIGATING / APPROACHING / ARRIVED`).
  - [ ] Tampilan koordinat posisi live $(X, Y)$ dari topik `/odom`.
  - [ ] Checkpoint target aktif (`current_waypoint`).
  - [ ] Tombol kendali misi: **Start Misi**, **Pause**, dan **Emergency Stop (Software)**.

---

## 🟣 FASE 6: Gladi Bersih Venue & Simulasi Hari Pameran

- [ ] **Adaptasi Lingkungan Venue (Rektorat UNESA):**
  - [ ] Kalibrasi ambang batas citra di bawah pencahayaan aula pameran (antisipasi silau lantai mengilap).
  - [ ] Uji deteksi ArUco pada sudut pandang pencahayaan pameran.
- [ ] **Keamanan & Keandalan Daya:**
  - [ ] Uji tombol E-stop fisik: memutus daya motor seketika dalam segala kondisi gerak.
  - [ ] Manajemen baterai: pastikan tegangan baterai motor dan komputer stabil untuk 5-10 kali demo berturut-turut.
  - [ ] Baterai cadangan dan charger siap siaga di lokasi.
- [ ] **Kemudahan Operasional Demo:**
  - [ ] Master launch file siap pakai: cukup 1 perintah untuk menyalakan seluruh robot.
  - [ ] Buku panduan singkat pengoperasian (*SOP Demo*) untuk tim yang berjaga di booth.
  - [ ] **Code Freeze:** Tidak ada modifikasi kode besar pada H-3 pameran.
