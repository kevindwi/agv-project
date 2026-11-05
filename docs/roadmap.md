# 🗺️ Roadmap Pengembangan AGV (Capstone Design ARASHI)
> **Pembaruan:** November 2026  
> **Target Utama:** Pameran Capstone Design di Rektorat UNESA (Awal Desember 2026).  
> **Arsitektur Acuan:** Arsitektur Modular ROS 2 Jazzy (`agv_interfaces`, `agv_bringup`, `agv_navigation`, `agv_mission`).

---

## 🎯 Skenario Demo Minimum (Target Akhir)

Robot AGV berjalan otonom di lintasan bergaris (*line-guided*):
1. **Start:** Operator menekan tombol Start (dari GUI Web atau tombol robot).
2. **Checkpoint 1 (CP1):** Robot melaju mengikuti garis $\to$ mendeteksi ArUco ID 1 di tepi jalur $\to$ otomatis melambat $\to$ konfirmasi titik singgah $\to$ lanjut melaju.
3. **Checkpoint 2 (CP2 - Percabangan/Tikungan):** Robot mendeteksi ArUco ID 2 $\to$ mengeksekusi manuver belok sesuai instruksi rute $\to$ mengunci kembali garis cabang.
4. **Checkpoint 3 (CP3 - Stasiun Akhir):** Robot mendeteksi ArUco ID 3 $\to$ melambat dan berhenti sempurna tepat di stasiun akhir.
5. **Telemetri IoT:** Posisi robot ($X, Y$), status misi (*NAVIGATING / APPROACHING / ARRIVED*), dan baterai terpantau di Web Dashboard.

---

## 🏆 Pencapaian yang Sudah Selesai (Milestone Terkini)

- [x] **Refaktor Arsitektur Modular ROS 2:**
  - `agv_interfaces`: Tipe pesan custom `ArucoDetection.msg` dan `MissionState.msg`.
  - `agv_bringup`: Driver kamera (`camera_node`) dan jembatan serial motor (`serial_bridge_node`) sudah terpisah dari algoritma navigasi.
  - `agv_navigation`: Kontroler PID garis (`line_follower_node`) dan odometri (`odometry_node`).
  - `agv_mission`: Detektor marker visual (`aruco_detector_node`) dan pengatur rute (`mission_controller_node`).
- [x] **Kalibrasi Kamera & Bench Test ArUco di Meja:**
  - Skrip kalibrasi papan catur mandiri (`calibrate_camera.py`) selesai.
  - Deteksi ID marker dan estimasi jarak meter 3D ($Z$) terverifikasi di meja tanpa menggerakkan robot.
- [x] **Komunikasi Firmware Dasar:**
  - Protokol serial dua arah: kirim perintah kecepatan `C,v,w\n` dan terima tick encoder `E,left,right\n`.

---

## 📌 Rencana Tahapan Menuju Demo Pameran

```text
[FASE 1: Validasi Motor & Odometri Fisik]
                      │
                      ▼
[FASE 2: Tuning Line Follower Fisik di Lantai]
                      │
                      ▼
[FASE 3: Integrasi Line Following + ArUco Stop]
                      │
                      ▼
[FASE 4: Skenario Multi-Checkpoint & Percabangan]
                      │
                      ▼
[FASE 5: Dashboard GUI / Telemetri IoT]
                      │
                      ▼
[FASE 6: Gladi Bersih Venue & Simulasi Pameran]
```

---

### 🟢 FASE 1: Validasi Gerak Motor & Kalibrasi Odometri Fisik
> **Fokus:** Memastikan robot fisik bergerak maju lurus saat diperintah maju, dan koordinat posisi di komputer sesuai dengan pergerakan nyata di lantai.

#### 1. Target Capaian:
- Perintah `/cmd_vel` positif membuat kedua roda berputar maju (bukan mundur atau berputar di tempat).
- Robot digerakkan lurus sejauh 2.0 meter di lantai $\to$ pembacaan topik `/odom` menunjukkan perpindahan $X \approx 2.0 \text{ m}$ ($\pm 5 \text{ cm}$).
- Robot diputar $360^\circ$ di tempat $\to$ pembacaan sudut orientasi Yaw ($\theta$) kembali ke titik awal ($\pm 10^\circ$).

#### 2. Teori & Konsep yang Perlu Dipahami:
- **Kinematika Differential Drive (DDMR):** Hubungan antara kecepatan linier robot $v$ (m/s), kecepatan sudut $\omega$ (rad/s), dengan kecepatan roda kiri $v_L$ dan kanan $v_R$:
  $$v_L = v - \frac{\omega \cdot b}{2}, \quad v_R = v + \frac{\omega \cdot b}{2}$$
  *(dengan $b$ = jarak antar-roda / wheel track).*
- **Resolusi Pulsa Encoder (CPR):** Pastikan perkalian pulsa gearbox benar:
  $$30 \text{ pulsa poros} \times 30 \text{ rasio gearbox} = 900 \text{ pulsa/putaran roda}.$$
- **Wheel Slip & Odometry Drift:** Mengapa roda karet di lantai keramik bisa meleset jika robot berakselerasi terlalu mendadak.

#### 3. Prosedur Uji Fisik:
1. Gantung roda robot (*jack up*) agar tidak menyentuh lantai.
2. Jalankan `serial_bridge_node` dan kirim perintah manual:
   ```bash
   ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}"
   ```
   *Amati: kedua roda harus berputar maju serempak.*
3. Letakkan robot di lantai, beri tanda lakban di lantai sepanjang 2 meter.
4. Buka terminal: `ros2 topic echo /odom` dan perhatikan `pose.pose.position.x`.
5. Dorong/jalankan robot tepat 2 meter $\to$ catat nilai $X$.
   - Jika di terminal terbaca 1.8 m (terlalu kecil), besarkan parameter `wheel_diameter_m` di `robot_params.yaml`.
   - Jika terbaca 2.2 m (terlalu besar), kecilkan parameter `wheel_diameter_m`.

---

### 🟡 FASE 2: Tuning Line Follower Fisik di Lintasan
> **Fokus:** Membuat robot mampu mengikuti lakban hitam di lantai dengan mulus pada kecepatan konstan.

#### 1. Target Capaian:
- Robot tidak keluar dari lintasan pada jalur lurus maupun tikungan bersudut tumpul/sedang.
- Robot bergerak tenang tanpa osilasi hebat (tidak mengayun/bergelombang ke kiri-kanan).

#### 2. Teori & Konsep yang Perlu Dipahami:
- **Region of Interest (ROI):** Membatasi area analisis gambar hanya di bagian bawah (lantai) agar lampu langit-langit tidak dianggap garis.
- **Tuning Intuitif Kontroler PID:**
  - **$K_p$ (Proportional):** Daya respons membelok. Terlalu kecil = telat belok; terlalu besar = robot bergetar/zig-zag hebat.
  - **$K_d$ (Derivative):** Rem getaran. Berfungsi meredam goyangan saat robot mendekati tengah garis.
  - **$K_i$ (Integral):** Biasanya diatur ke `0.0` untuk line follower robot agar tidak mengakumulasi kesalahan (*windup*).

#### 3. Prosedur Uji Fisik:
1. Pasang kamera dengan sudut kemiringan tetap menghadap ke lantai depan robot.
2. Jalankan node inti:
   ```bash
   ros2 launch agv_bringup agv_core.launch.py
   ros2 run agv_navigation line_follower_node --ros-args --params-file src/agv_navigation/config/pid_params.yaml
   ```
3. Mulai dengan $K_p = 0.8$, $K_i = 0.0$, $K_d = 0.2$, dan kecepatan rendah `linear_speed: 0.08` m/s.
4. Naikkan $K_p$ sedikit demi sedikit jika robot terlambat merespons tikungan.

---

### 🟠 FASE 3: Integrasi Line Following + ArUco Stop (Approach & Stop)
> **Fokus:** Menggabungkan penglihatan garis dengan penglihatan penanda ArUco pada Checkpoint pertama.

#### 1. Target Capaian:
- Robot melaju mengikuti garis $\to$ kamera melihat ArUco ID 1 di depan pada jarak $\approx 0.8 \text{ m}$.
- Pada jarak $\le 0.7 \text{ m}$, robot beralih ke mode `SLOW` (melambat perlahan).
- Pada jarak $\le 0.35 \text{ m}$ (jarak pemicu), robot berhenti tepat di depan marker (`STOP`).

#### 2. Teori & Konsep yang Perlu Dipahami:
- **Prioritas Kontrol (Arbiter / FSM Override):** Line follower menghasilkan kecepatan kemudi $\omega$ untuk menjaga robot di tengah garis, tetapi batas kecepatan maju $v$ dikendalikan oleh Mission Controller via topik `/mission/control` (`FOLLOW` $\to$ `SLOW` $\to$ `STOP`).
- **Debouncing Deteksi:** Menghindari false trigger (pemicu palsu). Marker harus terlihat minimal 3 frame berturut-turut sebelum robot memutuskan untuk merespons.

#### 3. Prosedur Uji Fisik:
1. Tempelkan marker ArUco ID 1 di tiang/kotak di ujung lintasan garis lurus (tinggi sejajar lensa kamera).
2. Jalankan ketiga subsistem:
   ```bash
   # Terminal 1: Hardware
   ros2 launch agv_bringup agv_core.launch.py
   # Terminal 2: Navigasi Garis
   ros2 run agv_navigation line_follower_node --ros-args --params-file src/agv_navigation/config/pid_params.yaml
   # Terminal 3: Misi ArUco
   ros2 run agv_mission aruco_detector_node --ros-args --params-file src/agv_mission/config/route_scenario.yaml
   ros2 run agv_mission mission_controller_node --ros-args --params-file src/agv_mission/config/route_scenario.yaml
   ```
3. Pantau topik status misi: `ros2 topic echo /mission/state`.
4. Amati apakah robot melambat dengan halus sebelum berhenti.

---

### 🔴 FASE 4: Skenario Multi-Checkpoint & Percabangan
> **Fokus:** Eksekusi skenario demo lengkap (CP1 $\to$ CP2 $\to$ CP3) termasuk pemilihan cabang/tikungan.

#### 1. Target Capaian:
- Checkpoint 1 (ID 1): Konfirmasi titik transit, robot tidak berhenti permanen, melainkan melambat sejenak lalu lanjut.
- Checkpoint 2 (ID 2): Robot membelok di persimpangan garis cabang (kiri atau kanan).
- Checkpoint 3 (ID 3): Robot berhenti total di stasiun akhir.

#### 2. Teori & Konsep yang Perlu Dipahami:
- **Manuver Persimpangan Garis (Branching):**
  Saat ada garis bercabang (T-junction atau Y-junction), kamera akan melihat garis melebar. Manuver belok yang paling andal untuk pemula adalah **Timed Turning / Odometry-Assisted Turning**:
  1. Begitu marker persimpangan terdeteksi, matikan tracking garis sementara.
  2. Putar robot di tempat dengan $\omega = \pm 0.4 \text{ rad/s}$ selama durasi tertentu (atau sampai sensor mendeteksi garis cabang baru).
  3. Kunci kembali ke mode pelacak garis biasa (`FOLLOW`).
- **State Latching:** Begitu Checkpoint 1 selesai dilewati, ID 1 harus diabaikan agar robot tidak membaca ulang titik yang sama saat masih berada di dekatnya.

---

### 🔵 FASE 5: Monitoring Telemetri & Kontrol IoT
> **Fokus:** Operator dapat memantau status robot dari jarak jauh tanpa melihat terminal layar hitam.

#### 1. Target Capaian:
- Dashboard Web sederhana (berjalan di browser laptop/tablet operator) yang menampilkan:
  - Koordinat posisi robot ($X, Y$).
  - Status Misi terkini (`NAVIGATING`, `APPROACHING`, `ARRIVED`).
  - Checkpoint yang sedang dituju.
  - Tombol darurat perangkat lunak (*Software Stop / Pause*).

#### 2. Teori & Konsep yang Perlu Dipahami:
- **ROSBridge Suite (`rosbridge_websocket`):** Jembatan resmi antara topik ROS 2 dengan aplikasi web berbasis HTML/JavaScript via protokol WebSocket port 9090.
- **Web Telemetry Stack:** HTML + JavaScript (`roslibjs`), sangat ringan, tidak membutuhkan cloud berbayar untuk demonstrasi di jaringan Wi-Fi lokal (*hotspot*).

#### 3. Prosedur Implementasi:
1. Instal paket ROSBridge:
   ```bash
   sudo apt install ros-jazzy-rosbridge-suite
   ```
2. Jalankan jembatan:
   ```bash
   ros2 launch rosbridge_server rosbridge_websocket_launch.xml
   ```
3. Buat halaman web statis sederhana `index.html` yang terhubung ke `ws://localhost:9090` untuk membaca topik `/mission/state` dan `/odom`.

---

### 🟣 FASE 6: Gladi Bersih Venue & Simulasi Hari Pameran
> **Fokus:** Mengantisipasi perbedaan kondisi lab dengan gedung pameran Rektorat UNESA.

#### 1. Checklist Kesiapan Demo:
- [ ] **Pencahayaan Ruangan:** Uji deteksi garis dan ArUco di ruangan dengan lantai mengilap (pantulan lampu plafon dapat mengganggu thresholding).
- [ ] **Baterai Cadangan & Manajemen Daya:** Pastikan tegangan baterai motor tidak drop di bawah batas aman saat robot berjalan berulang kali.
- [ ] **SOP Pengoperasian Mandiri:** Buat 1 file launch master (misalnya `agv_demo.launch.py`) agar robot bisa dinyalakan cukup dengan 1 perintah terminal:
  ```bash
  ros2 launch agv_bringup agv_demo.launch.py
  ```
- [ ] **Tombol E-stop Fisik:** Pastikan tombol E-stop merah di bodi robot dapat memutus daya motor secara instan jika terjadi anomali lintasan.
- [ ] **Code Freeze:** H-5 sebelum pameran, hentikan penambahan fitur baru dan fokus 100% pada keandalan dan konsistensi gerak.

---

## 📅 Matriks Jadwal Mingguan Menuju Pameran

| Minggu | Periode | Fokus Utama | Target Selesai |
| :--- | :--- | :--- | :--- |
| **Minggu 1** | 5 – 11 Nov | **Fase 1 & Fase 2** | Validasi motor fisik, kalibrasi odometri lurus 2m, tuning PID line following di lantai. |
| **Minggu 2** | 12 – 18 Nov | **Fase 3 & Fase 4** | Integrasi ArUco Approach + Stop di CP1, uji manuver belok persimpangan CP2, skenario CP3. |
| **Minggu 3** | 19 – 25 Nov | **Fase 5** | Pembuatan dashboard web telemetri (ROSBridge/roslibjs), uji kontrol start/stop dari GUI. |
| **Minggu 4** | 26 Nov – 2 Des | **Fase 6** | Gladi bersih penuh di lintasan demo pameran, uji daya tahan baterai, code freeze. |
| **Hari H** | Awal Des | **Pameran Capstone** | Demonstrasi berjalan lancar di hadapan penguji dan pengunjung pameran. |
