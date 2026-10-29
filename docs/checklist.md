# Checklist Pengembangan AGV (Capstone Design ARASHI)

Checklist ini mengikuti tahapan pada [roadmap pengembangan](roadmap.md); nomor
bagian sama dengan nomor Tahap di roadmap. Tenggat: pameran awal Desember 2026
(Rektorat UNESA).
Item `[x]` berarti sudah tersedia di kode atau dokumentasi. Item tersebut belum tentu
sudah diuji pada robot fisik; validasi hardware ditandai terpisah.

## Kondisi sekarang

- [x] Membaca frame kamera V4L2.
- [x] Mendeteksi garis dan menghitung error posisi ternormalisasi.
- [x] PID garis menghasilkan perintah kecepatan putar `ω`.
- [x] Menerbitkan gambar debug ke `/line_tracker/debug_image`.
- [x] Menerbitkan `/cmd_vel` dari line tracker.
- [x] Mengirim `C,<linear_x>,<angular_z>` melalui serial ke firmware.
- [x] Menerima delta encoder `E,<left_ticks>,<right_ticks>` dari firmware.
- [x] Meneruskan delta encoder dari line tracker ke topik `/line_tracker/encoder_delta`.
- [x] Menghitung estimasi odometri DDMR global $(x, y, \theta)$ dan menerbitkan topik `/odom`, TF `odom -> base_link`, serta `/path` (`odometry_node`).
- [x] Menyediakan konfigurasi visualisasi RViz (`ros2_ws/line_tracker.rviz`).
- [ ] Memvalidasi arah belok dan semua satuan pada robot fisik.

## Masalah terbuka (known issues)

- [ ] **BUG KESELAMATAN:** E-stop yang sedang aktif masih bisa di-reset dan di-start lagi.
  Lihat Tahap 0. Selesaikan sebelum uji gerak di lantai dilanjutkan.

## Tahap 0 — Keselamatan, hardware, dan antarmuka dasar

- [x] Ukur diameter efektif kedua roda dan jarak antara pusat roda kiri-kanan (*wheel track*).
  - Diameter roda efektif: **15 cm (0,15 m)**; radius **0,075 m**.
  - *Wheel track* $b$: **48 cm (0,48 m)**.
- [x] Catat konfigurasi encoder: pulsa per putaran (*CPR*), rasio gearbox, dan resolusi efektif.
  - Encoder: **30 pulsa/putaran poros motor**.
  - Gearbox: **30:1**.
  - Resolusi roda: **900 pulsa/putaran roda** (30 × 30).
- [ ] Pastikan 30 pulsa/putaran sudah memperhitungkan mode dekode (x1/x2/x4); ukur
  dengan memutar roda tepat 1 putaran.
- [ ] Catat tegangan kerja motor dan port UART di dokumentasi hardware.
- [ ] Validasi arah putaran motor kiri/kanan dan polaritas hitungan pulsa encoder.
  - Prosedur ada di [`hardware.md`](hardware.md#41-arah-putaran-motor).
- [ ] Validasi koneksi ROS ke `Serial1` Arduino Mega dan pastikan port stabil.
- [x] Tambahkan **command timeout** 500 ms di firmware ketika state `MOVING`.
- [ ] Perbaiki E-stop: tombol yang aktif tidak boleh bisa di-reset/start lewat software.
  - [ ] E-stop memutus daya aktuator di jalur hardware (bukan hanya flag firmware).
  - [ ] Status E-stop *latched*: motor aktif lagi hanya setelah tombol dilepas fisik
    dan ada aksi start eksplisit di robot.
  - [ ] Perintah serial/ROS/IoT tidak dapat membatalkan atau mengabaikan E-stop.
- [ ] Uji fungsi E-stop fisik: selalu memutus pergerakan motor seketika (uji saat
  bergerak maju, belok, dan saat serial terputus).
- [x] Dokumentasikan satuan kanonik (m, rad, s), urutan token serial, dan simpan nilai kalibrasi di konfigurasi YAML.
  - Satuan, konvensi tanda, pinout, dan prosedur: [`hardware.md`](hardware.md).
  - Kalibrasi tersimpan di `ros2_ws/src/line_tracker_ros2/config/line_tracker.yaml`.

## Tahap 1 — Kontrol kecepatan tertutup & serial dua arah

- [x] Tentukan format perintah serial `C,<linear_x>,<angular_z>` dan balasan
  `E,<left_ticks>,<right_ticks>`.
- [x] Hitung target ticks kiri/kanan dari `cmd_vel` di firmware (kinematika balik DDMR
  dilakukan di firmware, bukan di ROS).
- [x] Implementasikan PID terpisah untuk roda kiri dan kanan; input-nya delta ticks
  per interval kontrol, output-nya PWM bertanda.
- [x] Implementasikan arah maju/mundur motor dari tanda PWM hasil PID.
- [x] Kirim delta ticks encoder dari firmware kembali ke ROS 2.
- [x] Tambahkan timeout perintah 500 ms di firmware.
- [ ] Tambahkan framing delimiter ketat dan checksum (CRC8/XOR) pada frame `C` dan `E`;
  frame dengan checksum salah ditolak.
- [ ] Uji korupsi frame: jalankan motor penuh dan pastikan tidak ada frame rusak
  yang diterima.
- [ ] Uji dan tuning PID. Gain saat ini nilai awal dan belum tervalidasi.
- [ ] Validasi tanda encoder dan arah motor untuk maju, mundur, serta putar di tempat.
- [ ] Ukur kecepatan aktual roda (RPM atau m/s) pada beban dan tegangan baterai berbeda,
  dan publikasikan ke ROS untuk telemetri.
- [ ] Uji perilaku motor saat kabel serial dicabut tiba-tiba (harus memicu timeout).

## Tahap 2 — Odometri DDMR dan estimasi pose di ROS 2

- [x] Pisahkan odometri dari node line tracker ke `odometry_node` khusus.
- [x] Konversikan delta pulsa encoder menjadi jarak roda $\Delta s_L$ dan $\Delta s_R$,
  hitung $\Delta s$, $\Delta\theta$, dan perbarui pose global $(x,y,\theta)$.
  - $d_t = \pi D / CPR = \pi(0{,}15)/900 \approx 0{,}0005236$ m/tick.
  - $\Delta s = (\Delta s_L + \Delta s_R)/2$; $\Delta\theta = (\Delta s_R - \Delta s_L)/b$, $b = 0{,}48$ m.
  - Pembaruan memakai heading tengah $\theta + \Delta\theta/2$. Rumus lengkap ada di roadmap.
- [x] Publikasikan `/odom` (`nav_msgs/msg/Odometry`) dengan frame `odom` dan `base_link`.
- [x] Publikasikan transformasi koordinat `odom -> base_link` menggunakan `tf2_ros`.
- [x] Publikasikan jejak pose `nav_msgs/msg/Path` pada topik `/path` untuk visualisasi RViz.
- [ ] Tetapkan dan validasi nilai kovariansi pose serta twist pada pesan odometri.
- [ ] Lakukan uji jalan lurus pada jarak fisik terukur (misal 2 meter) dan kalibrasi diameter roda.
- [ ] Lakukan uji putar 360° di tempat dan kalibrasi parameter jarak antar-roda (*wheel track*).
- [ ] Simpan hasil kalibrasi sebagai faktor skala di `line_tracker.yaml`.
- [ ] Visualisasikan pose, path, dan TF di RViz untuk memastikan pergerakan halus tanpa patah.
- [ ] Dokumentasikan laju akumulasi drift odometri per meter perjalanan.

## Tahap 2B — Orientasi IMU BNO055

- [ ] Tentukan sambungan I2C (disarankan ke Arduino Mega: SDA 20, SCL 21) dan catat di `hardware.md`.
- [ ] Baca yaw BNO055 dan kirim ke ROS (mis. frame `I,<yaw_rad>`) atau langsung dari komputer ROS.
- [ ] Mulai dengan mode fusi tanpa magnetometer; uji gangguan magnetik dari motor sebelum memakai NDOF.
- [ ] Publikasikan `sensor_msgs/msg/Imu` dengan frame dan tanda yaw konsisten dengan odometri.
- [ ] Bandingkan yaw IMU dengan $\Delta\theta$ odometri pada uji putar 360°.
- [ ] (Opsional) Fusi odometri + IMU dengan `robot_localization`.

## Tahap 3 — Eksperimen persepsi dan deteksi ArUco fisik

- [ ] Putuskan ArUco vs alternatif landmark/topologi (batas: akhir Minggu 6).
- [ ] Tentukan skema penempatan fisik marker ArUco (lantai atau tiang/dinding persimpangan).
- [ ] Evaluasi FOV kamera: pastikan kamera mampu melihat marker ArUco dengan jarak deselerasi yang aman sebelum tiba di intersection.
- [ ] Kalibrasi parameter intrinsik dan distorsi lensa kamera menggunakan `camera_calibration` ROS 2.
- [ ] Implementasikan node deteksi ArUco menggunakan OpenCV `cv2.aruco`.
- [ ] Implementasikan filter *debouncing*: konfirmasi deteksi stabil selama beberapa frame berturut-turut sebelum menandai waypoint terdeteksi.
- [ ] Ukur jarak efektif deteksi dan estimasi pose marker relatif terhadap kamera.
- [ ] Uji kehandalan deteksi pada berbagai kondisi pencahayaan, blur gerak, dan sudut kemiringan kamera.
- [ ] Tetapkan ukuran fisik marker dan jarak pemasangan yang optimal.

## Tahap 4 — Peta graf, misi, dan refaktorisasi ROS 2

### 4A — Wajib untuk demo

- [ ] `aruco_detector_node` (pemrosesan citra marker ArUco).
- [ ] `mission_controller_node` (state machine navigasi misi).
- [ ] Definisikan format peta lintasan berbasis graf (YAML/JSON): node waypoint (CP1–CP3),
  relasi edge, arah, batas kecepatan, instruksi percabangan, dan ID ArUco.
- [ ] Buat Waypoint & Route Manager untuk menghasilkan rute sekuensial dari titik awal ke tujuan.
- [ ] Definisikan FSM misi: `IDLE`, `NAVIGATING`, `AT_INTERSECTION`, `ARRIVED`, `PAUSED`, `ERROR`, `ESTOP`.
- [ ] Buat unit test untuk validasi integritas struktur file peta (tidak ada waypoint yatim/loop buntu tak terdefinisi).

### 4B — Refaktorisasi lanjutan (opsional, prioritas rendah)

- [x] `odometry_node` (perhitungan pose dan publish TF) sudah terpisah.
- [ ] `serial_bridge_node` (I/O UART dua arah, framing, checksum).
- [ ] `line_detector_node` (pemrosesan citra garis).

## Tahap 5 — Navigasi line tracking percabangan di intersection

- [ ] Implementasikan logika deselerasi saat odometri mendeteksi robot mendekati persimpangan.
- [ ] Implementasikan verifikasi ArUco di persimpangan yang sesuai dengan rute misi.
- [ ] Gunakan pose ArUco untuk mengoreksi akumulasi drift odometri (*checkpoint reset*).
- [ ] Kembangkan logika pemilihan cabang (*branch switching*) pada kontur garis (belok kiri, lurus, atau belok kanan).
- [ ] Implementasikan penguncian garis baru setelah menyelesaikan belokan di persimpangan.
- [ ] Implementasikan penanganan kondisi gagal:
  - [ ] Garis hilang di luar persimpangan: robot berhenti seketika.
  - [ ] ArUco tidak terdeteksi pada jarak perkiraan: robot melambat hingga berhenti (minta asistensi).
  - [ ] ArUco tidak cocok dengan urutan rute: robot menghentikan misi demi keselamatan.
- [ ] Uji coba navigasi otonom CP1 → CP2 → CP3 melewati minimal 3 persimpangan berbeda.

## Tahap 6 — Integrasi IoT: telemetry dan command tingkat misi

### 6A — Localhost + GUI web

- [ ] Backend localhost yang mengirim data telemetri dan menerima data command.
- [ ] Buat GUI web read-only untuk pengujian awal.
- [ ] Publikasikan telemetri: pose terkoreksi, ID waypoint saat ini, status FSM, kecepatan
  target/aktual roda, kualitas garis, ID ArUco terakhir, dan status koneksi/sensor.

### 6B — Database (Firebase)

- [ ] Hubungkan telemetri/log misi ke Firebase.
- [ ] Atur autentikasi dan *security rules* agar hanya operator berwenang yang dapat menulis command.
- [ ] Pastikan robot tetap berjalan aman dan demo tetap bisa dijalankan tanpa internet (fallback localhost).

### 6C — Command misi

- [ ] Rancang skema command berformat JSON berversi lengkap dengan ID pesan, timestamp, dan ACK/NACK.
  Tolak command kedaluwarsa atau duplikat.
- [ ] Implementasikan handler command tingkat misi: `go_to_waypoint` (CP1/CP2/CP3), `pause`,
  `resume`, `cancel_mission`, dan `ack_error`.
- [ ] Pastikan command IoT divalidasi ketat dan tidak dapat mengirimkan PWM mentah atau membatalkan E-stop fisik.
- [ ] Uji perilaku robot saat jaringan IoT putus tiba-tiba (robot harus menyelesaikan segmen aman atau berhenti aman).
- [ ] Ukur latensi telemetri dan command.

## Tahap 7 — Uji integrasi akhir dan dokumentasi

- [ ] Lakukan prosedur uji meja (roda terangkat) untuk seluruh alur dari command IoT hingga respon motor.
- [ ] Lakukan uji lintasan penuh tertutup dengan skenario persimpangan kompleks.
- [ ] Ulangi skenario demo minimum 5 kali berturut-turut tanpa intervensi.
- [ ] Uji dalam kondisi menyerupai venue (pencahayaan, lantai, jaringan Wi-Fi, baterai penuh/rendah).
- [ ] Rekam log dan *rosbag* untuk setiap uji penting.
- [ ] Dokumentasikan diagram wiring hardware final, pinout, dan parameter kalibrasi.
- [ ] Dokumentasikan prosedur peluncuran sistem (*launch files*), penanganan error (*recovery*), dan manual operasi.
- [ ] Rekam video demonstrasi misi penuh dari command IoT hingga robot tiba di tujuan dan melaporkan status selesai.

## Jalur paralel — Upgrade hardware dan mekanik (opsional)

- [ ] Putuskan ruang lingkup upgrade hardware/mekanik (batas: akhir Minggu 6; setelah itu dibekukan).
- [ ] Jika ada perubahan roda, sasis, atau posisi sensor: ulangi kalibrasi odometri, tuning PID, dan evaluasi FOV kamera.
- [ ] Catat setiap perubahan mekanik di dokumentasi hardware.
