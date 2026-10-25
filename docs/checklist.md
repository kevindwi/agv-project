# Checklist Pengembangan AGV

Checklist ini mengikuti urutan tahapan yang disesuaikan pada [roadmap pengembangan](roadmap.md).
Item bertanda `[x]` adalah kemampuan yang sudah tersedia di kode saat ini; tetap validasi
pada robot fisik sebelum menganggapnya selesai untuk operasi.

## 0. Kondisi sekarang

- [x] Membaca frame kamera V4L2.
- [x] Mendeteksi garis dan menghitung error posisi ternormalisasi.
- [x] PID menghasilkan perintah kecepatan putar `ω`.
- [x] Kinematika balik DDMR menghitung target kecepatan roda kiri dan kanan.
- [x] Menerbitkan gambar debug ke `/line_tracker/debug_image`.
- [x] Menerbitkan `/cmd_vel`, `/line_tracker/wheel_speeds`, dan `/motor_pwm`.
- [x] Mengirim PWM kiri/kanan melalui serial ke firmware.
- [ ] Memvalidasi arah belok dan semua satuan pada robot fisik.

## 1. Keselamatan, hardware, dan antarmuka dasar (Tahap 0)

- [ ] Ukur diameter efektif kedua roda dan jarak antara pusat roda kiri-kanan (*wheel track*).
- [ ] Catat konfigurasi encoder: pulsa per putaran (*CPR*), rasio gearbox, dan resolusi efektif.
- [ ] Validasi arah putaran motor kiri/kanan dan polaritas hitungan pulsa encoder.
- [ ] Validasi koneksi ROS ke `Serial1` Arduino Mega dan pastikan port stabil.
- [ ] Tambahkan **command timeout** di firmware: jika dalam $\ge 500\text{ ms}$ tidak ada frame serial valid, motor otomatis berhenti.
- [ ] Uji fungsi E-stop fisik: pastikan selalu memutus pergerakan motor seketika dan tidak dapat diabaikan oleh software.
- [ ] Dokumentasikan satuan kanonik (m, rad, s), urutan token serial, dan simpan nilai kalibrasi di konfigurasi YAML.

## 2. Kontrol kecepatan tertutup & serial dua arah (Tahap 1)

- [ ] Tentukan format frame serial target kecepatan roda bertanda (misal `mm/s`).
- [ ] Implementasikan perhitungan kecepatan aktual roda di firmware dari delta pulsa encoder dan interval waktu sampling.
- [ ] Implementasikan loop kontrol PI/PID kecepatan terpisah untuk roda kiri dan roda kanan pada firmware.
- [ ] Implementasikan arah maju, mundur, dan putar di tempat berdasarkan tanda target kecepatan.
- [ ] Buat pengiriman telemetri *delta ticks* atau kecepatan aktual dari firmware kembali ke ROS 2.
- [ ] Tambahkan proteksi serial: *delimiter* paket yang ketat dan *checksum* (CRC8/XOR) untuk menolak derau elektrik motor.
- [ ] Uji stabilitas kecepatan roda saat diberi beban dan saat tegangan baterai berubah.
- [ ] Uji perilaku motor saat kabel serial dicabut tiba-tiba (harus memicu timeout).

## 3. Odometri DDMR dan estimasi pose di ROS 2 (Tahap 2)

- [ ] Di ROS 2, konversikan delta pulsa encoder yang diterima menjadi jarak roda $\Delta s_L$ dan $\Delta s_R$.
- [ ] Hitung $\Delta s$, $\Delta\theta$, dan perbarui pose robot $(x, y, \theta)$ di koordinat global ROS 2.
- [ ] Publikasikan topik `nav_msgs/msg/Odometry` dengan frame `odom` dan `base_link`.
- [ ] Publikasikan transformasi koordinat `odom -> base_link` menggunakan `tf2_ros`.
- [ ] Lakukan uji jalan lurus pada jarak fisik terukur (misal 2 meter) dan kalibrasi diameter roda.
- [ ] Lakukan uji putar 360° di tempat dan kalibrasi parameter jarak antar-roda (*wheel track*).
- [ ] Visualisasikan pose, path, dan TF di RViz untuk memastikan pergerakan halus tanpa patah.
- [ ] Dokumentasikan laju akumulasi drift odometri per meter perjalanan.

## 4. Eksperimen persepsi dan deteksi ArUco fisik (Tahap 3)

- [ ] Tentukan skema penempatan fisik marker ArUco (lantai atau tiang/dinding persimpangan).
- [ ] Evaluasi FOV kamera: pastikan kamera mampu melihat marker ArUco dengan jarak deselerasi yang aman sebelum tiba di intersection.
- [ ] Kalibrasi parameter intrinsik dan distorsi lensa kamera menggunakan `camera_calibration` ROS 2.
- [ ] Implementasikan node deteksi ArUco menggunakan OpenCV `cv2.aruco`.
- [ ] Implementasikan filter *debouncing*: konfirmasi deteksi stabil selama beberapa frame berturut-turut sebelum menandai waypoint terdeteksi.
- [ ] Uji kehandalan deteksi pada berbagai kondisi pencahayaan, blur gerak, dan sudut kemiringan kamera.
- [ ] Tetapkan ukuran fisik marker dan jarak pemasangan yang optimal.

## 5. Refaktorisasi modular ROS 2 & format peta graf (Tahap 4)

- [ ] Pecah node monolitik `line_tracker_node` menjadi node modular:
  - [ ] `serial_bridge_node` (I/O UART dua arah, framing, checksum).
  - [ ] `odometry_node` (perhitungan pose dan publish TF).
  - [ ] `line_detector_node` (pemrosesan citra garis).
  - [ ] `aruco_detector_node` (pemrosesan citra marker ArUco).
  - [ ] `mission_controller_node` (state machine navigasi misi).
- [ ] Definisikan format file peta lintasan berbasis graf (YAML/JSON): node waypoint, relasi edge, arah, batas kecepatan, dan ID ArUco.
- [ ] Buat Waypoint & Route Manager untuk menghasilkan rute sekuensial dari titik awal ke tujuan.
- [ ] Definisikan FSM misi: `IDLE`, `NAVIGATING`, `AT_INTERSECTION`, `ARRIVED`, `PAUSED`, `ERROR`.
- [ ] Buat unit test untuk validasi integritas struktur file peta (tidak ada waypoint yatim/loop buntu tak terdefinisi).

## 6. Navigasi line tracking percabangan di intersection (Tahap 5)

- [ ] Implementasikan logika deselerasi saat odometri mendeteksi robot mendekati persimpangan.
- [ ] Implementasikan verifikasi ArUco di persimpangan yang sesuai dengan rute misi.
- [ ] Gunakan pose ArUco untuk mengoreksi akumulasi drift odometri (*checkpoint reset*).
- [ ] Kembangkan logika pemilihan cabang (*branch switching*) pada kontur garis (belok kiri, lurus, atau belok kanan).
- [ ] Implementasikan penguncian garis baru setelah menyelesaikan belokan di persimpangan.
- [ ] Implementasikan penanganan kondisi gagal:
  - [ ] Garis hilang di luar persimpangan: robot berhenti seketika.
  - [ ] ArUco tidak terdeteksi pada jarak perkiraan: robot melambat hingga berhenti (minta asistensi).
  - [ ] ArUco tidak cocok dengan urutan rute: robot menghentikan misi demi keselamatan.
- [ ] Uji coba navigasi otonom multititik melewati minimal 3 persimpangan berbeda.

## 7. Integrasi IoT: telemetry dan command tingkat misi (Tahap 6)

- [ ] Pilih protokol komunikasi IoT (misalnya MQTT) dan rancang arsitektur topiknya.
- [ ] Publikasikan telemetri read-only: pose terkoreksi, ID waypoint saat ini, status FSM misi, kecepatan roda, dan status koneksi.
- [ ] Buat dashboard monitoring web/desktop read-only untuk pengujian awal.
- [ ] Rancang skema command berformat JSON berversi lengkap dengan ID pesan, timestamp, dan ACK/NACK.
- [ ] Implementasikan handler command tingkat misi: `go_to_waypoint`, `pause`, `resume`, `cancel`.
- [ ] Pastikan command IoT divalidasi ketat dan tidak dapat mengirimkan PWM mentah atau membatalkan E-stop fisik.
- [ ] Uji perilaku robot saat jaringan IoT putus tiba-tiba (robot harus menyelesaikan segmen aman atau berhenti aman).

## 8. Uji integrasi akhir dan dokumentasi

- [ ] Lakukan prosedur uji meja (roda terangkat) untuk seluruh alur dari command IoT hingga respon motor.
- [ ] Lakukan uji lintasan penuh tertutup dengan skenario persimpangan kompleks.
- [ ] Dokumentasikan diagram wiring hardware final, pinout, dan parameter kalibrasi.
- [ ] Dokumentasikan prosedur peluncuran sistem (*launch files*), penanganan error (*recovery*), dan manual operasi.
- [ ] Rekam video demonstrasi misi penuh dari command IoT hingga robot tiba di tujuan dan melaporkan status selesai.
