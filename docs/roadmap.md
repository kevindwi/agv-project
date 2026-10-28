# Roadmap Pengembangan AGV (Capstone Design ARASHI)

Dokumen ini merinci tahapan untuk mencapai tiga tujuan proyek:

1. Navigasi berbasis waypoint pada lintasan terpandu (line-guided).
2. Line tracking (path tracking) dengan identifikasi intersection menggunakan ArUco
   atau metode landmark/topologi lintasan lain yang sesuai.
3. Integrasi IoT untuk telemetry dan command tingkat misi.

**Tenggat:** pameran awal Desember 2026, venue Rektorat UNESA. Checklist operasional
ada di [checklist.md](checklist.md).

**Skenario demo minimum:** robot menjalankan rute multititik melewati tiga checkpoint
(CP1, CP2, CP3) pada lintasan bergaris dengan marker ArUco, dengan tujuan dikirim dan
status dipantau dari GUI web melalui IoT. Semua prioritas di bawah mengacu ke skenario ini.

## Kondisi proyek saat ini

- Kamera, deteksi garis, dan PID line tracking tersedia di node ROS 2.
- ROS mengirim `cmd_vel` (`linear.x` dalam m/s dan `angular.z` dalam rad/s)
  melalui frame serial `C,<linear_x>,<angular_z>`.
- Firmware melakukan kinematika balik DDMR (`cmd_vel` → target ticks kiri/kanan per
  interval kontrol), membaca delta encoder, menjalankan PID per roda, lalu
  mengeluarkan PWM bertanda.
- Firmware mengirim delta encoder kembali ke ROS dengan frame
  `E,<left_ticks>,<right_ticks>`.
- `line_tracker_node` menerbitkan delta encoder ke `/line_tracker/encoder_delta`;
  `odometry_node` (sudah node terpisah) mengintegrasikannya menjadi pose dan twist
  pada `/odom`, TF `odom -> base_link`, serta jejak `/path` untuk RViz.
- Parameter fisik: roda Ø 0,15 m, *wheel track* $b$ = 0,48 m, encoder 30 pulsa/putaran
  motor × gearbox 30:1 = 900 pulsa/putaran roda.
- Kalibrasi, uji RViz, drift, dan validasi fisik odometri masih terbuka.
- Kontrol PID, tanda encoder, timeout, dan odometri belum divalidasi menyeluruh di
  robot fisik. Gain PID masih nilai awal.
- **Masalah keselamatan terbuka:** E-stop yang aktif masih bisa di-reset dan di-start
  lagi (lihat Tahap 0).
- Protokol serial belum memiliki checksum; pembacaan serial ROS masih berada di
  callback `line_tracker_node`.
- `line_tracker_node.py` masih monolitik (kamera + PID garis + serial).
- Belum ada: sensor orientasi (IMU), peta waypoint, route planner, deteksi ArUco,
  komunikasi IoT.

Prioritas saat ini adalah validasi kontrol roda dan odometri secara fisik, lalu
menambah ArUco, peta/misi minimum, dan IoT.

## Gambaran arsitektur yang dituju

```text
GUI web / IoT command (localhost, lalu Firebase)
          │ tujuan waypoint (CP1–CP3)
          ▼
Waypoint manager ──► perencana rute pada graf lintasan
                            │ target cabang & checkpoint
                            ▼
Kamera + Detektor ArUco ──► controller navigasi garis & percabangan
          │                          │ cmd_vel (m/s, rad/s)
          └── koreksi drift pose     ▼
                        serial bridge: C,<linear_x>,<angular_z>
                        (dua arah; checksum direncanakan)
                                     │
                                     ▼
   firmware: kinematika balik DDMR → target ticks roda → PID per roda → PWM
                                     │
                                     ▼
                             driver motor + motor
                                     │ pulsa encoder
                                     ▼
                        firmware: delta ticks (+ yaw IMU BNO055 via I2C)
                                     │ telemetry serial balik: E,... / I,...
                                     ▼
          ROS 2: odometry_node (+ fusi IMU, opsional) → pose (x, y, θ)
                                     │ pose, status, telemetry
                                     └────────────► IoT dashboard
```

> Catatan: kinematika balik DDMR dilakukan di **firmware**; ROS mengirim `cmd_vel`
> standar, bukan target roda. Dokumen dan kode harus konsisten pada satu pilihan ini.

---

## Jadwal usulan menuju pameran

Penomoran minggu mengikuti catatan tim (Minggu-2 = 5–9 Okt). Target awal (ArUco
Minggu-2, basic IoT Minggu-3, command IoT Minggu-4) **belum tercapai berdasarkan
checklist**, sehingga jadwal disusun ulang. Ini usulan; sesuaikan dengan kapasitas tim.

| Minggu | Tanggal | Fokus | Keluaran yang diharapkan |
| ---: | :--- | :--- | :--- |
| 5 | 26–30 Okt | Tahap 0–1: perbaikan E-stop, tanda encoder/arah motor, checksum serial, mulai tuning PID | Robot aman dan gerak maju/mundur/putar benar |
| 6 | 2–6 Nov | Tahap 2 + 2B: kalibrasi odometri, IMU BNO055. Paralel: Tahap 3 (ArUco) dan 6A (IoT localhost, telemetri) | Odometri tervalidasi; ArUco terdeteksi; GUI web menampilkan pose. **Keputusan upgrade mekanik/hardware** |
| 7 | 9–13 Nov | Tahap 4A: peta graf + FSM + route manager minimum; mulai Tahap 5 | Peta CP1–CP3 termuat; robot berhenti di checkpoint |
| 8 | 16–20 Nov | Tahap 5 + 6B/6C: navigasi percabangan, command IoT (`go_to_waypoint`, `pause`, `cancel`), Firebase | Rute CP1→CP2→CP3 dari command IoT |
| 9 | 23–27 Nov | Tahap 7: uji penuh, dokumentasi, video, simulasi kondisi venue. **Freeze fitur** | Sistem siap demo |
| 10 | 30 Nov–4 Des | Pameran; hanya perbaikan bug kritis | |

---

## Tahap 0 — Rapikan antarmuka dan keselamatan dasar

### Pekerjaan

- Tetapkan satuan dan konvensi arah di seluruh sistem: meter, detik, radian,
  `linear.x` dalam m/s, `angular.z` dalam rad/s, kiri/kanan sesuai urutan kanonik.
- Ukur diameter roda efektif dan jarak antara pusat roda kiri-kanan (*wheel track*);
  dokumentasikan nilai serta prosedur pengukurannya.
- Pastikan arah positif encoder, motor kiri, motor kanan, dan `angular.z` ROS
  konsisten.
- Tambahkan **command timeout** di firmware: bila tidak ada paket serial valid
  dalam batas waktu yang ditentukan (misalnya 500 ms), motor otomatis berhenti.
- **E-stop fisik harus selalu menghentikan motor seketika dan tidak dapat dibatalkan
  secara software oleh ROS maupun IoT.** Saat ini E-stop yang aktif masih bisa
  di-reset dan di-start lagi; ini harus diperbaiki sebelum pengujian di lantai
  dilanjutkan. Sasaran desain:
  - E-stop memutus daya aktuator di jalur hardware (kontak NC / relay / kontaktor),
    bukan hanya flag di firmware.
  - Status E-stop bersifat *latched*: motor baru boleh aktif lagi setelah tombol
    dilepas secara fisik **dan** ada aksi start eksplisit di robot.
  - Selama tombol masih tertekan, perintah reset/start dari software diabaikan.
- Catat konfigurasi hardware: pinout, rasio gearbox, pulsa encoder per putaran (CPR),
  mode dekode encoder (x1/x2/x4), tegangan kerja motor, dan port UART.

### Selesai jika

- Uji command roda kiri/kanan menghasilkan arah putaran yang benar pada robot fisik.
- E-stop dan command timeout terbukti menghentikan motor tanpa bergantung pada ROS,
  dan E-stop tidak bisa di-bypass lewat serial/ROS/IoT.
- Parameter fisik robot tercatat dan diintegrasikan ke file konfigurasi YAML.

---

## Tahap 1 — Kontrol kecepatan roda tertutup dan protokol serial dua arah

**Status: implementasi dasar sudah ada; tuning, checksum, dan validasi hardware masih terbuka.**

### Prinsip pemisahan tugas
Firmware Arduino Mega 2560 (8-bit AVR 16 MHz) tidak memiliki hardware floating-point
unit. Implementasi saat ini tetap memakai operasi `float` sederhana untuk konversi
`cmd_vel` ke target ticks dan menjalankan PID lokal. Perhitungan pose serta
trigonometri dilakukan di ROS 2.

### Pekerjaan

- [x] Ganti perintah PWM langsung dengan frame `C,<linear_x>,<angular_z>`; firmware
  menghitung target ticks kiri dan kanan untuk tiap interval kontrol.
- [x] Tambahkan PID terpisah untuk tiap roda; input PID adalah delta ticks terukur
  pada interval kontrol, output-nya PWM bertanda.
- [x] Tambahkan dukungan arah motor berdasarkan tanda output PWM.
- [x] Tambahkan frame balik `E,<left_ticks>,<right_ticks>` untuk delta encoder.
- [x] Tambahkan timeout perintah serial 500 ms di firmware.
- [ ] Tambahkan proteksi data serial: framing delimiter yang ketat dan checksum
  sederhana (CRC8 atau XOR) untuk mengantisipasi derau elektrik dari motor.
- [ ] Tuning gain PID dan validasi kecepatan roda pada beban serta tegangan baterai
  berbeda. Kontrol saat ini menggunakan target ticks per interval, belum estimator
  kecepatan dalam m/s atau RPM.
- [ ] Validasi arah dan tanda encoder pada gerak maju, mundur, dan putar di tempat.
- [ ] Tambahkan keluaran kecepatan aktual roda (RPM atau m/s) di ROS untuk telemetri
  dan tuning.

### Selesai jika

- Kedua roda dapat mempertahankan kecepatan target pada beban dan kondisi baterai berbeda.
- Perintah target negatif membalik arah roda secara mulus.
- Data telemetri encoder terbaca stabil di ROS 2 tanpa korupsi frame serial saat motor
  berputar (frame rusak ditolak oleh checksum).
- Kehilangan koneksi serial memicu timeout dan menghentikan motor secara aman.

---

## Tahap 2 — Odometri DDMR dan estimasi pose di ROS 2

**Status: implementasi dasar tersedia; kalibrasi dan validasi fisik masih terbuka.**

### Pekerjaan

- [x] Teruskan delta ticks dari line tracker ke `/line_tracker/encoder_delta`.
- [x] Pisahkan perhitungan odometri ke `odometry_node` dan konversikan delta ticks
  menjadi perpindahan roda kiri $\Delta s_L$ dan kanan $\Delta s_R$.
- [x] Hitung perpindahan robot di koordinat global (dengan $b$ = *wheel track* = 0,48 m,
  $d_t = \pi D / CPR = \pi \cdot 0{,}15 / 900 \approx 0{,}0005236$ m/tick):

  ```text
  ΔsL, ΔsR = nL × d_t, nR × d_t
  Δs       = (ΔsR + ΔsL) / 2
  Δθ       = (ΔsR - ΔsL) / b
  θ_mid    = θ + Δθ / 2
  x        = x + Δs × cos(θ_mid)
  y        = y + Δs × sin(θ_mid)
  θ        = θ + Δθ
  ```

  Pose awal $(0,0,0)$ menghadap sumbu global $+x$.
- [x] Publikasikan pose dan twist sebagai `nav_msgs/msg/Odometry` di `/odom` dengan
  frame `odom` dan `base_link`.
- [x] Publikasikan transformasi koordinat `odom -> base_link` via `tf2_ros`.
- [x] Publikasikan jejak pose `nav_msgs/msg/Path` di `/path`.
- [ ] Tetapkan kovariansi pose dan twist yang sesuai dengan karakteristik sensor.
- [ ] Kalibrasi diameter roda efektif dan *wheel track* $b$ menggunakan uji empiris,
  dan simpan hasilnya sebagai faktor skala (*scaling factor*) di konfigurasi YAML.

### Uji kalibrasi yang disarankan

1. **Uji jarak lurus:** Jalankan robot lurus sejauh jarak terukur (misal 2 meter);
   bandingkan jarak fisik dengan pembacaan odometri, lalu kalibrasi diameter roda.
2. **Uji putaran:** Putar robot di tempat sebanyak 360° (atau 5 putaran penuh); periksa
   galat orientasi akhir $\theta$, lalu kalibrasi parameter jarak antar roda $b$.
3. Dokumentasikan estimasi akumulasi drift per meter perjalanan pada permukaan operasi.
4. Di RViz, pilih Fixed Frame `odom`, tambahkan display `/odom`, `/path`, dan TF.

> Jika mekanik/roda diubah (lihat jalur Mekanik di bawah), seluruh kalibrasi di atas
> harus diulang.

### Selesai jika

- Odometri bergerak konsisten sesuai uji lurus dan putar dalam batas toleransi yang disepakati.
- Pose dan lintasan robot dapat divisualisasikan dengan mulus di RViz tanpa TF putus.
- Batasan drift odometri terdokumentasi dengan baik.

---

## Tahap 2B — Orientasi dengan IMU BNO055

Heading dari encoder saja mudah menyimpang saat roda selip. Catatan tim menetapkan
sensor orientasi BNO055 (I2C) sebagai bagian target awal.

### Pekerjaan

- Tentukan sambungan I2C: ke Arduino Mega (SDA 20, SCL 21; firmware mengirim yaw lewat
  frame serial tambahan, misalnya `I,<yaw_rad>`) atau langsung ke komputer ROS.
  Sambungan ke Mega lebih disarankan karena BNO055 dikenal bermasalah dengan
  *clock stretching* pada I2C hardware Raspberry Pi.
- Mulai dari mode fusi tanpa magnetometer (mode IMU), karena motor dan baterai dapat
  mengganggu magnetometer. Pindah ke NDOF hanya jika terbukti stabil.
- Publikasikan `sensor_msgs/msg/Imu` (mis. `/imu/data`) dengan frame yang konsisten
  terhadap `base_link`; samakan konvensi tanda yaw dengan $\theta$ odometri.
- Gunakan yaw IMU untuk (a) membandingkan dengan $\Delta\theta$ odometri pada uji putar,
  (b) mendeteksi selip, dan (c) opsional fusi dengan `robot_localization` (EKF).
- Pertimbangkan *heading hold* saat berbelok di persimpangan memakai yaw IMU.

### Selesai jika

- Yaw IMU stabil, tanda dan sumbu benar, dan galat putar 360° lebih kecil daripada
  odometri murni (atau perbedaannya terdokumentasi).

---

## Tahap 3 — Eksperimen persepsi dan deteksi ArUco fisik

### Alasan tahapan ini didahulukan sebelum perancangan peta
Format peta dan penempatan titik henti/intersection sangat bergantung pada batasan fisik optik:
jarak pandang deteksi, sudut kemiringan (*pitch*) kamera garis, pencahayaan, dan ukuran marker.
Eksperimen fisik harus dilakukan sebelum menetapkan spesifikasi graf peta.
Tahap ini dapat berjalan paralel dengan Tahap 2 dan 2B karena tidak bergantung pada odometri.

### Gerbang keputusan: ArUco atau alternatif

ArUco adalah pilihan utama tim, tetapi bukan syarat untuk memilih cabang. Persimpangan
dapat dikenali dari bentuk garis pada kamera, dan pilihan cabang ditentukan oleh rute
yang sedang dijalankan. Lokasi intersection dapat ditentukan dari jarak odometri sejak
checkpoint terakhir atau landmark lain (QR, RFID, sakelar lantai, sensor jarak).
Dengan odometri saja, drift tidak mendapat koreksi absolut; rute pendek dan uji
berulang diperlukan.

Putuskan di akhir Minggu 6: jika deteksi ArUco belum andal pada kecepatan operasi,
beralih ke alternatif di atas agar jadwal tidak mundur.

### Pekerjaan

- Tentukan skema penempatan ArUco: di lantai lintasan atau pada tiang/dinding samping.
- Evaluasi bidang pandang (*FOV*) kamera garis saat ini: apakah satu kamera cukup untuk
  melihat garis di bawah sekaligus mendeteksi ArUco di depan pada jarak deselerasi yang aman,
  atau apakah diperlukan sudut kamera khusus / kamera sekunder.
- Kalibrasi lensa kamera (intrinsik dan distorsi) menggunakan `camera_calibration` ROS 2.
- Implementasikan node deteksi ArUco (OpenCV `cv2.aruco`, Python/rclpy).
- Terapkan mekanisme *debouncing* dan konfirmasi: marker harus terdeteksi secara konsisten
  selama beberapa frame sebelum diterima sebagai sinyal waypoint.
- Ukur jarak efektif deteksi dan estimasi pose marker relatif terhadap kamera.
- Uji pada berbagai pencahayaan, *motion blur*, dan sudut kamera (kondisi ruang pameran
  berbeda dari lab).

### Selesai jika

- Kamera dapat mendeteksi ID ArUco secara andal pada kecepatan gerak robot yang direncanakan.
- Robot memiliki cukup jarak deselerasi antara saat marker pertama kali terdeteksi hingga
  mencapai pusat persimpangan.
- Parameter ukuran marker fisik dan jarak penempatan optimal telah ditetapkan dan dicatat.

---

## Tahap 4 — Peta graf, misi, dan refaktorisasi ROS 2

Tahap ini dibagi dua agar refaktorisasi besar tidak menghambat jalur ke demo.

### Tahap 4A — Wajib untuk demo

- Buat `aruco_detector_node` (dari Tahap 3) dan `mission_controller_node` sebagai node baru.
- Definisikan format peta lintasan berbasis graf (YAML/JSON):
  - **Node (Waypoint):** ID unik, pemetaan ke ID ArUco, koordinat perkiraan $(x, y)$,
    tipe titik (intersection, stop, charging). Untuk demo: CP1, CP2, CP3.
  - **Edge (Segmen Garis):** Koneksi antar-node, arah gerak, batas kecepatan, instruksi
    percabangan di persimpangan (belok kiri, lurus, belok kanan).
- Buat Waypoint & Route Manager yang menerima perintah tujuan dan menghasilkan daftar
  urutan waypoint serta instruksi percabangan yang harus dilalui.
- Definisikan state machine misi: `IDLE`, `NAVIGATING`, `AT_INTERSECTION`, `ARRIVED`,
  `PAUSED`, `ERROR`, dan `ESTOP` (status dari firmware, hanya dibaca oleh controller misi).

### Tahap 4B — Refaktorisasi lanjutan (prioritas lebih rendah)

Pecah node monolitik [`line_tracker_node.py`](ros2_ws/src/line_tracker_ros2/line_tracker_ros2/line_tracker_node.py)
menjadi node dengan tanggung jawab tunggal. `odometry_node` sudah terpisah.

1. `serial_bridge_node`: komunikasi serial dua arah, framing, checksum, publikasi
   raw encoder/ticks dan pengiriman `cmd_vel`.
2. `line_detector_node`: memproses frame kamera untuk mendeteksi garis dan menerbitkan error garis.

Kerjakan 4B hanya jika jalur 4A dan Tahap 5 sudah aman terhadap jadwal. Checksum serial
tetap dikerjakan di Tahap 1 pada kode yang ada, tanpa menunggu refaktorisasi.

### Selesai jika

- File peta graf dapat dimuat dan diverifikasi validitas koneksinya (tanpa waypoint
  yatim atau loop buntu tak terdefinisi).
- Waypoint Manager dapat menghasilkan urutan rute dari titik awal ke tujuan.
- Node baru berjalan lewat topik standar dan dapat diuji terpisah.

---

## Tahap 5 — Navigasi line tracking percabangan (Branching Line Tracking)

### Karakteristik navigasi
AGV ini adalah **Line-Guided AGV**, bukan AGV navigasi ruang bebas tanpa panduan.
Robot menempel pada garis fisik dan menggunakan odometri serta ArUco sebagai pemandu segmen
dan pengambil keputusan cabang di persimpangan:

```text
Ikuti garis segmen saat ini
          ↓
Odometri mendeteksi mendekati intersection → kurangi kecepatan
          ↓
Deteksi ArUco terkonfirmasi cocok dengan waypoint target
          ↓
Koreksi drift estimasi posisi odometri berdasarkan pose/posisi ArUco
          ↓
Eksekusi instruksi percabangan (pilih cabang garis: belok kiri / lurus / kanan)
          ↓
Kunci deteksi pada garis baru & lanjutkan navigasi segmen berikutnya
```

### Pekerjaan

- Implementasikan transisi state saat mendekati persimpangan berdasarkan estimasi jarak odometri
  dan deteksi visual ArUco.
- Kembangkan logika pemilihan cabang (*branch switching*) pada pengolah kontur garis kamera
  (misalnya memilih ROI kontur garis sebelah kiri saat rute memerintahkan belok kiri).
- Gunakan pose ArUco di persimpangan sebagai *ground-truth checkpoint* untuk mengoreksi
  akumulasi drift odometri.
- Implementasikan penanganan kondisi darurat:
  - Garis hilang di luar persimpangan: robot berhenti seketika.
  - ArUco tidak terdeteksi pada jarak perkiraan: robot melambat hingga berhenti dan meminta bantuan (*recovery mode*).
  - Terbaca marker tak dikenal / tidak sesuai urutan rute: robot berhenti dengan aman.

### Selesai jika

- Robot berhasil menyelesaikan rute multititik (CP1 → CP2 → CP3) yang melibatkan
  persimpangan (belok kiri, lurus, dan kanan) tanpa intervensi manual.
- Pemilihan cabang garis berjalan mulus tanpa kehilangan garis.
- Drift odometri terkoreksi setiap kali melewati persimpangan bertanda ArUco.

---

## Tahap 6 — Integrasi IoT: telemetry dan command tingkat misi

Dibagi tiga langkah mengikuti catatan tim (localhost → database → mekanisme waypoint).
Telemetri read-only hanya membutuhkan `/odom`, sehingga **6A dapat dimulai paralel
sejak Minggu 6**, tidak perlu menunggu Tahap 5.

### 6A — Localhost + GUI web

- Backend lokal di komputer robot (mis. WebSocket/REST) yang **mengirim data** telemetri
  dan **menerima data** command, ditampilkan di GUI web.
- Mulai dengan telemetri read-only, lalu tambah command.

### 6B — Database (Firebase)

- Simpan telemetri/log misi dan status ke Firebase untuk dashboard jarak jauh dan rekaman uji.
- Robot **harus tetap aman dan dapat menyelesaikan demo tanpa internet**. Jaringan di
  venue belum tentu andal, sehingga 6A (localhost) menjadi mode cadangan.
- Atur aturan keamanan (autentikasi dan *security rules*) agar hanya operator berwenang
  yang dapat menulis command.
- MQTT tetap menjadi alternatif jika Firebase menimbulkan latensi atau kendala jaringan.

### 6C — Mekanisme waypoint dan command

- Command tingkat misi: `go_to_waypoint(CP1|CP2|CP3)` atau urutan rute bernama.
- Kendali alur: `pause`, `resume`, `cancel_mission`, dan `ack_error` (reset/pemulihan kesalahan).
- **Catatan keselamatan:** IoT **dilarang keras** mengirimkan nilai PWM mentah atau
  kecepatan motor langsung tanpa melalui controller misi lokal di robot. Command IoT
  tidak boleh mengesampingkan tombol E-stop fisik.

### Telemetry yang berguna

- Pose/odometri terkoreksi dan ID waypoint saat ini.
- Kecepatan target dan aktual tiap roda (RPM atau m/s).
- Status misi dan FSM: `IDLE`, `NAVIGATING`, `AT_INTERSECTION`, `ARRIVED`, `PAUSED`, `ERROR`, `ESTOP`.
- Kualitas deteksi garis dan ID ArUco terakhir yang terverifikasi.
- Status komunikasi serial dan kesehatan sensor/node (termasuk IMU).

### Rekomendasi implementasi

- Pisahkan channel/koleksi untuk telemetri dan command.
- Definisikan skema payload terstruktur (JSON berversi) lengkap dengan ID pesan, timestamp, dan ACK/NACK.
  Tolak command kedaluwarsa atau duplikat.
- Terapkan safety timeout pada sisi IoT bridge: jika koneksi jaringan IoT terputus, robot tetap
  menyelesaikan segmen berjalan atau berhenti dengan aman sesuai kebijakan operasional.

### Selesai jika

- Dashboard IoT menampilkan telemetri robot secara realtime dengan latensi terkontrol.
- Operator dapat mengirim perintah tujuan dan membatalkan misi dari jarak jauh.
- Putusnya jaringan tidak menyebabkan bahaya atau perilaku aneh pada robot fisik.

---

## Tahap 7 — Uji integrasi akhir dan dokumentasi

- Uji meja (roda terangkat) untuk seluruh alur dari command IoT hingga respon motor.
- Uji lintasan penuh tertutup dengan skenario persimpangan kompleks, termasuk dengan
  kondisi pencahayaan dan jaringan yang menyerupai venue.
- Rekam log dan *rosbag* setiap uji; dokumentasikan wiring final, pinout, parameter
  kalibrasi, *launch files*, prosedur pemulihan, dan manual operasi.
- Rekam video demonstrasi misi penuh sebagai cadangan jika demo langsung bermasalah.

### Selesai jika

- Skenario demo minimum berjalan berulang (mis. 5 kali berturut-turut) tanpa intervensi.
- Dokumentasi dan video siap sebelum pameran.

---

## Jalur paralel — Upgrade hardware dan mekanik (opsional)

Catatan tim memuat upgrade hardware (jika memungkinkan) dan upgrade mekanik. Perubahan
ini berisiko terhadap jadwal karena mengubah diameter roda, *wheel track*, bobot, dan
posisi kamera/IMU, sehingga **kalibrasi odometri, tuning PID, dan FOV kamera harus
diulang**.

- Putuskan ruang lingkupnya paling lambat akhir Minggu 6; setelah itu perubahan
  mekanik dibekukan.
- Jika dikerjakan, lakukan sebelum kalibrasi final di Tahap 2, bukan sesudahnya.
- Tetapkan prioritas (mis. dudukan kamera/IMU, kekakuan sasis, kabel E-stop) dan catat
  tiap perubahan di dokumentasi hardware.

---

## Prioritas dan urutan tahapan

| Urutan | Fase | Fokus Utama | Alasan Rekayasa |
| ---: | :--- | :--- | :--- |
| **1** | **Tahap 0** | Keselamatan fisik (termasuk perbaikan E-stop), parameter mekanik, command timeout | Fondasi wajib sebelum motor diuji di lantai |
| **2** | **Tahap 1** | Kontrol kecepatan tertutup, protokol serial dua arah, checksum | Respon gerak linier dan terukur; proteksi noise |
| **3** | **Tahap 2 + 2B** | Odometri DDMR, kalibrasi, IMU BNO055 | Posisi dan orientasi tanpa membebani mikrokontroler |
| **3** (paralel) | **Tahap 3** | Uji persepsi optik dan ArUco fisik | Memastikan kamera mampu melihat marker sebelum mendesain peta |
| **3** (paralel) | **Tahap 6A** | IoT localhost, telemetri read-only | Hanya butuh `/odom`; mengurangi risiko di akhir |
| **4** | **Tahap 4A** | Peta graf, FSM misi, route manager | Representasi rute CP1–CP3 |
| **5** | **Tahap 5** | Navigasi line tracking percabangan | Navigasi otonom terpandu garis antar-waypoint |
| **6** | **Tahap 6B/6C** | Firebase, command misi | Menghubungkan robot ke dashboard secara aman |
| **7** | **Tahap 7** | Uji integrasi dan dokumentasi | Kesiapan demo |
| Opsional | **Tahap 4B**, jalur mekanik | Refaktorisasi lanjutan; upgrade hardware | Dikerjakan hanya jika jadwal memungkinkan |

---

## Saran tambahan

1. **Jaga isolasi komputasi:** Arduino Mega 2560 difokuskan hanya untuk I/O real-time (baca interupsi encoder, hitung PID motor, kontrol PWM, baca IMU, dan pantau E-Stop). Serahkan seluruh kalkulasi trigonometri, matriks, visi komputer, dan navigasi ke komputer Linux/ROS 2.
2. **Jangan andalkan odometri murni untuk rute panjang:** Selip roda pada lantai akan mendistorsi pose. Manfaatkan marker ArUco di intersection sebagai *anchor point* untuk mereset/mengoreksi akumulasi drift, dan IMU untuk heading.
3. **Peta sebagai Single Source of Truth:** Seluruh relasi node, ID ArUco, koneksi edge, dan arah belok didefinisikan dalam satu file konfigurasi terpusat (YAML), bukan di-hardcode ke dalam kode node.
4. **Catat log hasil uji:** Selalu rekam galat odometri, tingkat keberhasilan deteksi ArUco, dan log transisi FSM di setiap tahap pengujian untuk mempermudah isolasi masalah antara mekanik, sensor, kontrol, dan software.
5. **Rencanakan untuk kondisi venue:** Jaringan, pencahayaan, dan lantai di Rektorat UNESA berbeda dari lab. Siapkan mode offline (localhost), uji pencahayaan, dan baterai cadangan.
