# Roadmap Pengembangan AGV

Dokumen ini merinci tahapan lanjutan untuk mencapai tiga tujuan proyek:

1. Navigasi berbasis waypoint pada lintasan terpandu (line-guided).
2. Line tracking dengan identifikasi intersection menggunakan marker ArUco.
3. Integrasi IoT untuk telemetry dan command tingkat misi.

## Kondisi proyek saat ini

- Kamera, deteksi garis, dan PID line tracking tersedia di node ROS 2.
- ROS mengirim `cmd_vel` (`linear.x` dalam m/s dan `angular.z` dalam rad/s)
  melalui frame serial `C,<linear_x>,<angular_z>`.
- Firmware mengubah `cmd_vel` menjadi target ticks per interval kontrol,
  membaca delta encoder, menjalankan PID per roda, lalu mengeluarkan PWM bertanda.
- Firmware mengirim delta encoder kembali ke ROS dengan frame
  `E,<left_ticks>,<right_ticks>`.
- Node ROS mengakumulasi pose $(x,y,\theta)$ dari delta encoder, tetapi belum
  mempublikasikan `nav_msgs/Odometry` atau TF.
- Kontrol PID, tanda encoder, timeout, dan odometri belum divalidasi secara
  menyeluruh di robot fisik. Gain PID masih nilai awal.
- Protokol serial belum memiliki checksum; pembacaan serial ROS masih berada di
  callback node line tracker.
- Belum ada peta waypoint, path planner, deteksi ArUco, atau komunikasi IoT.
- Node ROS 2 masih bersifat monolitik dalam satu script (`line_tracker_node.py`).

Prioritas saat ini adalah validasi kontrol roda dan odometri secara fisik, lalu
mempublikasikan pose melalui antarmuka ROS standar sebelum waypoint dan navigasi
tingkat misi dibangun.

## Gambaran arsitektur yang dituju

```text
Rencana misi / IoT command
          │ tujuan waypoint
          ▼
Waypoint manager ──► perencana rute pada graf lintasan
                            │ target cabang & checkpoint
                            ▼
Kamera + Detektor ArUco ──► controller navigasi garis & percabangan
          │                          │ cmd_vel (m/s, rad/s)
          └── koreksi drift pose     ▼
                             DDMR kinematika balik
                                     │ target kecepatan roda (m/s)
                                     ▼
                        serial bridge (dua arah; checksum masih direncanakan)
                                     │ target roda bertanda
                                     ▼
                   firmware: kontrol PID kecepatan roda lokal
                                     │ PWM
                                     ▼
                             driver motor + motor
                                     │ pulsa encoder
                                     ▼
                        firmware: bacaan delta ticks
                                     │ telemetry serial balik
                                     ▼
                     ROS 2: node odometri & estimasi pose
                                     │ pose (x, y, θ), status, telemetry
                                     └────────────► IoT dashboard
```

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
- Pastikan E-stop fisik pada firmware selalu menghentikan motor seketika dan tidak
  dapat dibatalkan secara software oleh ROS maupun IoT.
- Catat konfigurasi hardware: pinout, rasio gearbox, pulsa encoder per putaran (CPR),
  tegangan kerja motor, dan port UART.

### Selesai jika

- Uji command roda kiri/kanan menghasilkan arah putaran yang benar pada robot fisik.
- E-stop dan command timeout terbukti menghentikan motor tanpa bergantung pada ROS.
- Parameter fisik robot tercatat dan diintegrasikan ke file konfigurasi YAML.

---

## Tahap 1 — Kontrol kecepatan roda tertutup dan protokol serial dua arah

**Status: implementasi dasar sudah ada; tuning dan validasi hardware masih terbuka.**

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
- [ ] Tuning gain PID dan validasi kecepatan roda pada beban serta tegangan baterai
  berbeda. Kontrol saat ini menggunakan target ticks per interval, belum estimator
  kecepatan dalam m/s atau ticks/s.
- [ ] Validasi arah dan tanda encoder pada gerak maju, mundur, dan putar di tempat.
- Tambahkan proteksi data serial (framing delimiter yang ketat dan checksum sederhana
  seperti CRC8 atau XOR) untuk mengantisipasi derau elektrik (*electrical noise*) dari motor.

### Selesai jika

- Kedua roda dapat mempertahankan kecepatan target pada beban dan kondisi baterai berbeda.
- Perintah target negatif membalik arah roda secara mulus.
- Data telemetri encoder terbaca stabil di ROS 2 tanpa korupsi frame serial saat motor berputar.
- Kehilangan koneksi serial memicu timeout dan menghentikan motor secara aman.

---

## Tahap 2 — Odometri DDMR dan estimasi pose di ROS 2

**Status: akumulasi pose dasar ada di node; publikasi standar ROS dan validasi belum ada.**

### Pekerjaan

- [x] Hitung perpindahan roda dan akumulasikan pose internal $(x,y,\theta)$ di node
  line tracker berdasarkan delta ticks dari firmware.
- [ ] Buat perhitungan odometri di ROS 2 berdasarkan delta ticks yang diterima dari firmware:
  konversi pulsa menjadi perpindahan roda kiri $\Delta s_L$ dan kanan $\Delta s_R$.
- Hitung perpindahan robot di koordinat global:

  ```text
  Δs     = (ΔsR + ΔsL) / 2
  Δθ     = (ΔsR - ΔsL) / b
  θ_mid  = θ + Δθ / 2
  x      = x + Δs × cos(θ_mid)
  y      = y + Δs × sin(θ_mid)
  θ      = θ + Δθ
  ```

- [ ] Publikasikan pose, twist, dan kovariansi sebagai `nav_msgs/msg/Odometry`.
- Publikasikan transformasi koordinat `odom -> base_link` via `tf2_ros`.
- Kalibrasi parameter efektif diameter roda dan jarak roda ($b$) menggunakan uji empiris.

### Uji kalibrasi yang disarankan

1. **Uji jarak lurus:** Jalankan robot lurus sejauh jarak terukur (misal 2 meter);
   bandingkan jarak fisik dengan pembacaan odometri, lalu kalibrasi diameter roda.
2. **Uji putaran:** Putar robot di tempat sebanyak 360° (atau 5 putaran penuh); periksa
   galat orientasi akhir $\theta$, lalu kalibrasi parameter jarak antar roda $b$.
3. Dokumentasikan estimasi akumulasi drift per meter perjalanan pada permukaan operasi.

### Selesai jika

- Odometri bergerak konsisten sesuai uji lurus dan putar dalam batas toleransi yang disepakati.
- Pose dan lintasan robot dapat divisualisasikan dengan mulus di RViz tanpa TF putus.
- Batasan drift odometri terdokumentasi dengan baik.

---

## Tahap 3 — Eksperimen persepsi dan deteksi ArUco fisik

### Alasan tahapan ini didahulukan sebelum perancangan peta
Format peta dan penempatan titik henti/intersection sangat bergantung pada batasan fisik optik:
jarak pandang deteksi, sudut kemiringan (*pitch*) kamera garis, pencahayaan, dan ukuran marker.
Eksperimen fisik harus dilakukan sebelum menetapkan spesifikasi graf peta.

### Pekerjaan

- Tentukan skema penempatan ArUco: di lantai lintasan atau pada tiang/dinding samping.
- Evaluasi bidang pandang (*FOV*) kamera garis saat ini: apakah satu kamera cukup untuk
  melihat garis di bawah sekaligus mendeteksi ArUco di depan pada jarak deselerasi yang aman,
  atau apakah diperlukan sudut kamera khusus / kamera sekunder.
- Kalibrasi lensa kamera (intrinsik dan distorsi) menggunakan `camera_calibration` ROS 2.
- Implementasikan node deteksi ArUco (OpenCV `cv2.aruco`).
- Terapkan mekanisme *debouncing* dan konfirmasi: marker harus terdeteksi secara konsisten
  selama beberapa frame sebelum diterima sebagai sinyal waypoint.
- Ukur jarak efektif deteksi dan estimasi pose marker relatif terhadap kamera.

### Selesai jika

- Kamera dapat mendeteksi ID ArUco secara andal pada kecepatan gerak robot yang direncanakan.
- Robot memiliki cukup jarak deselerasi antara saat marker pertama kali terdeteksi hingga
  mencapai pusat persimpangan.
- Parameter ukuran marker fisik dan jarak penempatan optimal telah ditetapkan dan dicatat.

---

## Tahap 4 — Refaktorisasi modular arsitektur ROS 2 & format peta graf

### Pekerjaan

- Pecah node monolitik [`line_tracker_node.py`](ros2_ws/src/line_tracker_ros2/line_tracker_ros2/line_tracker_node.py) menjadi node-node dengan tanggung jawab tunggal (*single responsibility*):
  1. `serial_bridge_node`: Komunikasi serial dua arah, publikasi raw encoder/ticks, dan pengiriman target kecepatan roda.
  2. `odometry_node`: Menghitung odometri pose dan mempublikasikan TF `odom -> base_link`.
  3. `line_detector_node`: Memproses frame kamera untuk mendeteksi garis dan menerbitkan error garis.
  4. `aruco_detector_node`: Memproses frame untuk deteksi ID ArUco dan pose marker.
  5. `mission_controller_node`: State machine pengatur navigasi dan eksekusi misi.
- Definisikan format peta lintasan berbasis graf (YAML/JSON):
  - **Node (Waypoint):** ID unik, pemetaan ke ID ArUco, koordinat perkiraan $(x, y)$, tipe titik (intersection, stop, charging).
  - **Edge (Segmen Garis):** Koneksi antar-node, arah gerak, batas kecepatan, instruksi percabangan di persimpangan (misal: belok kiri, lurus, belok kanan).
- Buat Waypoint & Route Manager yang menerima perintah tujuan dan menghasilkan daftar
  urutan waypoint serta instruksi percabangan yang harus dilalui.
- Definisikan state machine misi: `IDLE`, `NAVIGATING`, `AT_INTERSECTION`, `ARRIVED`, `PAUSED`, `ERROR`.

### Selesai jika

- Node-node ROS 2 berjalan modular, terhubung lewat topik standar, dan dapat diuji secara terpisah.
- File peta graf dapat dimuat dan diverifikasi validitas koneksinya.
- Waypoint Manager dapat menghasilkan urutan rute dari titik awal ke tujuan.

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

- Robot berhasil menyelesaikan rute multititik yang melibatkan beberapa persimpangan (belok kiri, lurus, dan kanan) tanpa intervensi manual.
- Pemilihan cabang garis berjalan mulus tanpa kehilangan garis.
- Drift odometri terkoreksi setiap kali melewati persimpangan bertanda ArUco.

---

## Tahap 6 — Integrasi IoT: telemetry dan command tingkat misi

### Telemetry yang berguna

- Pose/odometri terkoreksi dan ID waypoint saat ini.
- Kecepatan target dan aktual tiap roda.
- Status misi dan FSM: `IDLE`, `NAVIGATING`, `AT_INTERSECTION`, `ARRIVED`, `PAUSED`, `ERROR`, `ESTOP`.
- Kualitas deteksi garis dan ID ArUco terakhir yang terverifikasi.
- Status komunikasi serial dan kesehatan sensor/node.

### Command yang diizinkan

- Kirim tujuan misi: `go_to_waypoint(id)` atau urutan rute bernama.
- Kendali alur: `pause`, `resume`, `cancel_mission`.
- Reset status / pemulihan kesalahan (*error acknowledgement*).
- **Catatan keselamatan:** IoT **dilarang keras** mengirimkan nilai PWM mentah atau kecepatan motor langsung tanpa melalui controller misi lokal di robot. Command IoT tidak boleh mengesampingkan tombol E-stop fisik.

### Rekomendasi implementasi

- Gunakan broker ringan (misalnya MQTT) dengan channel terpisah untuk telemetri dan command.
- Definisikan skema payload terstruktur (JSON berversi) lengkap dengan ID pesan, timestamp, dan acknowledgement (ACK/NACK).
- Terapkan safety timeout pada sisi IoT bridge: jika koneksi jaringan IoT terputus, robot tetap menyelesaikan segmen berjalan atau berhenti dengan aman sesuai kebijakan operasional.

### Selesai jika

- Dashboard IoT menampilkan telemetri robot secara realtime dengan latensi terkontrol.
- Operator dapat mengirim perintah tujuan dan membatalkan misi dari jarak jauh.
- Putusnya jaringan tidak menyebabkan bahaya atau perilaku aneh pada robot fisik.

---

## Prioritas dan urutan tahapan yang disesuaikan

| Urutan | Fase | Fokus Utama | Alasan Rekayasa |
| ---: | :--- | :--- | :--- |
| **1** | **Tahap 0** | Keselamatan fisik, parameter mekanik, command timeout | Fondasi wajib sebelum motor diuji di lantai |
| **2** | **Tahap 1** | Kontrol kecepatan tertutup & protokol serial dua arah | Membuat respon gerak linier dan terukur; proteksi noise |
| **3** | **Tahap 2** | Odometri DDMR dan estimasi pose di ROS 2 | Mengetahui perpindahan fisik tanpa membebani mikrokontroler |
| **4** | **Tahap 3** | Uji persepsi optik dan deteksi ArUco fisik | Memastikan kamera mampu melihat marker sebelum mendesain peta |
| **5** | **Tahap 4** | Refaktorisasi modular ROS 2 & peta graf waypoint | Menata arsitektur kode bersih dan representasi rute |
| **6** | **Tahap 5** | Navigasi line tracking percabangan di intersection | Menyelesaikan navigasi otonom terpandu garis antar-waypoint |
| **7** | **Tahap 6** | Integrasi IoT (telemetri read-only, lalu command misi) | Menghubungkan robot ke lapisan cloud/dashboard secara aman |

---

## Saran tambahan

1. **Jaga isolasi komputasi:** Arduino Mega 2560 difokuskan hanya untuk I/O real-time (baca interupsi encoder, hitung PID motor, kontrol PWM, dan pantau E-Stop). Serahkan seluruh kalkulasi trigonometri, matriks, visi komputer, dan navigasi ke komputer Linux/ROS 2.
2. **Jangan andalkan odometri murni untuk rute panjang:** Selip roda pada lantai akan mendistorsi pose. Manfaatkan marker ArUco di intersection sebagai *anchor point* untuk mereset/mengoreksi akumulasi drift.
3. **Peta sebagai Single Source of Truth:** Seluruh relasi node, ID ArUco, koneksi edge, dan arah belok didefinisikan dalam satu file konfigurasi terpusat (YAML), bukan di-hardcode ke dalam kode node.
4. **Catat log hasil uji:** Selalu rekam galat odometri, tingkat keberhasilan deteksi ArUco, dan log transisi FSM di setiap tahap pengujian untuk mempermudah isolasi masalah antara mekanik, sensor, kontrol, dan software.
