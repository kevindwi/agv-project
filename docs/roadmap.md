# Roadmap Pengembangan AGV

Dokumen ini merinci tahapan lanjutan untuk mencapai tiga tujuan proyek:

1. Navigasi berbasis waypoint.
2. Path tracking, termasuk identifikasi intersection dengan marker ArUco.
3. Integrasi IoT untuk telemetry dan command.

## Kondisi proyek saat ini

- Kamera dan deteksi garis sudah ada di node ROS 2.
- PID menghasilkan perintah gerak dan kinematika balik DDMR menghitung target
  kecepatan roda.
- Target kecepatan roda masih dipetakan secara perkiraan ke PWM.
- Firmware membaca encoder, tetapi belum mengirim bacaan ke ROS 2 atau
  menggunakannya untuk odometri dan kontrol kecepatan tertutup.
- Belum ada peta waypoint, path planner, deteksi ArUco, atau komunikasi IoT.

Karena posisi robot belum diketahui dengan baik, fondasi encoder, odometri, dan
kalibrasi perlu dibuat sebelum waypoint dan path tracking tingkat misi.

## Gambaran arsitektur yang dituju

```text
Rencana misi / IoT command
          │ tujuan waypoint
          ▼
Waypoint manager ──► planner / pembuatan path
                            │ path
                            ▼
Kamera garis + deteksi ArUco ──► path tracker / pengendali
                  │                       │ cmd_vel (m/s, rad/s)
                  └── pose marker         ▼
                                  DDMR kinematika balik
                                          │ target kecepatan roda
                                          ▼
                               kontrol kecepatan roda
                                          │ PWM
                                          ▼
                                  firmware + motor
                                          │ encoder
                                          ▼
                         odometri / estimasi pose robot
                                          │ pose, status, telemetry
                                          └────────────► IoT dashboard
```

## Tahap 0 — Rapikan antarmuka dan keselamatan dasar

### Pekerjaan

- Tetapkan satuan dan konvensi arah di seluruh sistem: meter, detik, radian,
  `linear.x` dalam m/s, `angular.z` dalam rad/s, kiri/kanan sesuai urutan.
- Ukur diameter roda dan jarak antar roda; dokumentasikan nilai serta cara
  mengukurnya.
- Pastikan arah positif encoder, motor kiri, motor kanan, dan `angular.z` ROS
  konsisten.
- Tambahkan timeout perintah di firmware: bila tidak ada command dalam batas
  waktu yang ditentukan, motor berhenti.
- Pastikan E-stop firmware selalu mengalahkan command dari ROS maupun IoT.
- Catat konfigurasi hardware: pin, rasio gearbox, pulsa encoder per putaran,
  tegangan motor, dan port UART.

### Selesai jika

- Uji command roda kiri/kanan menghasilkan arah yang diperkirakan.
- E-stop dan timeout menghentikan motor tanpa bergantung pada koneksi ROS.
- Parameter fisik robot tercatat dan digunakan oleh node.

## Tahap 1 — Kontrol kecepatan roda dengan encoder

### Pekerjaan

- Ubah protokol ROS–firmware dari PWM langsung menjadi target kecepatan roda
  bertanda, misalnya mm/s untuk kiri dan kanan.
- Di firmware, hitung kecepatan aktual roda dari delta pulsa encoder dan interval
  waktu yang diketahui.
- Tambahkan pengendali kecepatan per roda (misalnya PI) yang mengubah selisih
  target dan aktual menjadi PWM.
- Dukungan tanda target harus mengatur arah motor, termasuk mundur dan putar di
  tempat.
- Kembalikan telemetry encoder/kecepatan aktual ke ROS; gunakan framing yang
  jelas dan validasi data serial.

### Selesai jika

- Kedua roda dapat mempertahankan target kecepatan rendah dan sedang.
- Perintah target negatif membalik arah roda dengan benar.
- Kecepatan aktual dapat diamati di ROS dan sebanding dengan target.
- Kehilangan komunikasi menghentikan motor.

## Tahap 2 — Odometri DDMR dan pose

### Pekerjaan

- Konversi delta pulsa encoder menjadi jarak roda kiri `ΔsL` dan kanan `ΔsR`.
- Hitung perpindahan robot:

  ```text
  Δs     = (ΔsR + ΔsL) / 2
  Δθ     = (ΔsR - ΔsL) / b
  θ_mid  = θ + Δθ / 2
  x      = x + Δs × cos(θ_mid)
  y      = y + Δs × sin(θ_mid)
  θ      = θ + Δθ
  ```

- Publikasikan pose dan kecepatan sebagai `nav_msgs/Odometry` dengan frame yang
  konsisten, misalnya `odom` dan `base_link`.
- Publikasikan transformasi `odom → base_link` untuk visualisasi ROS.
- Kalibrasikan parameter efektif roda dengan uji jarak lurus dan putaran.

### Uji kalibrasi yang disarankan

1. Jalankan robot lurus pada jarak terukur; bandingkan jarak fisik dengan
   odometri.
2. Putar robot 360 derajat di tempat; periksa galat arah dan posisi akhir.
3. Ulangi beberapa kali pada permukaan operasi; catat rata-rata dan variasi.

### Selesai jika

- Odometri bergerak sesuai uji lurus dan putar dalam toleransi yang disepakati.
- Pose dapat dilihat pada RViz tanpa transformasi putus.
- Keterbatasan drift dan selip dicatat.

## Tahap 3 — Waypoint dan peta intersection

### Definisi yang disarankan

Representasikan lintasan sebagai **graf waypoint**:

- Waypoint adalah titik tujuan, misalnya intersection, stasiun, atau area
  berhenti.
- Edge menghubungkan dua waypoint dan menyimpan informasi lintasan, arah masuk,
  jarak perkiraan, serta aturan belok.
- Waypoint dapat memiliki ID unik yang sama dengan ID ArUco di intersection.

Contoh:

```text
[Aruco 10 / W0] ── lurus ── [Aruco 11 / W1]
                               │
                               └── belok kanan ── [Aruco 12 / W2]
```

### Pekerjaan

- Buat format peta sederhana (YAML/JSON) berisi ID waypoint, koneksi, arah,
  aturan belok, dan batas kecepatan.
- Buat waypoint manager yang menerima tujuan dan menentukan urutan node pada
  graf.
- Implementasikan state mission: `IDLE`, `NAVIGATING`, `ARRIVED`, `PAUSED`,
  `ERROR`.
- Definisikan perilaku jika tujuan tidak terhubung, waypoint terlewat, atau
  marker tidak terbaca.

### Selesai jika

- Robot dapat menerima satu tujuan waypoint dan mengeluarkan urutan waypoint
  yang dapat diperiksa.
- Misi dapat dihentikan, dilanjutkan, dan dibatalkan dengan state yang jelas.

## Tahap 4 — Deteksi ArUco dan koreksi pose di intersection

### Rekomendasi pemakaian marker

- Letakkan marker di lokasi intersection yang terlihat dari kamera sebelum
  robot memasuki titik keputusan.
- Pakai ID marker sebagai identitas waypoint/intersection.
- Jika ukuran marker diketahui dan kamera dikalibrasi, estimasi pose relatif
  marker terhadap kamera untuk membantu koreksi posisi dan arah.
- Gunakan odometri untuk estimasi gerak di antara marker; gunakan marker untuk
  mengenali titik dan mengurangi drift saat marker terlihat.
- Terapkan debouncing/konfirmasi: marker harus terdeteksi stabil pada beberapa
  frame sebelum dianggap sebagai waypoint yang tercapai.
- Simpan kalibrasi kamera, ukuran marker, posisi marker terhadap pusat
  intersection, dan orientasi marker.

### Perhatian desain

- Marker ID hanya memberi identitas; ID saja tidak memberitahu jarak dan arah
  robot terhadap marker. Untuk koreksi pose, perlu ukuran marker yang diketahui,
  kalibrasi intrinsik kamera, dan transformasi kamera terhadap robot.
- Uji sudut pandang, tinggi kamera, pencahayaan, blur, ukuran marker di frame,
  dan occlusion pada kecepatan operasi.
- Hindari menempatkan marker tepat di area yang tertutup roda, barang, atau
  bagian robot.
- Siapkan fallback: jika marker tidak terbaca, robot melambat/berhenti dan
  meminta pemulihan, bukan mengambil cabang secara acak.

### Selesai jika

- ID intersection terbaca berulang pada kondisi lapangan yang ditentukan.
- Deteksi marker tidak memicu keputusan ganda saat robot diam atau bergerak
  melewati satu marker.
- Pose marker dapat dipakai untuk mengoreksi pose odometri di intersection.

## Tahap 5 — Path tracking antar-waypoint

### Pekerjaan

- Pisahkan **planner** yang menentukan lintasan dari **tracker** yang membuat
  robot mengikuti lintasan tersebut.
- Representasikan edge sebagai segmen garis, kurva, atau lintasan yang dibentuk
  dari titik-titik berurutan.
- Mulai dengan tracker sederhana (misalnya Pure Pursuit untuk path bertitik) atau
  line following yang dikaitkan dengan cabang/arah waypoint.
- Hitung command `v, ω`, batasi kecepatan serta percepatan, lalu teruskan ke
  kinematika DDMR dan kontrol roda.
- Perlambat sebelum intersection, verifikasi ID ArUco, lakukan keputusan belok,
  lalu lanjutkan ke waypoint berikutnya.
- Tambahkan toleransi tiba (jarak dan arah) serta kondisi berhenti.

### Urutan perilaku pada intersection

```text
ikuti path menuju intersection
        ↓
perlambat dan deteksi ArUco
        ↓
konfirmasi ID cocok dengan waypoint berikutnya
        ↓
pilih cabang dari rencana misi
        ↓
belok dan verifikasi marker/path berikutnya
```

### Selesai jika

- Robot mencapai beberapa waypoint berurutan tanpa intervensi manual pada
  kecepatan rendah.
- Robot mengambil cabang sesuai rute dan berhenti jika marker atau path hilang.
- Pengujian merekam tingkat keberhasilan, galat lintasan, dan waktu perjalanan.

## Tahap 6 — Integrasi IoT: telemetry dan command

### Telemetry yang berguna

- Pose/odometri dan waypoint saat ini.
- Kecepatan target dan aktual tiap roda.
- Status misi dan FSM: idle, berjalan, pause, selesai, error, E-stop.
- Kualitas deteksi garis/ArUco dan timestamp.
- Baterai, suhu, atau alarm hardware jika sensor tersedia.

### Command yang berguna

- Minta robot menuju waypoint atau menjalankan misi bernama.
- Pause, resume, cancel, dan request status.
- Ubah parameter operasi terbatas seperti batas kecepatan, dengan validasi.
- Hindari command IoT berupa PWM motor langsung; command sebaiknya berupa niat
  tingkat misi agar keselamatan dan kontrol tetap di robot.

### Rekomendasi implementasi

- Mulai dari broker/message transport yang sederhana (misalnya MQTT) jika
  arsitektur jaringan proyek belum ditentukan.
- Pisahkan channel telemetry dan command; gunakan ID pesan, timestamp, dan
  acknowledgement untuk command penting.
- Definisikan format payload berversi (misalnya JSON) dan dokumentasikan nama
  topic, satuan, QoS, serta perilaku saat koneksi terputus.
- Command harus tervalidasi di robot, memiliki batas waktu, dan tidak boleh
  membatalkan E-stop lokal.
- Jika koneksi IoT hilang, robot menyelesaikan perilaku aman yang sudah
  ditentukan (umumnya berhenti atau menyelesaikan segmen aman), bukan terus
  menerima command lama.
- Untuk jaringan di luar LAN tepercaya, gunakan autentikasi dan enkripsi.

### Selesai jika

- Dashboard menerima telemetry dengan satuan dan timestamp yang jelas.
- Operator dapat meminta waypoint, pause, resume, dan cancel.
- Command duplikat/kedaluwarsa tidak menjalankan aksi berulang atau berbahaya.
- Kehilangan jaringan dan restart broker ditangani dengan aman.

## Prioritas yang disarankan

| Urutan | Fokus | Alasan |
| ---: | --- | --- |
| 1 | Safety, parameter fisik, dan serial yang andal | Menjadi dasar uji motor yang aman |
| 2 | Kontrol kecepatan roda dari encoder | Membuat command gerak dapat diprediksi |
| 3 | Odometri dan visualisasi pose | Waypoint memerlukan estimasi posisi |
| 4 | Deteksi ArUco dan koreksi di intersection | Mengurangi drift dan memberi identitas node peta |
| 5 | Waypoint manager dan path tracking | Menyusun navigasi dari satu titik ke titik lain |
| 6 | IoT telemetry, lalu command misi | Telemetry lebih aman sebagai integrasi awal |

IoT dapat dikembangkan paralel sebagai **monitoring read-only** setelah topic ROS
dan satuannya stabil. Tunda command jarak jauh sampai timeout, E-stop, dan
perilaku saat koneksi hilang sudah diuji.

## Saran tambahan

1. **Jangan mengandalkan odometri saja untuk rute panjang.** Selip dan variasi
   permukaan membuat galat menumpuk. ArUco di intersection cocok menjadi
   checkpoint untuk mengoreksi posisi, tetapi tetap perlu validasi pose.
2. **Kalibrasi kamera sebelum mengandalkan pose ArUco.** Deteksi ID dapat bekerja
   tanpa pose akurat, tetapi estimasi jarak/orientasi butuh parameter kamera dan
   ukuran marker yang benar.
3. **Gunakan satu sumber kebenaran untuk peta.** ID ArUco, waypoint, cabang,
   dan aturan belok sebaiknya berasal dari satu file peta, bukan disalin ke
   beberapa node.
4. **Pisahkan ROS node berdasarkan tanggung jawab setelah alur stabil.** Contoh:
   camera/marker perception, odometry, wheel controller, waypoint manager,
   path tracker, dan IoT bridge. Untuk prototipe awal boleh tetap lebih sederhana.
5. **Catat hasil setiap uji.** Simpan parameter, permukaan, kecepatan, rute,
   galat akhir, deteksi marker gagal, dan tindakan recovery. Ini membantu
   membedakan masalah mekanik, sensor, kontrol, dan jaringan.

## Metrik evaluasi yang disarankan

- Galat jarak odometri setelah menempuh lintasan terukur.
- Galat sudut setelah putaran terukur.
- Persentase deteksi marker yang benar dan waktu deteksi.
- Persentase keberhasilan mencapai waypoint dan menyelesaikan rute.
- Galat silang lintasan (cross-track error) dan waktu perjalanan.
- Waktu dari command IoT diterima hingga acknowledgement, serta perilaku saat
  koneksi terputus.
