# Checklist Pengembangan AGV

Checklist ini mengikuti [roadmap pengembangan](roadmap.md). Item bertanda `[x]`
adalah kemampuan yang sudah tersedia di kode saat ini; tetap validasi di robot
fisik sebelum menganggapnya selesai untuk operasi.

## 0. Kondisi sekarang

- [x] Membaca frame kamera V4L2.
- [x] Mendeteksi garis dan menghitung error posisi ternormalisasi.
- [x] PID menghasilkan perintah kecepatan putar `ω`.
- [x] Kinematika balik DDMR menghitung target kecepatan roda kiri dan kanan.
- [x] Menerbitkan gambar debug ke `/line_tracker/debug_image`.
- [x] Menerbitkan `/cmd_vel`, `/line_tracker/wheel_speeds`, dan `/motor_pwm`.
- [x] Mengirim PWM kiri/kanan melalui serial ke firmware.
- [ ] Memvalidasi arah belok dan semua satuan pada robot fisik.

## 1. Keselamatan, hardware, dan antarmuka

- [ ] Ukur diameter efektif roda dan jarak antara pusat roda kiri-kanan.
- [ ] Catat encoder pulses per wheel revolution, termasuk rasio gearbox.
- [ ] Validasi arah motor kiri/kanan dan polaritas encoder.
- [ ] Validasi sambungan ROS ke `Serial1` Arduino Mega dan catat nama port.
- [ ] Tambahkan timeout command pada firmware; timeout harus menghentikan motor.
- [ ] Uji bahwa E-stop selalu menghentikan motor dan tidak dapat dibatalkan oleh ROS/IoT.
- [ ] Dokumentasikan satuan, urutan roda, format serial, dan versi protokol.

## 2. Kontrol kecepatan roda berbasis encoder

- [ ] Tentukan format command bertanda untuk target kecepatan roda (misalnya mm/s).
- [ ] Perbarui protokol serial ROS–firmware dan parser firmware.
- [ ] Hitung kecepatan aktual kiri/kanan dari delta encoder dan interval waktu.
- [ ] Implementasikan kontrol PI/PID kecepatan terpisah untuk tiap roda.
- [ ] Implementasikan arah maju/mundur dari tanda target roda.
- [ ] Kirim telemetry kecepatan aktual/encoder kembali ke ROS.
- [ ] Validasi target dan aktual pada beberapa kecepatan.
- [ ] Uji perilaku berhenti saat serial terputus atau data tidak valid.

## 3. Odometri DDMR

- [ ] Konversi pulsa encoder menjadi jarak roda kiri `ΔsL` dan kanan `ΔsR`.
- [ ] Hitung `Δs`, `Δθ`, lalu perbarui pose `(x, y, θ)`.
- [ ] Publikasikan `nav_msgs/Odometry` dengan frame `odom` dan `base_link`.
- [ ] Publikasikan transformasi `odom → base_link`.
- [ ] Uji lintasan lurus dengan jarak fisik terukur.
- [ ] Uji putaran 360° dan ukur galat sudut/posisi.
- [ ] Kalibrasi parameter roda dan jarak antar roda berdasarkan hasil uji.
- [ ] Visualisasikan pose/trajectory di RViz.
- [ ] Catat drift odometri pada permukaan operasi.

## 4. ArUco di intersection

- [ ] Pilih dictionary dan skema ID marker yang tidak duplikat.
- [ ] Buat tabel ID ArUco ↔ ID waypoint/intersection.
- [ ] Tentukan ukuran fisik dan posisi/orientasi pemasangan tiap marker.
- [ ] Kalibrasi intrinsik kamera dan simpan file kalibrasinya.
- [ ] Pasang marker agar terlihat sebelum robot mencapai titik keputusan.
- [ ] Implementasikan deteksi ID dan terbitkan hasil deteksi ke ROS.
- [ ] Jika dibutuhkan pose, hitung pose marker dengan ukuran marker dan kalibrasi kamera.
- [ ] Konfirmasi deteksi stabil beberapa frame sebelum menerima marker sebagai waypoint.
- [ ] Tangani marker salah, hilang, terduplikasi, atau terbaca berulang.
- [ ] Gunakan marker sebagai checkpoint untuk mengoreksi drift odometri.
- [ ] Uji pencahayaan, blur, sudut pandang, jarak, dan occlusion.

## 5. Peta, waypoint, dan path tracking

- [ ] Definisikan format peta (YAML/JSON): waypoint, ID ArUco, edge, arah, dan aturan belok.
- [ ] Validasi bahwa ID waypoint pada peta unik dan koneksi antar titik konsisten.
- [ ] Implementasikan waypoint manager dan pencarian rute dari posisi ke tujuan.
- [ ] Definisikan state misi: `IDLE`, `NAVIGATING`, `ARRIVED`, `PAUSED`, `ERROR`.
- [ ] Pilih metode tracking awal (line following berbasis cabang atau Pure Pursuit).
- [ ] Hasilkan `v, ω` dari path tracker dan batasi kecepatan/percepatan.
- [ ] Perlambat saat mendekati intersection.
- [ ] Verifikasi ID ArUco sesuai waypoint berikutnya sebelum memilih cabang.
- [ ] Tambahkan toleransi tiba berdasarkan jarak dan arah.
- [ ] Berhenti dengan aman saat path atau marker tidak terdeteksi.
- [ ] Uji rute beberapa waypoint dengan kecepatan rendah.
- [ ] Ukur keberhasilan waypoint, cross-track error, dan waktu perjalanan.

## 6. IoT telemetry dan command

- [ ] Pilih transport/message broker (contoh: MQTT) dan dokumentasikan alasannya.
- [ ] Tetapkan skema payload berversi, timestamp, satuan, dan ID pesan.
- [ ] Publikasikan telemetry pose, waypoint, status misi, kecepatan, dan sensor.
- [ ] Buat dashboard monitoring read-only terlebih dahulu.
- [ ] Definisikan command tingkat misi: tujuan waypoint, pause, resume, cancel, status.
- [ ] Tambahkan validasi command, acknowledgement, dan penolakan command kedaluwarsa.
- [ ] Pastikan command IoT tidak dapat menimpa E-stop atau mengirim PWM mentah.
- [ ] Tentukan perilaku robot ketika jaringan/broker terputus.
- [ ] Uji command duplikat, urutan pesan salah, dan koneksi putus-sambung.
- [ ] Aktifkan autentikasi/enkripsi jika komunikasi melewati jaringan tepercaya.

## 7. Uji akhir dan dokumentasi

- [ ] Buat prosedur uji meja dengan roda terangkat.
- [ ] Buat prosedur uji lintasan tertutup dengan E-stop mudah dijangkau.
- [ ] Simpan log konfigurasi, sensor, command, pose, dan hasil tiap uji.
- [ ] Dokumentasikan wiring final dan parameter kalibrasi.
- [ ] Dokumentasikan cara menjalankan seluruh node dan pemulihan setelah error.
- [ ] Tetapkan metrik penerimaan dan toleransi untuk waypoint/path tracking.
- [ ] Rekam demo misi dari command tujuan sampai robot tiba dan melaporkan status.
