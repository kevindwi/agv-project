# AGV Line Tracker

Proyek AGV dua roda penggerak (DDMR) yang mengikuti garis menggunakan kamera,
ROS 2, dan firmware Arduino Mega 2560.

## Alur kerja

```text
Kamera
  │ frame
  ▼
ROS 2: OpenCV mendeteksi garis → PID garis menghitung kecepatan putar ω
  │ perintah v dan ω (/cmd_vel)
  ▼
Serial UART: C,<linear_x>,<angular_z>\n
  │
  ▼
Firmware Arduino Mega:
  │ Kinematika balik DDMR: vL = v - (bω/2), vR = v + (bω/2)
  │ Target ticks dihitung per siklus kontrol (100 ms)
  │ Loop PID per roda → sinyal PWM bertanda & DIR → driver motor
  │
  └─ Membaca delta tick encoder dan mengirim balik: E,<left_ticks>,<right_ticks>\n
  │
  ▼
ROS 2:
  │ /line_tracker/encoder_delta
  ▼
odometry_node:
  │ Mengintegrasikan tick menjadi pose global (x, y, θ)
  ▼
Topik /odom, TF odom -> base_link, dan /path untuk visualisasi RViz
```

Node `line_tracker_node` menerbitkan gambar debug, perintah `/cmd_vel`, dan
meneruskan delta encoder ke `/line_tracker/encoder_delta`. Node `odometry_node`
mengintegrasikan delta pulsa encoder menjadi posisi dan orientasi global
`(x, y, θ)` pada `/odom` serta menyiarkan TF dan jejak lintasan `/path`.

## Struktur proyek

```text
firmware/                         Firmware PlatformIO untuk Arduino Mega 2560
ros2_ws/src/line_tracker_ros2/    Package ROS 2 (line_tracker_node & odometry_node)
docs/checklist.md                 Checklist pengembangan proyek
docs/hardware.md                  Parameter fisik, konvensi, pinout, dan kalibrasi
docs/roadmap.md                   Roadmap tahapan pengembangan menuju pameran
docs/serial_protocol.md           Kontrak komunikasi serial dua arah ROS ↔ MCU
```

## Perangkat yang diperlukan

- Arduino Mega 2560 dan driver motor yang sesuai.
- Dua motor penggerak dan encoder.
- Kamera yang dikenali Linux sebagai perangkat V4L2 (`/dev/videoN`).
- Komputer Linux dengan ROS 2 dan koneksi serial ke UART `Serial1` Arduino.
- Permukaan dengan garis kontras yang terlihat oleh kamera.

### Sambungan UART

Firmware menerima perintah pada `Serial1`; USB onboard Arduino Mega menyediakan
`Serial` untuk log debug. Karena itu port node ROS harus berupa adaptor USB-to-TTL
yang terhubung ke UART1, bukan otomatis USB onboard Arduino.

| USB-to-TTL | Arduino Mega |
| --- | --- |
| TX | RX1, pin 19 |
| RX | TX1, pin 18 |
| GND | GND |

Pastikan level logika adaptor sesuai dan satukan ground. USB onboard dapat
dipakai terpisah untuk log firmware. Jangan buka port serial yang sama dari dua
program sekaligus.

## Persiapan ROS 2

Contoh berikut memakai variabel `ROS_DISTRO` yang sudah diset (misalnya
`humble`, `iron`, atau `jazzy`):

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
sudo apt update
sudo apt install \
  python3-colcon-common-extensions \
  python3-opencv \
  python3-serial \
  ros-$ROS_DISTRO-cv-bridge \
  ros-$ROS_DISTRO-rqt-image-view
```

Pastikan perangkat kamera dan UART ditemukan:

```bash
ls /dev/video*
ls /dev/ttyUSB* /dev/ttyACM*
```

`camera_index: 2` membuka `/dev/video2`. Sesuaikan parameter bila kamera berada
pada indeks lain.

## Build dan flash firmware

Pasang PlatformIO Core (`pio`) dari
[dokumentasi PlatformIO](https://docs.platformio.org/en/latest/core/installation/index.html),
lalu jalankan dari root proyek:

```bash
cd firmware
pio run
pio run -t upload
```

Jika ada beberapa board, pilih port secara eksplisit:

```bash
pio run -t upload --upload-port /dev/ttyACM0
```

Firmware menerima perintah kecepatan `C,<linear_x>,<angular_z>\n` pada `Serial1`
dan membalas delta ticks `E,<left_ticks>,<right_ticks>\n`. Lihat
[protokol serial](docs/serial_protocol.md) untuk detailnya.

## Build ROS 2

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
cd ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Di terminal baru, source kembali ROS 2 dan workspace sebelum menjalankan paket.

## Konfigurasi

Edit [`line_tracker.yaml`](ros2_ws/src/line_tracker_ros2/config/line_tracker.yaml):

| Parameter | Default | Arti |
| --- | ---: | --- |
| `camera_index` | `3` | Indeks V4L2, misalnya `/dev/video3` |
| `serial_port` | `/dev/ttyUSB0` | Port adaptor UART yang terhubung ke `Serial1` |
| `baud_rate` | `115200` | Harus sama dengan firmware |
| `kp`, `ki`, `kd` | `1.0, 0.0, 0.0` | Gain PID; output menjadi `ω` (rad/s) |
| `linear_speed_mps` | `0.024` | Kecepatan maju robot `v` (m/s) |
| `wheel_diameter_m` | `0.15` | Diameter efektif roda (m) |
| `wheel_separation_m` | `0.48` | Jarak pusat roda kiri-kanan `b` (m) |
| `gear_ratio` | `30.0` | Rasio gearbox 30:1 |
| `encoder_pulses_per_motor_rev` | `30` | Pulsa encoder per putaran poros motor |
| `encoder_pulses_per_wheel_rev` | `900` | Pulsa per putaran roda (30 × 30) |
| `max_angular_speed_radps` | `1.0` | Batas nilai absolut `ω` |
| `max_wheel_speed_mps` | `0.30` | Skala perkiraan untuk memetakan kecepatan roda ke PWM |
| `publish_rate_hz` | `30.0` | Frekuensi pemrosesan kamera |

Nilai `wheel_diameter_m` (15 cm) dan `wheel_separation_m` (48 cm) sudah diukur
pada robot. Rincian dan prosedur kalibrasi ada di
[`docs/hardware.md`](docs/hardware.md). `max_wheel_speed_mps` masih perkiraan,
bukan kalibrasi otomatis. Tuning PID dan kecepatan sebaiknya dimulai dengan nilai
rendah.

## Menjalankan sistem

1. Untuk pemeriksaan awal, angkat roda dari lantai. Pastikan tombol E-stop
   berfungsi.
2. Flash firmware dan pastikan UART1 tersambung. Catat nama port adaptor.
3. Sesuaikan `camera_index` dan `serial_port` pada YAML.
4. Jalankan node:

   ```bash
   source /opt/ros/$ROS_DISTRO/setup.bash
   cd ros2_ws
   source install/setup.bash
   ros2 launch line_tracker_ros2 line_tracker.launch.py
   ```

5. Firmware mulai dalam status `IDLE`. Lepaskan E-stop jika aktif, lalu tekan
   tombol Start/Stop yang terhubung ke pin 40 untuk masuk `MOVING`.
6. Uji arah dan respons motor dengan hati-hati. Gunakan E-stop bila gerak tidak
   sesuai.

Jalankan node tanpa launch file dengan:

```bash
ros2 run line_tracker_ros2 line_tracker_node --ros-args \
  -p camera_index:=2 \
  -p serial_port:=/dev/ttyUSB0 \
  -p baud_rate:=115200
```

## Visualisasi dan topik

Tidak ada jendela `cv2.imshow`. Untuk melihat frame kamera dengan overlay ROI,
garis tengah, dan centroid, jalankan:

```bash
ros2 run rqt_image_view rqt_image_view
```

Pilih `/line_tracker/debug_image`. Untuk memantau keluaran numerik dan odometri:

```bash
ros2 topic list
ros2 topic echo /cmd_vel
ros2 topic echo /line_tracker/encoder_delta
ros2 topic echo /odom
ros2 topic echo /path
```

Untuk visualisasi odometri dan jejak robot di RViz:

```bash
rviz2 -d ros2_ws/line_tracker.rviz
```

| Topik | Tipe | Isi |
| --- | --- | --- |
| `/line_tracker/debug_image` | `sensor_msgs/Image` | Frame debug kamera (overlay ROI dan garis) |
| `/cmd_vel` | `geometry_msgs/Twist` | Perintah kecepatan: `linear.x` (m/s), `angular.z` (rad/s) |
| `/line_tracker/encoder_delta` | `std_msgs/Int64MultiArray` | Delta tick `[left, right]` dari serial firmware |
| `/odom` | `nav_msgs/Odometry` | Estimasi pose $(x, y, \theta)$ dan twist robot di frame `odom` |
| `/path` | `nav_msgs/Path` | Riwayat jejak perpindahan posisi robot untuk RViz |

## Batas implementasi saat ini

- Jika garis tidak ditemukan, node mengirim perintah berhenti (`linear.x = 0, angular.z = 0`). Frame kamera yang gagal dibaca dilewati.
- Kontrol kecepatan loop tertutup PID firmware dan gain kalibrasi masih berupa nilai awal dan memerlukan validasi menyeluruh pada robot fisik di lantai.
- Protokol serial belum dilengkapi checksum (CRC8/XOR); frame rusak karena derau kelistrikan motor belum terfilter secara otomatis.
- Deteksi ArUco, peta rute graf lintasan, dan integrasi IoT masih dalam perencanaan/tahap pengembangan berikutnya (lihat `docs/roadmap.md`).

## Troubleshooting

### Kamera tidak terbuka

- Periksa `ls /dev/video*` dan sesuaikan `camera_index`.
- Tutup aplikasi lain yang sedang menggunakan kamera.
- Pastikan pengguna memiliki izin akses perangkat kamera.

### Port serial gagal dibuka

- Periksa `serial_port` dan `ls /dev/ttyUSB* /dev/ttyACM*`.
- Tutup Serial Monitor atau proses lain yang memakai port yang sama.
- Pastikan port ROS berasal dari adaptor UART pada pin `Serial1`, bukan hanya
  USB onboard Arduino Mega.

### Motor tidak bergerak

- Pastikan firmware dalam status `MOVING` dengan menekan tombol Start/Stop.
- Pastikan E-stop tidak aktif dan catu daya driver motor tersedia.
- Periksa log firmware di USB onboard untuk memastikan perintah diterima.
- Periksa status FSM (`IDLE`, `ESTOP`, atau `ERROR`).

### Gambar tidak muncul

- Pastikan node berjalan dan kamera menghasilkan frame.
- Source ROS 2 dan workspace pada terminal rqt.
- Pastikan `/line_tracker/debug_image` muncul di `ros2 topic list`.

## Dokumentasi terkait

- [Checklist tahapan pengembangan](docs/checklist.md)
- [Roadmap tahapan pengembangan](docs/roadmap.md)
- [Hardware, satuan, dan kalibrasi](docs/hardware.md)
- [Protokol serial](docs/serial_protocol.md)
- [Panduan firmware](firmware/README.md)
- [Panduan paket ROS 2](ros2_ws/src/line_tracker_ros2/README.md)
