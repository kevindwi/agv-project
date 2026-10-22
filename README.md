# AGV Line Tracker

Proyek AGV dua roda penggerak (DDMR) yang mengikuti garis menggunakan kamera,
ROS 2, dan firmware Arduino Mega 2560.

## Alur kerja

```text
Kamera
  │ frame
  ▼
ROS 2: OpenCV mendeteksi garis → PID menghitung kecepatan putar ω
  │ perintah v dan ω
  ▼
Kinematika balik DDMR: vL = v - (bω/2), vR = v + (bω/2)
  │ kecepatan roda dipetakan ke PWM (perkiraan)
  ▼
Serial UART → firmware Arduino → driver motor
                              │
                              └─ encoder dibaca dan RPM dicetak untuk debug
```

Node ROS menerbitkan gambar debug, perintah gerak, dan target kecepatan roda.
Firmware saat ini menerima PWM, bukan target kecepatan roda. Encoder belum
dipakai untuk odometri, sehingga posisi `(x, y, θ)` belum dihitung atau
diterbitkan.

## Struktur proyek

```text
firmware/                         Firmware PlatformIO untuk Arduino Mega
ros2_ws/src/line_tracker_ros2/    Node kamera, PID, DDMR, dan serial
docs/serial_protocol.md           Kontrak komunikasi serial
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

Firmware menerima `<left_pwm>,<right_pwm>\n` pada `Serial1`. Lihat
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
| `camera_index` | `2` | Indeks V4L2, misalnya `/dev/video2` |
| `serial_port` | `/dev/ttyUSB0` | Port adaptor UART yang terhubung ke `Serial1` |
| `baud_rate` | `115200` | Harus sama dengan firmware |
| `kp`, `ki`, `kd` | `1.0, 0.0, 0.0` | Gain PID; output menjadi `ω` (rad/s) |
| `linear_speed_mps` | `0.15` | Kecepatan maju robot `v` (m/s) |
| `wheel_separation_m` | `0.30` | Jarak pusat roda kiri-kanan `b` (m), ukur robot |
| `max_angular_speed_radps` | `1.0` | Batas nilai absolut `ω` |
| `max_wheel_speed_mps` | `0.30` | Skala perkiraan untuk memetakan kecepatan roda ke PWM |
| `publish_rate_hz` | `30.0` | Frekuensi pemrosesan kamera |

`wheel_separation_m` perlu diukur pada robot. `max_wheel_speed_mps` adalah
perkiraan, bukan kalibrasi otomatis. Tuning PID dan kecepatan sebaiknya dimulai
dengan nilai rendah.

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

Pilih `/line_tracker/debug_image`. Untuk memantau keluaran numerik:

```bash
ros2 topic list
ros2 topic echo /line_tracker/normalized_error
ros2 topic echo /cmd_vel
ros2 topic echo /line_tracker/wheel_speeds
ros2 topic echo /motor_pwm
```

| Topik | Tipe | Isi |
| --- | --- | --- |
| `/line_tracker/debug_image` | `sensor_msgs/Image` | Frame debug kamera |
| `/line_tracker/normalized_error` | `std_msgs/Float64` | Error posisi garis ternormalisasi |
| `/cmd_vel` | `geometry_msgs/Twist` | `linear.x` (m/s), `angular.z` (rad/s) |
| `/line_tracker/wheel_speeds` | `std_msgs/Float64MultiArray` | `[v_kiri, v_kanan]` (m/s) |
| `/motor_pwm` | `std_msgs/Int32MultiArray` | `[PWM_kiri, PWM_kanan]` (0–255) |

## Batas implementasi saat ini

- Jika garis tidak ditemukan, node mengirim perintah berhenti. Frame kamera
  yang gagal dibaca dilewati.
- Kecepatan roda negatif dipotong menjadi PWM nol; firmware belum mengubah
  arah motor dari target kinematika.
- Encoder firmware menghitung RPM untuk log, tetapi belum mengirimkannya ke ROS
  atau menghitung odometri. Posisi `(x, y, θ)` belum tersedia.
- `/line_tracker/wheel_speeds` adalah target hasil perhitungan, bukan pengukuran
  kecepatan aktual roda.

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
- [Protokol serial](docs/serial_protocol.md)
- [Panduan firmware](firmware/README.md)
- [Panduan paket ROS 2](ros2_ws/src/line_tracker_ros2/README.md)
