# line_tracker_ros2

Package ROS 2 untuk sistem line tracking AGV: membaca kamera V4L2, mendeteksi garis menggunakan OpenCV, menghitung koreksi PID angular, mengirim perintah kecepatan `/cmd_vel` ke firmware via serial, serta mengintegrasikan delta encoder menjadi odometri global `/odom` dan visualisasi jejak `/path`.

## Struktur package

```text
line_tracker_ros2/
├── line_tracker_ros2/
│   ├── __init__.py
│   ├── line_tracker_node.py   # kamera, PID, pemilik tunggal keluaran serial, manuver garis
│   ├── odometry_node.py       # delta encoder -> /odom + TF odom/base_link + /path
│   ├── aruco_detector_node.py # ID dan pose ArUco dari /camera/image_raw
│   ├── mission_controller_node.py # FSM rute berbasis checkpoint ArUco
│   ├── camera.py              # akuisisi kamera V4L2 + deteksi kontur garis (ROI)
│   ├── pid.py                 # kontroler PID diskrit
│   └── serial_bridge.py       # protokol serial dua arah C/E (cmd_vel & encoder delta)
├── config/
│   └── line_tracker.yaml      # kalibrasi DDMR fisik, PID, dan konfigurasi serial
├── launch/
│   └── line_tracker.launch.py # launch tracker, odometri, detektor, dan FSM misi
├── resource/                  # ament index
├── test/
├── package.xml
├── setup.py
└── setup.cfg
```

## Parameter

Didefinisikan di [`config/line_tracker.yaml`](config/line_tracker.yaml):

### `line_tracker_node`

| Parameter | Default | Keterangan |
| --- | --- | --- |
| `camera_index` | `3` | Indeks perangkat kamera V4L2 (`/dev/videoN`) |
| `serial_port` | `/dev/ttyUSB0` | Port UART adapter ke `Serial1` Arduino Mega |
| `baud_rate` | `115200` | Baud rate komunikasi serial |
| `wheel_diameter_m` | `0.15` | Diameter efektif roda (15 cm) |
| `wheel_separation_m` | `0.48` | Jarak pusat roda kiri-kanan (48 cm) |
| `gear_ratio` | `30.0` | Rasio gearbox 30:1 |
| `encoder_pulses_per_wheel_rev` | `900` | Resolusi encoder per putaran roda |
| `kp`, `ki`, `kd` | `1.0, 0.0, 0.0` | Gain PID pengarah garis (`angular.z` rad/s) |
| `linear_speed_mps` | `0.30` | Kecepatan maju nominal robot (m/s) |
| `max_angular_speed_radps` | `1.0` | Batas kecepatan putar robot (rad/s) |
| `publish_rate_hz` | `30.0` | Frekuensi loop deteksi dan kontrol (Hz) |

### `odometry_node`

| Parameter | Default | Keterangan |
| --- | --- | --- |
| `wheel_diameter_m` | `0.15` | Diameter efektif roda (m) |
| `wheel_separation_m` | `0.48` | Jarak pusat roda kiri-kanan (m) |
| `encoder_pulses_per_wheel_rev` | `900` | Pulsa per putaran roda |
| `encoder_delta_topic` | `/line_tracker/encoder_delta` | Topik masukan delta pulsa |
| `odom_topic` | `/odom` | Topik keluaran odometri |
| `path_topic` | `/path` | Topik keluaran jejak lintasan RViz |
| `max_path_poses` | `10000` | Batas riwayat pose jejak lintasan |
| `path_publish_rate_hz` | `2.0` | Frekuensi publikasi jejak ke `/path` (Hz) |
| `odom_frame_id` | `odom` | Nama frame referensi odometri |
| `base_frame_id` | `base_link` | Nama frame base robot |

## Topik

| Topik | Tipe | Keterangan |
| --- | --- | --- |
| `/line_tracker/debug_image` | `sensor_msgs/Image` | Frame debug dengan overlay ROI dan posisi garis |
| `/cmd_vel` | `geometry_msgs/Twist` | Perintah kecepatan robot (`linear.x` m/s, `angular.z` rad/s) |
| `/line_tracker/encoder_delta` | `std_msgs/Int64MultiArray` | Delta tick `[left, right]` dari telemetri serial firmware |
| `/camera/image_raw` | `sensor_msgs/Image` | Frame kamera yang dibagikan ke detektor ArUco |
| `/aruco/detections` | `std_msgs/String` | JSON berisi ID dan pose marker relatif kamera bila kamera sudah dikalibrasi |
| `/mission/control` | `std_msgs/String` | Perintah FSM ke pemilik motor: `FOLLOW`, `LEFT`, `RIGHT`, `STOP` |
| `/line_tracker/status` | `std_msgs/String` | Status line following/manuver untuk FSM |
| `/mission/state` | `std_msgs/String` | Status FSM dan checkpoint yang sedang ditunggu dalam JSON |
| `/aruco/debug_image` | `sensor_msgs/Image` | Frame kamera dengan marker yang terdeteksi |
| `/odom` | `nav_msgs/Odometry` | Estimasi pose $(x, y, \theta)$ dan twist robot dalam frame `odom` |
| `/path` | `nav_msgs/Path` | Jejak pose robot di frame `odom` untuk RViz |

## Visualisasi & Monitoring

Launch file secara otomatis menjalankan `line_tracker_node` dan `odometry_node`:

```bash
ros2 launch line_tracker_ros2 line_tracker.launch.py
```

- Untuk melihat overlay deteksi kamera:
  ```bash
  ros2 run rqt_image_view rqt_image_view
  ```
  Pilih topik `/line_tracker/debug_image`.

- Untuk melihat pose robot, frame koordinat TF, dan jejak perpindahan lintasan:
  ```bash
  rviz2 -d ros2_ws/line_tracker.rviz
  ```
  Atau atur **Fixed Frame** ke `odom`, tambahkan display **TF**, **Odometry** (`/odom`), dan **Path** (`/path`).

## Build & Jalankan

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash

# Menjalankan via launch file
ros2 launch line_tracker_ros2 line_tracker.launch.py

# Atau override parameter via CLI
ros2 launch line_tracker_ros2 line_tracker.launch.py camera_index:=2 serial_port:=/dev/ttyUSB0
```

Launch menjalankan route contoh ArUco ID 1 → belok kiri, ID 2 → belok kanan, ID 3 →
berhenti. Ubah `route_json` pada `config/line_tracker.yaml` sesuai ID marker dan jalur
fisik. `trigger_distance_m` adalah jarak pemicu memakai kedalaman pose marker di frame
kamera; node memperlambat robot saat marker pertama dikonfirmasi, lalu menjalankan aksi
saat jarak mencapai ambang tersebut. Formatnya array JSON, misalnya:

```yaml
route_json: '[{"marker_id":7,"action":"straight","trigger_distance_m":0.35},{"marker_id":8,"action":"stop","trigger_distance_m":0.30}]'
```

`action` yang didukung: `straight`, `left`, `right`, `stop`. `marker_length_m` harus
sesuai ukuran sisi marker yang dicetak. Untuk pose 3D, masukkan matriks intrinsik 3×3
(urutan baris) dan koefisien distorsi hasil kalibrasi pada `camera_matrix` dan
`distortion_coefficients`. Tanpa kalibrasi, node tetap menerbitkan ID untuk waypoint,
tetapi tidak menerbitkan pose marker. Tanpa kalibrasi, mission controller memakai ID
sebagai pemicu langsung; pasang marker di posisi pemicu yang sudah diuji secara fisik.

Detektor membutuhkan OpenCV dengan modul `cv2.aruco` (build *contrib*). Node tracker
membagikan frame kameranya sehingga kedua node tidak membuka perangkat kamera yang sama.
Node tracker tetap menjadi satu-satunya penulis serial; mission controller meminta mode
melalui `/mission/control`. Belokan memakai odometri untuk rotasi awal lalu mencari garis
baru secara visual. Setel `turn_angle_rad`, kecepatan belok, dan ambang pencarian pada
kecepatan rendah sebelum menjalankan robot di lintasan penuh.

Protokol serial: [`../../../docs/serial_protocol.md`](../../../docs/serial_protocol.md).
