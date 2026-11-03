# line_tracker_ros2

Node ROS 2 untuk line tracking: membaca kamera, mendeteksi garis, menghitung
koreksi PID, lalu mengirim PWM roda ke firmware AGV melalui serial.

## Struktur package

```
line_tracker_ros2/
├── line_tracker_ros2/
│   ├── __init__.py
│   ├── line_tracker_node.py   # wiring: parameter -> kamera -> PID -> serial
│   ├── odometry_node.py       # delta encoder -> /odom + TF odom/base_link
│   ├── camera.py              # buka kamera + deteksi garis (ROI)
│   ├── pid.py                 # PIDController
│   └── serial_bridge.py       # kirim PWM ke firmware
├── config/
│   └── line_tracker.yaml      # PID, parameter DDMR, dan port
├── launch/
│   └── line_tracker.launch.py
├── resource/                  # ament index
├── test/
├── package.xml
├── setup.py
└── setup.cfg
```

## Parameter

| Parameter | Default | Keterangan |
| --- | --- | --- |
| `camera_index` | `2` | index perangkat kamera V4L2 |
| `serial_port` | `/dev/ttyUSB0` | port ke firmware |
| `baud_rate` | `115200` | baud serial |
| `kp`, `ki`, `kd` | `1.0, 0.0, 0.0` | gain PID; tuning memengaruhi `angular.z` (rad/s) |
| `linear_speed_mps` | `0.15` | kecepatan maju robot (m/s) |
| `wheel_separation_m` | `0.30` | jarak antara pusat roda kiri dan kanan (m), ukur pada robot |
| `max_angular_speed_radps` | `1.0` | batas kecepatan putar robot (rad/s) |
| `max_wheel_speed_mps` | `0.30` | skala perkiraan untuk memetakan kecepatan roda ke PWM |
| `publish_rate_hz` | `30.0` | frekuensi loop |
| `aruco_enabled` | `true` | aktifkan deteksi marker ArUco |
| `aruco_consecutive_frames` | `3` | filter debouncing konfirmasi marker stabil (frame) |
| `aruco_cooldown_seconds` | `2.5` | cooldown pencegahan trigger ganda (detik) |

## Topik

| Topik | Tipe | Keterangan |
| --- | --- | --- |
| `/line_tracker/debug_image` | `sensor_msgs/Image` | frame debug (untuk `rqt_image_view`, termasuk overlay ArUco) |
| `/line_tracker/normalized_error` | `std_msgs/Float64` | error posisi garis ternormalisasi |
| `/aruco/detected_id` | `std_msgs/Int32` | ID marker ArUco terkonfirmasi (waypoint / simpang) |
| `/mission/command` | `std_msgs/String` | Perintah misi (mis. `go_to CP2`, `pause`, `resume`, `cancel`) |
| `/mission/status` | `std_msgs/String` | Telemetri status FSM misi JSON (state, step, target, posisi) |
| `/line_tracker/branch_instruction` | `std_msgs/String` | Instruksi pemilihan cabang (`LEFT`, `RIGHT`, `STRAIGHT`, `STOP`, `RESUME`) |
| `/cmd_vel` | `geometry_msgs/Twist` | kecepatan robot: m/s dan rad/s |
| `/line_tracker/encoder_delta` | `std_msgs/Int64MultiArray` | delta tick `[left, right]` dari serial |
| `/odom` | `nav_msgs/Odometry` | pose dan kecepatan odometri dalam frame `odom` |
| `/path` | `nav_msgs/Path` | jejak pose robot di frame `odom` untuk RViz |

Launch file secara otomatis menjalankan `line_tracker_node`, `odometry_node`, dan `mission_controller_node`:

```bash
ros2 launch line_tracker_ros2 line_tracker.launch.py
```

Untuk mengirim perintah navigasi misi ke robot:
```bash
# Pergi ke Checkpoint 2
ros2 topic pub --once /mission/command std_msgs/msg/String "{data: 'go_to CP2'}"

# Jeda misi
ros2 topic pub --once /mission/command std_msgs/msg/String "{data: 'pause'}"

# Lanjutkan misi
ros2 topic pub --once /mission/command std_msgs/msg/String "{data: 'resume'}"

# Pantau telemetri misi
ros2 topic echo /mission/status
```
global dan menerbitkan TF `odom -> base_link`. Parameter dimensi roda, CPR,
topik, dan nama frame berada pada bagian `odometry_node` di YAML.

Untuk melihat jejak di RViz, jalankan sistem dan `rviz2`, set **Fixed Frame**
ke `odom`, lalu pilih **Add → By topic → `/path` → Path**. Tambahkan juga
**TF** jika ingin melihat sumbu frame robot. Panjang riwayat dibatasi oleh
`max_path_poses` (default 10.000 pose); `/path` diterbitkan 2 kali per detik
secara default.

Gambar kamera debug (frame dengan overlay ROI dan posisi garis) diterbitkan pada
`/line_tracker/debug_image`. Lihat dengan `rqt_image_view` dan pilih topik itu.

PID mengubah error garis menjadi kecepatan putar `angular.z`. Kinematika balik
DDMR menghitung kecepatan roda dari `linear.x` dan `angular.z`. Firmware saat
ini tetap menerima PWM positif, sehingga pemetaan ke PWM masih berupa skala
perkiraan; target kecepatan encoder dan gerak roda mundur belum diterapkan.

## Build & jalankan

```bash
cd ros2_ws
colcon build
source install/setup.bash

# via launch + config
ros2 launch line_tracker_ros2 line_tracker.launch.py

# atau langsung
ros2 run line_tracker_ros2 line_tracker_node --ros-args \
  -p serial_port:=/dev/ttyUSB0
```

Protokol serial: [`../../../docs/serial_protocol.md`](../../../docs/serial_protocol.md).
