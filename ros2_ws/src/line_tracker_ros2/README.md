# line_tracker_ros2

Node ROS 2 untuk line tracking: membaca kamera, mendeteksi garis, menghitung
koreksi PID, lalu mengirim PWM roda ke firmware AGV melalui serial.

## Struktur package

```
line_tracker_ros2/
├── line_tracker_ros2/
│   ├── __init__.py
│   ├── line_tracker_node.py   # wiring: parameter -> kamera -> PID -> serial
│   ├── camera.py              # buka kamera + deteksi garis (ROI)
│   ├── pid.py                 # PIDController
│   └── serial_bridge.py       # kirim PWM ke firmware
├── config/
│   └── line_tracker.yaml      # kp/ki/kd/base_speed/port
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
| `kp`, `ki`, `kd` | `5.0, 0.0, 0.0` | gain PID |
| `base_speed` | `25` | PWM acuan (0–255) |
| `publish_rate_hz` | `30.0` | frekuensi loop |
| `show_debug_window` | `true` | window OpenCV debug |

## Topik

| Topik | Tipe | Keterangan |
| --- | --- | --- |
| `/line_tracker/debug_image` | `sensor_msgs/Image` | frame debug (untuk `rqt_image_view`) |
| `/line_tracker/normalized_error` | `std_msgs/Float64` | error posisi garis ternormalisasi |
| `/motor_pwm` | `std_msgs/Int32MultiArray` | `[left_pwm, right_pwm]` |
| `/cmd_vel` | `geometry_msgs/Twist` | kecepatan robot (skala saat ini) |

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
