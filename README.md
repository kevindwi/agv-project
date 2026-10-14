# AGV Project

AGV (Automated Guided Vehicle) berbasis *line tracking*.

Alur sistem:

```
  Kamera (V4L2)
      │
      ▼
  ROS 2 node  ──  deteksi garis (OpenCV) ──► hitung error ──► PID
      │                                                       │
      │  publish: /line_tracker/debug_image                    │
      │           /line_tracker/normalized_error               │
      │           /motor_pwm, /cmd_vel                         │
      ▼                                                       │
  Serial (115200)  ◄────────  "<kiri>,<kanan>\n"  ────────────┘
      │
      ▼
  Firmware Arduino Mega (FreeRTOS)  ──  FSM  ──  Driver motor
```

## Struktur repo

```
agv-project/
├── firmware/               # PlatformIO: Arduino Mega 2560 + FreeRTOS
│   ├── include/            # header publik (.h)
│   ├── src/                # implementasi (.cpp)
│   ├── lib/                # library lokal
│   ├── test/               # unit test (pio test)
│   └── platformio.ini
├── ros2_ws/                # ROS 2 workspace
│   └── src/
│       └── line_tracker_ros2/   # package ament_python
│           ├── line_tracker_ros2/
│           ├── resource/
│           ├── test/
│           ├── package.xml
│           └── setup.py
├── docs/
│   └── serial_protocol.md  # KONTRAK firmware <-> ROS 2
└── README.md
```

## Prasyarat

- **Firmware**: [PlatformIO Core](https://docs.platformio.org/) (`pio`)
- **ROS 2**: distro Jazzy/Iron/Humble + `colcon`, `cv_bridge`
- **Python**: `opencv-python`, `pyserial`

## Build & jalankan

### Firmware

```bash
cd firmware
pio run                 # build
pio run -t upload       # flash ke board
pio device monitor      # lihat serial debug (115200)
```

### ROS 2

```bash
cd ros2_ws
colcon build
source install/setup.bash
ros2 run line_tracker_ros2 line_tracker_node --ros-args \
  -p serial_port:=/dev/ttyUSB0 -p baud_rate:=115200
```

## Dokumentasi

| Dokumen | Isi |
| --- | --- |
| [`docs/serial_protocol.md`](docs/serial_protocol.md) | Kontrak format data serial firmware ↔ ROS 2 |
