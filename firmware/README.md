# Firmware AGV (Arduino Mega 2560)

Firmware penerima perintah kecepatan dari ROS 2 melalui serial UART, menjalankan FSM,
menjalankan kinematika balik DDMR & PID kecepatan tertutup per roda, membaca encoder,
serta mengendalikan dua driver motor.

## Struktur

```text
firmware/
├── platformio.ini          # konfigurasi PlatformIO (board, lib)
├── include/                # header publik (.h)
│   ├── config.h            # pin mapping, parameter fisik, & konstanta PID/timing
│   ├── motor.h             # driver kendali motor DC (PWM, DIR, brake)
│   ├── button.h            # debouncing input tombol
│   ├── speed.h             # interrupt encoder roda
│   ├── pid.h               # algoritma PID diskrit untuk kecepatan roda
│   └── fsm.h               # finite state machine sistem AGV
├── src/                    # implementasi (.cpp)
│   ├── main.cpp            # inisialisasi & loop tombol/FSM
│   ├── app/                # logika aplikasi
│   │   ├── fsm.cpp         # FSM, parser serial C/E, & loop kontrol PID
│   │   └── pid.cpp         # implementasi PIDController
│   └── drivers/            # akses hardware
│       ├── motor.cpp
│       ├── button.cpp
│       └── speed.cpp
├── lib/                    # library lokal reusable
└── test/                   # unit test (pio test + Unity)
```

- `app/` = logika kontrol (FSM, parser protokol serial, closed-loop PID).
- `drivers/` = akses perangkat keras (driver motor, encoder interupsi, tombol).
- Semua berkas sumber di bawah `src/` dikompilasi otomatis oleh PlatformIO.

## Build & flash

```bash
pio run                 # build
pio run -t upload       # flash ke board
pio device monitor      # serial monitor debug 115200 (USB onboard)
```

## Protokol serial

- **Menerima frame perintah:** `C,<linear_x>,<angular_z>\n` (kecepatan m/s dan rad/s dari ROS 2).
- **Mengirim frame telemetri:** `E,<left_ticks>,<right_ticks>\n` (delta ticks encoder per siklus kontrol 100 ms).
- **Failsafe timeout:** Motor otomatis berhenti jika tidak ada perintah valid selama $\ge 500$ ms saat `MOVING`.

Detail lengkap ada di [`../docs/serial_protocol.md`](../docs/serial_protocol.md).
