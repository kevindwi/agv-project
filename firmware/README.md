# Firmware AGV (Arduino Mega 2560 + FreeRTOS)

Firmware penerima perintah PWM dari ROS 2 melalui serial, menjalankan FSM, dan
mengendalikan dua motor driver.

## Struktur

```
firmware/
├── platformio.ini          # konfigurasi PlatformIO (board, lib)
├── include/                # header publik (.h)
│   ├── config.h            # pin mapping + konstanta
│   ├── motor.h
│   ├── button.h
│   ├── speed.h
│   └── fsm.h
├── src/                    # implementasi (.cpp)
│   ├── main.cpp            # wiring setup()/loop() saja
│   ├── app/                # logika aplikasi
│   │   └── fsm.cpp
│   └── drivers/            # akses hardware
│       ├── motor.cpp
│       ├── button.cpp
│       └── speed.cpp
├── lib/                    # library lokal reusable
└── test/                   # unit test (pio test + Unity)
```

- `app/` = logika (FSM, alur). `drivers/` = akses perangkat keras (motor, tombol, encoder).
- Semua sumber di bawah `src/` dikompilasi otomatis oleh PlatformIO (rekursif).

## Build & flash

```bash
pio run                 # build
pio run -t upload       # flash ke board
pio device monitor      # serial debug 115200
```

## Protokol serial

Menerima frame ASCII: `<left_pwm>,<right_pwm>\n` (PWM 0–255).
Detail lengkap ada di [`../docs/serial_protocol.md`](../docs/serial_protocol.md).
