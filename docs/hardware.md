# Hardware & Kalibrasi Robot AGV

Sumber kebenaran untuk parameter fisik dan konvensi yang dipakai bersama oleh
firmware dan ROS 2. Jika nilai berubah, perbarui tabel di sini **dan** di
`ros2_ws/src/line_tracker_ros2/config/line_tracker.yaml`.

## 1. Parameter fisik

| Parameter | Nilai | Satuan |
| --- | --- | --- |
| Diameter efektif roda (`D`) | 0,15 | m |
| Radius roda (`r = D/2`) | 0,075 | m |
| Jarak pusat roda kiri–kanan / wheel track (`b`) | 0,48 | m |
| Rasio gearbox | 30:1 | – |
| Pulsa encoder per putaran poros motor (PPR) | 30 | pulsa/rev |
| Pulsa encoder per putaran roda | 900 | pulsa/rev |
| Interval sampling kecepatan (firmware) | 1000 | ms |

Turunan:

```text
PULSES_PER_WHEEL_REV = PPR x gear_ratio = 30 x 30 = 900 pulsa/putaran roda
Keliling roda        = pi x D          = pi x 0,15 = 0,4712 m
Jarak per pulsa      = 0,4712 / 900    = 0,5236 mm/pulsa
```

Nilai yang sama tersedia di:

- ROS 2: `config/line_tracker.yaml` (`wheel_diameter_m`, `wheel_separation_m`,
  `gear_ratio`, `encoder_pulses_per_motor_rev`, `encoder_pulses_per_wheel_rev`).
- Firmware: `firmware/include/config.h` (`PULSES_PER_REV`, `GEAR_RATIO`,
  `PULSES_PER_WHEEL_REV`, `SPEED_READ_INTERVAL_MS`).

## 2. Konvensi satuan dan tanda

| Besaran | Satuan | Tanda positif |
| --- | --- | --- |
| Panjang / posisi | m | – |
| Sudut | rad | putar kiri (CCW) |
| Waktu | s | – |
| `Twist.linear.x` | m/s | maju |
| `Twist.angular.z` | rad/s | putar kiri (CCW) |
| PWM motor | 0–255 | maju |
| Pulsa encoder | pulsa | maju (target; perlu validasi) |

- Urutan token serial: `<left_pwm>,<right_pwm>\n`.
- Motor kanan maju = `DIR_CW`, motor kiri maju = `DIR_CCW`
  (lihat `firmware/src/app/fsm.cpp`).
- `angular.z > 0` → roda kanan lebih cepat daripada roda kiri.
- Kinematika balik DDMR: `v_L = v - b·ω/2`, `v_R = v + b·ω/2`.

## 3. Pinout

Pin mapping lengkap ada di [`../firmware/include/config.h`](../firmware/include/config.h):

| Fungsi | Kiri | Kanan |
| --- | --- | --- |
| START/STOP | `L_START_STOP` (28) | `R_START_STOP` (22) |
| RUN/BRAKE | `L_RUN_BRAKE` (29) | `R_RUN_BRAKE` (23) |
| DIR | `L_DIR` (30) | `R_DIR` (24) |
| INT/VR | `L_INT_VR` (31) | `R_INT_VR` (25) |
| PWM | `L_PWM` (10) | `R_PWM` (9) |
| Encoder | `L_SPEED_OUT` (2, INT0) | `R_SPEED_OUT` (3, INT1) |
| ALARM RESET | `L_ALARM_RESET` (32) | `R_ALARM_RESET` (26) |

Tombol: `START_STOP_BTN_PIN` (40), `RESET_BTN_PIN` (42), `E_STOP_BTN_PIN` (37).
UART ke ROS: `Serial1` @ 115200. Debug: `Serial` (USB) @ 115200.

## 4. Prosedur validasi (fisik)

Item berikut **belum** tervalidasi di robot dan harus diuji sebelum dianggap
selesai.

### 4.1 Arah putaran motor

1. Angkat robot agar kedua roda bebas.
2. Kirim `left_pwm=50,right_pwm=50`.
3. Pastikan kedua roda mendorong robot ke **depan**.
4. Kirim `left_pwm=80,right_pwm=40`; robot harus berbelok ke **kanan** (CW).
5. Jika terbalik: perbaiki polaritas `DIR` atau urutan token serial.

### 4.2 Polaritas pulsa encoder

Firmware mencetak RPM roda bertanda (`+` maju, `-` mundur) beserta kecepatan
linier (m/s). Arah diturunkan dari pin `DIR` saat pulsa terjadi, karena encoder
saat ini hanya satu kanal.

1. Gerakkan robot maju pelan; pastikan RPM kedua roda **positif**.
2. Gerakkan mundur; pastikan RPM kedua roda **negatif**.
3. Jika terbalik, sesuaikan `forwardDirection` motor di `main.cpp`
   (`DIR_CW` untuk kanan, `DIR_CCW` untuk kiri).

Catatan: ini adalah arah *inferensi* dari perintah, bukan pengukuran. Untuk
pengukuran arah yang benar-benar terukur, diperlukan encoder quadrature dua
kanal (A/B).

### 4.3 Wheel track (putar 360°)

1. Tandai orientasi awal robot di lantai.
2. Perintahkan putar di tempat 360° (setelah kontrol kecepatan tertutup ada).
3. Ukur selisih sudut akhir; koreksi `wheel_separation_m` hingga tepat 360°.

### 4.4 Diameter efektif roda (jalan 2 m)

1. Tandai garis start dan finish berjarak 2 m.
2. Jalan lurus dan catat jumlah pulsa roda.
3. `D_efektif = (jarak tempuh x 900) / (pi x jumlah pulsa)`.
4. Perbarui `wheel_diameter_m` bila berbeda dari 0,15 m.

### 4.5 E-stop

Tekan E-stop saat motor bergerak: motor harus berhenti seketika, dan perintah
dari ROS/IoT tidak boleh menghidupkannya kembali sampai E-stop dilepas.

### 4.6 Koneksi serial

1. Cek port stabil: `ls -l /dev/ttyUSB*` dan `dmesg | tail`.
2. Jalankan node; pastikan log `Terhubung ke serial`.
3. Cabut kabel serial saat motor bergerak; firmware harus berhenti karena
   `SERIAL_COMMAND_TIMEOUT_MS` (500 ms).
