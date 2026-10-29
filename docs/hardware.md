# Hardware & Kalibrasi Robot AGV

Sumber kebenaran untuk parameter fisik dan konvensi yang dipakai bersama oleh
firmware dan ROS 2. Jika nilai berubah, perbarui tabel di sini **dan** di
`ros2_ws/src/line_tracker_ros2/config/line_tracker.yaml` serta `firmware/include/config.h`.

## 1. Parameter fisik

| Parameter | Nilai | Satuan | Keterangan |
| --- | --- | --- | --- |
| Diameter efektif roda (`D`) | 0,15 | m | 15 cm |
| Radius roda (`r = D/2`) | 0,075 | m | 7,5 cm |
| Jarak pusat roda kiri–kanan / wheel track (`b`) | 0,48 | m | 48 cm |
| Rasio gearbox | 30:1 | – | Reduksi putaran motor ke poros roda |
| Pulsa encoder per putaran poros motor (PPR) | 30 | pulsa/rev | Sinyal interupsi satu kanal |
| Pulsa encoder per putaran roda (`CPR`) | 900 | pulsa/rev | 30 PPR × rasio 30:1 |
| Periode siklus kontrol & telemetri (`CONTROL_PERIOD_MS`) | 100 | ms | Frekuensi kontrol PID & serial: 10 Hz |
| Interval sampling kecepatan akumulasi (`SPEED_READ_INTERVAL_MS`) | 1000 | ms | Interval kalkulasi RPM monitor debug |

Turunan:

```text
PULSES_PER_WHEEL_REV = PPR x gear_ratio = 30 x 30 = 900 pulsa/putaran roda
Keliling roda        = pi x D          = pi x 0,15 ≈ 0,471239 m
Jarak per pulsa (dt) = 0,471239 / 900  ≈ 0,0005236 m/pulsa (0,5236 mm/pulsa)
```

Nilai yang sama tersedia di:

- ROS 2: `ros2_ws/src/line_tracker_ros2/config/line_tracker.yaml` (`wheel_diameter_m`, `wheel_separation_m`,
  `gear_ratio`, `encoder_pulses_per_motor_rev`, `encoder_pulses_per_wheel_rev`).
- Firmware: `firmware/include/config.h` (`WHEEL_DIAMETER_M`, `WHEEL_SEPARATION_M`,
  `PULSES_PER_REV`, `GEAR_RATIO`, `PULSES_PER_WHEEL_REV`, `CONTROL_PERIOD_MS`).

## 2. Konvensi satuan dan tanda

| Besaran | Satuan | Tanda positif | Keterangan |
| --- | --- | --- | --- |
| Panjang / posisi global ($x, y$) | m | $+x$ maju, $+y$ kiri | Frame koordinat ROS REP-103 / REP-105 (`odom`, `base_link`) |
| Sudut heading ($\theta$) | rad | putar kiri (CCW) | $0$ rad menghadap sumbu $+x$ global |
| Waktu ($t$) | s | bertambah | `CONTROL_PERIOD_MS` = 0,1 s |
| `Twist.linear.x` ($v$) | m/s | maju | Kecepatan linier robot |
| `Twist.angular.z` ($\omega$) | rad/s | putar kiri (CCW) | Kecepatan putar robot |
| Target tick per interval | tick/100ms | maju | Dihitung dari kinematika balik di firmware |
| PWM internal motor | −255…255 | maju | Dihasilkan dari kontroler PID loop tertutup di firmware |
| Pulsa encoder ($\Delta s$) | pulsa | putaran maju | Dihitung per siklus kontrol 100 ms |

- Format komunikasi serial:
  - ROS 2 → Firmware: `C,<linear_x>,<angular_z>\n` (satuan: m/s dan rad/s).
  - Firmware → ROS 2: `E,<left_ticks>,<right_ticks>\n` (satuan: pulsa tick per interval 100 ms).
- Arah motor fisik: Motor kanan maju = `DIR_CW`, motor kiri maju = `DIR_CCW` (dikonfigurasi pada instansiasi objek `Motor` di `firmware/src/main.cpp`).
- `angular.z > 0` → roda kanan berputar lebih cepat daripada roda kiri (belok ke kiri).
- Kinematika balik DDMR (dihitung di firmware):
  $$v_L = v - \frac{b \cdot \omega}{2}, \quad v_R = v + \frac{b \cdot \omega}{2}$$

## 3. Pinout

Pin mapping lengkap tersedia di [`../firmware/include/config.h`](../firmware/include/config.h):

| Fungsi | Kiri | Kanan | Keterangan |
| --- | --- | --- | --- |
| START/STOP | `L_START_STOP` (28) | `R_START_STOP` (22) | Enable output driver |
| RUN/BRAKE | `L_RUN_BRAKE` (29) | `R_RUN_BRAKE` (23) | Rem aktif / rilis |
| DIR | `L_DIR` (30) | `R_DIR` (24) | Arah putaran (CW / CCW) |
| INT/VR | `L_INT_VR` (31) | `R_INT_VR` (25) | Mode kontrol kecepatan internal/eksternal |
| PWM | `L_PWM` (10) | `R_PWM` (9) | Sinyal PWM kecepatan (Timer AVR) |
| Encoder (SPEED_OUT) | `L_SPEED_OUT` (2, INT0) | `R_SPEED_OUT` (3, INT1) | Pin interupsi eksternal hardware |
| ALARM RESET | `L_ALARM_RESET` (32) | `R_ALARM_RESET` (26) | Reset driver motor bila terjadi fault |

Tombol kontrol:
- `START_STOP_BTN_PIN`: 40 (Active Low, internal pull-up)
- `RESET_BTN_PIN`: 42 (Active Low, internal pull-up)
- `E_STOP_BTN_PIN`: 37 (Active High, sensor darurat)

Port UART:
- **`Serial1` (Pin 19 RX1, Pin 18 TX1):** Port komunikasi data dua arah ke komputer ROS 2 (via USB-to-TTL adapter @ 115200 bps).
- **`Serial` (USB Onboard Arduino Mega):** Port monitor debug dan status FSM @ 115200 bps.

---

## 4. Prosedur validasi (fisik)

Item berikut **belum** divalidasi penuh di lantai robot dan harus diuji sebelum dianggap selesai.

### 4.1 Arah putaran motor

1. Angkat robot agar kedua roda bebas berputar tanpa menyentuh lantai.
2. Nyalakan catu daya dan pastikan firmware berada dalam state `MOVING` (tekan tombol Start/Stop).
3. Kirim perintah maju lurus dengan kecepatan rendah dari ROS 2:
   ```bash
   ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}"
   ```
   (atau kirim frame `C,0.1000,0.0000\n` lewat terminal serial).
4. Pastikan kedua roda berputar mendorong robot ke **depan**.
5. Kirim perintah belok kanan (CW):
   ```bash
   ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: -0.5}}"
   ```
   Pastikan roda kiri berputar maju dan roda kanan berputar mundur.
6. Jika terbalik: periksa polaritas `DIR_CW`/`DIR_CCW` pada `firmware/src/main.cpp` atau sambungan pin `DIR`.

### 4.2 Polaritas pulsa encoder

Firmware menghitung delta pulsa encoder setiap 100 ms dan mengirimkan telemetri `E,<left_ticks>,<right_ticks>` ke serial ROS (`Serial1`).

1. Putar roda kiri dan kanan secara manual ke arah **depan**.
2. Pantau output topik ROS 2:
   ```bash
   ros2 topic echo /line_tracker/encoder_delta
   ```
   atau pantau baris `E,...` di serial.
3. Pastikan delta ticks untuk kedua roda bernilai **positif**.
4. Jika berputar mundur, pastikan nilai delta ticks bernilai **negatif** (atau disesuaikan dengan arah `DIR`).
5. Catatan: Encoder saat ini menggunakan pembacaan satu kanal dengan arah diinferensi dari status `DIR`. Untuk pengukuran dua arah sejati tanpa inferensi sinyal kontrol, diperlukan encoder quadrature (fase A/B).

### 4.3 Wheel track (uji putar 360°)

1. Letakkan robot di lantai datar dan tandai orientasi awal robot.
2. Perintahkan putar di tempat sebanyak 360° menggunakan `cmd_vel` atau kontrol rotasi terukur.
3. Periksa orientasi akhir pada topik `/odom` atau visualisasi RViz `TF` `odom -> base_link`.
4. Jika robot fisik sudah berputar tepat 360° namun odometri belum mencapai $2\pi$ rad (atau sebaliknya):
   - Koreksi parameter `wheel_separation_m` di `line_tracker.yaml` dan `firmware/include/config.h`.

### 4.4 Diameter efektif roda (uji jalan 2 meter)

1. Tandai garis start dan finish berjarak tepat 2 meter di lantai.
2. Jalankan robot maju lurus dari garis start hingga garis finish.
3. Bandingkan perpindahan $x$ pada topik `/odom` dengan jarak fisik 2 meter.
4. Hitung diameter roda efektif terkoreksi:
   $$D_{\text{efektif}} = \frac{s_{\text{fisik}} \times \text{CPR}}{\pi \times \text{total\_pulsa}}$$
5. Perbarui `wheel_diameter_m` di file konfigurasi jika berbeda dari $0{,}15$ m.

### 4.5 Pengujian E-stop

1. Jalankan motor dalam status `MOVING`.
2. Tekan tombol E-stop fisik: motor kedua roda harus berhenti seketika.
3. Kirim perintah serial dari ROS 2: pastikan motor **tidak dapat dihidupkan kembali** selama status E-stop belum di-reset dan tombol belum dilepas secara fisik.
4. Verifikasi bahwa E-stop yang aktif tidak bisa di-bypass oleh software atau perintah IoT.

### 4.6 Pengujian koneksi serial & failsafe timeout

1. Pastikan port serial terdeteksi stabil:
   ```bash
   ls -l /dev/ttyUSB*
   dmesg | tail
   ```
2. Jalankan launch file ROS 2 (`ros2 launch line_tracker_ros2 line_tracker.launch.py`) dan pastikan log mencetak `Terhubung ke serial`.
3. Jalankan robot dalam status `MOVING`.
4. Cabut kabel serial atau hentikan node ROS 2: pastikan motor otomatis berhenti dalam waktu $\le 500$ ms karena mekanisme failsafe `SERIAL_COMMAND_TIMEOUT_MS`.
