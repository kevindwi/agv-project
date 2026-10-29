# Protokol Serial — ROS 2 ↔ Firmware AGV

Dokumen ini adalah **kontrak komunikasi serial** antara node ROS 2 dan firmware Arduino Mega 2560.
Setiap perubahan format data **wajib** diperbarui di sini agar kedua sisi tetap sinkron.

## 1. Ringkasan

| Item | Nilai |
| --- | --- |
| Arah | **Dua arah** (ROS 2 ↔ Firmware) |
| Media | UART / USB-serial (`Serial1` di firmware Arduino Mega 2560) |
| Baud rate | 115200 bps |
| Port (ROS 2) | `/dev/ttyUSB0` (dapat dikonfigurasi via parameter `serial_port`) |
| Port (Firmware) | `Serial1` (Pin 19: RX1, Pin 18: TX1) |
| Port Debug | `Serial` (USB onboard Arduino Mega) @ 115200 bps |
| Framing | Berakhir dengan baris baru `\n` (karakter `\r` diabaikan) |
| Encoding | ASCII |
| Periode Kontrol | 100 ms (`CONTROL_PERIOD_MS` = 10 Hz) |
| Failsafe Timeout | 500 ms (`SERIAL_COMMAND_TIMEOUT_MS`) |

---

## 2. Format Frame

### 2.1 Perintah Kecepatan: ROS 2 → Firmware (`C`)

Dikirim oleh ROS 2 (`line_tracker_node` melalui `serial_bridge.py`) untuk mengatur kecepatan linier dan angular robot.

```text
C,<linear_x>,<angular_z>\n
```

- **Prefix:** Karakter `C` diikuti tanda koma.
- `<linear_x>`: Kecepatan linier target robot dalam **m/s** (bilangan desimal floating-point, format 4 angka di belakang koma).
  - Positif: bergerak maju.
  - Negatif: bergerak mundur.
  - `0.0`: berhenti linier.
- `<angular_z>`: Kecepatan sudut target robot dalam **rad/s** (bilangan desimal floating-point, format 4 angka di belakang koma).
  - Positif: putar kiri (CCW / berlawanan jarum jam).
  - Negatif: putar kanan (CW / searah jarum jam).
  - `0.0`: lurus (tidak berputar).
- **Contoh:**
  - `C,0.3000,0.0000\n` : Maju lurus dengan kecepatan 0,3 m/s.
  - `C,0.2000,0.5000\n` : Maju 0,2 m/s sambil berbelok ke kiri 0,5 rad/s.
  - `C,0.0000,0.0000\n` : Berhenti seketika.

### 2.2 Telemetri Encoder: Firmware → ROS 2 (`E`)

Dikirim secara periodik oleh firmware setiap siklus kontrol (100 ms) saat robot dalam kondisi `MOVING`.

```text
E,<left_ticks>,<right_ticks>\n
```

- **Prefix:** Karakter `E` diikuti tanda koma.
- `<left_ticks>`: Delta pulsa (tick) encoder roda kiri yang terakumulasi selama interval kontrol 100 ms terakhir (bilangan bulat bertanda / `long`).
- `<right_ticks>`: Delta pulsa (tick) encoder roda kanan yang terakumulasi selama interval kontrol 100 ms terakhir (bilangan bulat bertanda / `long`).
- **Contoh:**
  - `E,12,12\n` : Kedua roda berputar maju dengan laju pulsa yang sama.
  - `E,8,15\n` : Roda kanan berputar lebih cepat dari roda kiri (robot berbelok ke kiri).

---

## 3. Semantik Field & Kinematika DDMR

Urutan kanonik untuk seluruh pembacaan/data roda: **kiri, kanan**.

| Pesan | Token ke-1 | Token ke-2 | Satuan |
| --- | --- | --- | --- |
| Perintah `C` | `linear_x` | `angular_z` | m/s, rad/s |
| Respon `E` | `left_ticks` | `right_ticks` | tick / 100 ms |

### Kinematika Balik DDMR di Firmware

Kinematika balik diferensial dihitung langsung pada firmware Arduino (`firmware/src/app/fsm.cpp`):

$$v_L = v - \frac{\omega \cdot b}{2}$$
$$v_R = v + \frac{\omega \cdot b}{2}$$

Di mana:
- $v$ = `linear_x` (m/s)
- $\omega$ = `angular_z` (rad/s)
- $b$ = `WHEEL_SEPARATION_M` = $0{,}48$ m (jarak antar-roda)

Target tick per siklus kontrol $\Delta t$ ($0{,}1$ s):

$$\text{metersPerTick} = \frac{\pi \cdot D}{\text{PULSES\_PER\_WHEEL\_REV}} = \frac{\pi \cdot 0{,}15}{900} \approx 0{,}0005236\text{ m/tick}$$
$$\text{targetLeftTicks} = \frac{v_L \cdot \Delta t}{\text{metersPerTick}}$$
$$\text{targetRightTicks} = \frac{v_R \cdot \Delta t}{\text{metersPerTick}}$$

Nilai target tick tersebut dimasukkan ke loop PID roda kiri dan kanan di firmware untuk menghasilkan nilai PWM bertanda ($-255 \dots 255$) pada driver motor.

---

## 4. Detail Implementasi

### Sisi ROS 2 (`ros2_ws/src/line_tracker_ros2/line_tracker_ros2/serial_bridge.py`)

- **Koneksi:** `serial.Serial(port, baud_rate, timeout=0.05)`, dengan `startup_delay=2.0` detik agar bootloader Arduino stabil.
- **Kirim Kecepatan:** `send_cmd_vel(linear_x, angular_z)` memformat frame `f"C,{linear_x:.4f},{angular_z:.4f}\n"`.
- **Baca Encoder:** `read_encoder_delta()` memeriksa antrean buffer serial (`in_waiting > 0`), membaca baris ASCII, memverifikasi prefix `E` dan panjang field = 3, lalu mengembalikan tuple `(left_ticks, right_ticks)`.
- **Integrasi Node:** Topik `/line_tracker/encoder_delta` (`std_msgs/msg/Int64MultiArray`) diterbitkan oleh `line_tracker_node` dan dikonsumsi oleh `odometry_node` untuk menghitung pose global $(x, y, \theta)$ dan TF `odom -> base_link`.

### Sisi Firmware (`firmware/src/app/fsm.cpp`)

- **Buffer Penerimaan:** `static char buffer[48]` di fungsi `handleSerial()`.
- **Proteksi Overflow:** Jika frame serial melebihi 47 karakter sebelum `\n`, karakter selanjutnya diabaikan (`discardLine = true`) sampai ditemukan karakter `\n` berikutnya.
- **Parsing:** Fungsi `parseCommand(char *line)` memvalidasi prefix `C,`, tokenisasi koma dengan `strchr`, dan parsing float dengan `strtod`. Frame ditolak jika mengandung karakter non-numerik atau `!isfinite()`.
- **Output Encoder:** Dipanggil di `update()` setiap kali interval `CONTROL_PERIOD_MS` (100 ms) terpenuhi dan state aktif `MOVING`.

---

## 5. Mekanisme Failsafe & Penanganan Error

1. **Command Timeout (`SERIAL_COMMAND_TIMEOUT_MS = 500 ms`):**
   - Jika robot berada dalam status `MOVING`, tetapi firmware tidak menerima frame `C,...` yang valid selama $\ge 500$ ms:
     - Motor kedua roda langsung dihentikan (`stopMotors()`).
     - State internal PID di-reset (`reset()`).
     - Pulsa encoder dibersihkan agar tidak menumpuk.
     - Pesan peringatan dikirim ke port monitor debug (`Serial`).
   - Motor akan kembali bergerak begitu frame perintah valid berikutnya diterima.

2. **Penghentian Saat Tidak `MOVING`:**
   - Dalam status selain `MOVING` (`IDLE`, `ESTOP`, `ERROR`), motor dipastikan berhenti dan pulsa encoder terus di-reset agar tidak ada akumulasi nilai lama saat transisi state.

3. **Batasan Saat Ini & Rencana Peningkatan:**
   - **Checksum / CRC:** Saat ini frame belum dilengkapi checksum. Frame rusak akibat noise motor berpotensi dibuang oleh validator `strtod` atau ditolak jika delimiter salah. Peningkatan checksum (CRC8/XOR) direncanakan pada Tahap 1 Roadmap.
   - **Telemetri Orientasi IMU:** Penambahan frame telemetri `I,<yaw_rad>\n` untuk sensor IMU BNO055 direncanakan pada Tahap 2B Roadmap.

---

## 6. Contoh Alur Pertukaran Data

```text
  ROS 2 (line_tracker_node)                       Firmware Arduino Mega
             │                                              │
             │─── "C,0.3000,0.0000\n" ────────────────────►│
             │    (Perintah: maju 0.3 m/s)                  │ Parse & hitung target tick
             │                                              │ Jalankan PID roda
             │◄── "E,14,14\n" ──────────────────────────────│ Kirim delta tick (100 ms)
             │    (Odometri mengupdate x, y, theta)         │
             │                                              │
             │─── "C,0.3000,-0.2000\n" ───────────────────►│
             │    (Perintah: maju belok kanan)              │
             │◄── "E,16,11\n" ──────────────────────────────│
             │                                              │
             │    [Koneksi serial terputus / lag > 500ms]   │
             │                                              │ Failsafe timeout tercapai
             │                                              │ -> Motor STOP seketika!
```
