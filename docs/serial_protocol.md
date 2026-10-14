# Protokol Serial — ROS 2 ↔ Firmware AGV

Dokumen ini adalah **kontrak** antara node ROS 2 dan firmware Arduino.
Setiap perubahan format data **wajib** diperbarui di sini agar kedua sisi tetap sinkron.

## 1. Ringkasan

| Item | Nilai |
| --- | --- |
| Arah | ROS 2 → Firmware (satu arah) |
| Media | UART / USB-serial (`Serial1` di firmware) |
| Baud rate | 115200 (default) |
| Port (ROS) | `/dev/ttyUSB0` — parameter `serial_port` |
| Framing | Berakhir dengan `\n`, karakter `\r` diabaikan |
| Encoding | ASCII |
| Isi frame | Dua bilangan bulat dipisah koma |

## 2. Format frame

```
<a>,<b>\n
```

- `<a>` dan `<b>`: bilangan bulat desimal PWM, rentang **0–255**.
- Contoh: `120,100\n`

## 3. Semantik field

| Field | Dikirim ROS 2 | Dibaca firmware |
| --- | --- | --- |
| token ke-1 | `pwm_kiri` (roda kiri) | `rightSpeed` (roda kanan) |
| token ke-2 | `pwm_kanan` (roda kanan) | `leftSpeed` (roda kiri) |

> **⚠️ Inkonsistensi (BUG) — perlu diputuskan.**
> Node ROS 2 (`line_tracker_node.py`) mengirim `f"{pwm_kiri},{pwm_kanan}\n"`, sedangkan firmware
> (`fsm.cpp::handleSerial`) memakai token pertama sebagai `rightSpeed`.
> Artinya **nilai PWM roda kiri dan kanan tertukar** di sisi firmware.
>
> Pilihan perbaikan (pilih salah satu, lalu samakan dokumen ini):
> 1. Ubah ROS agar mengirim `f"{pwm_kanan},{pwm_kiri}\n"`, **atau**
> 2. Ubah firmware: token pertama → `leftSpeed`, token kedua → `rightSpeed`.
>
> Rekomendasi: jadikan urutan kanonik **`kiri,kanan`** (ikuti sisi ROS) dan perbaiki firmware.

## 4. Detail implementasi

### Pengirim — `ros2_ws/src/line_tracker_ros2/line_tracker_ros2/line_tracker_node.py`

- Membuka port: `serial.Serial(port, baud, timeout=1)`, lalu `time.sleep(2)` agar MCU stabil.
- Nilai PWM di-clamp ke `0..255` sebelum dikirim: `max(0, min(255, int(round(pwm))))`.
- Kirim: `self.ser.write(f"{pwm_kiri},{pwm_kanan}\n".encode("utf-8"))`.

### Penerima — `firmware/src/fsm.cpp::handleSerial()`

- Buffer: `static char buf[16]` (maksimum **15 karakter** sebelum `\n`).
- Menumpuk karakter sampai `\n`; `\r` dibuang.
- Parsing: `strtok(buf, ",")` → token 1 ke `rightSpeed`, token 2 ke `leftSpeed` (via `atoi`).
- Setelah itu baris debug dicetak ke Serial USB: `R: <rightSpeed> | L: <leftSpeed>`.

## 5. Batasan & catatan

- **Tidak ada checksum / handshake.** Tidak ada validasi integritas atau ACK.
  Jika data korup, motor bisa bergerak salah. Pertimbangkan menambah checksum bila dianggap perlu.
- **Tidak ada telemetri balik.** Firmware belum mengirim data (mis. RPM encoder) ke ROS;
  output `Serial` hanya untuk debug manusia (`RPM: ...`).
- **Panjang frame.** Karena buffer firmware 16 byte, frame harus ≤ 15 karakter.
  `255,255` (7 karakter) aman, tetapi jangan menambah spasi/field tanpa memperbesar buffer.
- **Timeout.** Karena tak ada handshake, `time.sleep(2)` di sisi ROS adalah kompensasi sementara.

## 6. Contoh urutan

```
ROS 2  --("120,100\n")-->  Firmware
       kiri=120, kanan=100
Firmware (setelah perbaikan): leftSpeed=120, rightSpeed=100
```
