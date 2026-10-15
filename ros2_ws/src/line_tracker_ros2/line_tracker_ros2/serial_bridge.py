"""Jembatan serial dari ROS 2 ke firmware AGV.

Kontrak frame (lihat docs/serial_protocol.md):

    "<left_pwm>,<right_pwm>\\n"
"""

import time

import serial


class SerialBridge:
    """Mengirim perintah PWM roda kiri & kanan ke mikrokontroler via serial."""

    def __init__(
        self,
        port: str,
        baud_rate: int,
        timeout: float = 1.0,
        startup_delay: float = 2.0,
    ):
        self.port = port
        self.baud_rate = baud_rate
        self.timeout = timeout
        self.startup_delay = startup_delay
        self._serial = None

    def connect(self) -> None:
        """Buka port serial lalu tunggu mikrokontroler siap (reset saat dibuka)."""
        self._serial = serial.Serial(self.port, self.baud_rate, timeout=self.timeout)
        if self.startup_delay > 0:
            time.sleep(self.startup_delay)

    def is_connected(self) -> bool:
        """True bila port serial terbuka."""
        return self._serial is not None and self._serial.is_open

    def send_pwm(self, left_pwm: int, right_pwm: int) -> None:
        """Kirim frame PWM roda kiri & kanan."""
        if not self.is_connected():
            return
        frame = f"{int(left_pwm)},{int(right_pwm)}\n"
        self._serial.write(frame.encode("utf-8"))

    def close(self) -> None:
        """Tutup port serial dengan aman."""
        if self.is_connected():
            self._serial.close()
