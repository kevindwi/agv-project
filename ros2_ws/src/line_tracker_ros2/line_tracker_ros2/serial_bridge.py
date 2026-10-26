"""Jembatan serial dari ROS 2 ke firmware AGV.

Message :
    "C,<linear_x>,<angular_z>\\n" dari ROS, "E,<left_ticks>,<right_ticks>\\n" dari firmware
"""

import time

import serial


class SerialBridge:
    def __init__(
        self,
        port: str,
        baud_rate: int,
        timeout: float = 0.05,
        startup_delay: float = 2.0,
    ):
        self.port = port
        self.baud_rate = baud_rate
        self.timeout = timeout
        self.startup_delay = startup_delay
        self._serial = None

    def connect(self) -> None:
        self._serial = serial.Serial(self.port, self.baud_rate, timeout=self.timeout)
        if self.startup_delay > 0:
            time.sleep(self.startup_delay)

    def is_connected(self) -> bool:
        return self._serial is not None and self._serial.is_open

    def send_cmd_vel(self, linear_x: float, angular_z: float) -> None:
        if not self.is_connected():
            return
        frame = f"C,{linear_x:.4f},{angular_z:.4f}\n"
        self._serial.write(frame.encode("utf-8"))

    def read_encoder_delta(self):
        if not self.is_connected() or self._serial.in_waiting == 0:
            return None

        line = self._serial.readline().decode("ascii", errors="ignore").strip()
        fields = line.split(",")
        if len(fields) != 3 or fields[0] != "E":
            return None

        try:
            return int(fields[1]), int(fields[2])
        except ValueError:
            return None

    def close(self) -> None:
        if self.is_connected():
            self._serial.close()
