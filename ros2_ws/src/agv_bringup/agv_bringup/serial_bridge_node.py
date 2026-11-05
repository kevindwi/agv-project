"""Node serial bridge: jembatan komunikasi non-blocking antara ROS 2 dan firmware motor AGV."""

import time
from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import serial
from std_msgs.msg import Int64MultiArray


class SerialBridgeNode(Node):
    def __init__(self):
        super().__init__("serial_bridge_node")

        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("encoder_topic", "/agv/encoder_delta")
        self.declare_parameter("timeout_sec", 0.5)

        self.port = str(self.get_parameter("serial_port").value)
        self.baud_rate = int(self.get_parameter("baud_rate").value)
        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        encoder_topic = str(self.get_parameter("encoder_topic").value)
        self.watchdog_timeout = float(self.get_parameter("timeout_sec").value)

        self._serial = None
        self._rx_buffer = b""
        self.last_cmd_time = 0.0
        self.last_idle_heartbeat = 0.0

        # Publisher encoder
        self.encoder_publisher = self.create_publisher(Int64MultiArray, encoder_topic, 10)
        self.legacy_encoder_publisher = self.create_publisher(
            Int64MultiArray, "/line_tracker/encoder_delta", 10
        )

        # Subscriber cmd_vel
        self.create_subscription(Twist, cmd_vel_topic, self.cmd_vel_callback, 10)

        # Timer baca serial & keepalive (50 Hz / 20 ms)
        self.timer = self.create_timer(0.02, self.timer_callback)

        self._connect_serial()

    def _connect_serial(self):
        try:
            # timeout=0 agar pembacaan serial murni non-blocking tanpa membuat loop ROS macet
            self._serial = serial.Serial(self.port, self.baud_rate, timeout=0)
            time.sleep(1.0)
            self.get_logger().info(f"Terhubung ke serial hardware: {self.port} @ {self.baud_rate}")
        except OSError as e:
            self.get_logger().warn(f"Belum dapat membuka serial port {self.port}: {e}")
            self._serial = None

    def cmd_vel_callback(self, msg: Twist):
        """Kirim perintah gerak LANGSUNG seketika saat pesan /cmd_vel tiba."""
        now = time.monotonic()
        self.last_cmd_time = now
        self._send_command(msg.linear.x, msg.angular.z)

    def _send_command(self, linear_x: float, angular_z: float):
        if self._serial is None or not self._serial.is_open:
            return
        frame = f"C,{linear_x:.4f},{angular_z:.4f}\n"
        try:
            self._serial.write(frame.encode("utf-8"))
        except OSError as e:
            self.get_logger().warn(f"Gagal mengirim serial: {e}")

    def timer_callback(self):
        now = time.monotonic()

        # 1. HANYA kirim keepalive 'C,0,0' saat IDLE (tidak ada /cmd_vel aktif) setiap 150 ms
        #    Ini mencegah timeout Arduino tanpa mengganggu aliran /cmd_vel asli saat robot jalan
        if now - self.last_cmd_time > 0.25:
            if now - self.last_idle_heartbeat >= 0.15:
                self.last_idle_heartbeat = now
                self._send_command(0.0, 0.0)

        # 2. Coba hubungkan kembali jika port putus
        if self._serial is None or not self._serial.is_open:
            return

        # 3. Baca data serial secara NON-BLOCKING (menggunakan buffer stream)
        try:
            waiting = self._serial.in_waiting
            if waiting > 0:
                chunk = self._serial.read(waiting)
                self._rx_buffer += chunk

                while b"\n" in self._rx_buffer:
                    raw_line, self._rx_buffer = self._rx_buffer.split(b"\n", 1)
                    line = raw_line.decode("ascii", errors="ignore").strip()
                    if not line:
                        continue
                    fields = line.split(",")
                    if len(fields) == 3 and fields[0] == "E":
                        try:
                            left_ticks = int(fields[1])
                            right_ticks = int(fields[2])
                            msg = Int64MultiArray()
                            msg.data = [left_ticks, right_ticks]
                            self.encoder_publisher.publish(msg)
                            self.legacy_encoder_publisher.publish(msg)
                        except ValueError:
                            pass
        except OSError:
            self.get_logger().warn("Koneksi serial terputus.")
            try:
                self._serial.close()
            except Exception:
                pass
            self._serial = None

    def destroy_node(self):
        self._send_command(0.0, 0.0)
        if self._serial and self._serial.is_open:
            self._serial.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
