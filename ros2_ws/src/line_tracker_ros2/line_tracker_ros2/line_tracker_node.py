"""Node ROS 2 line tracker.

Hanya berisi wiring: parameter -> kamera -> PID -> serial/publisher.
Logika detail ada di `camera.py`, `pid.py`, dan `serial_bridge.py`.
"""

import sys
import time

import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, Int32MultiArray

from .camera import LineDetector, close_debug_windows
from .pid import PIDController
from .serial_bridge import SerialBridge

PWM_MIN = 0
PWM_MAX = 255


def clamp_pwm(value: float) -> int:
    """Batasi nilai PWM ke rentang aman 0–255."""
    return max(PWM_MIN, min(PWM_MAX, int(round(value))))


class LineTrackerNode(Node):
    """Mengikuti garis: deteksi -> PID -> kirim PWM ke firmware + topik ROS 2."""

    def __init__(self):
        super().__init__("line_tracker_node")

        # --- Parameter ---
        self.declare_parameter("camera_index", 2)
        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("kp", 5.0)
        self.declare_parameter("ki", 0.0)
        self.declare_parameter("kd", 0.0)
        self.declare_parameter("base_speed", 25)
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("show_debug_window", True)

        camera_index = self.get_parameter("camera_index").value
        serial_port = self.get_parameter("serial_port").value
        baud_rate = self.get_parameter("baud_rate").value
        publish_rate_hz = self.get_parameter("publish_rate_hz").value
        self.show_debug_window = self.get_parameter("show_debug_window").value

        # --- Kamera & deteksi garis ---
        self.line_detector = LineDetector(camera_index)
        if not self.line_detector.is_opened():
            self.get_logger().error("Tidak dapat membuka kamera")
            sys.exit(1)

        # --- Serial ke firmware ---
        self.serial_bridge = SerialBridge(serial_port, baud_rate)
        try:
            self.serial_bridge.connect()
            self.get_logger().info(
                f"Terhubung ke serial: {serial_port} @ {baud_rate}"
            )
        except OSError as error:
            self.get_logger().error(
                f"Gagal membuka serial port {serial_port}: {error}"
            )
            sys.exit(1)

        # --- PID ---
        self.pid = PIDController(
            kp=self.get_parameter("kp").value,
            ki=self.get_parameter("ki").value,
            kd=self.get_parameter("kd").value,
        )
        self.last_time = time.time()

        # --- Publisher ---
        self.debug_image_publisher = self.create_publisher(
            Image, "/line_tracker/debug_image", 10
        )
        self.normalized_error_publisher = self.create_publisher(
            Float64, "/line_tracker/normalized_error", 10
        )
        self.motor_pwm_publisher = self.create_publisher(
            Int32MultiArray, "/motor_pwm", 10
        )
        self.cmd_vel_publisher = self.create_publisher(Twist, "/cmd_vel", 10)

        self.bridge = CvBridge()

        # --- Timer pengganti while-loop (jangan pakai while True di ROS) ---
        self.timer = self.create_timer(1.0 / publish_rate_hz, self.timer_callback)
        self.get_logger().info("Line tracker node (PID) aktif.")

    # ------------------------------------------------------------------ #
    # Callback utama
    # ------------------------------------------------------------------ #
    def timer_callback(self):
        frame = self.line_detector.read_frame()
        if frame is None:
            self.get_logger().warn("Gagal menerima frame. Dilewati...")
            return

        detection = self.line_detector.detect(frame)
        self._publish_debug_image(detection.debug_image)

        # Tidak ada garis -> hentikan motor, jangan exit node
        if not detection.found:
            self._stop_motors()
            self._handle_debug_window(detection.debug_image)
            return

        self._publish_normalized_error(detection.normalized_error)

        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        # Parameter PID dibaca ulang tiap frame agar bisa diubah via `ros2 param set`
        self.pid.kp = self.get_parameter("kp").value
        self.pid.ki = self.get_parameter("ki").value
        self.pid.kd = self.get_parameter("kd").value
        base_speed = self.get_parameter("base_speed").value

        pid_output = self.pid.compute(detection.normalized_error, dt)

        left_pwm = clamp_pwm(base_speed + pid_output)
        right_pwm = clamp_pwm(base_speed - pid_output)

        # 1. Topik PWM roda [kiri, kanan]
        pwm_message = Int32MultiArray()
        pwm_message.data = [left_pwm, right_pwm]
        self.motor_pwm_publisher.publish(pwm_message)

        # 2. Topik kecepatan standar ROS 2
        twist_message = Twist()
        twist_message.linear.x = float(base_speed) / float(PWM_MAX)
        twist_message.angular.z = -float(pid_output) / 100.0
        self.cmd_vel_publisher.publish(twist_message)

        # 3. Kirim ke firmware: "<left_pwm>,<right_pwm>\n"
        try:
            self.serial_bridge.send_pwm(left_pwm, right_pwm)
        except OSError as error:
            self.get_logger().warn(f"Gagal mengirim data serial: {error}")

        self._handle_debug_window(detection.debug_image)

    # ------------------------------------------------------------------ #
    # Helper
    # ------------------------------------------------------------------ #
    def _publish_debug_image(self, debug_image):
        try:
            message = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
            self.debug_image_publisher.publish(message)
        except Exception as error:  # noqa: BLE001 - jangan matikan node karena gambar
            self.get_logger().error(f"Gagal publish debug image: {error}")

    def _publish_normalized_error(self, normalized_error: float):
        message = Float64()
        message.data = float(normalized_error)
        self.normalized_error_publisher.publish(message)

    def _stop_motors(self):
        """Hentikan motor (topik + firmware)."""
        pwm_message = Int32MultiArray()
        pwm_message.data = [0, 0]
        self.motor_pwm_publisher.publish(pwm_message)
        self.cmd_vel_publisher.publish(Twist())
        try:
            self.serial_bridge.send_pwm(0, 0)
        except OSError:
            pass

    def _handle_debug_window(self, debug_image):
        if not self.show_debug_window:
            return
        key = self.line_detector.show_debug(debug_image)
        if key == ord("q"):
            self.get_logger().info("Tombol 'q' ditekan. Mematikan node...")
            self._stop_motors()
            rclpy.shutdown()

    def destroy_node(self):
        self.get_logger().info("Melepas kamera & serial...")
        self._stop_motors()
        self.line_detector.release()
        self.serial_bridge.close()
        close_debug_windows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LineTrackerNode()
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
