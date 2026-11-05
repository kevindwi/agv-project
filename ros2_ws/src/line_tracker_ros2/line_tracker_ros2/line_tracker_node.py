import math
import sys
import time

import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int64MultiArray, String

from .camera import LineDetector
from .pid import PIDController
from .serial_bridge import SerialBridge


class LineTrackerNode(Node):
    def __init__(self):
        super().__init__("line_tracker_node")

        # Parameter
        self.declare_parameter("camera_index", 3)

        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("baud_rate", 115200)

        self.declare_parameter("kp", 1.0)
        self.declare_parameter("ki", 0.0)
        self.declare_parameter("kd", 0.5)
        self.declare_parameter("wheel_separation_m", 0.48)
        self.declare_parameter("turn_angle_rad", 1.50)
        self.declare_parameter("turn_speed_radps", 0.45)
        self.declare_parameter("turn_timeout_sec", 12.0)
        self.declare_parameter("reacquire_frames", 4)
        self.declare_parameter("reacquire_error_threshold", 0.25)
        self.declare_parameter("reacquire_search_angle_rad", 0.55)
        self.declare_parameter("slow_speed_mps", 0.10)

        self.linear_speed = 0.10  # m/s
        self.slow_speed = max(0.0, float(self.get_parameter("slow_speed_mps").value))
        self.max_angular_speed = 1.0  # rad/s
        self.max_wheel_speed = 0.30  # m/s
        self.publish_rate_hz = 30.0

        self.wheel_separation = float(self.get_parameter("wheel_separation_m").value)

        camera_index = self.get_parameter("camera_index").value
        serial_port = self.get_parameter("serial_port").value
        baud_rate = self.get_parameter("baud_rate").value

        # Kamera & deteksi garis
        self.line_detector = LineDetector(camera_index or 0)
        if not self.line_detector.is_opened():
            self.get_logger().error("Tidak dapat membuka kamera")
            sys.exit(1)

        # Serial ke firmware
        self.serial_bridge = SerialBridge(
            serial_port or "/dev/ttyUSB0", baud_rate or 115200
        )
        try:
            self.serial_bridge.connect()
            self.get_logger().info(f"Terhubung ke serial: {serial_port} @ {baud_rate}")
        except OSError as error:
            self.get_logger().error(f"Gagal membuka serial port {serial_port}: {error}")
            # sys.exit(1)

        # PID
        self.pid = PIDController(
            kp=self.get_parameter("kp").value or 0.0,
            ki=self.get_parameter("ki").value or 0.0,
            kd=self.get_parameter("kd").value or 0.0,
        )
        self.last_time = time.time()
        self.control_mode = "FOLLOW"
        self.turn_direction = 0
        self.turn_start_yaw = None
        self.turn_started_at = None
        self.current_yaw = 0.0
        self.reacquire_count = 0
        self.seek_start_yaw = None
        self.turn_angle = float(self.get_parameter("turn_angle_rad").value)
        self.turn_speed = abs(float(self.get_parameter("turn_speed_radps").value))
        self.turn_timeout = max(
            1.0, float(self.get_parameter("turn_timeout_sec").value)
        )
        self.reacquire_frames = max(
            1, int(self.get_parameter("reacquire_frames").value)
        )
        self.reacquire_error_threshold = float(
            self.get_parameter("reacquire_error_threshold").value
        )
        self.reacquire_search_angle = abs(
            float(self.get_parameter("reacquire_search_angle_rad").value)
        )

        # Publisher
        self.bridge = CvBridge()

        self.debug_image_publisher = self.create_publisher(
            Image, "/line_tracker/debug_image", 10
        )
        self.cmd_vel_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.encoder_delta_publisher = self.create_publisher(
            Int64MultiArray, "/line_tracker/encoder_delta", 10
        )
        self.image_publisher = self.create_publisher(Image, "/camera/image_raw", 5)
        self.status_publisher = self.create_publisher(
            String, "/line_tracker/status", 10
        )
        self.create_subscription(String, "/mission/control", self.control_callback, 10)
        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)

        # Timer
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.timer_callback)
        self.get_logger().info("Line tracker node (PID) aktif.")

    def timer_callback(self):
        # Baca telemetri setiap callback, termasuk saat garis tidak ditemukan.
        delta = self.serial_bridge.read_encoder_delta()
        if delta is not None:
            encoder_message = Int64MultiArray()
            encoder_message.data = [int(delta[0]), int(delta[1])]
            self.encoder_delta_publisher.publish(encoder_message)

        # ------ Line Detector ------
        frame = self.line_detector.read_frame()
        if frame is None:
            self.get_logger().warn("Gagal menerima frame. Dilewati...")
            return

        detection = self.line_detector.detect(frame)
        self._publish_debug_image(detection.debug_image)
        try:
            self.image_publisher.publish(
                self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            )
        except Exception as error:  # noqa: BLE001
            self.get_logger().warning(f"Gagal publish frame kamera: {error}")

        if self.control_mode == "STOP":
            self._stop_motors()
            self._publish_status("STOPPED")
            return
        if self.control_mode == "TURN":
            self._run_turn()
            return
        if self.control_mode == "SEEK_LINE":
            self._seek_line(detection)
            return

        # Tidak ada garis -> hentikan motor, jangan exit node
        if not detection.found:
            self._stop_motors()
            self._publish_status("LINE_LOST")
            return

        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        linear_speed = (
            self.slow_speed if self.control_mode == "SLOW" else float(self.linear_speed)
        )
        wheel_separation = float(self.wheel_separation)
        max_angular_speed = abs(float(self.max_angular_speed))

        angular_speed = self.pid.compute(detection.normalized_error, dt)
        angular_speed = max(-max_angular_speed, min(max_angular_speed, angular_speed))

        # Kinematika balik DDMR: (v, omega) -> kecepatan roda kiri/kanan.
        left_speed = linear_speed - (angular_speed * wheel_separation / 2.0)
        right_speed = linear_speed + (angular_speed * wheel_separation / 2.0)

        self.get_logger().debug(
            f"v={linear_speed:.3f} m/s, omega={angular_speed:.3f} rad/s; "
            f"vL={left_speed:.3f}, vR={right_speed:.3f} m/s; "
        )

        # Topik kecepatan standar ROS 2
        twist_message = Twist()
        twist_message.linear.x = linear_speed
        twist_message.angular.z = angular_speed
        self._publish_status("FOLLOWING")

        self._send_twist(twist_message)

    # ------------------------------------------------------------------ #
    # Helper
    # ------------------------------------------------------------------ #
    def control_callback(self, message: String):
        """Accept mission modes; this node remains the sole serial motor writer."""
        command = message.data.strip().upper()
        if command in ("FOLLOW", "SLOW", "STOP"):
            self.control_mode = command
            if command in ("FOLLOW", "SLOW"):
                self.pid.reset()
        elif command in ("LEFT", "RIGHT"):
            self.turn_direction = 1 if command == "LEFT" else -1
            self.turn_start_yaw = self.current_yaw
            self.turn_started_at = time.monotonic()
            self.reacquire_count = 0
            self.control_mode = "TURN"
        else:
            self.get_logger().warning(f"Perintah misi tidak dikenal: {message.data}")

    def odom_callback(self, message: Odometry):
        q = message.pose.pose.orientation
        self.current_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def _run_turn(self):
        if self.turn_start_yaw is None:
            self.turn_start_yaw = self.current_yaw
            self.turn_started_at = time.monotonic()
        if time.monotonic() - self.turn_started_at > self.turn_timeout:
            self.control_mode = "STOP"
            self._stop_motors()
            self._publish_status("TURN_FAILED")
            return
        turned = self._angle_distance(self.current_yaw, self.turn_start_yaw)
        if abs(turned) >= self.turn_angle:
            self.control_mode = "SEEK_LINE"
            self.seek_start_yaw = self.current_yaw
            self._stop_motors()
            self._publish_status("SEEKING_LINE")
            return
        twist = Twist()
        twist.angular.z = self.turn_direction * self.turn_speed
        self._send_twist(twist)
        self._publish_status(
            "TURNING_LEFT" if self.turn_direction > 0 else "TURNING_RIGHT"
        )

    def _seek_line(self, detection):
        if (
            detection.found
            and abs(detection.normalized_error) <= self.reacquire_error_threshold
        ):
            self.reacquire_count += 1
        else:
            self.reacquire_count = 0
        if self.reacquire_count >= self.reacquire_frames:
            self.control_mode = "FOLLOW"
            self.turn_start_yaw = None
            self.pid.reset()
            self._stop_motors()
            self._publish_status("TURN_COMPLETE")
            return
        searched = self._angle_distance(self.current_yaw, self.seek_start_yaw)
        if abs(searched) >= self.reacquire_search_angle:
            self.control_mode = "STOP"
            self._stop_motors()
            self._publish_status("TURN_FAILED")
            return
        twist = Twist()
        twist.angular.z = self.turn_direction * self.turn_speed * 0.45
        self._send_twist(twist)
        self._publish_status("SEEKING_LINE")

    def _send_twist(self, twist):
        self.cmd_vel_publisher.publish(twist)
        try:
            self.serial_bridge.send_cmd_vel(twist.linear.x, twist.angular.z)
        except OSError as error:
            self.get_logger().warning(f"Gagal mengirim perintah serial: {error}")

    def _publish_status(self, status):
        self.status_publisher.publish(String(data=status))

    @staticmethod
    def _angle_distance(angle, reference):
        return math.atan2(math.sin(angle - reference), math.cos(angle - reference))

    def _publish_debug_image(self, image):
        """Publish frame dengan overlay ROI dan posisi garis."""
        try:
            message = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
            self.debug_image_publisher.publish(message)
        except Exception as error:  # noqa: BLE001 - kegagalan visualisasi tak hentikan kontrol
            self.get_logger().error(f"Gagal publish debug image: {error}")

    def _stop_motors(self):
        """Hentikan motor (topik + firmware)."""
        self.cmd_vel_publisher.publish(Twist())
        # self.wheel_speed_publisher.publish(Float64MultiArray())
        try:
            self.serial_bridge.send_cmd_vel(0.0, 0.0)
        except OSError:
            pass

    def destroy_node(self):
        self.get_logger().info("Melepas kamera & serial...")
        self._stop_motors()
        self.line_detector.release()
        self.serial_bridge.close()
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
