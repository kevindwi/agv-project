import sys
import time

import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int64MultiArray

from .camera import LineDetector
from .pid import PIDController
from .serial_bridge import SerialBridge


class LineTrackerNode(Node):
    def __init__(self):
        super().__init__("line_tracker_node")

        # Parameter ROS
        self.declare_parameter("camera_index", 3)
        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("baud_rate", 115200)

        self.declare_parameter("kp", 1.0)
        self.declare_parameter("ki", 0.0)
        self.declare_parameter("kd", 0.0)
        self.declare_parameter("wheel_separation_m", 0.48)

        # Kecepatan & Pengaturan Gerak
        self.linear_speed = 0.30  # m/s
        self.max_angular_speed = 1.0  # rad/s
        self.publish_rate_hz = 30.0

        # State Machine
        self.turn_lock_duration = 0.6  # detik
        self.last_fork_time = 0.0
        self.last_known_error = 0.0
        self.last_seen_time = time.time()  # Penanda waktu saat garis terakhir kali terlihat

        self.wheel_separation = float(self.get_parameter("wheel_separation_m").value)

        camera_index = self.get_parameter("camera_index").value
        serial_port = self.get_parameter("serial_port").value
        baud_rate = self.get_parameter("baud_rate").value

        self.line_detector = LineDetector(camera_index or 0)
        if not self.line_detector.is_opened():
            self.get_logger().error("Tidak dapat membuka kamera")
            sys.exit(1)

        self.serial_bridge = SerialBridge(serial_port or "/dev/ttyUSB0", baud_rate or 115200)
        try:
            self.serial_bridge.connect()
            self.get_logger().info(f"Terhubung ke serial: {serial_port} @ {baud_rate}")
        except OSError as error:
            self.get_logger().error(f"Gagal membuka serial port {serial_port}: {error}")

        self.pid = PIDController(
            kp=self.get_parameter("kp").value or 0.0,
            ki=self.get_parameter("ki").value or 0.0,
            kd=self.get_parameter("kd").value or 0.0,
        )
        self.last_time = time.time()

        self.bridge = CvBridge()
        self.debug_image_publisher = self.create_publisher(Image, "/line_tracker/debug_image", 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.encoder_delta_publisher = self.create_publisher(Int64MultiArray, "/line_tracker/encoder_delta", 10)

        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.timer_callback)
        self.get_logger().info("Line tracker node aktif.")

    def timer_callback(self):
        delta = self.serial_bridge.read_encoder_delta()
        if delta is not None:
            encoder_message = Int64MultiArray()
            encoder_message.data = [int(delta[0]), int(delta[1])]
            self.encoder_delta_publisher.publish(encoder_message)

        frame = self.line_detector.read_frame()
        if frame is None:
            return

        detection = self.line_detector.detect(frame)
        self._publish_debug_image(detection.debug_image)

        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        # PERBAIKAN: Pemulihan saat garis hilang (Baik pertigaan maupun belokan 90° biasa)
        if not detection.found:
            # Jika garis hilang kurang dari 1 detik yang lalu dan sebelumnya ada offset error, lakukan pivot turn
            if (now - self.last_seen_time) < 1.0 and abs(self.last_known_error) > 0.3:
                twist_message = Twist()
                twist_message.linear.x = 0.0  # Putar di tempat
                direction = 1.0 if self.last_known_error >= 0 else -1.0
                twist_message.angular.z = direction * self.max_angular_speed
                
                self.cmd_vel_publisher.publish(twist_message)
                try:
                    self.serial_bridge.send_cmd_vel(twist_message.linear.x, twist_message.angular.z)
                except OSError:
                    pass
                return
            else:
                self._stop_motors()
                return

        # Simpan state garis saat masih terlihat
        self.last_known_error = detection.normalized_error
        self.last_seen_time = now

        if detection.is_fork:
            self.last_fork_time = now

        is_turning = (now - self.last_fork_time) < self.turn_lock_duration

        # Kecepatan Adaptif Dinamis
        error_scale = max(0.3, 1.0 - abs(detection.normalized_error))
        linear_speed = float(self.linear_speed) * error_scale
        
        if is_turning:
            linear_speed *= 0.4

        wheel_separation = float(self.wheel_separation)
        max_angular_speed = abs(float(self.max_angular_speed))

        angular_speed = self.pid.compute(detection.normalized_error, dt)
        angular_speed = max(-max_angular_speed, min(max_angular_speed, angular_speed))

        twist_message = Twist()
        twist_message.linear.x = linear_speed
        twist_message.angular.z = angular_speed
        self.cmd_vel_publisher.publish(twist_message)

        try:
            self.serial_bridge.send_cmd_vel(linear_speed, angular_speed)
        except OSError as error:
            self.get_logger().warn(f"Gagal mengirim data serial: {error}")

    def _publish_debug_image(self, image):
        try:
            message = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
            self.debug_image_publisher.publish(message)
        except Exception as error:
            self.get_logger().error(f"Gagal publish debug image: {error}")

    def _stop_motors(self):
        self.cmd_vel_publisher.publish(Twist())
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