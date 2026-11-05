"""Node pelacak garis murni: memproses gambar kamera -> PID -> publish /cmd_vel."""

import time
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String

from .pid import PIDController


class LineFollowerNode(Node):
    def __init__(self):
        super().__init__("line_follower_node")

        # Parameter
        self.declare_parameter("kp", 1.0)
        self.declare_parameter("ki", 0.0)
        self.declare_parameter("kd", 0.5)
        self.declare_parameter("linear_speed", 0.10)
        self.declare_parameter("slow_speed", 0.04)
        self.declare_parameter("max_angular_speed", 1.0)
        self.declare_parameter("max_acceleration", 0.08)
        self.declare_parameter("roi_top_ratio", 0.70)
        self.declare_parameter("roi_bottom_ratio", 0.90)
        self.declare_parameter("roi_left_ratio", 0.20)
        self.declare_parameter("roi_right_ratio", 0.80)

        self.kp = float(self.get_parameter("kp").value)
        self.ki = float(self.get_parameter("ki").value)
        self.kd = float(self.get_parameter("kd").value)
        self.linear_speed = float(self.get_parameter("linear_speed").value)
        self.slow_speed = float(self.get_parameter("slow_speed").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        self.max_acceleration = float(self.get_parameter("max_acceleration").value)

        self.roi_top = float(self.get_parameter("roi_top_ratio").value)
        self.roi_bottom = float(self.get_parameter("roi_bottom_ratio").value)
        self.roi_left = float(self.get_parameter("roi_left_ratio").value)
        self.roi_right = float(self.get_parameter("roi_right_ratio").value)

        self.pid = PIDController(self.kp, self.ki, self.kd)
        self.bridge = CvBridge()
        self.last_time = time.monotonic()
        self.mission_state = "FOLLOW"  # FOLLOW, SLOW, STOP
        self.current_linear_speed = 0.0  # Kecepatan aktual bertahap (ramping)
        self.lost_frames = 0
        self.max_lost_frames = 8  # Toleransi flicker ~0.25 detik sebelum berhenti
        self.last_twist = Twist()

        # Publisher & Subscriber
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.debug_pub = self.create_publisher(Image, "/line_tracker/debug_image", 5)
        self.error_pub = self.create_publisher(Float32, "/line_follower/error", 10)

        self.create_subscription(Image, "/camera/image_raw", self.image_callback, 5)
        self.create_subscription(String, "/mission/control", self.mission_callback, 10)
        self.add_on_set_parameters_callback(self.on_set_parameters)

        self.get_logger().info("LineFollowerNode aktif (murni vision & PID realtime tuning).")

    def on_set_parameters(self, params):
        for param in params:
            if param.name == "kp":
                self.pid.kp = float(param.value)
                self.get_logger().info(f"[TUNING] Kp diperbarui -> {self.pid.kp}")
            elif param.name == "ki":
                self.pid.ki = float(param.value)
                self.get_logger().info(f"[TUNING] Ki diperbarui -> {self.pid.ki}")
            elif param.name == "kd":
                self.pid.kd = float(param.value)
                self.get_logger().info(f"[TUNING] Kd diperbarui -> {self.pid.kd}")
            elif param.name == "linear_speed":
                self.linear_speed = float(param.value)
                self.get_logger().info(f"[TUNING] Speed diperbarui -> {self.linear_speed}")
        return SetParametersResult(successful=True)

    def mission_callback(self, msg: String):
        command = msg.data.strip().upper()
        if command in ("FOLLOW", "SLOW", "STOP"):
            self.mission_state = command
            if command == "FOLLOW":
                self.pid.reset()

    def image_callback(self, msg: Image):
        now = time.monotonic()
        dt = max(0.001, min(0.1, now - self.last_time))
        self.last_time = now

        # Hitung target kecepatan berdasarkan status misi
        if self.mission_state == "STOP":
            target_speed = 0.0
        elif self.mission_state == "SLOW":
            target_speed = self.slow_speed
        else:
            target_speed = self.linear_speed

        # Ramp kecepatan linier (Slew rate limiter untuk akselerasi & deselerasi halus)
        max_delta_v = self.max_acceleration * dt
        if self.current_linear_speed < target_speed:
            self.current_linear_speed = min(target_speed, self.current_linear_speed + max_delta_v)
        elif self.current_linear_speed > target_speed:
            self.current_linear_speed = max(target_speed, self.current_linear_speed - max_delta_v)

        # Jika misi STOP dan kecepatan sudah melandai ke nol, berhenti total
        if self.mission_state == "STOP" and self.current_linear_speed <= 0.005:
            self.current_linear_speed = 0.0
            self.cmd_vel_pub.publish(Twist())
            self.pid.reset()
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"Gagal decode frame: {e}")
            return

        h, w, _ = frame.shape
        r_top = int(h * self.roi_top)
        r_bottom = int(h * self.roi_bottom)
        r_left = int(w * self.roi_left)
        r_right = int(w * self.roi_right)

        roi = frame[r_top:r_bottom, r_left:r_right]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        # Deteksi garis gelap/hitam di atas permukaan terang
        _, binary = cv2.threshold(
            blurred, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU
        )

        moments = cv2.moments(binary)
        debug_img = frame.copy()
        cv2.rectangle(debug_img, (r_left, r_top), (r_right, r_bottom), (0, 255, 0), 2)

        if moments["m00"] > 100:
            self.lost_frames = 0
            cx_local = int(moments["m10"] / moments["m00"])
            cx_global = r_left + cx_local
            cy_global = int((r_top + r_bottom) / 2)

            # Error ternormalisasi: -1.0 (kiri) s/d +1.0 (kanan)
            roi_width = r_right - r_left
            normalized_error = (cx_local - (roi_width / 2.0)) / (roi_width / 2.0)
            self.error_pub.publish(Float32(data=float(normalized_error)))

            angular_z = self.pid.compute(normalized_error, dt)
            angular_z = max(-self.max_angular_speed, min(self.max_angular_speed, angular_z))

            twist = Twist()
            twist.linear.x = self.current_linear_speed
            twist.angular.z = angular_z
            self.last_twist = twist
            self.cmd_vel_pub.publish(twist)

            # Gambar titik centroid pada debug image
            cv2.circle(debug_img, (cx_global, cy_global), 6, (0, 0, 255), -1)
            cv2.putText(
                debug_img,
                f"Err: {normalized_error:.2f} | Speed: {self.current_linear_speed:.2f}",
                (r_left, r_top - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2,
            )
        else:
            self.lost_frames += 1
            if self.lost_frames <= self.max_lost_frames:
                # Glitch filter: jika baru hilang 1-8 frame, pertahankan kecepatan ramping saat ini
                twist = Twist()
                twist.linear.x = self.current_linear_speed
                twist.angular.z = self.last_twist.angular.z
                self.cmd_vel_pub.publish(twist)
                cv2.putText(
                    debug_img,
                    f"COASTING ({self.lost_frames}/{self.max_lost_frames})",
                    (r_left, r_top - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 255),
                    2,
                )
            else:
                # Benar-benar keluar dari garis > 0.25 detik -> rem aman
                self.current_linear_speed = 0.0
                self.last_twist = Twist()
                self.cmd_vel_pub.publish(Twist())
                self.pid.reset()
                cv2.putText(
                    debug_img,
                    "LINE LOST",
                    (r_left, r_top - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 0, 255),
                    2,
                )
            cv2.putText(
                debug_img,
                "LINE LOST",
                (r_left, r_top - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 0, 255),
                2,
            )

        try:
            self.debug_pub.publish(
                self.bridge.cv2_to_imgmsg(debug_img, encoding="bgr8")
            )
        except Exception:
            pass

    def destroy_node(self):
        self.cmd_vel_pub.publish(Twist())
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerNode()
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
