"""Node pelacak garis + manuver belok: kamera -> PID -> publish /cmd_vel."""

import math
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String

from .pid import PIDController

IMAGE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


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
        self.declare_parameter("processing_fps", 15.0)
        self.declare_parameter("debug_publish_rate_hz", 5.0)

        # Parameter manuver belok (Fase 4)
        self.declare_parameter("turn_angle_rad", 1.57)
        self.declare_parameter("turn_speed_radps", 0.40)
        self.declare_parameter("turn_timeout_sec", 12.0)
        self.declare_parameter("reacquire_frames", 4)
        self.declare_parameter("reacquire_error_threshold", 0.25)
        self.declare_parameter("reacquire_search_angle_rad", 0.55)
        self.declare_parameter("status_topic", "/line_follower/state")

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
        self.processing_fps = max(
            1.0, float(self.get_parameter("processing_fps").value)
        )
        self.debug_publish_rate = max(
            0.0, float(self.get_parameter("debug_publish_rate_hz").value)
        )

        self.turn_angle = abs(float(self.get_parameter("turn_angle_rad").value))
        self.turn_speed = max(
            0.05, abs(float(self.get_parameter("turn_speed_radps").value))
        )
        if self.turn_angle > math.pi:
            self.get_logger().warning(
                "turn_angle_rad > 180 derajat tidak didukung oleh pengukuran yaw absolut; "
                "gunakan sudut <= 3.14 rad."
            )
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
        status_topic = str(self.get_parameter("status_topic").value)

        self.pid = PIDController(self.kp, self.ki, self.kd)
        self.bridge = CvBridge()
        self.last_time = time.monotonic()
        self.last_process_time = 0.0
        cv2.setNumThreads(1)
        # FOLLOW, SLOW, STOP, TURN, SEEK_LINE
        self.mission_state = "FOLLOW"
        self.current_linear_speed = 0.0  # Kecepatan aktual bertahap (ramping)
        self.lost_frames = 0
        self.max_lost_frames = 8  # Toleransi flicker ~0.25 detik sebelum berhenti
        self.last_twist = Twist()

        # State manuver belok
        self.current_yaw = 0.0
        self.odom_received = False
        self.turn_direction = 0
        self.turn_start_yaw = None
        self.turn_started_at = None
        self.seek_start_yaw = None
        self.seek_started_at = None
        self.reacquire_count = 0
        self.last_status = ""
        self.last_status_time = 0.0
        self.last_debug_publish_time = 0.0

        # Publisher & Subscriber
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.debug_pub = self.create_publisher(Image, "/line_tracker/debug_image", 1)
        self.error_pub = self.create_publisher(Float32, "/line_follower/error", 10)
        self.status_pub = self.create_publisher(String, status_topic, 10)

        self.create_subscription(
            Image, "/camera/image_raw", self.image_callback, IMAGE_QOS
        )
        self.create_subscription(String, "/mission/control", self.mission_callback, 10)
        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.add_on_set_parameters_callback(self.on_set_parameters)

        self.get_logger().info(
            "LineFollowerNode aktif (vision + PID + manuver belok odometri)."
        )

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
                self.get_logger().info(
                    f"[TUNING] Speed diperbarui -> {self.linear_speed}"
                )
        return SetParametersResult(successful=True)

    def mission_callback(self, msg: String):
        command = msg.data.strip().upper()
        if command in ("FOLLOW", "SLOW", "STOP"):
            self.mission_state = command
            if command in ("FOLLOW", "SLOW"):
                self.pid.reset()
            if command == "STOP":
                self.current_linear_speed = 0.0
                self.turn_start_yaw = None
                self.seek_start_yaw = None
            return

        if command in ("LEFT", "RIGHT", "TURN_LEFT", "TURN_RIGHT"):
            self.turn_direction = 1 if command in ("LEFT", "TURN_LEFT") else -1
            self.turn_start_yaw = self.current_yaw
            self.turn_started_at = time.monotonic()
            self.seek_start_yaw = None
            self.seek_started_at = None
            self.reacquire_count = 0
            self.current_linear_speed = 0.0
            self.pid.reset()
            self.mission_state = "TURN"
            self.get_logger().info(
                f"Manuver belok {'kiri' if self.turn_direction > 0 else 'kanan'} dimulai."
            )
            self._publish_status(
                "TURNING_LEFT" if self.turn_direction > 0 else "TURNING_RIGHT",
                force=True,
            )
            return

        self.get_logger().warning(f"Perintah misi tidak dikenal: {msg.data}")

    def odom_callback(self, msg: Odometry):
        q = msg.pose.pose.orientation
        self.current_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self.odom_received = True

    def image_callback(self, msg: Image):
        now = time.monotonic()
        if now - self.last_process_time < 1.0 / self.processing_fps:
            return
        self.last_process_time = now
        dt = max(0.001, min(0.1, now - self.last_time))
        self.last_time = now

        # Hitung target kecepatan berdasarkan status misi
        if self.mission_state in ("STOP", "TURN", "SEEK_LINE"):
            target_speed = 0.0
        elif self.mission_state == "SLOW":
            target_speed = self.slow_speed
        else:
            target_speed = self.linear_speed

        # Ramp kecepatan linier (Slew rate limiter untuk akselerasi & deselerasi halus)
        max_delta_v = self.max_acceleration * dt
        if self.current_linear_speed < target_speed:
            self.current_linear_speed = min(
                target_speed, self.current_linear_speed + max_delta_v
            )
        elif self.current_linear_speed > target_speed:
            self.current_linear_speed = max(
                target_speed, self.current_linear_speed - max_delta_v
            )

        # Jika misi STOP dan kecepatan sudah melandai ke nol, berhenti total
        if self.mission_state == "STOP" and self.current_linear_speed <= 0.005:
            self.current_linear_speed = 0.0
            self.cmd_vel_pub.publish(Twist())
            self.pid.reset()
            self._publish_status("STOPPED")
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
        line_found = moments["m00"] > 100
        normalized_error = 0.0
        cx_global = None
        cy_global = int((r_top + r_bottom) / 2)
        roi_width = r_right - r_left

        if line_found:
            cx_local = int(moments["m10"] / moments["m00"])
            cx_global = r_left + cx_local
            # Error ternormalisasi: -1.0 (kiri) s/d +1.0 (kanan)
            normalized_error = (cx_local - (roi_width / 2.0)) / (roi_width / 2.0)

        debug_due = (
            self.debug_pub.get_subscription_count() > 0
            and self.debug_publish_rate > 0.0
            and now - self.last_debug_publish_time >= 1.0 / self.debug_publish_rate
        )
        debug_img = frame.copy() if debug_due else None
        if debug_img is not None:
            cv2.rectangle(
                debug_img, (r_left, r_top), (r_right, r_bottom), (0, 255, 0), 2
            )

        # Mode manuver belok: kendalikan rotasi di tempat lebih dulu
        if self.mission_state == "TURN":
            self._run_turn()
            self._publish_debug(debug_img)
            return

        # Mode mencari garis cabang setelah sudut target tercapai
        if self.mission_state == "SEEK_LINE":
            self._seek_line(line_found, normalized_error)
            self._publish_debug(debug_img)
            return

        if line_found:
            self.lost_frames = 0
            self.error_pub.publish(Float32(data=float(normalized_error)))

            angular_z = self.pid.compute(normalized_error, dt)
            angular_z = max(
                -self.max_angular_speed, min(self.max_angular_speed, angular_z)
            )

            twist = Twist()
            twist.linear.x = self.current_linear_speed
            twist.angular.z = angular_z
            self.last_twist = twist
            self.cmd_vel_pub.publish(twist)

            # Gambar titik centroid pada debug image
            if debug_img is not None and cx_global is not None:
                cv2.circle(debug_img, (cx_global, cy_global), 6, (0, 0, 255), -1)
            if debug_img is not None:
                cv2.putText(
                    debug_img,
                    f"Err: {normalized_error:.2f} | Speed: {self.current_linear_speed:.2f}",
                    (r_left, r_top - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2,
                )
            self._publish_status(
                "SLOWING" if self.mission_state == "SLOW" else "FOLLOWING"
            )
        else:
            self.lost_frames += 1
            if self.lost_frames <= self.max_lost_frames:
                # Glitch filter: jika baru hilang 1-8 frame, pertahankan kecepatan ramping saat ini
                twist = Twist()
                twist.linear.x = self.current_linear_speed
                twist.angular.z = self.last_twist.angular.z
                self.cmd_vel_pub.publish(twist)
                if debug_img is not None:
                    cv2.putText(
                        debug_img,
                        f"COASTING ({self.lost_frames}/{self.max_lost_frames})",
                        (r_left, r_top - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 255),
                        2,
                    )
                self._publish_status("COASTING")
            else:
                # Benar-benar keluar dari garis > 0.25 detik -> rem aman
                self.current_linear_speed = 0.0
                self.last_twist = Twist()
                self.cmd_vel_pub.publish(Twist())
                self.pid.reset()
                if debug_img is not None:
                    cv2.putText(
                        debug_img,
                        "LINE LOST",
                        (r_left, r_top - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 0, 255),
                        2,
                    )
                self._publish_status("LINE_LOST")

        self._publish_debug(debug_img)

    def _run_turn(self):
        """Putar robot di tempat hingga sudut target tercapai."""
        now = time.monotonic()
        if self.turn_started_at is None:
            self.turn_started_at = now
        if self.turn_start_yaw is None:
            self.turn_start_yaw = self.current_yaw

        if now - self.turn_started_at > self.turn_timeout:
            self._fail_turn("Waktu putar habis sebelum sudut target tercapai")
            return

        if self.odom_received:
            turned = abs(self._angle_distance(self.current_yaw, self.turn_start_yaw))
        else:
            # Fallback berbasis waktu bila /odom belum tersedia
            turned = (now - self.turn_started_at) * self.turn_speed

        if turned >= self.turn_angle:
            self.mission_state = "SEEK_LINE"
            self.seek_start_yaw = self.current_yaw
            self.seek_started_at = now
            self.reacquire_count = 0
            self.cmd_vel_pub.publish(Twist())
            self._publish_status("SEEKING_LINE", force=True)
            return

        twist = Twist()
        twist.angular.z = self.turn_direction * self.turn_speed
        self.cmd_vel_pub.publish(twist)
        self._publish_status(
            "TURNING_LEFT" if self.turn_direction > 0 else "TURNING_RIGHT"
        )

    def _seek_line(self, line_found: bool, normalized_error: float):
        """Cari garis cabang baru setelah putaran, lalu kembali ke FOLLOW."""
        now = time.monotonic()
        if line_found and abs(normalized_error) <= self.reacquire_error_threshold:
            self.reacquire_count += 1
        else:
            self.reacquire_count = 0

        if self.reacquire_count >= self.reacquire_frames:
            self.mission_state = "FOLLOW"
            self.turn_start_yaw = None
            self.seek_start_yaw = None
            self.reacquire_count = 0
            self.current_linear_speed = 0.0
            self.pid.reset()
            self.cmd_vel_pub.publish(Twist())
            self._publish_status("TURN_COMPLETE", force=True)
            return

        if self.seek_started_at is None:
            self.seek_started_at = now

        if self.odom_received:
            searched = abs(self._angle_distance(self.current_yaw, self.seek_start_yaw))
        else:
            searched = (now - self.seek_started_at) * self.turn_speed * 0.45

        if searched >= self.reacquire_search_angle:
            self._fail_turn("Garis cabang tidak ditemukan setelah belok")
            return

        twist = Twist()
        twist.angular.z = self.turn_direction * self.turn_speed * 0.45
        self.cmd_vel_pub.publish(twist)
        self._publish_status("SEEKING_LINE")

    def _fail_turn(self, reason: str):
        self.get_logger().error(reason)
        self.mission_state = "STOP"
        self.current_linear_speed = 0.0
        self.turn_start_yaw = None
        self.seek_start_yaw = None
        self.cmd_vel_pub.publish(Twist())
        self._publish_status("TURN_FAILED", force=True)

    def _publish_debug(self, debug_img):
        if debug_img is None or self.debug_pub.get_subscription_count() == 0:
            return
        now = time.monotonic()
        if (
            self.debug_publish_rate <= 0.0
            or now - self.last_debug_publish_time < 1.0 / self.debug_publish_rate
        ):
            return
        try:
            self.debug_pub.publish(
                self.bridge.cv2_to_imgmsg(debug_img, encoding="bgr8")
            )
            self.last_debug_publish_time = now
        except Exception:
            pass

    def _publish_status(self, status: str, force: bool = False):
        now = time.monotonic()
        # Deduplikasi + heartbeat 0.5 s agar status tidak hilang bila subscriber telat
        if force or status != self.last_status or now - self.last_status_time >= 0.5:
            self.status_pub.publish(String(data=status))
            self.last_status = status
            self.last_status_time = now

    @staticmethod
    def _angle_distance(angle: float, reference: float) -> float:
        return math.atan2(math.sin(angle - reference), math.cos(angle - reference))

    def destroy_node(self):
        self.cmd_vel_pub.publish(Twist())
        self.status_pub.publish(String(data="STOPPED"))
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
