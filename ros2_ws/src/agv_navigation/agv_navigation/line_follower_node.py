"""Node pelacak garis + manuver belok: kamera -> PID -> publish /cmd_vel."""

import math
import time

import cv2
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

from .line_detector import LineDetector
from .pid import PIDController
from .turn_maneuver import TurnManeuverController

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
        self.declare_parameter("turn_angle_rad", 1.35)
        self.declare_parameter("turn_speed_radps", 0.35)
        self.declare_parameter("min_turn_speed_radps", 0.12)
        self.declare_parameter("yaw_kp", 1.2)
        self.declare_parameter("seek_kp", 1.2)
        self.declare_parameter("turn_timeout_sec", 12.0)
        self.declare_parameter("reacquire_frames", 3)
        self.declare_parameter("reacquire_error_threshold", 0.15)
        self.declare_parameter("reacquire_search_angle_rad", 0.60)
        self.declare_parameter("status_topic", "/line_follower/state")

        self.linear_speed = float(self.get_parameter("linear_speed").value)
        self.slow_speed = float(self.get_parameter("slow_speed").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        self.max_acceleration = float(self.get_parameter("max_acceleration").value)
        self.processing_fps = max(
            1.0, float(self.get_parameter("processing_fps").value)
        )
        self.debug_publish_rate = max(
            0.0, float(self.get_parameter("debug_publish_rate_hz").value)
        )
        status_topic = str(self.get_parameter("status_topic").value)

        # Sub-komponen modular
        self.pid = PIDController(
            float(self.get_parameter("kp").value),
            float(self.get_parameter("ki").value),
            float(self.get_parameter("kd").value),
        )
        self.detector = LineDetector(
            float(self.get_parameter("roi_top_ratio").value),
            float(self.get_parameter("roi_bottom_ratio").value),
            float(self.get_parameter("roi_left_ratio").value),
            float(self.get_parameter("roi_right_ratio").value),
        )
        self.turn_ctrl = TurnManeuverController(
            turn_angle_rad=float(self.get_parameter("turn_angle_rad").value),
            turn_speed_radps=float(self.get_parameter("turn_speed_radps").value),
            min_turn_speed_radps=float(
                self.get_parameter("min_turn_speed_radps").value
            ),
            yaw_kp=float(self.get_parameter("yaw_kp").value),
            seek_kp=float(self.get_parameter("seek_kp").value),
            turn_timeout_sec=float(self.get_parameter("turn_timeout_sec").value),
            reacquire_frames=int(self.get_parameter("reacquire_frames").value),
            reacquire_error_threshold=float(
                self.get_parameter("reacquire_error_threshold").value
            ),
            reacquire_search_angle_rad=float(
                self.get_parameter("reacquire_search_angle_rad").value
            ),
        )

        self.bridge = CvBridge()
        self.last_time = time.monotonic()
        self.last_process_time = 0.0
        cv2.setNumThreads(1)

        # State navigasi
        self.mission_state = "FOLLOW"  # FOLLOW, SLOW, STOP, TURN, SEEK_LINE
        self.current_linear_speed = 0.0
        self.lost_frames = 0
        self.max_lost_frames = 8
        self.last_twist = Twist()

        # State odometri & status
        self.current_yaw = 0.0
        self.odom_received = False
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
            "LineFollowerNode aktif (vision + PID + manuver belok odometri modular)."
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
            prev_state = self.mission_state
            self.mission_state = command
            if command in ("FOLLOW", "SLOW") and prev_state not in ("FOLLOW", "SLOW"):
                self.pid.reset()
            if command == "STOP":
                self.current_linear_speed = 0.0
                self.turn_ctrl.reset()
            return

        if command in ("LEFT", "RIGHT", "TURN_LEFT", "TURN_RIGHT"):
            direction = 1 if command in ("LEFT", "TURN_LEFT") else -1
            self.turn_ctrl.start_turn(direction, self.current_yaw)
            self.current_linear_speed = 0.0
            self.pid.reset()
            self.mission_state = "TURN"
            self.get_logger().info(
                f"Manuver belok {'kiri' if direction > 0 else 'kanan'} dimulai."
            )
            self._publish_status(
                "TURNING_LEFT" if direction > 0 else "TURNING_RIGHT", force=True
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

        # Update kecepatan linier dengan slew rate limiter
        self._update_linear_speed(dt)
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

        create_debug = (
            self.debug_pub.get_subscription_count() > 0
            and self.debug_publish_rate > 0.0
            and now - self.last_debug_publish_time >= 1.0 / self.debug_publish_rate
        )

        # 1. Deteksi garis via modul LineDetector (perlebar ROI hanya saat SEEK_LINE)
        res = self.detector.detect(
            frame,
            create_debug=create_debug,
            status_text="LINE LOST"
            if self.lost_frames > self.max_lost_frames
            else f"COASTING ({self.lost_frames}/{self.max_lost_frames})"
            if self.lost_frames > 0
            else "",
            linear_speed=self.current_linear_speed,
            wide_roi=(self.mission_state == "SEEK_LINE"),
        )

        # 2. Mode belok di tempat
        if self.mission_state == "TURN":
            event, angular_z = self.turn_ctrl.update_turn(
                self.current_yaw, self.odom_received, line_found=res.line_found
            )
            if event == "TIMEOUT":
                self._fail_turn("Waktu putar habis sebelum sudut target tercapai")
            elif event == "REACHED_TARGET":
                self.mission_state = "SEEK_LINE"
                self.cmd_vel_pub.publish(Twist())
                self._publish_status("SEEKING_LINE", force=True)
            else:
                twist = Twist()
                twist.angular.z = angular_z
                self.cmd_vel_pub.publish(twist)
                self._publish_status(
                    "TURNING_LEFT"
                    if self.turn_ctrl.turn_direction > 0
                    else "TURNING_RIGHT"
                )
            if create_debug:
                debug_image = (
                    res.debug_image if res.debug_image is not None else frame
                )
                self._publish_debug(debug_image)
            return

        # 3. Mode pencarian garis setelah belok (LCTT)
        if self.mission_state == "SEEK_LINE":
            event, angular_z = self.turn_ctrl.update_seek(
                self.current_yaw,
                self.odom_received,
                res.line_found,
                res.normalized_error,
                max_angular_speed=self.max_angular_speed,
            )
            if event == "FAILED":
                self._fail_turn("Garis cabang tidak ditemukan setelah belok")
            elif event == "COMPLETE":
                self.mission_state = "FOLLOW"
                self.current_linear_speed = 0.0
                self.pid.reset()
                self.cmd_vel_pub.publish(Twist())
                self._publish_status("TURN_COMPLETE", force=True)
            else:
                twist = Twist()
                twist.angular.z = angular_z
                self.cmd_vel_pub.publish(twist)
                self._publish_status("SEEKING_LINE")
            if create_debug:
                # NumPy arrays tidak dapat dipakai sebagai boolean; pilih eksplisit.
                debug_image = (
                    res.debug_image if res.debug_image is not None else frame
                )
                self._publish_debug(debug_image)
            return

        # 4. Mode Line Following normal
        if res.line_found:
            self.lost_frames = 0
            self.error_pub.publish(Float32(data=float(res.normalized_error)))

            angular_z = -self.pid.compute(res.normalized_error, dt)
            angular_z = max(
                -self.max_angular_speed, min(self.max_angular_speed, angular_z)
            )

            twist = Twist()
            twist.linear.x = self.current_linear_speed
            twist.angular.z = angular_z
            self.last_twist = twist
            self.cmd_vel_pub.publish(twist)
            self._publish_status(
                "SLOWING" if self.mission_state == "SLOW" else "FOLLOWING"
            )
        else:
            self.lost_frames += 1
            if self.lost_frames <= self.max_lost_frames:
                # Glitch filter: pertahankan kecepatan selama coasting
                twist = Twist()
                twist.linear.x = self.current_linear_speed
                twist.angular.z = self.last_twist.angular.z
                self.cmd_vel_pub.publish(twist)
                self._publish_status("COASTING")
            else:
                # Garis benar-benar hilang -> rem aman
                self.current_linear_speed = 0.0
                self.last_twist = Twist()
                self.cmd_vel_pub.publish(Twist())
                self.pid.reset()
                self._publish_status("LINE_LOST")

        if create_debug and res.debug_image is not None:
            self._publish_debug(res.debug_image)

    def _update_linear_speed(self, dt: float):
        """Slew rate limiter untuk akselerasi & deselerasi linier halus."""
        if self.mission_state in ("STOP", "TURN", "SEEK_LINE"):
            target = 0.0
        elif self.mission_state == "SLOW":
            target = self.slow_speed
        else:
            target = self.linear_speed

        max_delta = self.max_acceleration * dt
        if self.current_linear_speed < target:
            self.current_linear_speed = min(
                target, self.current_linear_speed + max_delta
            )
        elif self.current_linear_speed > target:
            self.current_linear_speed = max(
                target, self.current_linear_speed - max_delta
            )

    def _fail_turn(self, reason: str):
        self.get_logger().error(reason)
        self.mission_state = "STOP"
        self.current_linear_speed = 0.0
        self.turn_ctrl.reset()
        self.cmd_vel_pub.publish(Twist())
        self._publish_status("TURN_FAILED", force=True)

    def _publish_debug(self, debug_img):
        try:
            self.debug_pub.publish(
                self.bridge.cv2_to_imgmsg(debug_img, encoding="bgr8")
            )
            self.last_debug_publish_time = time.monotonic()
        except Exception:
            pass

    def _publish_status(self, status: str, force: bool = False):
        now = time.monotonic()
        if force or status != self.last_status or now - self.last_status_time >= 0.5:
            self.status_pub.publish(String(data=status))
            self.last_status = status
            self.last_status_time = now

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
