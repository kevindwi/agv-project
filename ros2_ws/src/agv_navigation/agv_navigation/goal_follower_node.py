"""Node navigasi titik-ke-titik (Point-to-Point) berbasis frame odom lokal.

Menerima 2D Goal Pose dari RViz (/goal_pose), membandingkan terhadap posisi
saat ini (/odom), dan menerbitkan /cmd_vel menggunakan kontroler proporsional
dengan pembatas akselerasi untuk mencegah roda selip.
"""

import math
import time

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker

import tf2_geometry_msgs
from tf2_ros import Buffer, TransformListener


def normalize_angle(angle: float) -> float:
    """Normalisasikan sudut ke rentang [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(z: float, w: float) -> float:
    """Konversi kuaternion planar 2D (qz, qw) ke sudut yaw dalam radian."""
    return 2.0 * math.atan2(z, w)


class GoalFollowerNode(Node):
    """Pengontrol navigasi titik-ke-titik lokal untuk differential drive."""

    def __init__(self):
        super().__init__("goal_follower_node")

        # Frame dan Topik
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("base_frame_id", "base_link")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("status_topic", "/goal_follower/state")
        self.declare_parameter("marker_topic", "/goal_follower/active_goal_marker")

        # Parameter Kontrol Kecepatan
        self.declare_parameter("kp_linear", 0.5)
        self.declare_parameter("kp_angular", 1.2)
        self.declare_parameter("max_linear_speed", 0.15)
        self.declare_parameter("min_linear_speed", 0.04)
        self.declare_parameter("max_angular_speed", 0.60)
        self.declare_parameter("min_angular_speed", 0.12)
        self.declare_parameter("max_linear_accel", 0.15)
        self.declare_parameter("max_angular_accel", 0.80)

        # Toleransi & Ambang Batas
        self.declare_parameter("xy_goal_tolerance", 0.05)       # 5 cm
        self.declare_parameter("yaw_goal_tolerance", 0.10)      # ~5.7 derajat
        self.declare_parameter("heading_tolerance_to_drive", 0.30)  # ~17 derajat
        self.declare_parameter("align_final_yaw", True)
        self.declare_parameter("goal_timeout_sec", 60.0)
        self.declare_parameter("control_rate_hz", 20.0)

        # Ambil nilai parameter
        self.odom_frame = str(self.get_parameter("odom_frame_id").value)
        self.base_frame = str(self.get_parameter("base_frame_id").value)
        goal_topic = str(self.get_parameter("goal_topic").value)
        odom_topic = str(self.get_parameter("odom_topic").value)
        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        status_topic = str(self.get_parameter("status_topic").value)
        marker_topic = str(self.get_parameter("marker_topic").value)

        self.kp_linear = float(self.get_parameter("kp_linear").value)
        self.kp_angular = float(self.get_parameter("kp_angular").value)
        self.max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self.min_linear_speed = float(self.get_parameter("min_linear_speed").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        self.min_angular_speed = float(self.get_parameter("min_angular_speed").value)
        self.max_linear_accel = float(self.get_parameter("max_linear_accel").value)
        self.max_angular_accel = float(self.get_parameter("max_angular_accel").value)

        self.xy_tolerance = float(self.get_parameter("xy_goal_tolerance").value)
        self.yaw_tolerance = float(self.get_parameter("yaw_goal_tolerance").value)
        self.heading_tolerance = float(self.get_parameter("heading_tolerance_to_drive").value)
        self.align_final_yaw = bool(self.get_parameter("align_final_yaw").value)
        self.goal_timeout = float(self.get_parameter("goal_timeout_sec").value)
        control_rate = max(1.0, float(self.get_parameter("control_rate_hz").value))

        # TF2 listener untuk menangani frame konversi bila goal dikirim bukan dalam frame odom
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # State Robot
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.odom_received = False

        # State Target
        self.active_goal = False
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_yaw = 0.0
        self.goal_start_time = 0.0

        # State Mesin: IDLE, ROTATING_TO_HEADING, DRIVING_TO_POINT, ROTATING_TO_FINAL_HEADING, ARRIVED, TIMEOUT
        self.state = "IDLE"
        self.last_status = ""
        self.last_status_time = 0.0

        # Kecepatan aktual untuk pemulusan akselerasi (ramping)
        self.current_linear_cmd = 0.0
        self.current_angular_cmd = 0.0
        self.last_control_time = time.monotonic()

        # Publisher dan Subscriber
        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.status_pub = self.create_publisher(String, status_topic, 10)
        self.marker_pub = self.create_publisher(Marker, marker_topic, 10)

        self.goal_sub = self.create_subscription(
            PoseStamped, goal_topic, self.goal_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, odom_topic, self.odom_callback, 10
        )

        # Loop kontrol periodik
        self.timer = self.create_timer(1.0 / control_rate, self.control_loop)

        self.get_logger().info(
            f"Goal Follower aktif! Menunggu target di {goal_topic} (Frame acuan: '{self.odom_frame}')"
        )

    def odom_callback(self, msg: Odometry):
        """Update posisi dan heading robot saat ini dari /odom."""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.current_yaw = quaternion_to_yaw(qz, qw)
        self.odom_received = True

    def goal_callback(self, msg: PoseStamped):
        """Terima target baru dari RViz 2D Goal Pose."""
        target_pose = msg
        frame_id = msg.header.frame_id or self.odom_frame

        # Jika target dikirim dalam frame selain odom, coba transformasi
        if frame_id != self.odom_frame:
            try:
                target_pose = self.tf_buffer.transform(
                    msg, self.odom_frame, timeout=Duration(seconds=0.5)
                )
            except Exception as e:
                self.get_logger().warn(
                    f"Tidak dapat mentransformasikan goal dari '{frame_id}' ke '{self.odom_frame}': {e}. Menggunakan koordinat mentah."
                )

        self.target_x = target_pose.pose.position.x
        self.target_y = target_pose.pose.position.y
        qz = target_pose.pose.orientation.z
        qw = target_pose.pose.orientation.w
        self.target_yaw = quaternion_to_yaw(qz, qw)

        self.active_goal = True
        self.goal_start_time = time.monotonic()
        self.state = "ROTATING_TO_HEADING"

        self.get_logger().info(
            f"Target baru diterima: X={self.target_x:.3f} m, Y={self.target_y:.3f} m, "
            f"Yaw={math.degrees(self.target_yaw):.1f}°"
        )

        self._publish_marker()
        self._publish_status("NEW_GOAL_RECEIVED", force=True)

    def control_loop(self):
        """Loop kontrol periodik navigasi titik-ke-titik."""
        now = time.monotonic()
        dt = max(1e-4, min(0.2, now - self.last_control_time))
        self.last_control_time = now

        if not self.active_goal:
            if self.current_linear_cmd != 0.0 or self.current_angular_cmd != 0.0:
                self._stop_robot()
            return

        if not self.odom_received:
            self.get_logger().warn_throttle(2.0, "Menunggu data /odom...")
            return

        # Cek batas waktu (timeout)
        if self.goal_timeout > 0.0 and (now - self.goal_start_time) > self.goal_timeout:
            self.get_logger().warn(f"Goal timeout setelah {self.goal_timeout:.1f} detik.")
            self._abort_goal("TIMEOUT")
            return

        # Hitung jarak dan sudut relatif ke target posisi
        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y
        rho = math.hypot(dx, dy)
        target_heading = math.atan2(dy, dx)
        heading_error = normalize_angle(target_heading - self.current_yaw)

        # 1. Apakah posisi (X, Y) sudah dalam batas toleransi?
        if rho <= self.xy_tolerance:
            if self.align_final_yaw:
                yaw_error = normalize_angle(self.target_yaw - self.current_yaw)
                if abs(yaw_error) <= self.yaw_tolerance:
                    self._complete_goal()
                    return
                # Putar di tempat menyelaraskan orientasi akhir
                self.state = "ROTATING_TO_FINAL_HEADING"
                target_v = 0.0
                target_w = self._calc_angular_vel(yaw_error)
            else:
                self._complete_goal()
                return
        else:
            # 2. Belum sampai di posisi: putar ke arah target atau maju sambil mengarahkan
            if self.state == "ROTATING_TO_HEADING":
                if abs(heading_error) <= self.heading_tolerance:
                    self.state = "DRIVING_TO_POINT"
                    target_v = 0.0
                    target_w = 0.0
                else:
                    target_v = 0.0
                    target_w = self._calc_angular_vel(heading_error)
            elif self.state == "DRIVING_TO_POINT":
                # Jika heading meleset terlalu jauh (> 60 derajat), putar di tempat kembali
                if abs(heading_error) > 1.05:
                    self.state = "ROTATING_TO_HEADING"
                    target_v = 0.0
                    target_w = self._calc_angular_vel(heading_error)
                else:
                    # Maju dengan kecepatan proporsional terhadap sisa jarak,
                    # dikalikan cosinus kesalahan heading agar melambat jika sedikit serong
                    facing_scale = max(0.0, math.cos(heading_error))
                    raw_linear = max(self.min_linear_speed, min(self.max_linear_speed, self.kp_linear * rho))
                    target_v = raw_linear * facing_scale
                    target_w = max(
                        -self.max_angular_speed,
                        min(self.max_angular_speed, self.kp_angular * heading_error),
                    )
            else:
                # Transisi default jika dari state lain
                self.state = "ROTATING_TO_HEADING"
                target_v = 0.0
                target_w = self._calc_angular_vel(heading_error)

        # Terapkan pembatas akselerasi (ramping) agar roda tidak selip di lantai keramik
        self._apply_velocity_ramping(target_v, target_w, dt)
        self._publish_status(self.state)

    def _calc_angular_vel(self, error: float) -> float:
        """Hitung kecepatan angular dengan clamping min/max."""
        raw_w = self.kp_angular * error
        clamped_w = max(-self.max_angular_speed, min(self.max_angular_speed, raw_w))
        if abs(clamped_w) < self.min_angular_speed:
            return math.copysign(self.min_angular_speed, error)
        return clamped_w

    def _apply_velocity_ramping(self, target_v: float, target_w: float, dt: float):
        """Batasi perubahan kecepatan mendadak."""
        max_dv = self.max_linear_accel * dt
        dv = target_v - self.current_linear_cmd
        if abs(dv) > max_dv:
            self.current_linear_cmd += math.copysign(max_dv, dv)
        else:
            self.current_linear_cmd = target_v

        max_dw = self.max_angular_accel * dt
        dw = target_w - self.current_angular_cmd
        if abs(dw) > max_dw:
            self.current_angular_cmd += math.copysign(max_dw, dw)
        else:
            self.current_angular_cmd = target_w

        cmd = Twist()
        cmd.linear.x = self.current_linear_cmd
        cmd.angular.z = self.current_angular_cmd
        self.cmd_vel_pub.publish(cmd)

    def _stop_robot(self):
        """Hentikan robot seketika."""
        self.current_linear_cmd = 0.0
        self.current_angular_cmd = 0.0
        self.cmd_vel_pub.publish(Twist())

    def _complete_goal(self):
        """Selesaikan navigasi saat target tercapai."""
        self._stop_robot()
        self.active_goal = False
        self.state = "ARRIVED"
        self._publish_status("ARRIVED", force=True)
        self.get_logger().info("Target berhasil dicapai!")

    def _abort_goal(self, reason: str):
        """Batalkan target karena kegagalan atau timeout."""
        self._stop_robot()
        self.active_goal = False
        self.state = reason
        self._publish_status(reason, force=True)

    def _publish_marker(self):
        """Publikasikan visualisasi target aktif di RViz."""
        marker = Marker()
        marker.header.frame_id = self.odom_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "active_goal"
        marker.id = 1
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        marker.pose.position.x = self.target_x
        marker.pose.position.y = self.target_y
        marker.pose.position.z = 0.05

        half_yaw = self.target_yaw / 2.0
        marker.pose.orientation.z = math.sin(half_yaw)
        marker.pose.orientation.w = math.cos(half_yaw)

        # Ukuran panah: panjang 0.3m, diameter 0.06m
        marker.scale.x = 0.35
        marker.scale.y = 0.06
        marker.scale.z = 0.06

        # Warna cyan terang transparan
        marker.color.r = 0.0
        marker.color.g = 0.9
        marker.color.b = 1.0
        marker.color.a = 0.85

        self.marker_pub.publish(marker)

    def _publish_status(self, status: str, force: bool = False):
        """Publikasikan status dengan heartbeat 0.5 detik."""
        now = time.monotonic()
        if force or status != self.last_status or now - self.last_status_time >= 0.5:
            self.status_pub.publish(String(data=status))
            self.last_status = status
            self.last_status_time = now

    def destroy_node(self):
        self._stop_robot()
        self.status_pub.publish(String(data="STOPPED"))
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GoalFollowerNode()
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
