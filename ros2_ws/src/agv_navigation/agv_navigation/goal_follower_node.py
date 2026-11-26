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

from .goal_controller import PointToPointController, quaternion_to_yaw


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
        self.declare_parameter("xy_goal_tolerance", 0.05)
        self.declare_parameter("yaw_goal_tolerance", 0.10)
        self.declare_parameter("heading_tolerance_to_drive", 0.30)
        self.declare_parameter("align_final_yaw", True)
        self.declare_parameter("goal_timeout_sec", 60.0)
        self.declare_parameter("control_rate_hz", 20.0)

        self.odom_frame = str(self.get_parameter("odom_frame_id").value)
        self.base_frame = str(self.get_parameter("base_frame_id").value)
        goal_topic = str(self.get_parameter("goal_topic").value)
        odom_topic = str(self.get_parameter("odom_topic").value)
        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        status_topic = str(self.get_parameter("status_topic").value)
        marker_topic = str(self.get_parameter("marker_topic").value)
        self.goal_timeout = float(self.get_parameter("goal_timeout_sec").value)
        control_rate = max(1.0, float(self.get_parameter("control_rate_hz").value))

        # Kontroler P2P
        self.controller = PointToPointController(
            kp_linear=float(self.get_parameter("kp_linear").value),
            kp_angular=float(self.get_parameter("kp_angular").value),
            max_linear_speed=float(self.get_parameter("max_linear_speed").value),
            min_linear_speed=float(self.get_parameter("min_linear_speed").value),
            max_angular_speed=float(self.get_parameter("max_angular_speed").value),
            min_angular_speed=float(self.get_parameter("min_angular_speed").value),
            max_linear_accel=float(self.get_parameter("max_linear_accel").value),
            max_angular_accel=float(self.get_parameter("max_angular_accel").value),
            xy_tolerance=float(self.get_parameter("xy_goal_tolerance").value),
            yaw_tolerance=float(self.get_parameter("yaw_goal_tolerance").value),
            heading_tolerance=float(
                self.get_parameter("heading_tolerance_to_drive").value
            ),
            align_final_yaw=bool(self.get_parameter("align_final_yaw").value),
        )

        # TF2 listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # State Robot & Target
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.odom_received = False

        self.active_goal = False
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_yaw = 0.0
        self.goal_start_time = 0.0

        self.state = "IDLE"
        self.last_status = ""
        self.last_status_time = 0.0
        self.last_control_time = time.monotonic()

        # Publisher dan Subscriber
        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.status_pub = self.create_publisher(String, status_topic, 10)
        self.marker_pub = self.create_publisher(Marker, marker_topic, 10)

        self.create_subscription(
            PoseStamped, goal_topic, self.goal_callback, 10
        )
        self.create_subscription(
            Odometry, odom_topic, self.odom_callback, 10
        )

        self.timer = self.create_timer(1.0 / control_rate, self.control_loop)
        self.get_logger().info(
            f"Goal Follower aktif! Menunggu target di {goal_topic} (Frame: '{self.odom_frame}')"
        )

    def odom_callback(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.current_yaw = quaternion_to_yaw(qz, qw)
        self.odom_received = True

    def goal_callback(self, msg: PoseStamped):
        target_pose = msg
        frame_id = msg.header.frame_id or self.odom_frame

        if frame_id != self.odom_frame:
            try:
                target_pose = self.tf_buffer.transform(
                    msg, self.odom_frame, timeout=Duration(seconds=0.5)
                )
            except Exception as e:
                self.get_logger().warn(
                    f"Gagal transformasi goal dari '{frame_id}' ke '{self.odom_frame}': {e}"
                )

        self.target_x = target_pose.pose.position.x
        self.target_y = target_pose.pose.position.y
        qz = target_pose.pose.orientation.z
        qw = target_pose.pose.orientation.w
        self.target_yaw = quaternion_to_yaw(qz, qw)

        self.active_goal = True
        self.goal_start_time = time.monotonic()
        self.state = "ROTATING_TO_HEADING"
        self.controller.reset()

        self.get_logger().info(
            f"Target baru: X={self.target_x:.3f} m, Y={self.target_y:.3f} m, "
            f"Yaw={math.degrees(self.target_yaw):.1f}°"
        )
        self._publish_marker()
        self._publish_status("NEW_GOAL_RECEIVED", force=True)

    def control_loop(self):
        now = time.monotonic()
        dt = max(1e-4, min(0.2, now - self.last_control_time))
        self.last_control_time = now

        if not self.active_goal:
            if self.controller.current_v != 0.0 or self.controller.current_w != 0.0:
                self._stop_robot()
            return

        if not self.odom_received:
            self.get_logger().warn_throttle(2.0, "Menunggu data /odom...")
            return

        if self.goal_timeout > 0.0 and (now - self.goal_start_time) > self.goal_timeout:
            self.get_logger().warn(f"Goal timeout setelah {self.goal_timeout:.1f} detik.")
            self._abort_goal("TIMEOUT")
            return

        new_state, cmd_v, cmd_w, rho = self.controller.step(
            self.current_x,
            self.current_y,
            self.current_yaw,
            self.target_x,
            self.target_y,
            self.target_yaw,
            self.state,
            dt,
        )

        if new_state == "ARRIVED":
            self._complete_goal()
            return

        self.state = new_state
        twist = Twist()
        twist.linear.x = cmd_v
        twist.angular.z = cmd_w
        self.cmd_vel_pub.publish(twist)
        self._publish_status(self.state)

    def _complete_goal(self):
        self._stop_robot()
        self.active_goal = False
        self.state = "ARRIVED"
        self.get_logger().info("Target tercapai dengan sukses!")
        self._publish_status("GOAL_REACHED", force=True)
        self._clear_marker()

    def _abort_goal(self, reason: str):
        self._stop_robot()
        self.active_goal = False
        self.state = reason
        self._publish_status(f"GOAL_ABORTED_{reason}", force=True)
        self._clear_marker()

    def _stop_robot(self):
        self.controller.reset()
        self.cmd_vel_pub.publish(Twist())

    def _publish_status(self, status: str, force: bool = False):
        now = time.monotonic()
        if force or status != self.last_status or now - self.last_status_time >= 0.5:
            self.status_pub.publish(String(data=status))
            self.last_status = status
            self.last_status_time = now

    def _publish_marker(self):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.odom_frame
        marker.ns = "active_goal"
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = self.target_x
        marker.pose.position.y = self.target_y
        marker.pose.position.z = 0.05
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.controller.xy_tolerance * 2.0
        marker.scale.y = self.controller.xy_tolerance * 2.0
        marker.scale.z = 0.02
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.2
        marker.color.a = 0.6
        self.marker_pub.publish(marker)

    def _clear_marker(self):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.odom_frame
        marker.ns = "active_goal"
        marker.id = 0
        marker.action = Marker.DELETE
        self.marker_pub.publish(marker)

    def destroy_node(self):
        self._stop_robot()
        self._publish_status("STOPPED")
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
