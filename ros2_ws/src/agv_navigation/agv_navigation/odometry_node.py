"""Estimasi odometri DDMR dari delta encoder dan publikasi odom, TF, dan visualisasi 3D robot."""

import math
from geometry_msgs.msg import Point, PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Int64MultiArray
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import MarkerArray

from .robot_visualizer import create_robot_markers


class OdometryNode(Node):
    """Integrasikan tick encoder menjadi pose odom, menyiarkan TF, dan menerbitkan visualisasi robot."""

    def __init__(self):
        super().__init__("odometry_node")
        self.declare_parameter("wheel_diameter_m", 0.15)
        self.declare_parameter("wheel_separation_m", 0.48)
        self.declare_parameter("encoder_pulses_per_wheel_rev", 900)
        self.declare_parameter("encoder_delta_topic", "/agv/encoder_delta")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("path_topic", "/path")
        self.declare_parameter("marker_topic", "/robot_marker")
        self.declare_parameter("max_path_poses", 10000)
        self.declare_parameter("path_publish_rate_hz", 2.0)
        self.declare_parameter("odom_publish_rate_hz", 20.0)
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("base_frame_id", "base_link")

        self.wheel_diameter = float(self.get_parameter("wheel_diameter_m").value)
        self.wheel_separation = float(self.get_parameter("wheel_separation_m").value)
        self.encoder_cpr = int(self.get_parameter("encoder_pulses_per_wheel_rev").value)

        self.odom_frame_id = str(self.get_parameter("odom_frame_id").value)
        self.base_frame_id = str(self.get_parameter("base_frame_id").value)
        encoder_topic = str(self.get_parameter("encoder_delta_topic").value)
        odom_topic = str(self.get_parameter("odom_topic").value)
        path_topic = str(self.get_parameter("path_topic").value)
        marker_topic = str(self.get_parameter("marker_topic").value)
        self.max_path_poses = int(self.get_parameter("max_path_poses").value)
        path_publish_rate = float(self.get_parameter("path_publish_rate_hz").value)
        odom_publish_rate = float(self.get_parameter("odom_publish_rate_hz").value)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.last_encoder_time = None

        self.odom_publisher = self.create_publisher(Odometry, odom_topic, 10)
        self.path_publisher = self.create_publisher(Path, path_topic, 10)
        self.marker_publisher = self.create_publisher(MarkerArray, marker_topic, 10)

        self.path = Path()
        self.path.header.frame_id = self.odom_frame_id
        self.transform_broadcaster = TransformBroadcaster(self)

        self.encoder_subscription = self.create_subscription(
            Int64MultiArray, encoder_topic, self.encoder_callback, 10
        )

        # Timer periodik: Memastikan TF dan Odometry selalu aktif & realtime di RViz bahkan saat diam
        self.odom_timer = self.create_timer(1.0 / odom_publish_rate, self.publish_odom_and_tf)
        self.path_timer = self.create_timer(1.0 / path_publish_rate, self.publish_path)

        self.get_logger().info(
            f"Odometri aktif: {encoder_topic} -> {odom_topic}, TF {self.odom_frame_id} -> {self.base_frame_id}"
        )

    def encoder_callback(self, message: Int64MultiArray):
        if len(message.data) != 2:
            return

        left_ticks, right_ticks = message.data
        distance_per_tick = math.pi * self.wheel_diameter / self.encoder_cpr
        left_distance = left_ticks * distance_per_tick
        right_distance = right_ticks * distance_per_tick
        delta_distance = (left_distance + right_distance) / 2.0
        delta_theta = (right_distance - left_distance) / self.wheel_separation

        midpoint_theta = self.theta + delta_theta / 2.0
        self.x += delta_distance * math.cos(midpoint_theta)
        self.y += delta_distance * math.sin(midpoint_theta)
        self.theta += delta_theta

        now_seconds = self.get_clock().now().nanoseconds * 1e-9
        dt = None if self.last_encoder_time is None else now_seconds - self.last_encoder_time
        self.last_encoder_time = now_seconds

        self.linear_velocity = 0.0 if dt is None or dt <= 0.0 else delta_distance / dt
        self.angular_velocity = 0.0 if dt is None or dt <= 0.0 else delta_theta / dt

        # Rekam jejak pose ke path
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.header.frame_id = self.odom_frame_id
        pose_stamped.pose.position.x = self.x
        pose_stamped.pose.position.y = self.y
        half_theta = self.theta / 2.0
        pose_stamped.pose.orientation.z = math.sin(half_theta)
        pose_stamped.pose.orientation.w = math.cos(half_theta)
        self.path.poses.append(pose_stamped)
        if len(self.path.poses) > self.max_path_poses:
            self.path.poses = self.path.poses[-self.max_path_poses:]

        # Publikasikan segera
        self.publish_odom_and_tf()

    def publish_odom_and_tf(self):
        """Siarkan TF odom -> base_link, publikasikan /odom, dan marker 3D robot."""
        stamp = self.get_clock().now().to_msg()
        half_theta = self.theta / 2.0
        qz = math.sin(half_theta)
        qw = math.cos(half_theta)

        # 1. Pesan Odometry
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = self.linear_velocity
        odom.twist.twist.angular.z = self.angular_velocity
        self.odom_publisher.publish(odom)

        # 2. Siaran TF Transform
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.odom_frame_id
        transform.child_frame_id = self.base_frame_id
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self.transform_broadcaster.sendTransform(transform)

        # 3. Visualisasi Marker 3D Robot di RViz
        marker_array = create_robot_markers(
            stamp, self.base_frame_id, self.wheel_separation, self.wheel_diameter
        )
        self.marker_publisher.publish(marker_array)

    def publish_path(self):
        if not self.path.poses:
            return
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.path_publisher.publish(self.path)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
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
