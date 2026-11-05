"""Estimasi odometri DDMR dari delta encoder dan publikasi odom serta TF."""

import math
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Int64MultiArray
from tf2_ros import TransformBroadcaster


class OdometryNode(Node):
    """Integrasikan tick encoder menjadi pose global odom."""

    def __init__(self):
        super().__init__("odometry_node")
        self.declare_parameter("wheel_diameter_m", 0.15)
        self.declare_parameter("wheel_separation_m", 0.48)
        self.declare_parameter("encoder_pulses_per_wheel_rev", 900)
        self.declare_parameter("encoder_delta_topic", "/agv/encoder_delta")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("path_topic", "/path")
        self.declare_parameter("max_path_poses", 10000)
        self.declare_parameter("path_publish_rate_hz", 2.0)
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
        self.max_path_poses = int(self.get_parameter("max_path_poses").value)
        path_publish_rate = float(self.get_parameter("path_publish_rate_hz").value)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_stamp = None

        self.odom_publisher = self.create_publisher(Odometry, odom_topic, 10)
        self.path_publisher = self.create_publisher(Path, path_topic, 10)
        self.path = Path()
        self.path.header.frame_id = self.odom_frame_id
        self.transform_broadcaster = TransformBroadcaster(self)

        self.encoder_subscription = self.create_subscription(
            Int64MultiArray, encoder_topic, self.encoder_callback, 10
        )
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

        stamp = self.get_clock().now().to_msg()
        now_seconds = stamp.sec + stamp.nanosec * 1e-9
        dt = None if self.last_stamp is None else now_seconds - self.last_stamp
        self.last_stamp = now_seconds

        linear_velocity = 0.0 if dt is None or dt <= 0.0 else delta_distance / dt
        angular_velocity = 0.0 if dt is None or dt <= 0.0 else delta_theta / dt

        half_theta = self.theta / 2.0
        qz = math.sin(half_theta)
        qw = math.cos(half_theta)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = linear_velocity
        odom.twist.twist.angular.z = angular_velocity
        self.odom_publisher.publish(odom)

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.odom_frame_id
        transform.child_frame_id = self.base_frame_id
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self.transform_broadcaster.sendTransform(transform)

        pose_stamped = PoseStamped()
        pose_stamped.header = odom.header
        pose_stamped.pose = odom.pose.pose
        self.path.poses.append(pose_stamped)
        if len(self.path.poses) > self.max_path_poses:
            self.path.poses = self.path.poses[-self.max_path_poses:]

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
