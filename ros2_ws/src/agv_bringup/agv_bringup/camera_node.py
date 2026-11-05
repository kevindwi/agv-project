"""Node terdedikasi untuk membaca kamera USB dan mempublikasikan Image ke ROS 2."""

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image


class CameraNode(Node):
    def __init__(self):
        super().__init__("camera_node")

        self.declare_parameter("camera_index", 2)
        self.declare_parameter("frame_width", 640)
        self.declare_parameter("frame_height", 480)
        self.declare_parameter("fps", 30.0)
        self.declare_parameter("image_topic", "/camera/image_raw")

        self.camera_index = int(self.get_parameter("camera_index").value)
        self.frame_width = int(self.get_parameter("frame_width").value)
        self.frame_height = int(self.get_parameter("frame_height").value)
        self.fps = float(self.get_parameter("fps").value)
        image_topic = str(self.get_parameter("image_topic").value)

        self.bridge = CvBridge()
        self.image_publisher = self.create_publisher(Image, image_topic, 10)

        self.capture = None
        self._init_camera()

        timer_period = 1.0 / max(1.0, self.fps)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info(
            f"CameraNode aktif: index={self.camera_index}, resolusi={self.frame_width}x{self.frame_height}, fps={self.fps}"
        )

    def _init_camera(self):
        if self.capture is not None and self.capture.isOpened():
            self.capture.release()

        self.capture = cv2.VideoCapture(self.camera_index, cv2.CAP_V4L2)
        if not self.capture.isOpened():
            self.get_logger().warn(f"Belum dapat membuka kamera pada index {self.camera_index}. Menunggu...")
            return

        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
        self.capture.set(cv2.CAP_PROP_FPS, self.fps)
        self.get_logger().info(f"Kamera index {self.camera_index} berhasil dibuka.")

    def timer_callback(self):
        if self.capture is None or not self.capture.isOpened():
            self._init_camera()
            return

        ret, frame = self.capture.read()
        if not ret or frame is None:
            self.get_logger().warn("Gagal membaca frame kamera.")
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_optical_frame"
        self.image_publisher.publish(msg)

    def destroy_node(self):
        if self.capture is not None and self.capture.isOpened():
            self.capture.release()
            self.get_logger().info("Kamera dilepas.")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
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
