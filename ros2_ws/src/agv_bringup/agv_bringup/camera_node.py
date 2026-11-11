"""Node terdedikasi untuk membaca kamera USB dan mempublikasikan Image ke ROS 2."""

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Image


IMAGE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    # Reliable cocok dengan ros2 topic hz / image viewer, antrean tetap satu frame.
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)


class CameraNode(Node):
    def __init__(self):
        super().__init__("camera_node")

        self.declare_parameter("camera_index", 2)
        self.declare_parameter("frame_width", 640)
        self.declare_parameter("frame_height", 480)
        self.declare_parameter("fps", 30.0)
        self.declare_parameter("pixel_format", "MJPG")
        self.declare_parameter("image_topic", "/camera/image_raw")

        self.camera_index = int(self.get_parameter("camera_index").value)
        self.frame_width = int(self.get_parameter("frame_width").value)
        self.frame_height = int(self.get_parameter("frame_height").value)
        self.fps = float(self.get_parameter("fps").value)
        self.pixel_format = str(self.get_parameter("pixel_format").value).strip().upper()
        image_topic = str(self.get_parameter("image_topic").value)

        self.bridge = CvBridge()
        # Video adalah data terbaru; jangan menumpuk frame lama saat konsumen sibuk.
        self.image_publisher = self.create_publisher(
            Image, image_topic, IMAGE_QOS
        )

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

        if len(self.pixel_format) == 4:
            fourcc = cv2.VideoWriter_fourcc(*self.pixel_format)
            if not self.capture.set(cv2.CAP_PROP_FOURCC, fourcc):
                self.get_logger().warning(
                    f"Kamera menolak format {self.pixel_format}; memakai format default driver."
                )
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
        self.capture.set(cv2.CAP_PROP_FPS, self.fps)
        # Minta V4L2 menyimpan sesedikit mungkin frame agar read() memberi frame baru.
        self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        actual_width = int(self.capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(self.capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = self.capture.get(cv2.CAP_PROP_FPS)
        actual_fourcc = int(self.capture.get(cv2.CAP_PROP_FOURCC))
        actual_format = "".join(
            chr((actual_fourcc >> (8 * i)) & 0xFF) for i in range(4)
        )
        self.get_logger().info(
            f"Kamera index={self.camera_index}, mode={actual_width}x{actual_height} "
            f"@ {actual_fps:.1f} FPS, format={actual_format!r} (diminta {self.pixel_format})."
        )
        if len(self.pixel_format) == 4 and actual_format != self.pixel_format:
            self.get_logger().warning(
                f"Format aktual {actual_format!r} berbeda dari permintaan "
                f"{self.pixel_format!r}; periksa mode yang didukung kamera."
            )

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
