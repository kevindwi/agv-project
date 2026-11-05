"""Deteksi ArUco marker dan publikasi pesan ArucoDetection terstruktur."""

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image

from agv_interfaces.msg import ArucoDetection


class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__("aruco_detector_node")

        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("detection_topic", "/aruco/detection")
        self.declare_parameter("debug_topic", "/aruco/debug_image")
        self.declare_parameter("dictionary_id", 0)  # DICT_4X4_50
        self.declare_parameter("marker_length_m", 0.10)
        self.declare_parameter("camera_matrix", [0.0] * 9)
        self.declare_parameter("distortion_coefficients", [0.0] * 5)

        self.marker_length = float(self.get_parameter("marker_length_m").value)
        dictionary_id = int(self.get_parameter("dictionary_id").value)

        if not hasattr(cv2, "aruco"):
            raise RuntimeError("cv2.aruco tidak ditemukan. Pastikan opencv-contrib terinstal.")

        self.aruco = cv2.aruco
        self.dictionary = self.aruco.getPredefinedDictionary(dictionary_id)

        # Kompatibilitas API OpenCV
        if hasattr(self.aruco, "ArucoDetector"):
            self.detector_parameters = self.aruco.DetectorParameters()
            self.detector = self.aruco.ArucoDetector(self.dictionary, self.detector_parameters)
        else:
            self.detector_parameters = self.aruco.DetectorParameters_create()
            self.detector = None

        self.camera_matrix = self._parse_matrix("camera_matrix", (3, 3))
        self.distortion = self._parse_matrix("distortion_coefficients", (1, 5))

        self.bridge = CvBridge()
        detection_topic = str(self.get_parameter("detection_topic").value)
        debug_topic = str(self.get_parameter("debug_topic").value)
        image_topic = str(self.get_parameter("image_topic").value)

        self.detection_publisher = self.create_publisher(ArucoDetection, detection_topic, 10)
        self.debug_publisher = self.create_publisher(Image, debug_topic, 5)

        self.create_subscription(Image, image_topic, self.image_callback, 5)
        self.get_logger().info(f"ArucoDetectorNode aktif. Subscribed ke {image_topic}")

    def _parse_matrix(self, name: str, shape: tuple):
        val = self.get_parameter(name).value
        if not val or len(val) != shape[0] * shape[1]:
            return None
        arr = np.array(val, dtype=np.float64).reshape(shape)
        if not np.any(arr):
            return None
        return arr

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"Gagal konversi frame kamera: {e}")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if self.detector is not None:
            corners, ids, _ = self.detector.detectMarkers(gray)
        else:
            corners, ids, _ = self.aruco.detectMarkers(
                gray, self.dictionary, parameters=self.detector_parameters
            )

        debug_img = frame.copy()

        if ids is not None and len(ids) > 0:
            flat_ids = ids.flatten()
            self.aruco.drawDetectedMarkers(debug_img, corners, ids)

            for i, marker_id in enumerate(flat_ids):
                marker_corners = corners[i][0]
                dist_z = 0.0

                # 1. Gunakan Pose Estimation jika camera_matrix sudah dikalibrasi
                if self.camera_matrix is not None:
                    dist_coeffs = self.distortion if self.distortion is not None else np.zeros((1, 5))
                    rvecs, tvecs, _ = self.aruco.estimatePoseSingleMarkers(
                        [corners[i]], self.marker_length, self.camera_matrix, dist_coeffs
                    )
                    dist_z = float(tvecs[0][0][2])
                    # Gambar sumbu 3D
                    cv2.drawFrameAxes(
                        debug_img, self.camera_matrix, dist_coeffs,
                        rvecs[0], tvecs[0], self.marker_length * 0.5
                    )
                else:
                    # 2. Estimasi geometri sederhana jika belum kalibrasi
                    # Jarak Z ~ (focal_length * real_width) / pixel_width
                    pixel_w = np.linalg.norm(marker_corners[0] - marker_corners[1])
                    focal_px = 500.0  # Perkiraan wajar kamera 640x480
                    dist_z = float((focal_px * self.marker_length) / max(1.0, pixel_w))

                # Publish pesan ROS 2 terstruktur
                det_msg = ArucoDetection()
                det_msg.id = int(marker_id)
                det_msg.distance_m = float(dist_z)
                self.detection_publisher.publish(det_msg)

                # Overlay teks di debug image
                c = marker_corners[0]
                cv2.putText(
                    debug_img,
                    f"ID: {marker_id} | Dist: {dist_z:.2f}m",
                    (int(c[0]), int(c[1]) - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 255),
                    2,
                )

        try:
            self.debug_publisher.publish(self.bridge.cv2_to_imgmsg(debug_img, encoding="bgr8"))
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
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
