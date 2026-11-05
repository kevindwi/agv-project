"""Detect ArUco landmarks in frames published by the line tracker camera."""

import json

import cv2
import numpy as np
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


class ArucoDetectorNode(Node):
    """Publish marker IDs and, when calibrated, camera-relative marker poses."""

    def __init__(self):
        super().__init__("aruco_detector_node")
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("detections_topic", "/aruco/detections")
        self.declare_parameter("debug_topic", "/aruco/debug_image")
        self.declare_parameter("dictionary_id", 0)  # DICT_4X4_50
        self.declare_parameter("marker_length_m", 0.10)
        self.declare_parameter("camera_matrix", [0.0] * 9)
        self.declare_parameter("distortion_coefficients", [0.0] * 5)

        dictionary_id = int(self.get_parameter("dictionary_id").value)
        if not hasattr(cv2, "aruco"):
            raise RuntimeError(
                "cv2.aruco tidak tersedia; instal OpenCV contrib (opencv-contrib-python "
                "atau paket ROS/OpenCV contrib yang sesuai)."
            )
        self.aruco = cv2.aruco
        self.dictionary = self.aruco.getPredefinedDictionary(dictionary_id)
        # OpenCV 4.6 exposes DetectorParameters but its default constructor can
        # segfault inside detectMarkers on some builds. Use the legacy factory
        # unless the newer ArucoDetector API is available.
        if hasattr(self.aruco, "ArucoDetector"):
            self.detector_parameters = self.aruco.DetectorParameters()
        else:
            self.detector_parameters = self.aruco.DetectorParameters_create()
        self.detector = (
            self.aruco.ArucoDetector(self.dictionary, self.detector_parameters)
            if hasattr(self.aruco, "ArucoDetector")
            else None
        )
        self.marker_length = float(self.get_parameter("marker_length_m").value)
        self.camera_matrix = self._matrix_parameter("camera_matrix", (3, 3))
        self.distortion = self._matrix_parameter("distortion_coefficients")
        if self.camera_matrix is not None and self.distortion is None:
            self.distortion = np.zeros((1, 5), dtype=np.float64)
        self.bridge = CvBridge()

        self.detections_publisher = self.create_publisher(
            String, str(self.get_parameter("detections_topic").value), 10
        )
        self.debug_publisher = self.create_publisher(
            Image, str(self.get_parameter("debug_topic").value), 5
        )
        self.subscription = self.create_subscription(
            Image,
            str(self.get_parameter("image_topic").value),
            self.image_callback,
            5,
        )
        self.get_logger().info("Detektor ArUco aktif")

    def _matrix_parameter(self, name, shape=None):
        values = self.get_parameter(name).value
        if not values:
            return None
        matrix = np.asarray(values, dtype=np.float64)
        if shape is not None:
            if matrix.size != shape[0] * shape[1]:
                raise ValueError(f"{name} harus berisi {shape[0] * shape[1]} angka")
            if not np.any(matrix):
                return None
            matrix = matrix.reshape(shape)
        else:
            matrix = matrix.reshape(1, -1)
        return matrix

    def image_callback(self, message):
        try:
            frame = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except Exception as error:  # noqa: BLE001
            self.get_logger().warning(f"Frame kamera tidak dapat dibaca: {error}")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if self.detector is not None:
            corners, ids, _rejected = self.detector.detectMarkers(gray)
        else:
            corners, ids, _rejected = self.aruco.detectMarkers(
                gray, self.dictionary, parameters=self.detector_parameters
            )

        detections = []
        debug = frame.copy()
        if ids is not None:
            ids = ids.flatten().tolist()
            self.aruco.drawDetectedMarkers(debug, corners, np.asarray(ids).reshape(-1, 1))
            poses = [None] * len(ids)
            if self.camera_matrix is not None and self.marker_length > 0:
                rvecs, tvecs, _ = self.aruco.estimatePoseSingleMarkers(
                    corners, self.marker_length, self.camera_matrix, self.distortion
                )
                poses = [
                    {
                        "translation_m": [float(value) for value in tvec.flatten()],
                        "rotation_vector_rad": [float(value) for value in rvec.flatten()],
                    }
                    for rvec, tvec in zip(rvecs, tvecs)
                ]
            detections = [
                {"id": int(marker_id), "pose_camera": pose}
                for marker_id, pose in zip(ids, poses)
            ]
        output = String()
        output.data = json.dumps({"detections": detections}, separators=(",", ":"))
        self.detections_publisher.publish(output)
        try:
            self.debug_publisher.publish(
                self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
            )
        except Exception as error:  # noqa: BLE001
            self.get_logger().warning(f"Gagal publish debug ArUco: {error}")


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
