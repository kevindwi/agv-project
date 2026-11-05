"""State Machine Misi: mengendalikan rute AGV berdasarkan deteksi ArUco."""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from agv_interfaces.msg import ArucoDetection, MissionState


class MissionControllerNode(Node):
    def __init__(self):
        super().__init__("mission_controller_node")

        # Parameter
        self.declare_parameter("confirm_frames", 3)
        self.declare_parameter("trigger_distance_m", 0.35)
        self.declare_parameter("slow_distance_m", 0.70)
        self.declare_parameter("route_ids", [1, 2, 3])

        self.confirm_frames = int(self.get_parameter("confirm_frames").value)
        self.trigger_distance = float(self.get_parameter("trigger_distance_m").value)
        self.slow_distance = float(self.get_parameter("slow_distance_m").value)
        self.route = [int(i) for i in self.get_parameter("route_ids").value]

        self.route_idx = 0
        self.state = "NAVIGATING" if self.route else "IDLE"
        self.candidate_id = None
        self.candidate_count = 0
        self.latched_id = None

        # Publisher & Subscriber
        self.control_pub = self.create_publisher(String, "/mission/control", 10)
        self.state_pub = self.create_publisher(MissionState, "/mission/state", 10)

        self.create_subscription(ArucoDetection, "/aruco/detection", self.detection_callback, 10)

        # Timer status (2 Hz)
        self.timer = self.create_timer(0.5, self.publish_state)

        # Beri perintah jalan awal jika ada rute
        if self.route:
            self._send_control("FOLLOW")
            self.get_logger().info(f"Misi dimulai! Rute ArUco target: {self.route}")

    def detection_callback(self, msg: ArucoDetection):
        if self.state in ("ARRIVED", "ERROR", "IDLE"):
            return

        if self.route_idx >= len(self.route):
            self.state = "ARRIVED"
            self._send_control("STOP")
            return

        target_id = self.route[self.route_idx]

        # Abaikan marker yang baru saja dilewati (latch)
        if self.latched_id == msg.id:
            return

        # Cek apakah ID yang terlihat adalah checkpoint berikutnya
        if msg.id == target_id:
            if self.candidate_id == target_id:
                self.candidate_count += 1
            else:
                self.candidate_id = target_id
                self.candidate_count = 1

            if self.candidate_count >= self.confirm_frames:
                self._handle_checkpoint(msg.id, msg.distance_m)
        else:
            self.candidate_count = 0

    def _handle_checkpoint(self, marker_id: int, distance_m: float):
        # Jika masih cukup jauh, perintahkan robot melambat
        if distance_m > self.trigger_distance:
            if distance_m <= self.slow_distance and self.state != "APPROACHING":
                self.get_logger().info(f"Checkpoint {marker_id} terlihat pada {distance_m:.2f}m. Melambat...")
                self.state = "APPROACHING"
                self._send_control("SLOW")
            return

        # Jarak sudah mencapai batas pemicu (<= trigger_distance)
        self.get_logger().info(f"Checkpoint {marker_id} TERCAPAI pada jarak {distance_m:.2f}m!")
        self.latched_id = marker_id
        self.candidate_count = 0

        # Cek apakah ini checkpoint terakhir
        is_last = (self.route_idx == len(self.route) - 1)
        if is_last:
            self.state = "ARRIVED"
            self.get_logger().info("Semua checkpoint selesai! Robot berhenti.")
            self._send_control("STOP")
        else:
            self.route_idx += 1
            next_target = self.route[self.route_idx]
            self.state = "NAVIGATING"
            self.get_logger().info(f"Lanjut ke checkpoint berikutnya: ID {next_target}")
            self._send_control("FOLLOW")

    def _send_control(self, cmd: str):
        msg = String()
        msg.data = cmd
        self.control_pub.publish(msg)

    def publish_state(self):
        msg = MissionState()
        msg.state = self.state
        msg.current_waypoint = self.route[self.route_idx] if self.route_idx < len(self.route) else -1
        self.state_pub.publish(msg)

    def destroy_node(self):
        self._send_control("STOP")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MissionControllerNode()
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
