"""Controller Misi Navigasi Waypoint & Persimpangan AGV (Tahap 4A).

State Machine: IDLE, NAVIGATING, AT_INTERSECTION, ARRIVED, PAUSED, ERROR, ESTOP.
Mengatur eksekusi rute sekuensial antar-checkpoint (CP1, CP2, CP3) berdasarkan
deteksi marker ArUco dan menerbitkan instruksi percabangan ke line tracker.
"""

from enum import Enum
import json
import os
from typing import List, Optional

from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Int32, String

from .route_manager import RouteManager, RouteStep


class MissionState(str, Enum):
    IDLE = "IDLE"
    NAVIGATING = "NAVIGATING"
    AT_INTERSECTION = "AT_INTERSECTION"
    ARRIVED = "ARRIVED"
    PAUSED = "PAUSED"
    ERROR = "ERROR"
    ESTOP = "ESTOP"


class MissionControllerNode(Node):
    """Pengendali misi navigasi graf lintasan AGV."""

    def __init__(self):
        super().__init__("mission_controller_node")

        # Tentukan default path map file
        try:
            pkg_share = get_package_share_directory("line_tracker_ros2")
            default_map = os.path.join(pkg_share, "config", "track_graph.yaml")
        except Exception:
            default_map = os.path.join(os.getcwd(), "config", "track_graph.yaml")

        self.declare_parameter("map_file", default_map)
        self.declare_parameter("initial_node", "CP1")
        self.declare_parameter("status_publish_rate_hz", 5.0)

        map_path = str(self.get_parameter("map_file").value)
        self.current_node = str(self.get_parameter("initial_node").value)
        status_rate = float(self.get_parameter("status_publish_rate_hz").value)

        # Inisialisasi Route Manager
        self.route_manager = RouteManager()
        try:
            self.route_manager.load_map(map_path)
            self.get_logger().info(f"Peta graf berhasil dimuat dari: {map_path}")
        except Exception as error:
            self.get_logger().error(f"Gagal memuat peta graf: {error}")
            self.state = MissionState.ERROR
            return

        # State navigasi
        self.state = MissionState.IDLE
        self.target_node: Optional[str] = None
        self.current_route: List[RouteStep] = []
        self.current_step_index: int = 0
        self.last_detected_aruco_id: Optional[int] = None
        self.current_x: float = 0.0
        self.current_y: float = 0.0

        # Publisher & Subscriber
        self.aruco_sub = self.create_subscription(
            Int32, "/aruco/detected_id", self.aruco_callback, 10
        )
        self.command_sub = self.create_subscription(
            String, "/mission/command", self.command_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10
        )

        self.branch_pub = self.create_publisher(
            String, "/line_tracker/branch_instruction", 10
        )
        self.status_pub = self.create_publisher(
            String, "/mission/status", 10
        )

        self.status_timer = self.create_timer(1.0 / status_rate, self.publish_status)
        self.get_logger().info(f"Mission Controller Node aktif. Posisi awal: {self.current_node}, State: {self.state}")

    def odom_callback(self, message: Odometry):
        self.current_x = message.pose.pose.position.x
        self.current_y = message.pose.pose.position.y

    def command_callback(self, message: String):
        """Parsing dan eksekusi perintah misi (JSON atau text)."""
        raw = message.data.strip()
        action = ""
        target = ""

        # Coba parsing JSON format: {"action": "go_to", "target": "CP2"}
        try:
            payload = json.loads(raw)
            action = payload.get("action", "").lower()
            target = payload.get("target", "").upper()
        except json.JSONDecodeError:
            # Fallback ke format teks: "GO_TO CP2", "PAUSE", "RESUME", "CANCEL"
            parts = raw.split()
            if parts:
                action = parts[0].lower()
                if len(parts) > 1:
                    target = parts[1].upper()

        self.get_logger().info(f"[MissionCommand] Menerima aksi: '{action}', target: '{target}'")

        if action in ("go_to", "goto", "start"):
            self.start_mission(target)
        elif action == "pause":
            self.pause_mission()
        elif action == "resume":
            self.resume_mission()
        elif action in ("cancel", "stop"):
            self.cancel_mission()
        elif action in ("ack_error", "reset"):
            self.state = MissionState.IDLE
            self.get_logger().info("Status error di-reset. Kembali ke IDLE.")
        else:
            self.get_logger().warn(f"Perintah tidak dikenal: '{raw}'")

    def start_mission(self, target_node: str):
        """Rencanakan dan mulai eksekusi rute ke checkpoint tujuan."""
        if not target_node:
            self.get_logger().error("Target node kosong!")
            return

        if target_node not in self.route_manager.nodes:
            self.get_logger().error(f"Target node '{target_node}' tidak ada di peta!")
            self.state = MissionState.ERROR
            return

        try:
            steps = self.route_manager.plan_route(self.current_node, target_node)
        except Exception as error:
            self.get_logger().error(f"Gagal merencanakan rute: {error}")
            self.state = MissionState.ERROR
            return

        if not steps:
            self.get_logger().info(f"Robot sudah berada di {target_node}. Misi langsung selesai.")
            self.state = MissionState.ARRIVED
            self.send_branch_instruction("STOP")
            return

        self.target_node = target_node
        self.current_route = steps
        self.current_step_index = 0
        self.state = MissionState.NAVIGATING

        # Mulai navigasi: aktifkan line tracker dan arahkan ke segmen pertama
        first_step = self.current_route[0]
        self.send_branch_instruction("RESUME")
        self.send_branch_instruction(first_step.branch_action)

        self.get_logger().info(
            f"Misi dimulai! {self.current_node} -> {self.target_node} "
            f"({len(self.current_route)} segmen). Menuju {first_step.to_node} (ArUco ID {first_step.target_aruco_id})"
        )

    def pause_mission(self):
        if self.state in (MissionState.NAVIGATING, MissionState.AT_INTERSECTION):
            self.state = MissionState.PAUSED
            self.send_branch_instruction("STOP")
            self.get_logger().info("Misi DIJEDA. Motor dihentikan.")

    def resume_mission(self):
        if self.state == MissionState.PAUSED and self.current_route:
            self.state = MissionState.NAVIGATING
            self.send_branch_instruction("RESUME")
            current_step = self.current_route[self.current_step_index]
            self.send_branch_instruction(current_step.branch_action)
            self.get_logger().info(f"Misi DILANJUTKAN menuju {current_step.to_node}.")

    def cancel_mission(self):
        self.state = MissionState.IDLE
        self.current_route = []
        self.target_node = None
        self.send_branch_instruction("STOP")
        self.get_logger().info("Misi DIBATALKAN. Kembali ke IDLE.")

    def aruco_callback(self, message: Int32):
        """Respons ketika marker ArUco terdeteksi dan terkonfirmasi."""
        aruco_id = message.data
        self.last_detected_aruco_id = aruco_id
        node_name = self.route_manager.get_node_by_aruco_id(aruco_id)

        self.get_logger().info(f"[ArUcoEvent] Terdeteksi ArUco ID {aruco_id} (Node: {node_name})")

        # Jika tidak sedang dalam misi navigasi, cukup update posisi saat ini
        if self.state not in (MissionState.NAVIGATING, MissionState.AT_INTERSECTION):
            if node_name:
                self.current_node = node_name
            return

        if self.current_step_index >= len(self.current_route):
            return

        expected_step = self.current_route[self.current_step_index]

        # Validasi apakah marker cocok dengan target segmen saat ini
        if aruco_id == expected_step.target_aruco_id:
            arrived_node = expected_step.to_node
            self.current_node = arrived_node

            if expected_step.target_type == "intersection":
                # Tiba di persimpangan
                self.state = MissionState.AT_INTERSECTION
                self.get_logger().info(
                    f"Persimpangan {arrived_node} tercapai! "
                    f"Mengeksekusi cabang: {expected_step.branch_action}"
                )

                # Kirim instruksi belok ke line tracker
                self.send_branch_instruction(expected_step.branch_action)

                # Melangkah ke segmen berikutnya
                self.current_step_index += 1
                if self.current_step_index < len(self.current_route):
                    next_step = self.current_route[self.current_step_index]
                    self.state = MissionState.NAVIGATING
                    self.get_logger().info(
                        f"Melanjutkan segmen berikutnya menuju {next_step.to_node} (ArUco ID {next_step.target_aruco_id})"
                    )

            elif expected_step.target_type == "checkpoint":
                # Tiba di titik checkpoint (tujuan akhir segmen atau tujuan misi)
                self.current_step_index += 1
                if self.current_step_index >= len(self.current_route):
                    # Target misi akhir tercapai!
                    self.state = MissionState.ARRIVED
                    self.send_branch_instruction("STOP")
                    self.get_logger().info(
                        f"Checkpoint Tujuan {arrived_node} TERCAPAI! Misi SELESAI dengan sukses."
                    )
                    self.current_route = []
                else:
                    # Checkpoint perantara (transit)
                    next_step = self.current_route[self.current_step_index]
                    self.send_branch_instruction(next_step.branch_action)
                    self.get_logger().info(f"Transit di {arrived_node}, melanjutkan ke {next_step.to_node}")
        else:
            self.get_logger().warn(
                f"Marker ArUco tidak sesuai rute: terbaca {aruco_id} ({node_name}), "
                f"diharapkan {expected_step.target_aruco_id} ({expected_step.to_node})"
            )

    def send_branch_instruction(self, action: str):
        """Kirim perintah ke line tracker (/line_tracker/branch_instruction)."""
        msg = String()
        msg.data = action
        self.branch_pub.publish(msg)

    def publish_status(self):
        """Publikasikan telemetri status misi secara periodik (JSON)."""
        next_node = None
        next_aruco = None
        branch_action = None
        remaining_steps = 0

        if self.current_route and self.current_step_index < len(self.current_route):
            step = self.current_route[self.current_step_index]
            next_node = step.to_node
            next_aruco = step.target_aruco_id
            branch_action = step.branch_action
            remaining_steps = len(self.current_route) - self.current_step_index

        payload = {
            "state": self.state.value,
            "current_node": self.current_node,
            "target_node": self.target_node,
            "next_node": next_node,
            "next_aruco_id": next_aruco,
            "branch_action": branch_action,
            "remaining_steps": remaining_steps,
            "last_aruco_id": self.last_detected_aruco_id,
            "x": round(self.current_x, 3),
            "y": round(self.current_y, 3),
        }

        msg = String()
        msg.data = json.dumps(payload)
        self.status_pub.publish(msg)


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
