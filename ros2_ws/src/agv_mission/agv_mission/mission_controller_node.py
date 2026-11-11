"""State Machine Misi: mengendalikan rute AGV berdasarkan deteksi ArUco."""

import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from agv_interfaces.msg import ArucoDetection, MissionState


VALID_ACTIONS = ("straight", "left", "right", "stop")


class MissionControllerNode(Node):
    def __init__(self):
        super().__init__("mission_controller_node")

        # Parameter rute & deteksi
        self.declare_parameter("confirm_frames", 3)
        self.declare_parameter("trigger_distance_m", 0.35)
        self.declare_parameter("slow_distance_m", 0.70)
        self.declare_parameter("route_ids", [1, 2, 3])
        # Default harus berupa STRING_ARRAY (bukan list kosong) agar tipe parameter
        # tidak terinferensi sebagai BYTE_ARRAY.
        self.declare_parameter("route_actions", ["straight", "left", "stop"])
        self.declare_parameter("transit_hold_sec", 0.8)
        self.declare_parameter("detection_timeout_sec", 0.5)
        self.declare_parameter("control_topic", "/mission/control")
        self.declare_parameter("detection_topic", "/aruco/detection")
        self.declare_parameter("line_status_topic", "/line_follower/state")
        self.declare_parameter("fail_on_line_lost", False)

        self.confirm_frames = max(1, int(self.get_parameter("confirm_frames").value))
        self.trigger_distance = float(self.get_parameter("trigger_distance_m").value)
        self.slow_distance = float(self.get_parameter("slow_distance_m").value)
        self.transit_hold_sec = max(0.0, float(self.get_parameter("transit_hold_sec").value))
        self.detection_timeout = max(
            0.05, float(self.get_parameter("detection_timeout_sec").value)
        )
        self.fail_on_line_lost = bool(self.get_parameter("fail_on_line_lost").value)

        if self.trigger_distance <= 0.0:
            raise ValueError("trigger_distance_m harus positif")
        if self.slow_distance <= self.trigger_distance:
            raise ValueError("slow_distance_m harus lebih besar dari trigger_distance_m")

        self.route = [int(marker_id) for marker_id in self.get_parameter("route_ids").value]
        route_actions = [
            str(action).strip().lower()
            for action in self.get_parameter("route_actions").value
        ]
        if len(route_actions) != len(self.route):
            raise ValueError(
                "route_actions harus memiliki jumlah yang sama dengan route_ids "
                f"({len(route_actions)} != {len(self.route)})"
            )
        for index, action in enumerate(route_actions):
            if action not in VALID_ACTIONS:
                raise ValueError(
                    f"Aksi route_actions[{index}] tidak valid: {action}. "
                    f"Pilihan: {', '.join(VALID_ACTIONS)}"
                )
        self.route_actions = route_actions

        control_topic = str(self.get_parameter("control_topic").value)
        detection_topic = str(self.get_parameter("detection_topic").value)
        line_status_topic = str(self.get_parameter("line_status_topic").value)

        # State machine
        self.route_idx = 0
        self.state = "NAVIGATING" if self.route else "IDLE"
        self.candidate_id = None
        self.candidate_count = 0
        self.candidate_last_seen = 0.0
        self.visited_ids = set()
        self.transit_resume_at = None
        self.line_status = ""
        self.last_state_publish = 0.0

        # Publisher & Subscriber
        self.control_pub = self.create_publisher(String, control_topic, 10)
        self.state_pub = self.create_publisher(MissionState, "/mission/state", 10)

        self.create_subscription(
            ArucoDetection, detection_topic, self.detection_callback, 10
        )
        self.create_subscription(
            String, line_status_topic, self.status_callback, 10
        )

        # Tick 10 Hz: timeout debouncing, jeda transit, dan publish status
        self.timer = self.create_timer(0.1, self.tick)

        # Beri perintah jalan awal jika ada rute
        if self.route:
            self._send_control("FOLLOW")
            self.get_logger().info(
                f"Misi dimulai! Rute ArUco: {list(zip(self.route, self.route_actions))}"
            )
        else:
            self.get_logger().info("Route kosong; misi dalam keadaan IDLE.")

    def detection_callback(self, msg: ArucoDetection):
        if self.state in ("ARRIVED", "ERROR", "IDLE"):
            return

        if self.route_idx >= len(self.route):
            self._arrive()
            return

        target_id = self.route[self.route_idx]

        # Latch: marker yang sudah pernah dilewati tidak boleh memicu ulang
        if msg.id in self.visited_ids:
            return

        # Abaikan marker lain (mis. checkpoint yang belum menjadi target)
        if msg.id != target_id:
            return

        distance_m = float(msg.distance_m)
        if distance_m <= 0.05:
            self.get_logger().warn(
                f"Jarak marker {msg.id} tidak valid ({distance_m:.3f} m); diabaikan.",
                throttle_duration_sec=2.0,
            )
            return

        now = time.monotonic()
        if (
            msg.id == self.candidate_id
            and (now - self.candidate_last_seen) <= self.detection_timeout
        ):
            self.candidate_count += 1
        else:
            self.candidate_id = int(msg.id)
            self.candidate_count = 1
        self.candidate_last_seen = now

        # Debouncing: butuh confirm_frames deteksi berturut-turut
        if self.candidate_count < self.confirm_frames:
            return

        self._handle_checkpoint(int(msg.id), distance_m)

    def _handle_checkpoint(self, marker_id: int, distance_m: float):
        # Jika masih cukup jauh, perintahkan robot melambat
        if distance_m > self.trigger_distance:
            if (
                distance_m <= self.slow_distance
                and self.state not in ("APPROACHING", "TRANSIT", "TURNING")
            ):
                self.get_logger().info(
                    f"Checkpoint {marker_id} terlihat pada {distance_m:.2f} m. Melambat..."
                )
                self.state = "APPROACHING"
                self._send_control("SLOW")
                self.publish_state()
            return

        # Jarak sudah mencapai batas pemicu (<= trigger_distance)
        self.get_logger().info(
            f"Checkpoint {marker_id} TERCAPAI pada jarak {distance_m:.2f} m!"
        )
        self.visited_ids.add(marker_id)
        self.candidate_id = None
        self.candidate_count = 0

        action = self.route_actions[self.route_idx]

        if action == "stop" or self.route_idx == len(self.route) - 1:
            self._arrive()
            return

        if action in ("left", "right"):
            # Latch sudah aktif; line follower akan berputar lalu mencari garis baru
            self.state = "TURNING"
            self.get_logger().info(
                f"Checkpoint {marker_id}: manuver belok {action.upper()}."
            )
            self._send_control(action.upper())
            self.publish_state()
            return

        # action == "straight": transit singkat lalu lanjut
        if self.transit_hold_sec > 0.0:
            self.state = "TRANSIT"
            self._send_control("SLOW")
            self.transit_resume_at = time.monotonic() + self.transit_hold_sec
            self.get_logger().info(
                f"Checkpoint {marker_id}: transit {self.transit_hold_sec:.1f} detik."
            )
            self.publish_state()
        else:
            self._advance_route()

    def status_callback(self, msg: String):
        status = msg.data.strip().upper()
        self.line_status = status

        if self.state == "TURNING":
            if status == "TURN_COMPLETE":
                self._advance_route()
            elif status == "TURN_FAILED":
                self._fail("Garis cabang tidak berhasil ditemukan setelah belok")
        elif (
            status == "LINE_LOST"
            and self.fail_on_line_lost
            and self.state in ("NAVIGATING", "APPROACHING", "TRANSIT")
        ):
            self._fail("Garis hilang saat mengikuti lintasan")

    def _advance_route(self):
        self.route_idx += 1
        self.candidate_id = None
        self.candidate_count = 0
        self.transit_resume_at = None

        if self.route_idx >= len(self.route):
            self._arrive()
            return

        self.state = "NAVIGATING"
        self.get_logger().info(
            f"Lanjut ke checkpoint berikutnya: ID {self.route[self.route_idx]} "
            f"({self.route_actions[self.route_idx]})"
        )
        self._send_control("FOLLOW")
        self.publish_state()

    def _arrive(self):
        if self.state == "ARRIVED":
            return
        self.state = "ARRIVED"
        self.transit_resume_at = None
        self.get_logger().info("Semua checkpoint selesai! Robot berhenti.")
        self._send_control("STOP")
        self.publish_state()

    def _fail(self, reason: str):
        if self.state == "ERROR":
            return
        self.get_logger().error(reason)
        self.state = "ERROR"
        self.transit_resume_at = None
        self._send_control("STOP")
        self.publish_state()

    def _send_control(self, cmd: str):
        self.control_pub.publish(String(data=cmd))

    def tick(self):
        now = time.monotonic()

        # Debouncing: reset bila marker target tidak terlihat cukup lama
        if (
            self.candidate_id is not None
            and now - self.candidate_last_seen > self.detection_timeout
        ):
            self.candidate_id = None
            self.candidate_count = 0

        # Jeda transit CP1 selesai -> lanjut ke checkpoint berikutnya
        if self.transit_resume_at is not None and now >= self.transit_resume_at:
            self.transit_resume_at = None
            self._advance_route()

        # Publish status misi 2 Hz
        if now - self.last_state_publish >= 0.5:
            self.publish_state()
            self.last_state_publish = now

    def publish_state(self):
        msg = MissionState()
        msg.state = self.state
        if self.state in ("ARRIVED", "ERROR", "IDLE") or self.route_idx >= len(self.route):
            msg.current_waypoint = -1
        else:
            msg.current_waypoint = self.route[self.route_idx]
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
