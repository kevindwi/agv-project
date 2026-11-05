"""Marker-sequenced mission FSM for line-guided AGV routes."""

import json

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


class MissionControllerNode(Node):
    """Validate expected ArUco checkpoints and command the line tracker FSM."""

    def __init__(self):
        super().__init__("mission_controller_node")
        self.declare_parameter("route_json", "[]")
        self.declare_parameter("confirm_frames", 4)
        self.declare_parameter("marker_clear_frames", 4)
        self.declare_parameter("approach_lost_frames", 8)
        self.declare_parameter("control_topic", "/mission/control")
        self.declare_parameter("detections_topic", "/aruco/detections")

        self.route = json.loads(str(self.get_parameter("route_json").value))
        if not isinstance(self.route, list):
            raise ValueError("route_json harus berupa JSON array")
        self._validate_route()
        self.confirm_frames = max(1, int(self.get_parameter("confirm_frames").value))
        self.marker_clear_frames = max(1, int(self.get_parameter("marker_clear_frames").value))
        self.approach_lost_frames = max(
            1, int(self.get_parameter("approach_lost_frames").value)
        )
        self.route_index = 0
        self.state = "NAVIGATING" if self.route else "IDLE"
        self.candidate_id = None
        self.candidate_count = 0
        self.latched_id = None
        self.clear_count = 0
        self.approach_lost_count = 0
        self.approaching_marker = False
        self.tracker_status = ""

        self.control_publisher = self.create_publisher(
            String, str(self.get_parameter("control_topic").value), 10
        )
        self.state_publisher = self.create_publisher(String, "/mission/state", 10)
        self.create_subscription(
            String,
            str(self.get_parameter("detections_topic").value),
            self.detections_callback,
            10,
        )
        self.create_subscription(String, "/line_tracker/status", self.status_callback, 10)
        self.timer = self.create_timer(0.5, self.publish_state)
        if self.route:
            self._send_control("FOLLOW")
        self.get_logger().info(
            f"Mission FSM aktif: {len(self.route)} checkpoint dalam rute"
        )

    def _validate_route(self):
        seen = set()
        for index, waypoint in enumerate(self.route):
            if not isinstance(waypoint, dict) or "marker_id" not in waypoint:
                raise ValueError(f"Waypoint route[{index}] wajib punya marker_id")
            marker_id = int(waypoint["marker_id"])
            if marker_id in seen:
                raise ValueError(f"ID ArUco duplikat di rute: {marker_id}")
            seen.add(marker_id)
            action = str(waypoint.get("action", "straight")).lower()
            if action not in ("left", "right", "straight", "stop"):
                raise ValueError(f"Aksi waypoint tidak valid: {action}")
            waypoint["marker_id"] = marker_id
            waypoint["action"] = action
            trigger_distance = float(waypoint.get("trigger_distance_m", 0.35))
            if trigger_distance <= 0.0:
                raise ValueError(f"trigger_distance_m harus positif pada route[{index}]")
            waypoint["trigger_distance_m"] = trigger_distance

    def detections_callback(self, message):
        try:
            payload = json.loads(message.data)
            detections = payload.get("detections", [])
            visible_ids = sorted({int(item["id"]) for item in detections})
            poses = {int(item["id"]): item.get("pose_camera") for item in detections}
        except (ValueError, TypeError, KeyError, AttributeError) as error:
            self.get_logger().warning(f"Pesan deteksi ArUco tidak valid: {error}")
            return

        if self.state in ("ARRIVED", "ERROR", "IDLE"):
            return
        if self.latched_id is not None:
            if self.latched_id in visible_ids:
                self.clear_count = 0
                return
            self.clear_count += 1
            if self.clear_count < self.marker_clear_frames:
                return
            self.latched_id = None
            self.clear_count = 0

        if not visible_ids:
            self.candidate_id = None
            self.candidate_count = 0
            return
        expected = self.route[self.route_index]["marker_id"]
        route_ids = {waypoint["marker_id"] for waypoint in self.route}
        unknown_ids = [marker_id for marker_id in visible_ids if marker_id not in route_ids]
        if unknown_ids:
            self._fail(f"Marker tidak dikenal terlihat: {unknown_ids}")
            return
        if expected not in visible_ids:
            # Marker checkpoint berikutnya bisa terlihat dari jauh; tunggu ID yang
            # sedang dituju tanpa menghentikan robot untuk marker rute masa depan.
            self.candidate_id = None
            self.candidate_count = 0
            if self.approaching_marker:
                self.approach_lost_count += 1
                if self.approach_lost_count >= self.approach_lost_frames:
                    self._fail(f"Marker checkpoint {expected} hilang saat pendekatan")
            return
        self.approach_lost_count = 0
        marker_id = expected
        if marker_id == self.candidate_id:
            self.candidate_count += 1
        else:
            self.candidate_id = marker_id
            self.candidate_count = 1
        if self.candidate_count >= self.confirm_frames:
            self._approach_or_accept(marker_id, poses.get(marker_id))

    def _approach_or_accept(self, marker_id, pose):
        waypoint = self.route[self.route_index]
        trigger_distance = waypoint["trigger_distance_m"]
        if pose is not None:
            try:
                distance = float(pose["translation_m"][2])
            except (KeyError, IndexError, TypeError, ValueError):
                self._fail(f"Pose marker {marker_id} tidak valid")
                return
            if distance <= 0.0:
                self._fail(f"Pose marker {marker_id} berada di belakang kamera")
                return
            if distance > trigger_distance:
                if not self.approaching_marker:
                    self.get_logger().info(
                        f"Marker {marker_id} terdeteksi pada {distance:.2f} m; melambat"
                    )
                    self._send_control("SLOW")
                    self.state = "APPROACHING"
                    self.approaching_marker = True
                return
        elif not self.approaching_marker:
            self.get_logger().warning(
                f"Pose marker {marker_id} tidak tersedia; memakai pemicu ID tanpa jarak."
            )
        self._accept_waypoint(marker_id)

    def _accept_waypoint(self, marker_id):
        waypoint = self.route[self.route_index]
        action = waypoint["action"]
        self.latched_id = marker_id
        self.clear_count = 0
        self.approaching_marker = False
        self.approach_lost_count = 0
        self.candidate_count = 0
        self.get_logger().info(f"Checkpoint {marker_id} terverifikasi, aksi={action}")
        if action == "stop":
            self.state = "ARRIVED"
            self._send_control("STOP")
            return
        if action in ("left", "right"):
            self.state = "TURNING"
            self._send_control(action.upper())
            return
        self._advance_route()

    def status_callback(self, message):
        self.tracker_status = message.data
        if self.state == "TURNING" and message.data == "TURN_COMPLETE":
            self._advance_route()
        elif self.state == "TURNING" and message.data == "TURN_FAILED":
            self._fail("Garis cabang tidak berhasil ditemukan setelah belok")
        elif message.data == "LINE_LOST" and self.state in ("NAVIGATING", "APPROACHING"):
            self._fail("Garis hilang saat mengikuti lintasan")

    def _advance_route(self):
        self.route_index += 1
        if self.route_index >= len(self.route):
            self.state = "ARRIVED"
            self._send_control("STOP")
            return
        self.state = "NAVIGATING"
        self.candidate_id = None
        self.candidate_count = 0
        self.approaching_marker = False
        self.approach_lost_count = 0
        self._send_control("FOLLOW")

    def _fail(self, reason):
        if self.state == "ERROR":
            return
        self.get_logger().error(reason)
        self.state = "ERROR"
        self._send_control("STOP")

    def _send_control(self, command):
        self.control_publisher.publish(String(data=command))

    def publish_state(self):
        message = String()
        if self.state in ("ARRIVED", "ERROR", "IDLE"):
            current = "none"
        else:
            current = str(self.route[self.route_index]["marker_id"])
        message.data = json.dumps(
            {
                "state": self.state,
                "current_marker_id": current,
                "route_index": self.route_index,
                "tracker_status": self.tracker_status,
            },
            separators=(",", ":"),
        )
        self.state_publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = MissionControllerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node._send_control("STOP")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
