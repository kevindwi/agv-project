"""State Machine Misi: mengendalikan rute AGV berdasarkan deteksi ArUco."""

import math
import time

from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from agv_interfaces.msg import ArucoDetection, MissionState
from .route_manager import RouteManager


class MissionControllerNode(Node):
    def __init__(self):
        super().__init__("mission_controller_node")

        # Parameter rute & deteksi
        self.declare_parameter("confirm_frames", 3)
        self.declare_parameter("trigger_distance_m", 0.35)
        self.declare_parameter("creep_distance_m", 0.66)
        self.declare_parameter("use_dynamic_creep", True)
        self.declare_parameter("camera_offset_m", 0.45)
        self.declare_parameter("camera_pitch_deg", 45.0)
        self.declare_parameter("center_tolerance_m", 0.05)
        self.declare_parameter("slow_distance_m", 0.70)
        self.declare_parameter("route_ids", [1, 2, 3])
        self.declare_parameter("route_actions", ["straight", "left", "stop"])
        self.declare_parameter("transit_hold_sec", 0.8)
        self.declare_parameter("detection_timeout_sec", 0.5)
        self.declare_parameter("control_topic", "/mission/control")
        self.declare_parameter("detection_topic", "/aruco/detection")
        self.declare_parameter("line_status_topic", "/line_follower/state")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("fail_on_line_lost", False)
        # New optional parameters for no‑slow‑approach mode
        self.declare_parameter("slow_down_on_approach", False)
        self.declare_parameter("creep_with_slow_speed", False)

        trigger_dist = float(self.get_parameter("trigger_distance_m").value)
        slow_dist = float(self.get_parameter("slow_distance_m").value)
        if trigger_dist <= 0.0:
            raise ValueError("trigger_distance_m harus positif")
        if slow_dist <= trigger_dist:
            raise ValueError("slow_distance_m harus lebih besar dari trigger_distance_m")

        self.creep_distance = max(0.0, float(self.get_parameter("creep_distance_m").value))
        self.use_dynamic_creep = bool(self.get_parameter("use_dynamic_creep").value)
        self.camera_offset_m = max(0.0, float(self.get_parameter("camera_offset_m").value))
        self.camera_pitch_deg = float(self.get_parameter("camera_pitch_deg").value)
        self.transit_hold_sec = max(
            0.0, float(self.get_parameter("transit_hold_sec").value)
        )
        self.fail_on_line_lost = bool(self.get_parameter("fail_on_line_lost").value)
        # Read optional parameters for no‑slow‑approach mode
        self.slow_down_on_approach = bool(self.get_parameter("slow_down_on_approach").value)
        self.creep_with_slow_speed = bool(self.get_parameter("creep_with_slow_speed").value)

        # Pengelola rute & debounce
        self.route_mgr = RouteManager(
            route_ids=[int(x) for x in self.get_parameter("route_ids").value],
            route_actions=[
                str(x) for x in self.get_parameter("route_actions").value
            ],
            confirm_frames=int(self.get_parameter("confirm_frames").value),
            detection_timeout_sec=float(
                self.get_parameter("detection_timeout_sec").value
            ),
            trigger_distance_m=trigger_dist,
            slow_distance_m=slow_dist,
            center_tolerance_m=float(
                self.get_parameter("center_tolerance_m").value
            ),
        )

        control_topic = str(self.get_parameter("control_topic").value)
        detection_topic = str(self.get_parameter("detection_topic").value)
        line_status_topic = str(self.get_parameter("line_status_topic").value)
        odom_topic = str(self.get_parameter("odom_topic").value)

        # State machine
        self.state = "NAVIGATING" if self.route_mgr.route else "IDLE"
        self.transit_resume_at = None
        self.line_status = ""
        self.last_state_publish = 0.0

        # Odometry tracking & Creeping
        self.current_x = 0.0
        self.current_y = 0.0
        self.odom_received = False
        self.creep_start_x = 0.0
        self.creep_start_y = 0.0
        self.active_creep_distance = self.creep_distance
        self.creep_pending_action = None
        self.creep_pending_marker_id = None

        # Publisher & Subscriber
        self.control_pub = self.create_publisher(String, control_topic, 10)
        self.state_pub = self.create_publisher(MissionState, "/mission/state", 10)

        self.create_subscription(
            ArucoDetection, detection_topic, self.detection_callback, 10
        )
        self.create_subscription(
            String, line_status_topic, self.status_callback, 10
        )
        self.create_subscription(
            Odometry, odom_topic, self.odom_callback, 10
        )

        self.timer = self.create_timer(0.1, self.tick)

        if self.route_mgr.route:
            self._send_control("FOLLOW")
            self.get_logger().info(
                f"Misi dimulai! Rute ArUco: {list(zip(self.route_mgr.route, self.route_mgr.actions))} "
                f"| Creep Offset: {self.creep_distance:.2f} m"
            )
        else:
            self.get_logger().info("Route kosong; misi dalam keadaan IDLE.")

    def odom_callback(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.odom_received = True

        if self.state == "CREEPING":
            self._check_creep_progress()

    def detection_callback(self, msg: ArucoDetection):
        if self.state in ("ARRIVED", "ERROR", "IDLE", "CREEPING"):
            return

        if self.route_mgr.is_completed:
            self._arrive()
            return

        # Ambil offset translation X jika pose terlampir
        offset_x = None
        if hasattr(msg, "pose_camera"):
            pose = getattr(msg, "pose_camera")
            if isinstance(pose, dict):
                trans = pose.get("translation_m")
                if isinstance(trans, (list, tuple)) and len(trans) >= 1:
                    offset_x = trans[0]

        event, action = self.route_mgr.process_detection(
            int(msg.id), float(msg.distance_m), offset_x
        )

        if event == "NEED_CENTERING":
            self._send_control("FOLLOW")
            return

        if event == "SLOW":
            if self.slow_down_on_approach and self.state not in ("APPROACHING", "TRANSIT", "TURNING", "CREEPING"):
                self.get_logger().info(
                    f"Checkpoint {msg.id} terlihat pada {msg.distance_m:.2f} m. Melambat..."
                )
                self.state = "APPROACHING"
                cmd = "SLOW" if self.creep_with_slow_speed else "FOLLOW"
                self._send_control(cmd)
                self.publish_state()
            # If slow_down_on_approach is False, treat as normal FOLLOW (no slow)
            return

        if event == "TRIGGER" and action is not None:
            if self.use_dynamic_creep and float(msg.distance_m) > 0.0:
                floor_dist = float(msg.distance_m) * math.cos(
                    math.radians(self.camera_pitch_deg)
                )
                target_creep = max(0.0, floor_dist + self.camera_offset_m)
            else:
                target_creep = self.creep_distance

            # Jika target_creep > 0 dan odom aktif, rayap maju sebelum eksekusi aksi
            if target_creep > 0.0 and self.odom_received:
                self.state = "CREEPING"
                self.creep_start_x = self.current_x
                self.creep_start_y = self.current_y
                self.active_creep_distance = target_creep
                self.creep_pending_action = action
                self.creep_pending_marker_id = int(msg.id)
                self.get_logger().info(
                    f"Checkpoint {msg.id} terpicu pada {msg.distance_m:.2f} m! "
                    f"Merayap maju {self.active_creep_distance:.2f} m "
                    f"menuju persimpangan sebelum aksi '{action}'..."
                )
                cmd = "SLOW" if self.creep_with_slow_speed else "FOLLOW"
                self._send_control(cmd)
                self.publish_state()
            else:
                self._execute_checkpoint_action(int(msg.id), action)

    def _check_creep_progress(self):
        dx = self.current_x - self.creep_start_x
        dy = self.current_y - self.creep_start_y
        dist_traveled = math.hypot(dx, dy)

        if dist_traveled >= self.active_creep_distance:
            self.get_logger().info(
                f"Perayapan selesai ({dist_traveled:.2f} m >= {self.active_creep_distance:.2f} m). "
                f"Tiba tepat di persimpangan! Mengeksekusi aksi: {self.creep_pending_action}"
            )
            action = self.creep_pending_action
            marker_id = self.creep_pending_marker_id
            self.creep_pending_action = None
            self.creep_pending_marker_id = None
            self._execute_checkpoint_action(marker_id, action)

    def _execute_checkpoint_action(self, marker_id: int, action: str):
        self.get_logger().info(f"Checkpoint {marker_id} TERCAPAI! Aksi: {action}")
        if action == "stop":
            self._arrive()
            return

        if action in ("left", "right"):
            self.state = "TURNING"
            self._send_control(action.upper())
            self.publish_state()
            return

        # action == "straight"
        if self.transit_hold_sec > 0.0:
            self.state = "TRANSIT"
            self._send_control("SLOW")
            self.transit_resume_at = time.monotonic() + self.transit_hold_sec
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
            and self.state in ("NAVIGATING", "APPROACHING", "TRANSIT", "CREEPING")
        ):
            self._fail("Garis hilang saat mengikuti lintasan")

    def _advance_route(self):
        has_next = self.route_mgr.advance()
        self.transit_resume_at = None
        self.creep_pending_action = None
        self.creep_pending_marker_id = None

        if not has_next:
            self._arrive()
            return

        self.state = "NAVIGATING"
        self.get_logger().info(
            f"Lanjut ke checkpoint: ID {self.route_mgr.current_waypoint} ({self.route_mgr.current_action})"
        )
        self._send_control("FOLLOW")
        self.publish_state()

    def _arrive(self):
        if self.state == "ARRIVED":
            return
        self.state = "ARRIVED"
        self.transit_resume_at = None
        self.creep_pending_action = None
        self.get_logger().info("Semua checkpoint selesai! Robot berhenti tepat di persimpangan.")
        self._send_control("STOP")
        self.publish_state()

    def _fail(self, reason: str):
        if self.state == "ERROR":
            return
        self.get_logger().error(reason)
        self.state = "ERROR"
        self.transit_resume_at = None
        self.creep_pending_action = None
        self._send_control("STOP")
        self.publish_state()

    def _send_control(self, cmd: str):
        self.control_pub.publish(String(data=cmd))

    def tick(self):
        now = time.monotonic()
        self.route_mgr.check_timeout(now)

        if self.state == "CREEPING":
            self._check_creep_progress()

        if self.transit_resume_at is not None and now >= self.transit_resume_at:
            self.transit_resume_at = None
            self._advance_route()

        if now - self.last_state_publish >= 0.5:
            self.publish_state()
            self.last_state_publish = now

    def publish_state(self):
        msg = MissionState()
        msg.state = self.state
        if self.state in ("ARRIVED", "ERROR", "IDLE"):
            msg.current_waypoint = -1
        else:
            msg.current_waypoint = self.route_mgr.current_waypoint
        self.state_pub.publish(msg)

    def destroy_node(self):
        try:
            self._send_control("STOP")
        except Exception:
            pass
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
