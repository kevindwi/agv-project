"""Mission Manager Node — Finite State Machine untuk navigasi lintasan H-track.

Node ini bertindak sebagai "otak" robot. Ia mengatur kapan robot harus mengikuti
garis (line tracking), kapan harus berhenti di persimpangan, dan kapan harus
berbelok menggunakan odometri. Arsitektur ini menghindari Nav2/AMCL/LiDAR.

Topologi komunikasi
-------------------
Subscriptions:
    /line_tracker/status  (std_msgs/String) : status dari line_tracker_node
        "ON_LINE"              -> robot di atas garis, PID aktif
        "INTERSECTION_DETECTED"-> persimpangan terdeteksi di kamera
        "LINE_LOST"            -> garis hilang
    /odom                 (nav_msgs/Odometry) : pose robot dari odometry_node

Publications:
    /mission_manager/control  (std_msgs/String) : "PAUSE" / "RESUME"
        -> dikirim ke line_tracker_node untuk on/off PID
    /mission_manager/cmd_vel  (geometry_msgs/Twist)
        -> dikirim ke line_tracker_node sebagai override saat PAUSE
        -> line_tracker_node meneruskan ke serial firmware
    /mission_manager/state    (std_msgs/String)
        -> nama state FSM saat ini untuk monitoring/debugging

Misi H-track (konfigurasi default)
------------------------------------
Lintasan berbentuk H:

    |   |        <- Dua garis vertikal atas (entry)
    |___|        <- Crossbar horizontal (persimpangan)
    |   |        <- Dua garis vertikal bawah
    |___|        <- Bagian bawah kotak (persimpangan sudut)

Misi default: masuk dari atas, lewati crossbar lurus, navigasi loop bawah,
kembali ke atas. Setiap persimpangan memicu aksi dari ``mission_waypoints``.

Cara mengubah misi
------------------
Edit parameter ``mission_waypoints`` di YAML atau langsung di kode.
Contoh untuk lintasan sederhana lurus-belok kanan-berhenti:

    mission_waypoints:
      - ["CP1", "TURN_RIGHT"]
      - ["GOAL", "STOP"]

Aksi yang didukung per waypoint:
    "CONTINUE"    - lewati persimpangan lurus (line tracking tetap aktif)
    "TURN_LEFT"   - belok kiri 90° menggunakan odometri (line tracking off)
    "TURN_RIGHT"  - belok kanan 90° menggunakan odometri (line tracking off)
    "STOP"        - hentikan misi (GOAL_REACHED)
"""

import math
from enum import Enum, auto

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

# ---------------------------------------------------------------------------
# State Machine
# ---------------------------------------------------------------------------


class MissionState(Enum):
    """State-state FSM Mission Manager."""

    IDLE = auto()  # Menunggu; belum ada misi aktif
    FOLLOW_LINE = auto()  # PID line tracking aktif
    AT_INTERSECTION = auto()  # Persimpangan terdeteksi; memilih aksi
    TURNING = auto()  # Berbelok menggunakan odometri (PID off)
    GOAL_REACHED = auto()  # Misi selesai; robot berhenti


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------


class MissionManagerNode(Node):
    """State machine tingkat misi untuk lintasan H-track tanpa LiDAR/Nav2."""

    def __init__(self):
        super().__init__("mission_manager_node")

        # ---- Parameter -------------------------------------------------------
        self.declare_parameter("turn_speed_radps", 0.5)
        self.declare_parameter("turn_angle_deg", 90.0)
        self.declare_parameter("turn_angle_tolerance_deg", 3.0)
        self.declare_parameter("intersection_debounce_distance_m", 0.30)
        self.declare_parameter("auto_start", True)

        self._turn_speed: float = self.get_parameter("turn_speed_radps").value
        self._turn_angle: float = math.radians(
            self.get_parameter("turn_angle_deg").value
        )
        self._turn_tolerance: float = math.radians(
            self.get_parameter("turn_angle_tolerance_deg").value
        )
        self._debounce_dist: float = self.get_parameter(
            "intersection_debounce_distance_m"
        ).value
        auto_start: bool = self.get_parameter("auto_start").value

        # ---- Misi H-track (urutan aksi di setiap persimpangan) ---------------
        # Format: list of [checkpoint_id: str, action: str]
        # Urutan sesuai dengan urutan persimpangan yang ditemui robot.
        # Lintasan H-track (masuk dari kiri atas, searah jarum jam):
        #   - Persimpangan 1: crossbar tengah -> lurus ke bawah
        #   - Persimpangan 2: sudut kiri bawah -> belok kanan
        #   - Persimpangan 3: sudut kanan bawah -> belok kanan
        #   - Persimpangan 4: crossbar tengah (dari bawah) -> lurus ke atas
        #   - Selesai di kanan atas
        self._mission_waypoints: list[tuple[str, str]] = [
            # ("CP_crossbar_turun",  "CONTINUE"),
            ("CP_sudut_kiri_bawah", "TURN_RIGHT"),
            ("CP_sudut_kanan_bawah", "TURN_RIGHT"),
            ("CP_crossbar_naik", "CONTINUE"),
            ("GOAL", "STOP"),
        ]
        self._waypoint_index: int = 0

        # ---- State & odometri ------------------------------------------------
        self._state: MissionState = MissionState.IDLE
        self._turn_direction: int = 0  # +1 = kiri (CCW), -1 = kanan (CW)
        self._turn_start_yaw: float = 0.0
        self._current_yaw: float = 0.0
        self._current_x: float = 0.0
        self._current_y: float = 0.0
        self._last_intersection_x: float = -99.0
        self._last_intersection_y: float = -99.0

        # ---- Subscriber -------------------------------------------------------
        self.create_subscription(
            String, "/line_tracker/status", self._on_line_status, 10
        )
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)

        # ---- Publisher --------------------------------------------------------
        self._control_pub = self.create_publisher(
            String, "/mission_manager/control", 10
        )
        self._cmd_vel_pub = self.create_publisher(Twist, "/mission_manager/cmd_vel", 10)
        self._state_pub = self.create_publisher(String, "/mission_manager/state", 10)

        # ---- Timer utama (20 Hz) ---------------------------------------------
        self.create_timer(0.05, self._update)

        self.get_logger().info("Mission Manager Node aktif. State: IDLE")
        self.get_logger().info(
            f"Misi: {[f'{wp[0]}={wp[1]}' for wp in self._mission_waypoints]}"
        )

        if auto_start:
            self._transition_to(MissionState.FOLLOW_LINE)

    # =========================================================================
    # Subscriber callbacks
    # =========================================================================

    def _on_line_status(self, msg: String) -> None:
        """Terima status garis dari line_tracker_node."""
        if self._state != MissionState.FOLLOW_LINE:
            return  # Hanya proses saat sedang follow line

        if msg.data == "INTERSECTION_DETECTED":
            dist = math.hypot(
                self._current_x - self._last_intersection_x,
                self._current_y - self._last_intersection_y,
            )
            if dist >= self._debounce_dist:
                self._last_intersection_x = self._current_x
                self._last_intersection_y = self._current_y
                self._transition_to(MissionState.AT_INTERSECTION)

    def _on_odom(self, msg: Odometry) -> None:
        """Update pose robot dari odometry_node."""
        self._current_x = msg.pose.pose.position.x
        self._current_y = msg.pose.pose.position.y
        # Ekstrak yaw dari quaternion (REP 103: z-up, yaw = rotasi di bidang XY)
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._current_yaw = math.atan2(siny_cosp, cosy_cosp)

    # =========================================================================
    # Update loop (20 Hz)
    # =========================================================================

    def _update(self) -> None:
        """Loop utama FSM — dipanggil oleh timer 20 Hz."""
        if self._state == MissionState.TURNING:
            self._update_turning()

        # Selalu publikasikan nama state untuk monitoring
        self._state_pub.publish(String(data=self._state.name))

    def _update_turning(self) -> None:
        """Periksa apakah belokan sudah selesai berdasarkan delta yaw odometri."""
        yaw_delta = _angle_diff(self._current_yaw, self._turn_start_yaw)
        target_delta = self._turn_direction * self._turn_angle  # bertanda

        remaining = abs(target_delta) - abs(yaw_delta)

        if remaining <= self._turn_tolerance:
            # Belokan selesai -> berhenti lalu lanjut ikut garis
            self._publish_cmd_vel(0.0, 0.0)
            self.get_logger().info(
                f"Belokan selesai. Delta yaw: {math.degrees(yaw_delta):.1f} deg"
            )
            self._transition_to(MissionState.FOLLOW_LINE)
        else:
            # Terus berputar
            angular = self._turn_direction * self._turn_speed
            self._publish_cmd_vel(0.0, angular)

    # =========================================================================
    # State transitions
    # =========================================================================

    def _transition_to(self, new_state: MissionState) -> None:
        """Pindah ke state baru dan jalankan aksi masuk (entry action)."""
        old_state = self._state
        self._state = new_state
        self.get_logger().info(f"State: {old_state.name} -> {new_state.name}")

        if new_state == MissionState.FOLLOW_LINE:
            # Aktifkan PID line tracking di line_tracker_node
            self._send_control("RESUME")

        elif new_state == MissionState.AT_INTERSECTION:
            # Hentikan PID; eksekusi instruksi waypoint
            self._send_control("PAUSE")
            self._publish_cmd_vel(0.0, 0.0)  # Pastikan robot berhenti sejenak
            self._handle_intersection()

        elif new_state == MissionState.TURNING:
            # PID sudah di-PAUSE dari AT_INTERSECTION; catat yaw awal
            self._turn_start_yaw = self._current_yaw

        elif new_state == MissionState.GOAL_REACHED:
            self._send_control("PAUSE")
            self._publish_cmd_vel(0.0, 0.0)
            self.get_logger().info("GOAL REACHED! Misi selesai.")

    def _handle_intersection(self) -> None:
        """Ambil aksi dari daftar waypoint dan jalankan."""
        if self._waypoint_index >= len(self._mission_waypoints):
            self.get_logger().warn(
                "Persimpangan terdeteksi tapi waypoint sudah habis. "
                "Lanjutkan ikut garis."
            )
            self._transition_to(MissionState.FOLLOW_LINE)
            return

        wp_id, action = self._mission_waypoints[self._waypoint_index]
        self._waypoint_index += 1
        self.get_logger().info(
            f"Waypoint [{self._waypoint_index}/{len(self._mission_waypoints)}] "
            f"ID={wp_id} -> Aksi: {action}"
        )

        if action == "TURN_LEFT":
            self._turn_direction = +1  # +angular = CCW = kiri
            self._transition_to(MissionState.TURNING)

        elif action == "TURN_RIGHT":
            self._turn_direction = -1  # -angular = CW = kanan
            self._transition_to(MissionState.TURNING)

        elif action == "CONTINUE":
            # Lurus melalui persimpangan; aktifkan kembali line tracking
            self._transition_to(MissionState.FOLLOW_LINE)

        elif action == "STOP":
            self._transition_to(MissionState.GOAL_REACHED)

        else:
            self.get_logger().warn(
                f"Aksi tidak dikenal: '{action}'. Lanjutkan ikut garis."
            )
            self._transition_to(MissionState.FOLLOW_LINE)

    # =========================================================================
    # Publisher helpers
    # =========================================================================

    def _send_control(self, cmd: str) -> None:
        """Kirim perintah PAUSE / RESUME ke line_tracker_node."""
        self._control_pub.publish(String(data=cmd))

    def _publish_cmd_vel(self, linear: float, angular: float) -> None:
        """Publikasikan cmd_vel override (diteruskan ke serial oleh line_tracker_node)."""
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self._cmd_vel_pub.publish(msg)


# ---------------------------------------------------------------------------
# Utility
# ---------------------------------------------------------------------------


def _angle_diff(a: float, b: float) -> float:
    """Selisih sudut bertanda (a - b), dibungkus ke rentang [-pi, pi]."""
    diff = a - b
    while diff > math.pi:
        diff -= 2.0 * math.pi
    while diff < -math.pi:
        diff += 2.0 * math.pi
    return diff


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def main(args=None):
    rclpy.init(args=args)
    node = MissionManagerNode()
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
