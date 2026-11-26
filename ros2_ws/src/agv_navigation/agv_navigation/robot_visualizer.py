"""Helper visualisasi bentuk fisik 3D robot di RViz."""

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray


def create_robot_markers(
    stamp, base_frame_id: str, wheel_separation: float, wheel_diameter: float
) -> MarkerArray:
    """Buat visualisasi bentuk fisik robot dan panah arah hadap di RViz."""
    marker_array = MarkerArray()

    # Marker 1: Bodi / Sasis Robot (Box biru transparan)
    chassis = Marker()
    chassis.header.stamp = stamp
    chassis.header.frame_id = base_frame_id
    chassis.ns = "robot_model"
    chassis.id = 0
    chassis.type = Marker.CUBE
    chassis.action = Marker.ADD
    chassis.pose.position.x = 0.0
    chassis.pose.position.y = 0.0
    chassis.pose.position.z = 0.08  # Posisi tengah ketinggian sasis
    chassis.pose.orientation.w = 1.0
    chassis.scale.x = 0.50  # Panjang 50 cm
    chassis.scale.y = 0.45  # Lebar 45 cm
    chassis.scale.z = 0.12  # Tinggi 12 cm
    chassis.color.r = 0.1
    chassis.color.g = 0.5
    chassis.color.b = 0.9
    chassis.color.a = 0.75
    marker_array.markers.append(chassis)

    # Marker 2: Panah Arah Hadap Robot (Arrow kuning terang menunjuk ke depan +X)
    heading_arrow = Marker()
    heading_arrow.header.stamp = stamp
    heading_arrow.header.frame_id = base_frame_id
    heading_arrow.ns = "robot_heading"
    heading_arrow.id = 1
    heading_arrow.type = Marker.ARROW
    heading_arrow.action = Marker.ADD
    heading_arrow.points = [
        Point(x=0.0, y=0.0, z=0.15),
        Point(x=0.45, y=0.0, z=0.15),
    ]
    heading_arrow.scale.x = 0.05  # Diameter batang
    heading_arrow.scale.y = 0.10  # Diameter kepala panah
    heading_arrow.scale.z = 0.10  # Panjang kepala panah
    heading_arrow.color.r = 1.0
    heading_arrow.color.g = 0.9
    heading_arrow.color.b = 0.0
    heading_arrow.color.a = 1.0
    marker_array.markers.append(heading_arrow)

    # Marker 3: Roda Kiri (Cylinder hitam di sisi kiri)
    wheel_left = Marker()
    wheel_left.header.stamp = stamp
    wheel_left.header.frame_id = base_frame_id
    wheel_left.ns = "robot_wheels"
    wheel_left.id = 2
    wheel_left.type = Marker.CYLINDER
    wheel_left.action = Marker.ADD
    wheel_left.pose.position.x = 0.0
    wheel_left.pose.position.y = wheel_separation / 2.0
    wheel_left.pose.position.z = wheel_diameter / 2.0
    wheel_left.pose.orientation.x = 0.7071
    wheel_left.pose.orientation.w = 0.7071
    wheel_left.scale.x = wheel_diameter
    wheel_left.scale.y = wheel_diameter
    wheel_left.scale.z = 0.05
    wheel_left.color.r = 0.15
    wheel_left.color.g = 0.15
    wheel_left.color.b = 0.15
    wheel_left.color.a = 1.0
    marker_array.markers.append(wheel_left)

    # Marker 4: Roda Kanan (Cylinder hitam di sisi kanan)
    wheel_right = Marker()
    wheel_right.header.stamp = stamp
    wheel_right.header.frame_id = base_frame_id
    wheel_right.ns = "robot_wheels"
    wheel_right.id = 3
    wheel_right.type = Marker.CYLINDER
    wheel_right.action = Marker.ADD
    wheel_right.pose.position.x = 0.0
    wheel_right.pose.position.y = -wheel_separation / 2.0
    wheel_right.pose.position.z = wheel_diameter / 2.0
    wheel_right.pose.orientation.x = 0.7071
    wheel_right.pose.orientation.w = 0.7071
    wheel_right.scale.x = wheel_diameter
    wheel_right.scale.y = wheel_diameter
    wheel_right.scale.z = 0.05
    wheel_right.color.r = 0.15
    wheel_right.color.g = 0.15
    wheel_right.color.b = 0.15
    wheel_right.color.a = 1.0
    marker_array.markers.append(wheel_right)

    return marker_array
