import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("agv_bringup")
    camera_config = os.path.join(pkg_share, "config", "camera_params.yaml")
    robot_config = os.path.join(pkg_share, "config", "robot_params.yaml")

    camera_node = Node(
        package="agv_bringup",
        executable="camera_node",
        name="camera_node",
        output="screen",
        parameters=[camera_config],
    )

    serial_bridge_node = Node(
        package="agv_bringup",
        executable="serial_bridge_node",
        name="serial_bridge_node",
        output="screen",
        parameters=[robot_config],
    )

    return LaunchDescription([
        camera_node,
        serial_bridge_node,
    ])
