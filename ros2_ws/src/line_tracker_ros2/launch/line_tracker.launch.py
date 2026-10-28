"""Launch file untuk node line tracker."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    config_file = LaunchConfiguration("config_file").perform(context)

    # Hanya parameter yang dideklarasikan dan digunakan oleh node.
    overrides = {}
    parameter_types = {
        "camera_index": int,
        "serial_port": str,
        "baud_rate": int,
        "kp": float,
        "ki": float,
        "kd": float,
    }

    for key, value_type in parameter_types.items():
        val = LaunchConfiguration(key).perform(context).strip()
        if val != "":
            overrides[key] = value_type(val)

    node_params = [config_file]
    if overrides:
        node_params.append(overrides)

    node = Node(
        package="line_tracker_ros2",
        executable="line_tracker_node",
        name="line_tracker_node",
        output="screen",
        emulate_tty=True,
        parameters=node_params,
    )

    odometry_node = Node(
        package="line_tracker_ros2",
        executable="odometry_node",
        name="odometry_node",
        output="screen",
        parameters=[config_file],
    )

    return [node, odometry_node]


def generate_launch_description():
    package_share = get_package_share_directory("line_tracker_ros2")
    default_config_path = os.path.join(package_share, "config", "line_tracker.yaml")

    launch_args = [
        DeclareLaunchArgument(
            "config_file",
            default_value=default_config_path,
            description="Path ke file konfigurasi YAML",
        ),
        DeclareLaunchArgument(
            "camera_index",
            default_value="",
            description="Index perangkat V4L2 (/dev/videoN), meng-override YAML jika diisi",
        ),
        DeclareLaunchArgument(
            "serial_port",
            default_value="",
            description="Port serial adaptor UART, meng-override YAML jika diisi",
        ),
        DeclareLaunchArgument(
            "baud_rate",
            default_value="",
            description="Baud rate komunikasi serial, meng-override YAML jika diisi",
        ),
        DeclareLaunchArgument(
            "kp",
            default_value="",
            description="Gain proportional PID line tracker, meng-override YAML jika diisi",
        ),
        DeclareLaunchArgument(
            "ki",
            default_value="",
            description="Gain Integral PID, meng-override YAML jika diisi",
        ),
        DeclareLaunchArgument(
            "kd",
            default_value="",
            description="Gain Derivative PID, meng-override YAML jika diisi",
        ),
        OpaqueFunction(function=launch_setup),
    ]

    return LaunchDescription(launch_args)
