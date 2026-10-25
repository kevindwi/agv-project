"""Launch file untuk node line tracker."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    config_file = LaunchConfiguration("config_file").perform(context)

    # Kumpulkan parameter overrides dari CLI jika diisi oleh user
    overrides = {}
    param_keys = [
        "camera_index",
        "serial_port",
        "baud_rate",
        "linear_speed_mps",
        "wheel_separation_m",
        "max_angular_speed_radps",
        "max_wheel_speed_mps",
        "publish_rate_hz",
        "kp",
        "ki",
        "kd",
    ]

    for key in param_keys:
        val = LaunchConfiguration(key).perform(context).strip()
        if val != "":
            try:
                if "." in val:
                    overrides[key] = float(val)
                else:
                    overrides[key] = int(val)
            except ValueError:
                overrides[key] = val

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

    return [node]


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
            "linear_speed_mps",
            default_value="",
            description="Kecepatan maju linier robot (m/s), meng-override YAML jika diisi",
        ),
        DeclareLaunchArgument(
            "wheel_separation_m",
            default_value="",
            description="Jarak tapak roda kiri-kanan (m), meng-override YAML jika diisi",
        ),
        DeclareLaunchArgument(
            "max_angular_speed_radps",
            default_value="",
            description="Batas kecepatan putar robot (rad/s), meng-override YAML jika diisi",
        ),
        DeclareLaunchArgument(
            "max_wheel_speed_mps",
            default_value="",
            description="Skala kecepatan roda maks untuk pemetaan PWM, meng-override YAML jika diisi",
        ),
        DeclareLaunchArgument(
            "publish_rate_hz",
            default_value="",
            description="Frekuensi pemrosesan kamera (Hz), meng-override YAML jika diisi",
        ),
        DeclareLaunchArgument(
            "kp",
            default_value="",
            description="Gain Proportional PID, meng-override YAML jika diisi",
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
