"""Launch file untuk node line tracker."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("line_tracker_ros2")
    config_file = os.path.join(package_share, "config", "line_tracker.yaml")

    return LaunchDescription(
        [
            Node(
                package="line_tracker_ros2",
                executable="line_tracker_node",
                name="line_tracker_node",
                output="screen",
                parameters=[config_file],
            )
        ]
    )
