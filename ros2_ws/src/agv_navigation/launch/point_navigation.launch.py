import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_nav_share = get_package_share_directory("agv_navigation")
    default_params_file = os.path.join(pkg_nav_share, "config", "pid_params.yaml")

    # Declare arguments
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params_file,
        description="Path to parameters YAML file for navigation and odometry nodes",
    )

    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Whether to start RViz2 automatically",
    )

    params_file = LaunchConfiguration("params_file")
    use_rviz = LaunchConfiguration("use_rviz")

    # Node: Odometry
    odometry_node = Node(
        package="agv_navigation",
        executable="odometry_node",
        name="odometry_node",
        output="screen",
        parameters=[params_file],
    )

    # Node: Goal Follower (Point-to-Point)
    goal_follower_node = Node(
        package="agv_navigation",
        executable="goal_follower_node",
        name="goal_follower_node",
        output="screen",
        parameters=[params_file],
    )

    # Node: RViz2
    # Cek lokasi file rviz default di project workspace atau share
    rviz_config_path = os.path.expanduser("~/dev/agv-project/ros2_ws/line_tracker.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_path] if os.path.exists(rviz_config_path) else [],
        output="screen",
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        params_file_arg,
        use_rviz_arg,
        odometry_node,
        goal_follower_node,
        rviz_node,
    ])
