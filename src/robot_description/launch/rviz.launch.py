"""
Run this on the VISUALIZATION PC, not the Jetson.

It does NOT start robot_state_publisher or any hardware/controller nodes - it only
opens RViz2, which pulls the robot model over the /robot_description topic (the
Jetson's robot_state_publisher, started by robot_control/launch/bringup.launch.py,
publishes this as a transient-local topic) and subscribes to /tf and
/diff_drive_controller/odom directly.

Requirements for this to "just work":
  - Same ROS_DOMAIN_ID as the Jetson (export ROS_DOMAIN_ID=<n> on both machines)
  - Same DDS setup / on the same network segment (multicast reachable), matching
    the CycloneDDS config you're already running on the Jetson
  - The Jetson-side stack (bringup.launch.py) must already be running

Usage:
  ros2 launch robot_description rviz.launch.py
  ros2 launch robot_description rviz.launch.py rviz_config:=/path/to/other.rviz
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("robot_description")
    default_rviz_config = os.path.join(pkg_share, "rviz", "rover.rviz")

    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=default_rviz_config,
        description="Path to the RViz config file to load",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")],
    )

    return LaunchDescription(
        [
            rviz_config_arg,
            rviz_node,
        ]
    )
