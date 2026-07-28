#!/usr/bin/env python3
"""
slam.launch.py

Brings up the LiDAR driver, FAST-LIO, and the TF bridge together.

Does NOT launch the rover's ros2_control stack - run bringup.launch.py
separately so you can restart SLAM without power-cycling the motors.

Does NOT launch rviz - the robot is headless. Run rviz2 on a laptop
on the same ROS_DOMAIN_ID.

    ros2 launch robot_control slam.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration('config_file')
    world_frame = LaunchConfiguration('world_frame')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file', default_value='mid360.yaml',
            description='FAST-LIO config yaml under fast_lio/config'),
        DeclareLaunchArgument(
            'world_frame', default_value='camera_init',
            description="FAST-LIO's fixed world frame"),

        # 1. Livox driver - must use the CustomMsg (msg_*) launch so each
        #    point carries a timestamp for motion undistortion.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('livox_ros_driver2'),
                'launch_ROS2', 'msg_MID360_launch.py',
            ]))),

        # 2. FAST-LIO, headless.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('fast_lio'), 'launch', 'mapping.launch.py',
            ])),
            launch_arguments={
                'config_file': config_file,
                'rviz': 'false',
            }.items()),

        # 3. Bridge FAST-LIO's odometry into the robot's TF tree.
        Node(
            package='robot_control',
            executable='fastlio_tf_bridge.py',
            name='fastlio_tf_bridge',
            output='screen',
            parameters=[{
                'odom_topic': '/Odometry',
                'world_frame': world_frame,
                'robot_root_frame': 'base_footprint',
                'lidar_frame': 'livox_frame',
                # Must match mapping/extrinsic_T in the FAST-LIO config.
                'extrinsic_t': [-0.011, -0.02329, 0.04412],
            }]),
    ])
