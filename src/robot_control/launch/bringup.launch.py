"""
Single launch file for the whole hardware stack. Run this on the Jetson.

It starts, in order:
  1. robot_state_publisher   - publishes /robot_description (topic) and static TF
  2. ros2_control_node        - loads the ODrive S1 SystemInterface plugin, opens CAN
  3. joint_state_broadcaster  - spawned once ros2_control_node is up
  4. diff_drive_controller    - spawned once joint_state_broadcaster has finished
                                activating, subscribes /cmd_vel, publishes wheel odom

Usage:
  ros2 launch robot_control bringup.launch.py
  ros2 launch robot_control bringup.launch.py can_interface:=can1

Before running this for the first time:
  - Bring the CAN interface up yourself first (this launch file does NOT do it -
    see sample_can.py's bring_up_can() for the exact ip link sequence), and make
    sure each ODrive S1 has a unique, already-calibrated node_id matching
    robot_description/urdf/rover.ros2_control.xacro (0/1/2/3 = FL/FR/RL/RR by
    default).
  - Chock the wheels or put the rover up on a stand for the first activation -
    on_activate() puts all 4 axes into CLOSED_LOOP_CONTROL immediately.
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    can_interface_arg = DeclareLaunchArgument(
        "can_interface",
        default_value="can0",
        description="SocketCAN interface the ODrive S1 hardware interface should use",
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("robot_description"), "urdf", "rover.urdf.xacro"]
            ),
            " can_interface:=",
            LaunchConfiguration("can_interface"),
        ]
    )

    # Wrapped in ParameterValue(..., value_type=str) to prevent ROS 2 from attempting to parse URDF as YAML
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    controllers_yaml = PathJoinSubstitution(
        [FindPackageShare("robot_control"), "config", "controllers.yaml"]
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers_yaml],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--controller-manager", "/controller_manager", "--ros-args", "--remap", "/diff_drive_controller/cmd_vel_unstamped:=/cmd_vel",],
        output="screen",
    )

    # Spawn joint_state_broadcaster as soon as ros2_control_node has started, then
    # spawn diff_drive_controller only once that first spawner has finished - this
    # avoids a race against controller_manager's services still coming up.
    delayed_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager_node,
            on_start=[joint_state_broadcaster_spawner],
        )
    )

    delayed_diff_drive_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_controller_spawner],
        )
    )

    return LaunchDescription(
        [
            can_interface_arg,
            robot_state_publisher_node,
            controller_manager_node,
            delayed_joint_state_broadcaster,
            delayed_diff_drive_controller,
        ]
    )
