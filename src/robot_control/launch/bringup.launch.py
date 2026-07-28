"""
bringup.launch.py

Single entry point for the whole robot: rover hardware, camera, LiDAR
driver, FAST-LIO SLAM, and the TF bridge - started in dependency order.

STARTUP SEQUENCE
----------------
  t=0s    robot_state_publisher   - /robot_description + static TF
          ros2_control_node       - ODrive S1 plugin, opens CAN
          camera_publisher        - USB camera (independent of everything)
            -> joint_state_broadcaster spawns once controller_manager starts
            -> diff_drive_controller spawns once that finishes

  t=5s    livox_ros_driver2       - LiDAR starts after the rover so the
                                    URDF/TF tree already exists when points
                                    begin arriving

  t=10s   fast_lio                - needs /livox/lidar + /livox/imu already
          fastlio_tf_bridge         streaming, otherwise it logs "No point,
                                    skip this scan" until they appear

The delays are TimerActions rather than process-exit event handlers because
the LiDAR and SLAM stages are IncludeLaunchDescription, which don't emit the
process lifecycle events OnProcessStart/OnProcessExit rely on. Raise
lidar_delay / slam_delay if a stage is still coming up when the next starts.

USAGE
-----
  # everything, SLAM included (default)
  ros2 launch robot_control bringup.launch.py

  # rover only - no LiDAR, no SLAM. Wheel odometry TF is enabled
  # automatically in this mode so rviz still has a usable odom -> base tree.
  ros2 launch robot_control bringup.launch.py use_slam:=false

  # other overrides
  ros2 launch robot_control bringup.launch.py can_interface:=can1
  ros2 launch robot_control bringup.launch.py camera_device:=/dev/video2
  ros2 launch robot_control bringup.launch.py start_camera:=false
  ros2 launch robot_control bringup.launch.py lidar_delay:=8.0 slam_delay:=15.0

THE enable_odom_tf TRADEOFF (handled automatically here)
--------------------------------------------------------
Only one publisher may own the transform into base_footprint:
  - SLAM running     -> fastlio_tf_bridge publishes camera_init -> base_footprint,
                        so diff_drive_controller MUST NOT publish
                        odom -> base_footprint (two parents breaks TF)
  - SLAM not running -> nothing else parents base_footprint, so diff_drive
                        SHOULD publish it, or base_footprint is orphaned and
                        rviz silently displays nothing
This file sets enable_odom_tf to the inverse of use_slam via a parameter
override on controller_manager, so controllers.yaml never needs editing
when switching modes.

BEFORE FIRST RUN
----------------
  - Bring the CAN interface up yourself (this file does NOT do it).
  - Chock the wheels or put the rover on a stand - on_activate() puts all
    4 axes into CLOSED_LOOP_CONTROL immediately.
  - rviz is NOT launched here (robot is headless). Run rviz2 on a laptop
    with a matching ROS_DOMAIN_ID, Fixed Frame = camera_init.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ---------------- launch arguments ----------------
    can_interface_arg = DeclareLaunchArgument(
        "can_interface", default_value="can0",
        description="SocketCAN interface for the ODrive S1 hardware interface")

    use_slam_arg = DeclareLaunchArgument(
        "use_slam", default_value="true",
        description="Start LiDAR driver + FAST-LIO + TF bridge. When false, "
                    "diff_drive_controller publishes odom->base_footprint instead")

    start_camera_arg = DeclareLaunchArgument(
        "start_camera", default_value="true",
        description="Start the camera_feed publisher")
    camera_device_arg = DeclareLaunchArgument("camera_device", default_value="/dev/video0")
    camera_width_arg = DeclareLaunchArgument("camera_width", default_value="640")
    camera_height_arg = DeclareLaunchArgument("camera_height", default_value="480")
    camera_fps_arg = DeclareLaunchArgument("camera_fps", default_value="30.0")

    config_file_arg = DeclareLaunchArgument(
        "config_file", default_value="mid360.yaml",
        description="FAST-LIO config yaml under fast_lio/config")
    world_frame_arg = DeclareLaunchArgument(
        "world_frame", default_value="camera_init",
        description="FAST-LIO's fixed world frame - use as rviz Fixed Frame")

    lidar_delay_arg = DeclareLaunchArgument(
        "lidar_delay", default_value="5.0",
        description="Seconds after the rover stack before starting the LiDAR driver")
    slam_delay_arg = DeclareLaunchArgument(
        "slam_delay", default_value="10.0",
        description="Seconds after the rover stack before starting FAST-LIO")

    use_slam = LaunchConfiguration("use_slam")

    # enable_odom_tf must be the inverse of use_slam - see module docstring.
    enable_odom_tf = PythonExpression(["'false' if '", use_slam, "' == 'true' else 'true'"])

    # ---------------- robot description ----------------
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([
            FindPackageShare("robot_description"), "urdf", "rover.urdf.xacro"]),
        " can_interface:=", LaunchConfiguration("can_interface"),
    ])
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    controllers_yaml = PathJoinSubstitution(
        [FindPackageShare("robot_control"), "config", "controllers.yaml"])

    # ---------------- stage 1: rover ----------------
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # The dict after controllers_yaml overrides the yaml's own value, so
    # enable_odom_tf follows use_slam without editing controllers.yaml.
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            controllers_yaml,
            {"diff_drive_controller.enable_odom_tf": ParameterValue(
                enable_odom_tf, value_type=bool)},
        ],
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
        arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
        remappings=[("/diff_drive_controller/cmd_vel_unstamped", "/cmd_vel")],
        output="screen",
    )

    camera_publisher_node = Node(
        package="camera_feed",
        executable="camera_publisher",
        name="camera_publisher",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_camera")),
        parameters=[{
            "device": LaunchConfiguration("camera_device"),
            "width": LaunchConfiguration("camera_width"),
            "height": LaunchConfiguration("camera_height"),
            "fps": LaunchConfiguration("camera_fps"),
            # Fixed physical fact about this rover's mount, not a per-run option.
            "rotate_180": True,
        }],
    )

    # Chain the spawners so they don't race controller_manager's services.
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

    # ---------------- stage 2: LiDAR driver ----------------
    lidar_stage = TimerAction(
        period=LaunchConfiguration("lidar_delay"),
        actions=[
            GroupAction(
                condition=IfCondition(use_slam),
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(PathJoinSubstitution([
                            FindPackageShare("livox_ros_driver2"),
                            "launch_ROS2", "msg_MID360_launch.py",
                        ]))),
                ]),
        ])

    # ---------------- stage 3: FAST-LIO + TF bridge ----------------
    slam_stage = TimerAction(
        period=LaunchConfiguration("slam_delay"),
        actions=[
            GroupAction(
                condition=IfCondition(use_slam),
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(PathJoinSubstitution([
                            FindPackageShare("fast_lio"), "launch", "mapping.launch.py",
                        ])),
                        launch_arguments={
                            "config_file": LaunchConfiguration("config_file"),
                            "rviz": "false",
                        }.items()),
                    Node(
                        package="robot_control",
                        executable="fastlio_tf_bridge.py",
                        name="fastlio_tf_bridge",
                        output="screen",
                        parameters=[{
                            "odom_topic": "/Odometry",
                            "world_frame": LaunchConfiguration("world_frame"),
                            "robot_root_frame": "base_footprint",
                            "lidar_frame": "livox_frame",
                            # Must match mapping/extrinsic_T in the FAST-LIO config.
                            # This is the Mid-360's INTERNAL lidar<->imu offset, NOT
                            # the chassis mounting - that lives in rover.urdf.xacro.
                            "extrinsic_t": [-0.011, -0.02329, 0.04412],
                        }]),
                ]),
        ])

    return LaunchDescription([
        can_interface_arg,
        use_slam_arg,
        start_camera_arg,
        camera_device_arg,
        camera_width_arg,
        camera_height_arg,
        camera_fps_arg,
        config_file_arg,
        world_frame_arg,
        lidar_delay_arg,
        slam_delay_arg,

        robot_state_publisher_node,
        controller_manager_node,
        camera_publisher_node,
        delayed_joint_state_broadcaster,
        delayed_diff_drive_controller,

        lidar_stage,
        slam_stage,
    ])
