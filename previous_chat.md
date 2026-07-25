
/
For a 4 wheeled bot with Odrive S1s and say i want to implement differential dirve

Mainly for SLAM Nav2 and FAST-LIO2 Mapping

How does the control system should look like, 
Im thinking ros2 control cpp package seperately for controlling the motors

then i build the package one by one for mapping and slamming eventually to Nav2


Architected layered ROS2 control stack integrating ODrive, SLAM, and navigation systems
Architected layered ROS2 control stack integrating ODrive, SLAM, and navigation systems
Good architecture instinct — separating the hardware interface from everything else is the right call. Here's how I'd lay this out end to end.

Overall layered architecture
Nav2 (planner/controller) → /cmd_vel
        ↓
diff_drive_controller (ros2_controllers)
        ↓ (per-wheel velocity commands via hardware_interface)
Custom SystemInterface plugin (CAN ↔ ODrive S1 x4)
        ↓
CAN bus → 4x ODrive S1 → motors + onboard encoders
        ↑ (position/velocity feedback)
Custom SystemInterface plugin
        ↑
diff_drive_controller → wheel odometry
        ↑
robot_localization EKF ← IMU, wheel odom, FAST-LIO2 odom
        ↑
Nav2 (costmaps use FAST-LIO2 cloud/scan + EKF odom)
1. Hardware interface package (your instinct is right)
Build a standalone ros2_control SystemInterface C++ plugin, e.g. odrive_s1_hw_interface. This is only responsible for:

Opening a SocketCAN interface (can0) on startup
Sending Set_Axis_State (closed-loop control) to all 4 ODrives on activate
Writing Set_Input_Vel (0x0d) per wheel joint each control loop
Reading Get_Encoder_Estimates (0x09) feedback for position/velocity state interfaces
A CAN heartbeat/watchdog check — if a node stops responding, go to safe state
There is a community package, odrive_ros2_control, that already implements exactly this pattern (CAN-based SystemInterface for ODrive) — worth pulling up as a reference even if you write your own, since it saves you re-deriving the CAN message IDs and axis-state sequencing.

For a 4-wheel diff/skid-steer setup, each ODrive S1 drives one axis (one motor), so you'll have 4 joints: front_left_wheel_joint, front_right_wheel_joint, rear_left_wheel_joint, rear_right_wheel_joint, each exposing velocity command interface and position+velocity state interfaces.

2. Controller layer
Use the stock diff_drive_controller from ros2_controllers — it supports multiple wheel names per side (left_wheel_names: [front_left, rear_left], right_wheel_names: [front_right, rear_right]), so a 4-wheel skid-steer maps cleanly onto it without writing a custom controller. It:

Subscribes /cmd_vel
Converts to per-wheel velocity setpoints, sends through the hardware interface
Publishes wheel odometry + optionally odom → base_link tf
Keep this in its own <robot>_control package: controllers.yaml + a launch file that brings up controller_manager with the hardware interface plugin and diff_drive_controller + joint_state_broadcaster.

3. Important: don't trust wheel odom too much
4-wheel skid-steer has scrub/slip on turns, so wheel odometry alone will drift badly — this matters for both SLAM and Nav2. Two common patterns:

Fuse, don't replace: robot_localization EKF fuses wheel odom (short-term smooth velocity) + IMU + FAST-LIO2's odometry output, publishing a single continuous odom → base_link. Only one node should publish that tf — usually you set diff_drive_controller to not publish tf, and let the EKF or FAST-LIO2 own it.
FAST-LIO2 as primary odom source: since it's LiDAR-inertial, it's far more accurate than skid-steer wheel odom on uneven ground. Many teams feed FAST-LIO2's odometry directly as the odom frame source for Nav2, and only use wheel odom as a coarse EKF prior or fallback.
Decide this early since it affects your tf tree and which frame Nav2's costmaps key off of.

4. Package breakdown I'd suggest
robot_description/        # URDF/xacro, ros2_control tags for 4 wheel joints
odrive_s1_hw_interface/    # your SystemInterface plugin (CAN comms only)
robot_control/             # controllers.yaml, controller_manager launch
robot_localization/        # ekf.yaml, FAST-LIO2 config/launch
robot_navigation/          # Nav2 params.yaml, BT xml, costmap configs
robot_bringup/             # top-level launch files tying it all together
This mirrors your instinct — hardware interface is fully decoupled, so you can bench-test motor control (spin wheels via ros2 topic pub /cmd_vel) before FAST-LIO2 or Nav2 ever enter the picture. Good order of operations:

Get odrive_s1_hw_interface + diff_drive_controller spinning wheels correctly, verify wheel odom against ground truth (mark wheel travel, compare)
Bring up FAST-LIO2 standalone, verify odometry/map quality independent of the robot base
Add EKF fusion, settle the tf tree (map → odom → base_link → wheels)
Bring up Nav2 on top, tune costmaps against your FAST-LIO2 point cloud/derived scan
One CAN-layer detail worth flagging now: with 4 ODrive S1s on one bus, make sure your CAN bitrate and per-axis node IDs are set (each S1 needs a unique node_id before this works), and budget your control loop rate against CAN bus bandwidth — polling 4 axes for encoder feedback plus sending 4 velocity commands every cycle adds up if you're running controller_manager at 100+ Hz.

Want me to sketch the actual SystemInterface header/CAN message handling, or the URDF ros2_control block for the 4 wheel joints first?


You’ve used 75% of your weekly limit
Get more usage







Claude is AI and can make mistakes. Please double-check responses.
