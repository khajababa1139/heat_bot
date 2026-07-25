# rover_ws

Three packages for the 4-wheel differential-drive rover: description, ODrive S1
hardware interface, and controller bring-up. No simulation, no Gazebo - this
targets real CAN hardware on the Jetson only.

```
src/
  robot_description/       URDF/xacro, RViz config + launch (for the viz PC)
  odrive_s1_hw_interface/   ros2_control SystemInterface plugin (C++, raw SocketCAN)
  robot_control/            controller_manager config + the bring-up launch file
```

## 1. Dimensions used, and what I assumed

Everything below is derived directly from the numbers you gave me. A few
sub-details weren't specified, so I picked reasonable placeholder values -
they only affect visuals/inertia, not the kinematics that actually matter for
odometry:

| Quantity | Value | Source |
|---|---|---|
| Chassis (enclosure) L x W x H | 0.34 x 0.20 x 0.10 m | given |
| Wheel radius | 0.075 m | given (15 cm diameter) |
| Track width (L/R wheel-center distance) | 0.38 m | given |
| Wheelbase (F/R wheel-center distance) | 0.40 m | given |
| Side rail length | 0.50 m | given |
| Enclosure top height above ground | 0.22 m | given |
| Wheel width | 0.04 m | **assumed** - not specified |
| Rail cross-section | 0.03 x 0.03 m | **assumed** - not specified |
| Chassis/rail/wheel mass + inertia | placeholder values in the xacro | **assumed** - refine once you know real masses |

The vertical stack (base_footprint -> base_link -> chassis/wheels) is derived,
not guessed:

```
enclosure_bottom_to_ground = enclosure_top_to_ground - chassis_height = 0.12 m
axle_to_chassis_bottom     = enclosure_bottom_to_ground - wheel_radius = 0.045 m
```

So `base_link` sits at axle height (0.075 m above `base_footprint`), and the
chassis box sits 0.045 m above that - i.e. the side rails carry the motors
4.5 cm below the enclosure's underside. If that doesn't match reality (e.g.
the rails mount flush with the enclosure bottom instead), it's one number to
change in `rover.urdf.xacro` (`axle_to_chassis_bottom`).

Frame tree: `base_footprint -> base_link -> {chassis_link, left_rail_link,
right_rail_link, front_left_wheel_link, front_right_wheel_link,
rear_left_wheel_link, rear_right_wheel_link}`. All four wheel joints are
`continuous`, spinning about the local Y axis (REP-103: X forward, Z up).

## 2. CAN node ID mapping

`odrive_s1_hw_interface` and `robot_control` both assume this default
mapping, set as xacro args in `rover.ros2_control.xacro`:

| Joint | ODrive CAN node_id |
|---|---|
| `front_left_wheel_joint` | 0 |
| `front_right_wheel_joint` | 1 |
| `rear_left_wheel_joint` | 2 |
| `rear_right_wheel_joint` | 3 |

**This does not match `sample_can.py`'s `NODE_IDS = [0, 1]`** - that script
only ever exercised two axes on the bench. Before running `bringup.launch.py`
for real, confirm each ODrive S1's `axis0.config.can.node_id` is unique and
set to 0/1/2/3 (or override the `fl_node_id`/`fr_node_id`/`rl_node_id`/
`rr_node_id` args in `rover.ros2_control.xacro` if you've wired it up
differently).

## 3. How the hardware interface talks to the ODrives

`odrive_s1_hw_interface` is a `hardware_interface::SystemInterface` plugin
using the same CAN Simple protocol IDs as `sample_can.py`
(`arbitration_id = (node_id << 5) | cmd_id`):

- **on_configure**: opens the SocketCAN socket (does *not* bring the
  interface up - that's still your job beforehand, exactly like
  `bring_up_can()` in the sample script).
- **on_activate**: clears errors, sets velocity control mode + input mode
  passthrough, requests `CLOSED_LOOP_CONTROL` on all 4 axes, then waits (up to
  1 s each) for each axis's heartbeat to confirm it actually got there. If any
  axis doesn't confirm, activation fails and whatever did arm is walked back
  to `IDLE`.
- **read()**: non-blocking drain of whatever's on the bus (heartbeats +
  `Get_Encoder_Estimates`), converts turns/turns-per-second to rad/rad-per-s.
  If a wheel's feedback goes stale (no frame for 5x `can_timeout_ms`), it logs
  a throttled warning and holds the last known state rather than snapping to
  zero.
- **write()**: converts the controller's rad/s command back to turns/s,
  clamps to `max_velocity_turns_per_sec` (5.0, matching your Nav2/diff-drive
  ceiling) as a second line of defense on top of the controller-level limits,
  and sends `Set_Input_Vel`.
- **on_deactivate**: zeroes velocity, then sets all axes back to `IDLE`.

**Known simplification, said plainly:** `read()` polls the socket
non-blocking each control cycle rather than running a dedicated CAN RX thread
feeding a lock-free buffer. That's standard practice for ros2_control CAN
interfaces at 100 Hz-class update rates and is what similar community plugins
do, but it isn't hard-real-time. If you push `update_rate` much higher, or
need deterministic timing guarantees, that RX thread is the next
improvement to make.

**I could not compile this in this sandbox** - there's no ROS 2 toolchain or
apt access to ROS package repos here, only pip/npm/GitHub. I've followed the
standard Humble `hardware_interface::SystemInterface` API (the same pattern
`ros2_control_demos` uses: `CallbackReturn` for lifecycle callbacks,
`hardware_interface::return_type` for `read()`/`write()`), but please
`colcon build` on the Jetson and send me any compiler errors - header/API
details can shift slightly between minor ros2_control releases and I'd
rather fix a real error than have you guess at one.

## 4. Controller configuration

`diff_drive_controller` treats this as a standard 2-wheels-per-side
diff-drive (`left_wheel_names`/`right_wheel_names` each list 2 joints),
`wheel_separation = 0.38`, `wheel_radius = 0.075`, `open_loop: false` (trusts
ODrive encoder velocity feedback rather than just the last command - matters
for the scrub/slip four-wheel outdoor turning produces).

Velocity limits are derived from your 5 turns/s hardware ceiling:
`linear.x.max_velocity = 2.356 m/s`, `angular.z.max_velocity = 12.4 rad/s`
(the in-place-spin theoretical max - very fast, tune it down once you've
seen the chassis move). See the comments in `controllers.yaml` for the math.

## 5. Build

```bash
cd rover_ws
colcon build --symlink-install
source install/setup.bash
```

## 6. Run

On the **Jetson** (after CAN is up and ODrives are calibrated, per your
existing workflow):

```bash
ros2 launch robot_control bringup.launch.py
# or, if your CAN interface isn't can0:
ros2 launch robot_control bringup.launch.py can_interface:=can1
```

On the **visualization PC** (same `ROS_DOMAIN_ID`, same network/DDS
discovery as the Jetson - no ROS2 nodes need to be running there beforehand):

```bash
ros2 launch robot_description rviz.launch.py
```

RViz's RobotModel display pulls `/robot_description` as a topic (published
transient-local by the Jetson's `robot_state_publisher`), and subscribes to
`/tf` and `/diff_drive_controller/odom` directly - it doesn't run a second
`robot_state_publisher` locally.

## 7. First-run sanity checklist

- Wheels off the ground / rover on a stand for the very first activation.
- `ros2 control list_hardware_interfaces` and `ros2 control list_controllers`
  to confirm everything came up `active`.
- `ros2 topic echo /diff_drive_controller/odom` while spinning a wheel by hand
  to sanity-check sign/direction before commanding any velocity.
- Low-speed test: `ros2 run teleop_twist_keyboard teleop_twist_keyboard` before
  anything closed-loop (Nav2, etc.) touches `/cmd_vel`.

## 8. Not built yet (intentionally - you said not everything at once)

SLAM (FAST-LIO2), the EKF fusion node, and Nav2 aren't part of this drop.
Once this stack is verified on hardware, the previous architecture we
discussed (wheel odom + IMU + FAST-LIO2 -> `robot_localization` EKF -> Nav2)
layers on top without changes to these three packages.
