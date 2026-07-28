#!/usr/bin/env python3
"""
fastlio_tf_bridge.py

Connects FAST-LIO's odometry output to the robot's URDF TF tree.

THE PROBLEM THIS SOLVES
-----------------------
FAST-LIO publishes:            camera_init -> body      (body == the IMU frame)
robot_state_publisher owns:    base_footprint -> base_link -> livox_frame -> ...

These are two disconnected trees. You cannot simply add a static transform
between livox_frame and body, because livox_frame already has a parent
(base_link, from the URDF fixed joint) and every TF frame may have exactly
one parent.

The fix is to add ONE new edge upstream of the robot's root:

    camera_init -> base_footprint

computed as:

    T(camera_init -> base_footprint)
        = T(camera_init -> body)          [from FAST-LIO /Odometry]
        * inverse( T(base_footprint -> body) )

where T(base_footprint -> body) is the fixed chain

    base_footprint -> livox_frame         [looked up from tf2 / the URDF]
        * livox_frame -> body             [the LiDAR<->IMU extrinsic]

Everything downstream of base_footprint stays exactly as
robot_state_publisher already publishes it. Nothing in the URDF changes.

EXTRINSIC SIGN CONVENTION
-------------------------
FAST-LIO's mapping/extrinsic_T is the LiDAR's position expressed in the IMU
frame, i.e. T(body -> livox_frame). This node therefore INVERTS it to get
T(livox_frame -> body). If your map and robot model appear offset from each
other by a few centimetres, this sign is the first thing to re-check.

With the Mid-360 values:
    extrinsic_T = [-0.011, -0.02329, 0.04412]   (body -> livox_frame)
    => livox_frame -> body = [0.011, 0.02329, -0.04412]

IMPORTANT - AVOID A DOUBLE PARENT ON base_footprint
---------------------------------------------------
diff_drive_controller publishes odom -> base_footprint when
enable_odom_tf: true. If this node also publishes camera_init ->
base_footprint, base_footprint has two parents and the tree breaks.

Set enable_odom_tf: false in controllers.yaml before running this, or
fuse both sources in robot_localization and let the EKF own the transform.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener, TransformBroadcaster


def quat_to_matrix(x, y, z, w):
    """Quaternion -> 3x3 rotation matrix."""
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    xx, yy, zz = x * x * s, y * y * s, z * z * s
    xy, xz, yz = x * y * s, x * z * s, y * z * s
    wx, wy, wz = w * x * s, w * y * s, w * z * s
    return np.array([
        [1.0 - (yy + zz), xy - wz,         xz + wy],
        [xy + wz,         1.0 - (xx + zz), yz - wx],
        [xz - wy,         yz + wx,         1.0 - (xx + yy)],
    ])


def matrix_to_quat(m):
    """3x3 rotation matrix -> quaternion (x, y, z, w)."""
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0.0:
        s = np.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    return x, y, z, w


def make_transform(translation, rotation_matrix):
    """Build a 4x4 homogeneous transform."""
    t = np.eye(4)
    t[:3, :3] = rotation_matrix
    t[:3, 3] = translation
    return t


class FastLioTfBridge(Node):
    def __init__(self):
        super().__init__('fastlio_tf_bridge')

        self.declare_parameter('odom_topic', '/Odometry')
        self.declare_parameter('world_frame', 'camera_init')
        self.declare_parameter('robot_root_frame', 'base_footprint')
        self.declare_parameter('lidar_frame', 'livox_frame')
        # FAST-LIO's mapping/extrinsic_T, i.e. T(body -> lidar). Inverted below.
        self.declare_parameter('extrinsic_t', [-0.011, -0.02329, 0.04412])

        self.odom_topic = self.get_parameter('odom_topic').value
        self.world_frame = self.get_parameter('world_frame').value
        self.robot_root_frame = self.get_parameter('robot_root_frame').value
        self.lidar_frame = self.get_parameter('lidar_frame').value
        extrinsic_t = np.array(self.get_parameter('extrinsic_t').value, dtype=float)

        # extrinsic_T is T(body -> lidar); we need T(lidar -> body).
        # Rotation is identity for the Mid-360, so the inverse is just -T.
        self.t_lidar_to_body = make_transform(-extrinsic_t, np.eye(3))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Cached once robot_state_publisher is up - this chain is static.
        self.t_root_to_body = None

        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)

        self.get_logger().info(
            f"Bridging {self.world_frame} -> {self.robot_root_frame} "
            f"from {self.odom_topic}"
        )

    def lookup_static_chain(self):
        """Resolve base_footprint -> body once, via base_footprint -> livox_frame."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.robot_root_frame, self.lidar_frame, rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(
                f"Waiting for {self.robot_root_frame} -> {self.lidar_frame} "
                f"(is robot_state_publisher running?): {e}",
                throttle_duration_sec=5.0)
            return False

        tr = tf.transform.translation
        rot = tf.transform.rotation
        t_root_to_lidar = make_transform(
            [tr.x, tr.y, tr.z],
            quat_to_matrix(rot.x, rot.y, rot.z, rot.w))

        self.t_root_to_body = t_root_to_lidar @ self.t_lidar_to_body

        pos = self.t_root_to_body[:3, 3]
        self.get_logger().info(
            f"Resolved {self.robot_root_frame} -> body = "
            f"[{pos[0]:.5f}, {pos[1]:.5f}, {pos[2]:.5f}]")
        return True

    def odom_callback(self, msg: Odometry):
        if self.t_root_to_body is None:
            if not self.lookup_static_chain():
                return

        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        t_world_to_body = make_transform(
            [p.x, p.y, p.z],
            quat_to_matrix(q.x, q.y, q.z, q.w))

        # camera_init -> base_footprint = (camera_init -> body) * (body -> base_footprint)
        t_world_to_root = t_world_to_body @ np.linalg.inv(self.t_root_to_body)

        out = TransformStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.world_frame
        out.child_frame_id = self.robot_root_frame
        out.transform.translation.x = float(t_world_to_root[0, 3])
        out.transform.translation.y = float(t_world_to_root[1, 3])
        out.transform.translation.z = float(t_world_to_root[2, 3])
        qx, qy, qz, qw = matrix_to_quat(t_world_to_root[:3, :3])
        out.transform.rotation.x = float(qx)
        out.transform.rotation.y = float(qy)
        out.transform.rotation.z = float(qz)
        out.transform.rotation.w = float(qw)

        self.tf_broadcaster.sendTransform(out)


def main():
    rclpy.init()
    node = FastLioTfBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
