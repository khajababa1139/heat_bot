#!/usr/bin/env python3
"""
camera_publisher.py

Publishes a USB camera feed as both raw and JPEG-compressed images.

Topics (following the image_transport naming convention so that
republish/rqt_image_view pick them up automatically):

    /camera/image_raw              sensor_msgs/Image
    /camera/image_raw/compressed   sensor_msgs/CompressedImage

QoS
---
Defaults to a sensor-data-style profile: BEST_EFFORT + KEEP_LAST(1).
This is the right choice for a constantly-streaming camera - a dropped
frame is better than a backlog of stale ones, and there is no point
retransmitting a frame that is already obsolete by the time it arrives.

If you need every frame delivered (recording, offline processing), set
the 'reliability' parameter to 'reliable'.

IMPORTANT - rviz2 / subscriber QoS matching:
    A BEST_EFFORT publisher will NOT match a RELIABLE subscriber, and
    DDS gives no error when this happens - the topic simply shows up in
    'ros2 topic list' but no data ever arrives. If a viewer shows
    nothing, check its reliability setting first (in rviz2: expand the
    display's Topic entry -> Reliability Policy -> Best Effort).
"""

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge


class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')

        # ---- parameters ----
        self.declare_parameter('device', '/dev/video0')
        # Defaults tuned for LOW LATENCY over a network link, not maximum
        # image quality. Bandwidth scales with pixel count, so dropping to
        # 640x480 costs ~4x less than 720p and ~16x less than 2K.
        # Raise these if you are consuming the feed on the Jetson itself,
        # where network bandwidth is not a constraint.
        self.declare_parameter('width', 320)
        self.declare_parameter('height', 240)
        self.declare_parameter('fps', 25.0)
        self.declare_parameter('frame_id', 'camera_link')
        self.declare_parameter('jpeg_quality', 30)     # 1-100; lower = smaller frames
        self.declare_parameter('publish_raw', True)
        self.declare_parameter('publish_compressed', True)
        self.declare_parameter('reliability', 'best_effort')  # or 'reliable'
        self.declare_parameter('queue_depth', 1)
        # MJPG lets most UVC cameras hit high resolution at full framerate;
        # raw YUYV often caps at ~5 fps at 2K over USB.
        self.declare_parameter('fourcc', 'MJPG')
        # Set true if the camera is physically mounted upside down - rotates
        # every frame 180 degrees before publishing (applies to both raw and
        # compressed output, so downstream consumers never see the flip).
        self.declare_parameter('rotate_180', False)

        self.device = self.get_parameter('device').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.fps = self.get_parameter('fps').value
        self.frame_id = self.get_parameter('frame_id').value
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        self.publish_raw = self.get_parameter('publish_raw').value
        self.publish_compressed = self.get_parameter('publish_compressed').value
        reliability = str(self.get_parameter('reliability').value).lower()
        queue_depth = int(self.get_parameter('queue_depth').value)
        fourcc = str(self.get_parameter('fourcc').value)
        self.rotate_180 = bool(self.get_parameter('rotate_180').value)
        if self.rotate_180:
            self.get_logger().info("rotate_180 enabled - correcting for upside-down mount")

        # ---- QoS ----
        qos = QoSProfile(depth=queue_depth)
        qos.history = QoSHistoryPolicy.KEEP_LAST
        qos.durability = QoSDurabilityPolicy.VOLATILE
        if reliability == 'reliable':
            qos.reliability = QoSReliabilityPolicy.RELIABLE
        else:
            qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        self.get_logger().info(
            f"QoS: {qos.reliability.name}, KEEP_LAST({queue_depth})")

        # ---- publishers ----
        self.pub_raw = None
        self.pub_compressed = None
        if self.publish_raw:
            self.pub_raw = self.create_publisher(Image, 'camera/image_raw', qos)
        if self.publish_compressed:
            self.pub_compressed = self.create_publisher(
                CompressedImage, 'camera/image_raw/compressed', qos)
        if self.pub_raw is None and self.pub_compressed is None:
            self.get_logger().error("Both publish_raw and publish_compressed are false - nothing to do")
            raise SystemExit(1)

        self.bridge = CvBridge()

        # ---- capture ----
        self.cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.get_logger().error(
                f"Could not open {self.device}. Check it exists (ls /dev/video*) "
                f"and that this user is in the 'video' group.")
            raise SystemExit(1)

        if len(fourcc) == 4:
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        # Minimise latency - without this V4L2 buffers several frames and the
        # feed lags noticeably behind reality.
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.get_logger().info(
            f"Opened {self.device}: requested {self.width}x{self.height}@{self.fps}, "
            f"got {actual_w}x{actual_h}@{actual_fps:.1f}")
        if (actual_w, actual_h) != (self.width, self.height):
            self.get_logger().warn(
                "Camera did not accept the requested resolution - it fell back to "
                "the nearest supported mode. Run 'v4l2-ctl -d %s --list-formats-ext' "
                "to see what this device actually supports." % self.device)

        self.frame_count = 0
        self.fail_count = 0
        period = 1.0 / self.fps if self.fps > 0 else 1.0 / 30.0
        self.timer = self.create_timer(period, self.capture_and_publish)

    def capture_and_publish(self):
        ok, frame = self.cap.read()
        if not ok:
            self.fail_count += 1
            self.get_logger().warn(
                f"Frame grab failed (total {self.fail_count})",
                throttle_duration_sec=2.0)
            return

        if self.rotate_180:
            frame = cv2.rotate(frame, cv2.ROTATE_180)

        stamp = self.get_clock().now().to_msg()

        if self.pub_raw is not None:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header.stamp = stamp
            msg.header.frame_id = self.frame_id
            self.pub_raw.publish(msg)

        if self.pub_compressed is not None:
            ok_enc, buf = cv2.imencode(
                '.jpg', frame,
                [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
            if ok_enc:
                cmsg = CompressedImage()
                cmsg.header.stamp = stamp
                cmsg.header.frame_id = self.frame_id
                cmsg.format = 'jpeg'
                cmsg.data = buf.tobytes()
                self.pub_compressed.publish(cmsg)
            else:
                self.get_logger().warn("JPEG encode failed", throttle_duration_sec=2.0)

        self.frame_count += 1

    def destroy_node(self):
        if getattr(self, 'cap', None) is not None:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = CameraPublisher()
    except SystemExit:
        rclpy.shutdown()
        return
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
