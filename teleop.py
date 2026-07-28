#!/usr/bin/env python3
"""
wasd_teleop.py

Minimal WASD keyboard teleop for a differential-drive rover. Publishes
geometry_msgs/Twist - only linear.x and angular.z are ever set, since a
diff-drive rover has no strafe (no linear.y) to control.

Controls (hold or tap, no Enter needed):
    w / s   forward / backward   (linear.x = +/- linear_speed)
    a / d   turn left / right    (angular.z = +/- turn_speed)
    q / e   forward + turn combined (diagonal-feel curve, not strafing)
    space   stop immediately
    x       quit

NOTE ON TOPIC: bringup.launch.py remaps diff_drive_controller's
cmd_vel_unstamped subscription to /cmd_vel. So this publishes to /cmd_vel
by default, NOT /diff_drive_controller/cmd_vel_unstamped directly - that
literal topic has nothing subscribed to it under the current remap.
Override with --ros-args -p cmd_vel_topic:=... if that changes.
"""

import sys
import termios
import tty
import select

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


INSTRUCTIONS = """
WASD Teleop - non-holonomic (diff-drive)
-----------------------------------------
   w         forward
 a s d       turn left / backward / turn right
   q e       forward-left / forward-right curve

space  : stop
x      : quit

CTRL-C to force quit at any time.
"""

# key -> (linear.x multiplier, angular.z multiplier)
KEY_BINDINGS = {
    'w': (1.0, 0.0),
    's': (-1.0, 0.0),
    'a': (0.0, 1.0),
    'd': (0.0, -1.0),
    'q': (1.0, 1.0),
    'e': (1.0, -1.0),
}


class WasdTeleop(Node):
    def __init__(self):
        super().__init__('wasd_teleop')

        self.declare_parameter('cmd_vel_topic', '/diff_drive_controller/cmd_vel_unstamped')
        self.declare_parameter('linear_speed', 1.00)
        self.declare_parameter('turn_speed', 2.00)

        topic = self.get_parameter('cmd_vel_topic').value
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.turn_speed = float(self.get_parameter('turn_speed').value)

        self.pub = self.create_publisher(Twist, topic, 10)
        self.get_logger().info(
            f"Publishing to '{topic}'  (linear_speed={self.linear_speed}, "
            f"turn_speed={self.turn_speed})")

    def publish(self, lin_mult: float, ang_mult: float):
        msg = Twist()
        msg.linear.x = lin_mult * self.linear_speed
        msg.angular.z = ang_mult * self.turn_speed
        self.pub.publish(msg)

    def stop(self):
        self.publish(0.0, 0.0)


def get_key(settings, timeout=0.1):
    """Non-blocking single-character read from stdin."""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    key = sys.stdin.read(1) if rlist else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main():
    rclpy.init()
    node = WasdTeleop()

    settings = termios.tcgetattr(sys.stdin)
    print(INSTRUCTIONS)

    try:
        while rclpy.ok():
            key = get_key(settings)

            if key in KEY_BINDINGS:
                lin_mult, ang_mult = KEY_BINDINGS[key]
                node.publish(lin_mult, ang_mult)
            elif key == ' ':
                node.stop()
            elif key == 'x' or key == '\x03':  # 'x' or Ctrl-C
                break
            elif key == '':
                # No key pressed within timeout - stop so the rover
                # doesn't keep coasting after you let go.
                node.stop()

    except Exception as e:
        node.get_logger().error(f"Error: {e}")
    finally:
        node.stop()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
