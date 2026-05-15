#!/usr/bin/env python3
"""
ROS2 entry point wrapper for teleop_keyboard.
Re-exports main() from scripts/teleop_keyboard.py logic.
"""

import rclpy
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty

msg = """
Track Control:
   i
j  k  l
   ,

i: forward      ,: backward
j: turn left    l: turn right
k: STOP (sets speed to 0)
CTRL-C to quit
"""

moveBindings = {
    'i': ( 1.0,  0.0),
    ',': (-1.0,  0.0),
    'j': ( 0.0,  1.0),
    'l': ( 0.0, -1.0),
    'k': ( 0.0,  0.0),
}


def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    key = sys.stdin.read(1) if rlist else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main(args=None):
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init(args=args)
    node = rclpy.create_node('isaac_teleop_keyboard')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)

    speed = 1.0
    turn  = 1.0
    x     = 0.0
    th    = 0.0

    print(msg)

    try:
        while rclpy.ok():
            key = getKey(settings)

            if key in moveBindings:
                x  = moveBindings[key][0]
                th = moveBindings[key][1]
            elif key == 'k':
                x  = 0.0
                th = 0.0
                print("  [STOP COMMAND]")
            elif key == '\x03':
                break

            twist = Twist()
            twist.linear.x  = float(x * speed)
            twist.angular.z = float(th * turn)
            pub.publish(twist)

            if x != 0.0 or th != 0.0:
                print(f"  Driving: x={twist.linear.x:.1f} z={twist.angular.z:.1f}", end='\r')

    except Exception as e:
        print(f"\n[ERROR] {e}")

    finally:
        pub.publish(Twist())
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
