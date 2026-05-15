#!/usr/bin/env python3
"""
teleop_keyboard.py — Keyboard Teleoperation Wheelchair

Publishes geometry_msgs/Twist on /cmd_vel based on keyboard input.
The Isaac Sim simulation reads /cmd_vel and maps it to the PhysX
Surface Velocity API on the left/right track bodies.

Key Bindings:
    i : forward  (linear.x = +speed)
    , : backward (linear.x = -speed)
    j : turn left  (angular.z = +turn)
    l : turn right (angular.z = -turn)
    k : stop     (all velocities = 0)
    Ctrl-C : quit

"""

import rclpy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

msg = """
Track Control:
   i
j  k  l
   ,
i: forward      ,: backward
j: turn left    l: turn right
k: STOP
CTRL-C to quit
"""

moveBindings = {
    'i': ( 1.0,  0.0),   # Forward
    ',': (-1.0,  0.0),   # Backward
    'j': ( 0.0,  1.0),   # Turn left (positive angular.z = CCW)
    'l': ( 0.0, -1.0),   # Turn right
    'k': ( 0.0,  0.0),   # Stop
}

def getKey(settings):
    """Read a single keypress from stdin in raw terminal mode.

    Switches stdin to raw mode, waits up to 0.1s for a keypress,
    then restores the original terminal settings.

    Args:
        settings: Original terminal attributes from tcgetattr().

    Returns:
        str: The character pressed, or '' if no key within timeout.
    """
    tty.setraw(sys.stdin.fileno())          
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)  
    key = sys.stdin.read(1) if rlist else ''   
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)  
    return key

def main(args=None):
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init(args=args)
    node = rclpy.create_node('isaac_teleop_keyboard')
    pub  = node.create_publisher(Twist, '/cmd_vel', 10)

    speed = 2.0   # Linear speed multiplier (m/s) 
    turn  = 1.0   # Angular speed multiplier (rad/s)
    x     = 0.0   # Current linear velocity multiplier
    th    = 0.0   # Current angular velocity multiplier

    print(msg)

    try:
        while rclpy.ok():
            key = getKey(settings)

            if key in moveBindings:
                x  = moveBindings[key][0]  
                th = moveBindings[key][1]
            elif key == '\x03':   
                break

            twist = Twist()
            twist.linear.x  = float(x * speed)   # Forward/backward velocity
            twist.angular.z = float(th * turn)    # Turning velocity
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