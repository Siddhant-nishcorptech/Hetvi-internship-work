#!/usr/bin/env python3
"""
ROS2 entry point wrapper for balance_controller.
Imports and runs the balance controller from the scripts directory.
"""

import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import String, Float64MultiArray


def quaternion_to_euler(w, x, y, z):
    """Convert quaternion to (roll, pitch, yaw) in radians.

    Uses aerospace convention:
      roll  = rotation about X-axis
      pitch = rotation about Y-axis
      yaw   = rotation about Z-axis
    """
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class PIDController:
    """Simple PID controller with anti-windup integral clamping."""

    def __init__(self, kp, ki, kd, integral_limit=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = integral_limit
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = None

    def update(self, error, current_time):
        if self.prev_time is None:
            self.prev_time = current_time
            self.prev_error = error
            return 0.0

        dt = current_time - self.prev_time
        if dt <= 0.0:
            return 0.0

        p_term = self.kp * error

        self.integral += error * dt
        self.integral = max(-self.integral_limit,
                            min(self.integral_limit, self.integral))
        i_term = self.ki * self.integral

        d_term = self.kd * (error - self.prev_error) / dt

        self.prev_error = error
        self.prev_time = current_time

        return p_term + i_term + d_term

    def reset(self):
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = None


class BalanceController(Node):
    """ROS2 node: IMU -> PID -> fourbar crank commands."""

    def __init__(self):
        super().__init__('balance_controller')

        # Declare parameters
        self.declare_parameter('pitch_pid.kp', 2.0)
        self.declare_parameter('pitch_pid.ki', 0.1)
        self.declare_parameter('pitch_pid.kd', 0.5)
        self.declare_parameter('roll_pid.kp', 1.0)
        self.declare_parameter('roll_pid.ki', 0.05)
        self.declare_parameter('roll_pid.kd', 0.3)
        self.declare_parameter('stair_detection.pitch_threshold_deg', 8.0)
        self.declare_parameter('stair_detection.entry_duration_s', 0.5)
        self.declare_parameter('stair_detection.exit_threshold_deg', 3.0)
        self.declare_parameter('stair_detection.exit_duration_s', 1.0)
        self.declare_parameter('stair_mode.speed_multiplier', 0.5)
        self.declare_parameter('stair_mode.clearance_offset_rad', -0.15)
        self.declare_parameter('max_crank_offset_rad', 0.5)
        self.declare_parameter('control_rate_hz', 50.0)

        # Load parameters
        pitch_kp = self.get_parameter('pitch_pid.kp').value
        pitch_ki = self.get_parameter('pitch_pid.ki').value
        pitch_kd = self.get_parameter('pitch_pid.kd').value
        roll_kp = self.get_parameter('roll_pid.kp').value
        roll_ki = self.get_parameter('roll_pid.ki').value
        roll_kd = self.get_parameter('roll_pid.kd').value

        self.pitch_threshold = math.radians(
            self.get_parameter('stair_detection.pitch_threshold_deg').value)
        self.entry_duration = self.get_parameter(
            'stair_detection.entry_duration_s').value
        self.exit_threshold = math.radians(
            self.get_parameter('stair_detection.exit_threshold_deg').value)
        self.exit_duration = self.get_parameter(
            'stair_detection.exit_duration_s').value
        self.speed_multiplier = self.get_parameter(
            'stair_mode.speed_multiplier').value
        self.clearance_offset = self.get_parameter(
            'stair_mode.clearance_offset_rad').value
        self.max_crank_offset = self.get_parameter(
            'max_crank_offset_rad').value
        control_rate = self.get_parameter('control_rate_hz').value

        # PID controllers
        self.pitch_pid = PIDController(pitch_kp, pitch_ki, pitch_kd)
        self.roll_pid = PIDController(roll_kp, roll_ki, roll_kd)

        # Stair detection state machine
        self.state = "flat"
        self.pitch_exceed_start = None
        self.pitch_below_start = None

        # Latest IMU data
        self.latest_pitch = 0.0
        self.latest_roll = 0.0
        self.imu_received = False

        # ROS interfaces
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)
        self.state_pub = self.create_publisher(
            String, '/balancing/state', 10)
        self.crank_pub = self.create_publisher(
            Float64MultiArray, '/fourbar/crank_cmd', 10)
        self.speed_pub = self.create_publisher(
            Float64MultiArray, '/balancing/speed_scale', 10)

        self.create_timer(1.0 / control_rate, self.control_loop)

        self.get_logger().info(
            f'Balance controller started — '
            f'pitch PID=[{pitch_kp}, {pitch_ki}, {pitch_kd}], '
            f'roll PID=[{roll_kp}, {roll_ki}, {roll_kd}]')

    def imu_callback(self, msg):
        roll, pitch, _ = quaternion_to_euler(
            msg.orientation.w, msg.orientation.x,
            msg.orientation.y, msg.orientation.z)
        self.latest_roll = roll
        self.latest_pitch = pitch
        self.imu_received = True

    def update_stair_state(self, pitch, now):
        abs_pitch = abs(pitch)

        if self.state == "flat":
            if abs_pitch > self.pitch_threshold:
                if self.pitch_exceed_start is None:
                    self.pitch_exceed_start = now
                elif (now - self.pitch_exceed_start) >= self.entry_duration:
                    self.state = "ascending" if pitch > 0 else "descending"
                    self.pitch_exceed_start = None
                    self.pitch_below_start = None
                    self.get_logger().info(
                        f'Stair mode ENTERED: {self.state}')
            else:
                self.pitch_exceed_start = None

        elif self.state in ("ascending", "descending"):
            if abs_pitch < self.exit_threshold:
                if self.pitch_below_start is None:
                    self.pitch_below_start = now
                elif (now - self.pitch_below_start) >= self.exit_duration:
                    self.state = "flat"
                    self.pitch_below_start = None
                    self.pitch_exceed_start = None
                    self.pitch_pid.reset()
                    self.roll_pid.reset()
                    self.get_logger().info('Stair mode EXITED: flat')
            else:
                self.pitch_below_start = None

    def control_loop(self):
        if not self.imu_received:
            return

        now = time.monotonic()
        pitch = self.latest_pitch
        roll = self.latest_roll

        self.update_stair_state(pitch, now)

        # Publish current state
        state_msg = String()
        state_msg.data = self.state
        self.state_pub.publish(state_msg)

        # PID: target is 0 (level body)
        pitch_correction = self.pitch_pid.update(0.0 - pitch, now)
        roll_correction = self.roll_pid.update(0.0 - roll, now)

        # Clamp corrections
        pitch_correction = max(-self.max_crank_offset,
                               min(self.max_crank_offset, pitch_correction))
        roll_correction = max(-self.max_crank_offset * 0.5,
                              min(self.max_crank_offset * 0.5, roll_correction))

        # Stair mode: add clearance offset
        clearance = 0.0
        if self.state in ("ascending", "descending"):
            clearance = self.clearance_offset

        # Compute per-side crank commands
        left_crank = pitch_correction + roll_correction + clearance
        right_crank = pitch_correction - roll_correction + clearance

        # Final clamp
        left_crank = max(-self.max_crank_offset,
                         min(self.max_crank_offset, left_crank))
        right_crank = max(-self.max_crank_offset,
                          min(self.max_crank_offset, right_crank))

        # Publish crank commands
        cmd_msg = Float64MultiArray()
        cmd_msg.data = [left_crank, right_crank]
        self.crank_pub.publish(cmd_msg)

        # Publish speed scale for stair mode
        scale_msg = Float64MultiArray()
        speed_scale = self.speed_multiplier if self.state != "flat" else 1.0
        scale_msg.data = [speed_scale]
        self.speed_pub.publish(scale_msg)


def main(args=None):
    rclpy.init(args=args)
    node = BalanceController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
