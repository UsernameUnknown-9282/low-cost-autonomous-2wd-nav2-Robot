#!/usr/bin/env python3
"""
ROS 2 Humble Lightweight Motor Driver Node for 2WD Differential Drive Robot.

Hardware Target:
  - Raspberry Pi Zero 2 W
  - TB6612FNG Dual H-Bridge Motor Driver
  - Yellow TT DC Motors (with gearbox static friction compensation)

Kinematics & Control:
  - Differential drive kinematics (track width ~0.14 m, wheel radius ~0.033 m)
  - Deadband threshold & smooth PWM scaling (starts at ~28% min PWM to overcome TT stiction)
  - 300 ms safety watchdog timer to cut PWM on Wi-Fi drop or lost /cmd_vel
  - Automatic fallback to MockTB6612Driver when run in non-Pi environments (WSL2 / Linux PC)
"""

import sys
import os
import time
import math
from typing import Tuple, Optional

# Ensure local imports work regardless of working directory
current_dir = os.path.dirname(os.path.abspath(__file__))
if current_dir not in sys.path:
    sys.path.insert(0, current_dir)

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist

from tb6612_driver import (
    Direction,
    TB6612PinConfig,
    BaseTB6612Driver,
    create_tb6612_driver
)


class DifferentialDriveKinematics:
    """
    Kinematic calculations for 2WD differential drive robot.
    Computes left and right wheel velocities from linear (x) and angular (z) twist commands.
    """

    def __init__(self, track_width: float = 0.14, wheel_radius: float = 0.033):
        self.track_width = float(track_width)
        self.wheel_radius = float(wheel_radius)

    def twist_to_wheel_velocities(self, linear_x: float, angular_z: float) -> Tuple[float, float]:
        """
        Compute linear wheel velocities (m/s):
          v_left  = v_x - (omega_z * L / 2)
          v_right = v_x + (omega_z * L / 2)
        """
        if math.isnan(linear_x) or math.isnan(angular_z) or math.isinf(linear_x) or math.isinf(angular_z):
            return 0.0, 0.0

        half_track = (angular_z * self.track_width) / 2.0
        v_left = linear_x - half_track
        v_right = linear_x + half_track
        return v_left, v_right

    def wheel_velocities_to_twist(self, v_left: float, v_right: float) -> Tuple[float, float]:
        """
        Inverse kinematics: compute linear_x and angular_z from wheel speeds:
          v_x = (v_right + v_left) / 2
          omega_z = (v_right - v_left) / L
        """
        if math.isnan(v_left) or math.isnan(v_right) or math.isinf(v_left) or math.isinf(v_right):
            return 0.0, 0.0
        v_x = (v_right + v_left) / 2.0
        omega_z = (v_right - v_left) / max(self.track_width, 1e-6)
        return v_x, omega_z

    def desaturate_wheel_velocities(self, v_left: float, v_right: float, max_speed: float) -> Tuple[float, float]:
        """
        Scale both wheel velocities down proportionally if either exceeds max_speed,
        preserving the robot's commanded turning curvature (path radius).
        """
        max_allowed = max(1e-6, float(max_speed))
        peak_speed = max(abs(v_left), abs(v_right))
        if peak_speed > max_allowed:
            scale = max_allowed / peak_speed
            return v_left * scale, v_right * scale
        return v_left, v_right

    def wheel_velocities_to_angular(self, v_left: float, v_right: float) -> Tuple[float, float]:
        """Compute angular wheel speeds (rad/s): omega = v / R."""
        w_left = v_left / self.wheel_radius
        w_right = v_right / self.wheel_radius
        return w_left, w_right

    def wheel_velocities_to_rpm(self, v_left: float, v_right: float) -> Tuple[float, float]:
        """Compute wheel rotational speed in RPM."""
        w_left, w_right = self.wheel_velocities_to_angular(v_left, v_right)
        rpm_factor = 60.0 / (2.0 * 3.141592653589793)
        return w_left * rpm_factor, w_right * rpm_factor


class DeadbandPWMMapper:
    """
    Maps wheel linear velocity to TB6612 PWM duty cycle [0.0, 1.0] and Direction.
    Compensates for yellow TT gearbox static friction by stepping to min_pwm immediately
    when commanded above the deadband velocity threshold.
    """

    def __init__(
        self,
        min_pwm: float = 0.28,
        max_pwm: float = 1.00,
        deadband_speed: float = 0.02,
        max_wheel_speed: float = 0.40,
        invert_left: bool = False,
        invert_right: bool = False
    ):
        # Normalize PWM inputs if given in 0-100% range
        self.min_pwm = min_pwm / 100.0 if min_pwm > 1.0 else float(min_pwm)
        self.max_pwm = max_pwm / 100.0 if max_pwm > 1.0 else float(max_pwm)
        self.min_pwm = max(0.0, min(1.0, self.min_pwm))
        self.max_pwm = max(0.0, min(1.0, self.max_pwm))
        self.deadband_speed = max(0.0, float(deadband_speed))
        self.max_wheel_speed = max(self.deadband_speed + 1e-4, float(max_wheel_speed))
        self.invert_left = bool(invert_left)
        self.invert_right = bool(invert_right)

    def map_speed_to_pwm(self, speed_mps: float, invert: bool = False) -> Tuple[float, int]:
        """
        Map a single wheel's linear speed (m/s) to (duty_cycle, direction).

        Returns:
          duty_cycle: float in [0.0, 1.0]
          direction: Direction.FORWARD, Direction.REVERSE, or Direction.STOP
        """
        if speed_mps is None or math.isnan(speed_mps) or math.isinf(speed_mps):
            return 0.0, Direction.STOP

        abs_speed = abs(speed_mps)

        # Zero speed or below deadband threshold -> Stop motor completely
        if speed_mps == 0.0 or abs_speed < self.deadband_speed:
            return 0.0, Direction.STOP

        # Determine direction
        if speed_mps > 0:
            raw_dir = Direction.FORWARD
        else:
            raw_dir = Direction.REVERSE

        if invert:
            raw_dir = Direction.REVERSE if raw_dir == Direction.FORWARD else Direction.FORWARD

        # Linear scaling from min_pwm to max_pwm
        speed_range = max(self.max_wheel_speed - self.deadband_speed, 1e-6)
        normalized_speed = min(abs_speed - self.deadband_speed, speed_range) / speed_range
        normalized_speed = max(0.0, min(1.0, normalized_speed))

        duty = self.min_pwm + normalized_speed * (self.max_pwm - self.min_pwm)
        duty = max(self.min_pwm, min(self.max_pwm, duty))

        return duty, raw_dir

    def map_left(self, speed_mps: float) -> Tuple[float, int]:
        """Map left wheel linear speed using left motor polarity configuration."""
        return self.map_speed_to_pwm(speed_mps, self.invert_left)

    def map_right(self, speed_mps: float) -> Tuple[float, int]:
        """Map right wheel linear speed using right motor polarity configuration."""
        return self.map_speed_to_pwm(speed_mps, self.invert_right)

    def map_speeds(self, v_left: float, v_right: float) -> Tuple[Tuple[float, int], Tuple[float, int]]:
        """Map both wheel linear speeds simultaneously."""
        return self.map_left(v_left), self.map_right(v_right)


class MotorDriverNode(Node):
    """
    ROS 2 Node controlling TB6612FNG motor driver from /cmd_vel topics.
    """

    def __init__(self, driver: Optional[BaseTB6612Driver] = None):
        super().__init__('motor_driver_node')

        # Declare parameters
        self.declare_parameter('track_width', 0.14)
        self.declare_parameter('wheel_radius', 0.033)
        self.declare_parameter('max_wheel_speed', 0.40)
        self.declare_parameter('deadband_speed', 0.02)
        self.declare_parameter('min_pwm', 0.28)
        self.declare_parameter('max_pwm', 1.00)
        self.declare_parameter('pwm_frequency', 1000)
        self.declare_parameter('watchdog_timeout_sec', 0.300)
        self.declare_parameter('mock_hardware', False)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        # Pin assignments
        self.declare_parameter('pin_stby', 22)
        self.declare_parameter('pin_left_pwm', 12)
        self.declare_parameter('pin_left_ain1', 5)
        self.declare_parameter('pin_left_ain2', 6)
        self.declare_parameter('pin_right_pwm', 13)
        self.declare_parameter('pin_right_bin1', 23)
        self.declare_parameter('pin_right_bin2', 24)

        # Direction inversion flags
        self.declare_parameter('invert_left_motor', False)
        self.declare_parameter('invert_right_motor', False)

        # Retrieve parameter values
        self.track_width = self.get_parameter('track_width').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.max_wheel_speed = self.get_parameter('max_wheel_speed').value
        self.deadband_speed = self.get_parameter('deadband_speed').value
        self.min_pwm = self.get_parameter('min_pwm').value
        self.max_pwm = self.get_parameter('max_pwm').value
        self.pwm_frequency = self.get_parameter('pwm_frequency').value
        self.watchdog_timeout_sec = self.get_parameter('watchdog_timeout_sec').value
        self.mock_hardware = self.get_parameter('mock_hardware').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        self.invert_left = self.get_parameter('invert_left_motor').value
        self.invert_right = self.get_parameter('invert_right_motor').value

        self.pin_config = TB6612PinConfig(
            stby=self.get_parameter('pin_stby').value,
            pwma=self.get_parameter('pin_left_pwm').value,
            ain1=self.get_parameter('pin_left_ain1').value,
            ain2=self.get_parameter('pin_left_ain2').value,
            pwmb=self.get_parameter('pin_right_pwm').value,
            bin1=self.get_parameter('pin_right_bin1').value,
            bin2=self.get_parameter('pin_right_bin2').value
        )

        # Initialize kinematics and mapper
        self.kinematics = DifferentialDriveKinematics(
            track_width=self.track_width,
            wheel_radius=self.wheel_radius
        )
        self.mapper = DeadbandPWMMapper(
            min_pwm=self.min_pwm,
            max_pwm=self.max_pwm,
            deadband_speed=self.deadband_speed,
            max_wheel_speed=self.max_wheel_speed,
            invert_left=self.invert_left,
            invert_right=self.invert_right
        )

        def _safe_log(msg: str):
            if rclpy.ok():
                try:
                    self.get_logger().info(msg)
                    return
                except Exception:
                    pass
            print(msg, flush=True)

        # Initialize hardware or mock driver
        if driver is not None:
            self.driver = driver
        else:
            self.driver = create_tb6612_driver(
                pins=self.pin_config,
                frequency=self.pwm_frequency,
                force_mock=self.mock_hardware,
                log_callback=_safe_log
            )

        # Enable TB6612 (STBY = HIGH)
        self.driver.enable()

        # State tracking
        self.last_cmd_time: Optional[float] = None
        self.motors_active: bool = False
        self.watchdog_triggered: bool = False

        # Subscriber
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.sub_cmd_vel = self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self.cmd_vel_callback,
            qos
        )

        # 20 Hz Watchdog check timer (every 50 ms)
        self.watchdog_timer = self.create_timer(0.05, self._watchdog_check)

        self.get_logger().info(
            f"MotorDriverNode initialized successfully.\n"
            f"  Track width: {self.track_width:.3f} m, Wheel radius: {self.wheel_radius:.3f} m\n"
            f"  Deadband: {self.deadband_speed:.3f} m/s, Min PWM: {self.min_pwm * 100:.1f}%, Max PWM: {self.max_pwm * 100:.1f}%\n"
            f"  Watchdog Timeout: {self.watchdog_timeout_sec * 1000.0:.0f} ms\n"
            f"  Subscribed to: {self.cmd_vel_topic}"
        )

    def process_twist(self, linear_x: float, angular_z: float) -> Tuple[float, int, float, int]:
        """
        Process twist command into (left_duty, left_dir, right_duty, right_dir).
        Useful for both runtime operation and direct unit verification.
        """
        if math.isnan(linear_x) or math.isnan(angular_z) or math.isinf(linear_x) or math.isinf(angular_z):
            return 0.0, Direction.STOP, 0.0, Direction.STOP

        v_left, v_right = self.kinematics.twist_to_wheel_velocities(linear_x, angular_z)
        # Desaturate wheel speeds to preserve path curvature when speeds exceed max_wheel_speed
        v_left, v_right = self.kinematics.desaturate_wheel_velocities(v_left, v_right, self.max_wheel_speed)

        left_duty, left_dir = self.mapper.map_left(v_left)
        right_duty, right_dir = self.mapper.map_right(v_right)
        return left_duty, left_dir, right_duty, right_dir

    def cmd_vel_callback(self, msg: Twist) -> None:
        """Callback triggered upon receiving a Twist message on /cmd_vel."""
        now = time.monotonic()
        self.last_cmd_time = now

        linear_x = float(msg.linear.x)
        angular_z = float(msg.angular.z)

        # Safety check: Reject NaN or Inf commands immediately
        if math.isnan(linear_x) or math.isnan(angular_z) or math.isinf(linear_x) or math.isinf(angular_z):
            self.get_logger().warn("[cmd_vel] Received NaN/Inf in Twist command. Stopping motors for safety.")
            self.driver.stop()
            self.motors_active = False
            return

        # If watchdog was previously active, log recovery
        if self.watchdog_triggered:
            self.watchdog_triggered = False
            self.get_logger().info("[WATCHDOG] Failsafe cleared: /cmd_vel stream restored.")

        left_duty, left_dir, right_duty, right_dir = self.process_twist(linear_x, angular_z)

        # Apply motor commands to driver
        self.driver.set_motors(left_duty, left_dir, right_duty, right_dir)

        # Update active state
        self.motors_active = (left_dir != Direction.STOP or right_dir != Direction.STOP)

    def _watchdog_check(self) -> None:
        """Periodic safety check. Cuts motor power if no /cmd_vel received in 300 ms."""
        if self.last_cmd_time is None:
            # No commands have been received yet; motors remain stopped.
            return

        elapsed = time.monotonic() - self.last_cmd_time

        if elapsed > self.watchdog_timeout_sec:
            if not self.watchdog_triggered:
                if self.motors_active:
                    # Trigger failsafe
                    self.driver.stop()
                    self.motors_active = False
                    self.get_logger().warn(
                        f"[WATCHDOG] Failsafe triggered: No /cmd_vel received for "
                        f"{elapsed * 1000.0:.1f} ms (> {self.watchdog_timeout_sec * 1000.0:.0f} ms). "
                        f"Motors cut to STOP immediately."
                    )
                else:
                    self.driver.stop()
                self.watchdog_triggered = True

    def stop_motors(self) -> None:
        """Immediately stop both motors."""
        self.driver.stop()
        self.motors_active = False

    def destroy_node(self) -> None:
        """Safe shutdown of node and GPIO hardware."""
        try:
            self.driver.cleanup()
            self.motors_active = False
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = MotorDriverNode()
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        finally:
            node.destroy_node()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
