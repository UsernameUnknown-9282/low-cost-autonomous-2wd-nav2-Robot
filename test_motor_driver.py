#!/usr/bin/env python3
"""
Unit and Integration Tests for ROS 2 TB6612 Motor Driver Node and Differential Drive Kinematics.
"""

import sys
import os
import time
import pytest

# Ensure import paths resolve
current_dir = os.path.dirname(os.path.abspath(__file__))
if current_dir not in sys.path:
    sys.path.insert(0, current_dir)

from tb6612_driver import (
    Direction,
    TB6612PinConfig,
    MockTB6612Driver,
    create_tb6612_driver
)
from motor_driver_node import (
    DifferentialDriveKinematics,
    DeadbandPWMMapper,
    MotorDriverNode
)

import rclpy
from geometry_msgs.msg import Twist


# ============================================================================
# 1. Kinematics Tests
# ============================================================================

def test_kinematics_straight_forward():
    kin = DifferentialDriveKinematics(track_width=0.14, wheel_radius=0.033)
    v_l, v_r = kin.twist_to_wheel_velocities(0.2, 0.0)
    assert pytest.approx(v_l, abs=1e-5) == 0.2
    assert pytest.approx(v_r, abs=1e-5) == 0.2


def test_kinematics_straight_reverse():
    kin = DifferentialDriveKinematics(track_width=0.14, wheel_radius=0.033)
    v_l, v_r = kin.twist_to_wheel_velocities(-0.2, 0.0)
    assert pytest.approx(v_l, abs=1e-5) == -0.2
    assert pytest.approx(v_r, abs=1e-5) == -0.2


def test_kinematics_pure_spin_ccw():
    # Counter-clockwise: left wheel reverses, right wheel moves forward
    kin = DifferentialDriveKinematics(track_width=0.14, wheel_radius=0.033)
    v_l, v_r = kin.twist_to_wheel_velocities(0.0, 1.0)
    # v_diff = omega * L / 2 = 1.0 * 0.14 / 2 = 0.07 m/s
    assert pytest.approx(v_l, abs=1e-5) == -0.07
    assert pytest.approx(v_r, abs=1e-5) == 0.07


def test_kinematics_pure_spin_cw():
    # Clockwise: left wheel moves forward, right wheel reverses
    kin = DifferentialDriveKinematics(track_width=0.14, wheel_radius=0.033)
    v_l, v_r = kin.twist_to_wheel_velocities(0.0, -1.0)
    assert pytest.approx(v_l, abs=1e-5) == 0.07
    assert pytest.approx(v_r, abs=1e-5) == -0.07


def test_kinematics_curving_turn():
    kin = DifferentialDriveKinematics(track_width=0.14, wheel_radius=0.033)
    v_l, v_r = kin.twist_to_wheel_velocities(0.2, 0.5)
    # half_track = 0.5 * 0.14 / 2 = 0.035
    assert pytest.approx(v_l, abs=1e-5) == 0.165
    assert pytest.approx(v_r, abs=1e-5) == 0.235


def test_kinematics_angular_and_rpm_conversions():
    kin = DifferentialDriveKinematics(track_width=0.14, wheel_radius=0.033)
    v_l, v_r = 0.33, 0.33
    w_l, w_r = kin.wheel_velocities_to_angular(v_l, v_r)
    # omega = 0.33 / 0.033 = 10.0 rad/s
    assert pytest.approx(w_l, abs=1e-3) == 10.0
    assert pytest.approx(w_r, abs=1e-3) == 10.0

    rpm_l, rpm_r = kin.wheel_velocities_to_rpm(v_l, v_r)
    # 10 rad/s * 60 / (2 * pi) approx 95.49 RPM
    assert pytest.approx(rpm_l, abs=0.1) == 95.49
    assert pytest.approx(rpm_r, abs=0.1) == 95.49


# ============================================================================
# 2. Deadband & PWM Mapper Tests
# ============================================================================

def test_deadband_zero_and_below_threshold():
    mapper = DeadbandPWMMapper(
        min_pwm=0.28,
        max_pwm=1.00,
        deadband_speed=0.02,
        max_wheel_speed=0.40
    )

    # Exactly 0 m/s
    duty, direction = mapper.map_speed_to_pwm(0.0)
    assert duty == 0.0
    assert direction == Direction.STOP

    # Below deadband positive
    duty, direction = mapper.map_speed_to_pwm(0.015)
    assert duty == 0.0
    assert direction == Direction.STOP

    # Below deadband negative
    duty, direction = mapper.map_speed_to_pwm(-0.019)
    assert duty == 0.0
    assert direction == Direction.STOP


def test_deadband_step_jump_at_threshold():
    mapper = DeadbandPWMMapper(
        min_pwm=0.28,
        max_pwm=1.00,
        deadband_speed=0.02,
        max_wheel_speed=0.40
    )

    # Exactly at threshold positive -> min_pwm jump (0.28)
    duty, direction = mapper.map_speed_to_pwm(0.02)
    assert pytest.approx(duty, abs=1e-4) == 0.28
    assert direction == Direction.FORWARD

    # Exactly at threshold negative -> min_pwm jump (0.28)
    duty, direction = mapper.map_speed_to_pwm(-0.02)
    assert pytest.approx(duty, abs=1e-4) == 0.28
    assert direction == Direction.REVERSE


def test_deadband_linear_scaling():
    mapper = DeadbandPWMMapper(
        min_pwm=0.28,
        max_pwm=1.00,
        deadband_speed=0.02,
        max_wheel_speed=0.40
    )

    # Midpoint: speed = 0.21 (halfway between 0.02 and 0.40)
    # duty = 0.28 + 0.5 * (1.00 - 0.28) = 0.64
    duty, direction = mapper.map_speed_to_pwm(0.21)
    assert pytest.approx(duty, abs=1e-3) == 0.64
    assert direction == Direction.FORWARD


def test_deadband_max_speed_clamping():
    mapper = DeadbandPWMMapper(
        min_pwm=0.28,
        max_pwm=1.00,
        deadband_speed=0.02,
        max_wheel_speed=0.40
    )

    # At max speed
    duty, direction = mapper.map_speed_to_pwm(0.40)
    assert pytest.approx(duty, abs=1e-4) == 1.00
    assert direction == Direction.FORWARD

    # Above max speed (clamped to max_pwm)
    duty, direction = mapper.map_speed_to_pwm(0.85)
    assert pytest.approx(duty, abs=1e-4) == 1.00
    assert direction == Direction.FORWARD

    # Negative above max speed
    duty, direction = mapper.map_speed_to_pwm(-0.90)
    assert pytest.approx(duty, abs=1e-4) == 1.00
    assert direction == Direction.REVERSE


def test_deadband_motor_inversion():
    mapper = DeadbandPWMMapper(min_pwm=0.28, max_pwm=1.00, deadband_speed=0.02, max_wheel_speed=0.40)

    # Normal vs Inverted forward
    _, dir_norm = mapper.map_speed_to_pwm(0.20, invert=False)
    _, dir_inv = mapper.map_speed_to_pwm(0.20, invert=True)
    assert dir_norm == Direction.FORWARD
    assert dir_inv == Direction.REVERSE

    # Normal vs Inverted reverse
    _, dir_norm_rev = mapper.map_speed_to_pwm(-0.20, invert=False)
    _, dir_inv_rev = mapper.map_speed_to_pwm(-0.20, invert=True)
    assert dir_norm_rev == Direction.REVERSE
    assert dir_inv_rev == Direction.FORWARD


# ============================================================================
# 3. TB6612 Driver & Mock Hardware Tests
# ============================================================================

def test_mock_driver_pin_states():
    pins = TB6612PinConfig(stby=22, pwma=12, ain1=5, ain2=6, pwmb=13, bin1=23, bin2=24)
    driver = MockTB6612Driver(pins=pins)

    # Verify initial state
    assert driver.is_enabled is False
    assert driver.stby_val == 0

    # Enable
    driver.enable()
    assert driver.is_enabled is True
    assert driver.stby_val == 1

    # Forward on Left, Reverse on Right
    driver.set_motors(0.5, Direction.FORWARD, 0.75, Direction.REVERSE)
    assert driver.left_duty == 0.5
    assert driver.left_dir == Direction.FORWARD
    assert driver.ain1_val == 1
    assert driver.ain2_val == 0

    assert driver.right_duty == 0.75
    assert driver.right_dir == Direction.REVERSE
    assert driver.bin1_val == 0
    assert driver.bin2_val == 1

    # Stop command
    driver.stop()
    assert driver.left_duty == 0.0
    assert driver.left_dir == Direction.STOP
    assert driver.ain1_val == 0
    assert driver.ain2_val == 0
    assert driver.right_duty == 0.0
    assert driver.right_dir == Direction.STOP
    assert driver.bin1_val == 0
    assert driver.bin2_val == 0

    # Cleanup disables driver
    driver.cleanup()
    assert driver.is_enabled is False
    assert driver.stby_val == 0


def test_driver_fallback_factory():
    # In non-Pi environment without physical GPIO, should return MockTB6612Driver
    driver = create_tb6612_driver(force_mock=False)
    assert isinstance(driver, MockTB6612Driver)
    state = driver.get_state()
    assert state['is_mock'] is True


# ============================================================================
# 4. Watchdog & ROS 2 Integration Tests
# ============================================================================

@pytest.fixture(scope="module")
def ros_context():
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def test_node_twist_processing(ros_context):
    driver = MockTB6612Driver()
    node = MotorDriverNode(driver=driver)

    # Process forward twist
    left_duty, left_dir, right_duty, right_dir = node.process_twist(0.2, 0.0)
    assert left_dir == Direction.FORWARD
    assert right_dir == Direction.FORWARD
    assert left_duty > 0.5
    assert right_duty > 0.5

    # Process in-place spin twist (CCW: left reverse, right forward)
    left_duty, left_dir, right_duty, right_dir = node.process_twist(0.0, 1.0)
    assert left_dir == Direction.REVERSE
    assert right_dir == Direction.FORWARD
    assert left_duty > 0.28
    assert right_duty > 0.28

    node.destroy_node()


def test_node_watchdog_timeout_and_recovery(ros_context):
    driver = MockTB6612Driver()
    node = MotorDriverNode(driver=driver)

    # Simulate receiving cmd_vel
    twist = Twist()
    twist.linear.x = 0.2
    twist.angular.z = 0.0

    node.cmd_vel_callback(twist)

    # Motors should now be active and running
    assert node.motors_active is True
    assert driver.left_dir == Direction.FORWARD
    assert driver.right_dir == Direction.FORWARD
    assert driver.left_duty > 0.0
    assert driver.right_duty > 0.0
    assert node.watchdog_triggered is False

    # Simulate elapsed time > 300 ms (e.g. 350 ms)
    # We artificially set last_cmd_time to 400 ms in the past
    node.last_cmd_time = time.monotonic() - 0.400

    # Trigger watchdog check
    node._watchdog_check()

    # Watchdog should have triggered and cut motors to STOP immediately
    assert node.watchdog_triggered is True
    assert node.motors_active is False
    assert driver.left_duty == 0.0
    assert driver.right_duty == 0.0
    assert driver.left_dir == Direction.STOP
    assert driver.right_dir == Direction.STOP

    # Now simulate Wi-Fi recovery / new cmd_vel arriving
    twist.linear.x = 0.15
    node.cmd_vel_callback(twist)

    # Should recover from failsafe and resume motor control
    assert node.watchdog_triggered is False
    assert node.motors_active is True
    assert driver.left_dir == Direction.FORWARD
    assert driver.right_dir == Direction.FORWARD
    assert driver.left_duty > 0.0

    node.destroy_node()


# ============================================================================
# 5. Robustness & Edge Case Regression Tests
# ============================================================================

def test_deadband_zero_threshold_stops_at_zero():
    """Verify that setting deadband_speed=0.0 does NOT erroneously drive motors in reverse."""
    mapper = DeadbandPWMMapper(min_pwm=0.28, max_pwm=1.00, deadband_speed=0.0, max_wheel_speed=0.40)
    duty, direction = mapper.map_speed_to_pwm(0.0)
    assert duty == 0.0
    assert direction == Direction.STOP


def test_deadband_nan_and_inf_speed_handling():
    """Verify NaN and Inf speed inputs safely resolve to STOP with 0.0 duty."""
    mapper = DeadbandPWMMapper()
    assert mapper.map_speed_to_pwm(float('nan')) == (0.0, Direction.STOP)
    assert mapper.map_speed_to_pwm(float('inf')) == (0.0, Direction.STOP)
    assert mapper.map_speed_to_pwm(float('-inf')) == (0.0, Direction.STOP)


def test_mapper_helper_methods_and_polarity():
    """Verify map_left, map_right, and map_speeds respect motor polarity inversion."""
    mapper = DeadbandPWMMapper(min_pwm=0.28, max_pwm=1.00, deadband_speed=0.02, max_wheel_speed=0.40,
                               invert_left=True, invert_right=False)
    duty_l, dir_l = mapper.map_left(0.20)
    duty_r, dir_r = mapper.map_right(0.20)
    assert dir_l == Direction.REVERSE
    assert dir_r == Direction.FORWARD
    assert duty_l > 0.5
    assert duty_r > 0.5

    (l_res, r_res) = mapper.map_speeds(0.20, 0.20)
    assert l_res == (duty_l, dir_l)
    assert r_res == (duty_r, dir_r)


def test_inverse_kinematics_and_desaturation():
    """Verify inverse kinematics (wheel speeds -> twist) and velocity desaturation."""
    kin = DifferentialDriveKinematics(track_width=0.14, wheel_radius=0.033)

    # Round trip: twist -> wheels -> twist
    v_l, v_r = kin.twist_to_wheel_velocities(0.25, 0.50)
    v_x, omega_z = kin.wheel_velocities_to_twist(v_l, v_r)
    assert pytest.approx(v_x, abs=1e-5) == 0.25
    assert pytest.approx(omega_z, abs=1e-5) == 0.50

    # Desaturation: preserve curvature when one wheel exceeds max speed
    # v_l = 0.20, v_r = 0.60, max_speed = 0.40 -> scale by 0.40 / 0.60 = 2/3
    desat_l, desat_r = kin.desaturate_wheel_velocities(0.20, 0.60, 0.40)
    assert pytest.approx(desat_r, abs=1e-5) == 0.40
    assert pytest.approx(desat_l, abs=1e-5) == 0.20 * (0.40 / 0.60)
    # Curvature ratio preserved
    assert pytest.approx(desat_l / desat_r, abs=1e-5) == (0.20 / 0.60)


def test_kinematics_nan_inf_handling():
    """Verify that invalid Twist components in kinematics return safe zeros."""
    kin = DifferentialDriveKinematics(track_width=0.14, wheel_radius=0.033)
    assert kin.twist_to_wheel_velocities(float('nan'), 0.0) == (0.0, 0.0)
    assert kin.twist_to_wheel_velocities(0.0, float('inf')) == (0.0, 0.0)
    assert kin.wheel_velocities_to_twist(float('nan'), 0.2) == (0.0, 0.0)


def test_node_nan_inf_twist_safety(ros_context):
    """Verify node stops motors safely when a Twist containing NaN or Inf is received."""
    driver = MockTB6612Driver()
    node = MotorDriverNode(driver=driver)

    t = Twist()
    t.linear.x = float('nan')
    t.angular.z = 0.0

    node.cmd_vel_callback(t)
    state = driver.get_state()
    assert state['left_duty'] == 0.0
    assert state['right_duty'] == 0.0
    assert state['left_dir'] == "STOP"
    assert state['right_dir'] == "STOP"
    assert node.motors_active is False

    node.destroy_node()


def test_node_watchdog_timeout_when_already_stopped(ros_context):
    """Verify watchdog timeout properly registers when robot was previously commanded to stop."""
    driver = MockTB6612Driver()
    node = MotorDriverNode(driver=driver)

    t = Twist()
    t.linear.x = 0.0
    t.angular.z = 0.0
    node.cmd_vel_callback(t)
    assert node.watchdog_triggered is False

    # Simulate timeout
    node.last_cmd_time = time.monotonic() - 0.400
    node._watchdog_check()
    assert node.watchdog_triggered is True

    node.destroy_node()


def test_idempotent_cleanup():
    """Verify calling cleanup() multiple times is safe and idempotent."""
    driver = MockTB6612Driver()
    driver.cleanup()
    assert driver.is_cleaned_up is True
    # Second cleanup call should not raise or duplicate teardown
    driver.cleanup()
    assert driver.is_cleaned_up is True

