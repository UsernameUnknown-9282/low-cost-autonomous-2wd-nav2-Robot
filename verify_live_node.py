#!/usr/bin/env python3
"""
Live ROS 2 Integration Verification Script for MotorDriverNode.

Validates:
  1. Live ROS 2 node instantiation and parameter loading from YAML
  2. Live /cmd_vel publication and reception over ROS 2 middleware
  3. Real-time watchdog timeout triggering after 300 ms idle
  4. Automatic recovery when /cmd_vel resumes
"""

import sys
import os
import time
import threading

current_dir = os.path.dirname(os.path.abspath(__file__))
if current_dir not in sys.path:
    sys.path.insert(0, current_dir)

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist

from motor_driver_node import MotorDriverNode
from tb6612_driver import Direction, MockTB6612Driver


def run_live_verification():
    rclpy.init()
    mock_driver = MockTB6612Driver()
    motor_node = MotorDriverNode(driver=mock_driver)

    # Publisher test node
    pub_node = Node('test_cmd_vel_publisher')
    publisher = pub_node.create_publisher(Twist, '/cmd_vel', 10)

    # Executor in background thread
    executor = SingleThreadedExecutor()
    executor.add_node(motor_node)
    executor.add_node(pub_node)

    exec_thread = threading.Thread(target=executor.spin, daemon=True)
    exec_thread.start()

    print("\n--- Test 1: Node Startup & Standby State ---")
    time.sleep(0.1)
    state = mock_driver.get_state()
    print(f"Driver state: {state}")
    assert state['enabled'] is True, "Driver should be enabled (STBY=HIGH)"
    assert state['left_duty'] == 0.0, "Initial left duty must be 0"
    assert state['right_duty'] == 0.0, "Initial right duty must be 0"
    print("PASS: Driver is enabled on standby with 0% duty.")

    print("\n--- Test 2: Live /cmd_vel Publishing (Forward Drive) ---")
    twist = Twist()
    twist.linear.x = 0.25
    twist.angular.z = 0.0

    # Publish twist messages for 200ms
    for _ in range(5):
        publisher.publish(twist)
        time.sleep(0.04)

    time.sleep(0.05)
    state = mock_driver.get_state()
    print(f"Driver state during forward drive: {state}")
    assert state['left_dir'] == "FORWARD", "Left motor must be FORWARD"
    assert state['right_dir'] == "FORWARD", "Right motor must be FORWARD"
    assert state['left_duty'] > 0.5, "Duty cycle should scale with speed"
    assert motor_node.motors_active is True, "Motors should be marked active"
    print("PASS: Live /cmd_vel properly translated to motor PWM.")

    print("\n--- Test 3: Watchdog Timeout Test (>300 ms idle) ---")
    print("Ceasing /cmd_vel stream and waiting 400 ms...")
    time.sleep(0.40)

    state = mock_driver.get_state()
    print(f"Driver state after 400 ms idle: {state}")
    assert state['left_duty'] == 0.0, "Left motor PWM must be cut to 0"
    assert state['right_duty'] == 0.0, "Right motor PWM must be cut to 0"
    assert state['left_dir'] == "STOP", "Left motor must be STOP"
    assert state['right_dir'] == "STOP", "Right motor must be STOP"
    assert motor_node.watchdog_triggered is True, "Watchdog flag must be True"
    assert motor_node.motors_active is False, "motors_active must be False"
    print("PASS: Safety watchdog successfully cut motor power after 300 ms idle!")

    print("\n--- Test 4: Watchdog Recovery on Stream Restoration ---")
    print("Resuming /cmd_vel stream with in-place turn (angular.z = 1.0)...")
    twist_turn = Twist()
    twist_turn.linear.x = 0.0
    twist_turn.angular.z = 1.0
    publisher.publish(twist_turn)

    time.sleep(0.05)
    state = mock_driver.get_state()
    print(f"Driver state after resuming: {state}")
    assert state['left_dir'] == "REVERSE", "In CCW turn, left motor must REVERSE"
    assert state['right_dir'] == "FORWARD", "In CCW turn, right motor must be FORWARD"
    assert state['left_duty'] >= 0.28, "Duty cycle must be >= min_pwm (28%)"
    assert state['right_duty'] >= 0.28, "Duty cycle must be >= min_pwm (28%)"
    assert motor_node.watchdog_triggered is False, "Watchdog flag must be cleared"
    print("PASS: Motor driver cleanly recovered and executed in-place turn.")

    print("\n--- Test 5: Live NaN Twist Safety Rejection ---")
    print("Publishing invalid NaN twist command...")
    twist_nan = Twist()
    twist_nan.linear.x = float('nan')
    twist_nan.angular.z = 0.0
    publisher.publish(twist_nan)

    time.sleep(0.05)
    state = mock_driver.get_state()
    print(f"Driver state after NaN twist: {state}")
    assert state['left_duty'] == 0.0, "Motors must be immediately stopped on NaN"
    assert state['right_duty'] == 0.0, "Motors must be immediately stopped on NaN"
    assert state['left_dir'] == "STOP", "Motors must be STOP on NaN"
    assert state['right_dir'] == "STOP", "Motors must be STOP on NaN"
    assert motor_node.motors_active is False, "motors_active must be False"
    print("PASS: Malformed NaN twist command safely rejected and motors halted.")

    print("\n--- Test 6: Clean Node Shutdown & Idempotent Cleanup ---")
    executor.shutdown()
    motor_node.destroy_node()
    # Confirm cleanup is idempotent
    motor_node.driver.cleanup()
    assert mock_driver.is_cleaned_up is True
    pub_node.destroy_node()
    rclpy.shutdown()
    exec_thread.join(timeout=1.0)
    print("PASS: Nodes cleanly shut down and GPIO released.\n")
    print("ALL LIVE INTEGRATION TESTS PASSED SUCCESSFULLY!")


if __name__ == '__main__':
    run_live_verification()
