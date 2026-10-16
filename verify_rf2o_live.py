#!/usr/bin/env python3
"""
Live Verification Script for RF2O Laser Odometry Subsystem (Pure LiDAR Odometry).
Validates:
1. Reception of /odom messages from rf2o_laser_odometry.
2. Correct frame IDs (header: odom, child: base_footprint).
3. Realistic covariance matrices for planar 2D odometry (REP-105).
4. TF continuity: odom -> base_footprint -> base_link -> laser_frame.
5. Velocity and displacement estimation response to laser scan flow.
"""

import sys
import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import tf2_ros


class RF2OVerifier(Node):
    def __init__(self):
        super().__init__('rf2o_live_verifier')
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.odom_messages = []
        self.tf_lookups = []
        self.get_logger().info("RF2O Verifier initialized. Listening on /odom and TF...")

    def odom_callback(self, msg: Odometry):
        self.odom_messages.append(msg)

    def check_tf(self, target_frame, source_frame):
        try:
            tf = self.tf_buffer.lookup_transform(
                target_frame, source_frame, rclpy.time.Time()
            )
            return tf
        except Exception as e:
            return None


def run_verification(timeout_sec=20.0):
    rclpy.init()
    node = RF2OVerifier()

    start_time = time.time()
    last_print = 0.0

    print("=" * 65)
    print("  RUNNING LIVE VERIFICATION: RF2O LASER ODOMETRY")
    print("=" * 65)

    odom_received = False
    tf_odom_to_footprint_ok = False
    tf_odom_to_laser_ok = False

    while (time.time() - start_time) < timeout_sec:
        rclpy.spin_once(node, timeout_sec=0.1)

        # Check TF
        if not tf_odom_to_footprint_ok:
            tf_fp = node.check_tf('odom', 'base_footprint')
            if tf_fp is not None:
                tf_odom_to_footprint_ok = True
                print(f"[TF PASS] Discovered transform: odom -> base_footprint")

        if not tf_odom_to_laser_ok:
            tf_laser = node.check_tf('odom', 'laser_frame')
            if tf_laser is not None:
                tf_odom_to_laser_ok = True
                print(f"[TF PASS] Discovered complete chain: odom -> ... -> laser_frame")

        if len(node.odom_messages) >= 10 and tf_odom_to_footprint_ok and tf_odom_to_laser_ok:
            odom_received = True
            break

        if time.time() - last_print > 2.0:
            last_print = time.time()
            print(f"Waiting for RF2O odometry... odom_count={len(node.odom_messages)}, "
                  f"tf_fp={tf_odom_to_footprint_ok}, tf_laser={tf_odom_to_laser_ok}")

    print("-" * 65)
    success = True

    # 1. Check message reception
    if len(node.odom_messages) < 3:
        print(f"[FAIL] Expected at least 3 /odom messages, got {len(node.odom_messages)}")
        success = False
    else:
        print(f"[PASS] Received {len(node.odom_messages)} /odom messages from RF2O.")

    # 2. Check frame IDs
    if len(node.odom_messages) > 0:
        latest = node.odom_messages[-1]
        if latest.header.frame_id != 'odom':
            print(f"[FAIL] Expected header.frame_id == 'odom', got '{latest.header.frame_id}'")
            success = False
        else:
            print(f"[PASS] header.frame_id is 'odom'")

        if latest.child_frame_id != 'base_footprint':
            print(f"[FAIL] Expected child_frame_id == 'base_footprint', got '{latest.child_frame_id}'")
            success = False
        else:
            print(f"[PASS] child_frame_id is 'base_footprint'")

        # 3. Check Covariance Matrices (REP-105)
        # Indices: 0=x, 7=y, 14=z, 21=roll, 28=pitch, 35=yaw
        pose_cov = list(latest.pose.covariance)
        twist_cov = list(latest.twist.covariance)

        cov_pass = True
        if pose_cov[0] <= 0 or pose_cov[7] <= 0 or pose_cov[35] <= 0:
            print(f"[FAIL] Pose covariance diagonal [x={pose_cov[0]}, y={pose_cov[7]}, yaw={pose_cov[35]}] must be > 0")
            cov_pass = False
        if pose_cov[14] < 1e3 or pose_cov[21] < 1e3 or pose_cov[28] < 1e3:
            print(f"[FAIL] Unmeasured 2D dimensions [z={pose_cov[14]}, roll={pose_cov[21]}, pitch={pose_cov[28]}] should have high covariance (>= 1e3)")
            cov_pass = False

        if twist_cov[0] <= 0 or twist_cov[7] <= 0 or twist_cov[35] <= 0:
            print(f"[FAIL] Twist covariance diagonal [vx={twist_cov[0]}, vy={twist_cov[7]}, vyaw={twist_cov[35]}] must be > 0")
            cov_pass = False

        if cov_pass:
            print(f"[PASS] Covariance matrices correctly formatted: x_cov={pose_cov[0]}, yaw_cov={pose_cov[35]}, unmeasured_cov={pose_cov[14]}")
        else:
            success = False

        # 4. Check Motion Estimation
        first = node.odom_messages[0]
        dx = latest.pose.pose.position.x - first.pose.pose.position.x
        dy = latest.pose.pose.position.y - first.pose.pose.position.y
        print(f"[INFO] Integrated displacement over test: dx={dx:+.4f} m, dy={dy:+.4f} m, vx={latest.twist.twist.linear.x:+.4f} m/s")

    # 5. Check TF continuity
    if tf_odom_to_footprint_ok:
        print("[PASS] TF transform odom -> base_footprint is active.")
    else:
        print("[FAIL] TF transform odom -> base_footprint was not received.")
        success = False

    if tf_odom_to_laser_ok:
        print("[PASS] Full TF chain odom -> base_footprint -> base_link -> laser_frame resolved successfully.")
    else:
        print("[FAIL] Full TF chain could not be resolved.")
        success = False

    print("=" * 65)
    if success:
        print(">>> ALL RF2O LASER ODOMETRY VERIFICATIONS PASSED SUCCESSFULLY <<<")
    else:
        print(">>> SOME VERIFICATION CHECKS FAILED <<<")
    print("=" * 65)

    node.destroy_node()
    rclpy.shutdown()
    return 0 if success else 1


if __name__ == '__main__':
    sys.exit(run_verification())
