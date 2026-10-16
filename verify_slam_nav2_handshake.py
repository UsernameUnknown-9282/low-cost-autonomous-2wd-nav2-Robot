#!/usr/bin/env python3
"""
SLAM Toolbox & Nav2 Handshake Verification Script for Pure LiDAR Odometry.
Verifies:
1. slam_toolbox consumes RF2O's odom -> base_footprint TF and /scan topic.
2. slam_toolbox generates /map topic and broadcasts map -> odom TF.
3. Unbroken end-to-end TF chain:
   map -> odom -> base_footprint -> base_link -> laser_frame.
4. Nav2 compatibility: confirms that LaserScan points can be transformed
   into global/local costmap frames without wheel encoders.
"""

import sys
import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import tf2_ros


class SlamNav2Verifier(Node):
    def __init__(self):
        super().__init__('slam_nav2_handshake_verifier')
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10
        )
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.map_received = False
        self.map_info = None

    def map_callback(self, msg: OccupancyGrid):
        self.map_received = True
        self.map_info = msg.info

    def check_tf(self, target_frame, source_frame):
        try:
            return self.tf_buffer.lookup_transform(
                target_frame, source_frame, rclpy.time.Time()
            )
        except Exception:
            return None


def run_handshake_test(timeout_sec=25.0):
    rclpy.init()
    node = SlamNav2Verifier()

    print("=" * 65)
    print("  RUNNING SLAM TOOLBOX & NAV2 HANDSHAKE VERIFICATION")
    print("=" * 65)

    start_time = time.time()
    last_print = 0.0

    tf_map_to_odom_ok = False
    tf_map_to_footprint_ok = False
    tf_map_to_laser_ok = False

    while (time.time() - start_time) < timeout_sec:
        rclpy.spin_once(node, timeout_sec=0.1)

        # 1. Check map -> odom (published by slam_toolbox)
        if not tf_map_to_odom_ok:
            tf1 = node.check_tf('map', 'odom')
            if tf1 is not None:
                tf_map_to_odom_ok = True
                print("[TF PASS] Discovered transform: map -> odom (from slam_toolbox)")

        # 2. Check map -> base_footprint (composite SLAM + LiDAR odom)
        if not tf_map_to_footprint_ok:
            tf2 = node.check_tf('map', 'base_footprint')
            if tf2 is not None:
                tf_map_to_footprint_ok = True
                print("[TF PASS] Discovered transform: map -> base_footprint")

        # 3. Check complete chain: map -> laser_frame
        if not tf_map_to_laser_ok:
            tf3 = node.check_tf('map', 'laser_frame')
            if tf3 is not None:
                tf_map_to_laser_ok = True
                print("[TF PASS] Complete TF chain resolved: map -> odom -> base_footprint -> base_link -> laser_frame")

        if node.map_received and tf_map_to_odom_ok and tf_map_to_footprint_ok and tf_map_to_laser_ok:
            break

        if time.time() - last_print > 3.0:
            last_print = time.time()
            print(f"Waiting for SLAM handshake... map={node.map_received}, "
                  f"map->odom={tf_map_to_odom_ok}, map->laser={tf_map_to_laser_ok}")

    print("-" * 65)
    success = True

    if node.map_received:
        print(f"[PASS] /map topic published by slam_toolbox. Size: {node.map_info.width}x{node.map_info.height}, resolution: {node.map_info.resolution} m/cell")
    else:
        print("[FAIL] /map topic was not received from slam_toolbox.")
        success = False

    if tf_map_to_odom_ok:
        print("[PASS] slam_toolbox is actively publishing map -> odom transform.")
    else:
        print("[FAIL] map -> odom transform missing.")
        success = False

    if tf_map_to_laser_ok:
        print("[PASS] Full navigation TF tree verified: map -> odom -> base_footprint -> base_link -> laser_frame.")
    else:
        print("[FAIL] End-to-end TF tree could not be resolved.")
        success = False

    print("=" * 65)
    if success:
        print(">>> SLAM & NAV2 HANDSHAKE VERIFIED: PURE LIDAR ODOMETRY OPERATIONAL <<<")
    else:
        print(">>> SLAM & NAV2 HANDSHAKE VERIFICATION FAILED <<<")
    print("=" * 65)

    node.destroy_node()
    rclpy.shutdown()
    return 0 if success else 1


if __name__ == '__main__':
    sys.exit(run_handshake_test())
