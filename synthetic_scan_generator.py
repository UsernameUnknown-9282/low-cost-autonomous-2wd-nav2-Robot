#!/usr/bin/env python3
"""
Synthetic 2D LiDAR Scan Generator & Static TF Publisher for ROS 2.
Simulates a robot moving inside an enclosed environment (e.g. 6m x 6m room),
generating realistic LaserScan messages and broadcasting base_footprint -> base_link -> laser_frame
static transforms. Used for headless verification of LiDAR odometry (RF2O), SLAM, and Nav2
without requiring physical hardware or a heavy GUI simulator.
"""

import sys
import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


class SyntheticScanGenerator(Node):
    def __init__(self):
        super().__init__('synthetic_scan_generator')

        # Parameters
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('laser_frame', 'laser_frame')
        self.declare_parameter('base_link_frame', 'base_link')
        self.declare_parameter('base_footprint_frame', 'base_footprint')
        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('room_size_x', 6.0)
        self.declare_parameter('room_size_y', 6.0)
        self.declare_parameter('num_readings', 360)
        self.declare_parameter('linear_velocity', 0.10)   # 0.10 m/s forward
        self.declare_parameter('angular_velocity', 0.05)  # 0.05 rad/s yaw

        self.scan_topic = self.get_parameter('scan_topic').value
        self.laser_frame = self.get_parameter('laser_frame').value
        self.base_link_frame = self.get_parameter('base_link_frame').value
        self.base_footprint_frame = self.get_parameter('base_footprint_frame').value
        self.rate_hz = self.get_parameter('rate_hz').value
        self.room_x = self.get_parameter('room_size_x').value / 2.0
        self.room_y = self.get_parameter('room_size_y').value / 2.0
        self.num_readings = self.get_parameter('num_readings').value
        self.v_x = self.get_parameter('linear_velocity').value
        self.v_w = self.get_parameter('angular_velocity').value

        # Robot simulated ground-truth state
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.last_update_time = self.get_clock().now()

        # Static TF Broadcaster: base_footprint -> base_link -> laser_frame
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.publish_static_transforms()
        # Periodic static TF publish (1 Hz) to ensure late-joining nodes immediately have TF
        self.tf_timer = self.create_timer(1.0, self.publish_static_transforms)

        # LaserScan publisher using standard ROS 2 SensorDataQoS (Best Effort, Volatile)
        from rclpy.qos import qos_profile_sensor_data
        self.scan_pub = self.create_publisher(LaserScan, self.scan_topic, qos_profile_sensor_data)
        timer_period = 1.0 / self.rate_hz
        self.timer = self.create_timer(timer_period, self.publish_scan)

        self.get_logger().info(
            f"SyntheticScanGenerator started on {self.scan_topic} at {self.rate_hz} Hz. "
            f"Frames: {self.base_footprint_frame} -> {self.base_link_frame} -> {self.laser_frame}"
        )

    def publish_static_transforms(self):
        now = self.get_clock().now().to_msg()
        tfs = []

        # 1. base_footprint -> base_link (chassis height: 0.033 m = wheel radius)
        tf_footprint_to_link = TransformStamped()
        tf_footprint_to_link.header.stamp = now
        tf_footprint_to_link.header.frame_id = self.base_footprint_frame
        tf_footprint_to_link.child_frame_id = self.base_link_frame
        tf_footprint_to_link.transform.translation.x = 0.0
        tf_footprint_to_link.transform.translation.y = 0.0
        tf_footprint_to_link.transform.translation.z = 0.033
        tf_footprint_to_link.transform.rotation.x = 0.0
        tf_footprint_to_link.transform.rotation.y = 0.0
        tf_footprint_to_link.transform.rotation.z = 0.0
        tf_footprint_to_link.transform.rotation.w = 1.0
        tfs.append(tf_footprint_to_link)

        # 2. base_link -> laser_frame (LiDAR mount: x=0.04m, z=0.10m above chassis)
        tf_link_to_laser = TransformStamped()
        tf_link_to_laser.header.stamp = now
        tf_link_to_laser.header.frame_id = self.base_link_frame
        tf_link_to_laser.child_frame_id = self.laser_frame
        tf_link_to_laser.transform.translation.x = 0.04
        tf_link_to_laser.transform.translation.y = 0.0
        tf_link_to_laser.transform.translation.z = 0.10
        tf_link_to_laser.transform.rotation.x = 0.0
        tf_link_to_laser.transform.rotation.y = 0.0
        tf_link_to_laser.transform.rotation.z = 0.0
        tf_link_to_laser.transform.rotation.w = 1.0
        tfs.append(tf_link_to_laser)

        self.tf_static_broadcaster.sendTransform(tfs)

    def step_simulation(self):
        now = self.get_clock().now()
        dt = (now - self.last_update_time).nanoseconds * 1e-9
        self.last_update_time = now

        # Update pose based on commanded velocities
        self.robot_x += self.v_x * math.cos(self.robot_yaw) * dt
        self.robot_y += self.v_x * math.sin(self.robot_yaw) * dt
        self.robot_yaw += self.v_w * dt

        # Wrap yaw
        self.robot_yaw = math.atan2(math.sin(self.robot_yaw), math.cos(self.robot_yaw))

        # Bounce if near wall
        limit_x = self.room_x - 0.5
        limit_y = self.room_y - 0.5
        if abs(self.robot_x) > limit_x or abs(self.robot_y) > limit_y:
            self.v_x = -self.v_x

    def publish_scan(self):
        self.step_simulation()
        now = self.get_clock().now()

        scan = LaserScan()
        scan.header.stamp = now.to_msg()
        scan.header.frame_id = self.laser_frame

        angle_min = -math.pi
        angle_max = math.pi
        angle_inc = (angle_max - angle_min) / float(self.num_readings)

        scan.angle_min = angle_min
        scan.angle_max = angle_max
        scan.angle_increment = angle_inc
        scan.time_increment = 0.0
        scan.scan_time = 1.0 / self.rate_hz
        scan.range_min = 0.05
        scan.range_max = 12.0

        # Laser sensor position in global frame (approx robot_x + mount offset)
        laser_gx = self.robot_x + 0.04 * math.cos(self.robot_yaw)
        laser_gy = self.robot_y + 0.04 * math.sin(self.robot_yaw)

        ranges = []
        intensities = []

        for i in range(self.num_readings):
            beam_angle = angle_min + i * angle_inc
            global_angle = self.robot_yaw + beam_angle

            cos_a = math.cos(global_angle)
            sin_a = math.sin(global_angle)

            # Raycast against 4 rectangular room walls
            candidates = []
            if abs(cos_a) > 1e-6:
                # East wall (x = +room_x)
                d1 = (self.room_x - laser_gx) / cos_a
                if d1 > 0:
                    y_hit = laser_gy + d1 * sin_a
                    if -self.room_y <= y_hit <= self.room_y:
                        candidates.append(d1)
                # West wall (x = -room_x)
                d2 = (-self.room_x - laser_gx) / cos_a
                if d2 > 0:
                    y_hit = laser_gy + d2 * sin_a
                    if -self.room_y <= y_hit <= self.room_y:
                        candidates.append(d2)

            if abs(sin_a) > 1e-6:
                # North wall (y = +room_y)
                d3 = (self.room_y - laser_gy) / sin_a
                if d3 > 0:
                    x_hit = laser_gx + d3 * cos_a
                    if -self.room_x <= x_hit <= self.room_x:
                        candidates.append(d3)
                # South wall (y = -room_y)
                d4 = (-self.room_y - laser_gy) / sin_a
                if d4 > 0:
                    x_hit = laser_gx + d4 * cos_a
                    if -self.room_x <= x_hit <= self.room_x:
                        candidates.append(d4)

            dist = min(candidates) if candidates else 10.0
            # Add minor noise (+- 2mm)
            dist += np.random.normal(0, 0.002)
            dist = max(scan.range_min, min(scan.range_max, dist))

            ranges.append(float(dist))
            intensities.append(200.0)

        scan.ranges = ranges
        scan.intensities = intensities
        self.scan_pub.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = SyntheticScanGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
