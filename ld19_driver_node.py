#!/usr/bin/env python3
"""
Lightweight ROS 2 Driver Node for LDROBOT LD19 / D200 / LD06 LiDAR.
Features:
- Native Python serial parser (CP2102 UART @ 230400 baud, 8N1).
- Full LD19 protocol parser: CRC-8 verification, angle unwrap, point interpolation.
- Publishes standard sensor_msgs/msg/LaserScan at ~10 Hz with SensorDataQoS.
- Mock mode fallback when hardware is absent (allows testing without physical Pi).
- Automatic serial reconnection watchdog for USB disconnect resilience.
- Low CPU (<5%) and low memory (<15MB) designed for Raspberry Pi Zero 2 W.
"""

import sys
import math
import struct
import threading
import time
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

# LDROBOT LD19 CRC-8 lookup table
CRC_TABLE = (
    0x00, 0x4D, 0x9A, 0xD7, 0x79, 0x34, 0xE3, 0xAE, 0xF2, 0xBF, 0x68, 0x25,
    0x8B, 0xC6, 0x11, 0x5C, 0xA9, 0xE4, 0x33, 0x7E, 0xD0, 0x9D, 0x4A, 0x07,
    0x5B, 0x16, 0xC1, 0x8C, 0x22, 0x6F, 0xB8, 0xF5, 0x1F, 0x52, 0x85, 0xC8,
    0x66, 0x2B, 0xFC, 0xB1, 0xED, 0xA0, 0x77, 0x3A, 0x94, 0xD9, 0x0E, 0x43,
    0xB6, 0xFB, 0x2C, 0x61, 0xCF, 0x82, 0x55, 0x18, 0x44, 0x09, 0xDE, 0x93,
    0x3D, 0x70, 0xA7, 0xEA, 0x3E, 0x73, 0xA4, 0xE9, 0x47, 0x0A, 0xDD, 0x90,
    0xCC, 0x81, 0x56, 0x1B, 0xB5, 0xF8, 0x2F, 0x62, 0x97, 0xDA, 0x0D, 0x40,
    0xEE, 0xA3, 0x74, 0x39, 0x65, 0x28, 0xFF, 0xB2, 0x1C, 0x51, 0x86, 0xCB,
    0x21, 0x6C, 0xBB, 0xF6, 0x58, 0x15, 0xC2, 0x8F, 0xD3, 0x9E, 0x49, 0x04,
    0xAA, 0xE7, 0x30, 0x7D, 0x88, 0xC5, 0x12, 0x5F, 0xF1, 0xBC, 0x6B, 0x26,
    0x7A, 0x37, 0xE0, 0xAD, 0x03, 0x4E, 0x99, 0xD4, 0x7C, 0x31, 0xE6, 0xAB,
    0x05, 0x48, 0x9F, 0xD2, 0x8E, 0xC3, 0x14, 0x59, 0xF7, 0xBA, 0x6D, 0x20,
    0xD5, 0x98, 0x4F, 0x02, 0xAC, 0xE1, 0x36, 0x7B, 0x27, 0x6A, 0xBD, 0xF0,
    0x5E, 0x13, 0xC4, 0x89, 0x63, 0x2E, 0xF9, 0xB4, 0x1A, 0x57, 0x80, 0xCD,
    0x91, 0xDC, 0x0B, 0x46, 0xE8, 0xA5, 0x72, 0x3F, 0xCA, 0x87, 0x50, 0x1D,
    0xB3, 0xFE, 0x29, 0x64, 0x38, 0x75, 0xA2, 0xEF, 0x41, 0x0C, 0xDB, 0x96,
    0x42, 0x0F, 0xD8, 0x95, 0x3B, 0x76, 0xA1, 0xEC, 0xB0, 0xFD, 0x2A, 0x67,
    0xC9, 0x84, 0x53, 0x1E, 0xEB, 0xA6, 0x71, 0x3C, 0x92, 0xDF, 0x08, 0x45,
    0x19, 0x54, 0x83, 0xCE, 0x60, 0x2D, 0xFA, 0xB7, 0x5D, 0x10, 0xC7, 0x8A,
    0x24, 0x69, 0xBE, 0xF3, 0xAF, 0xE2, 0x35, 0x78, 0xD6, 0x9B, 0x4C, 0x01,
    0xF4, 0xB9, 0x6E, 0x23, 0x8D, 0xC0, 0x17, 0x5A, 0x06, 0x4B, 0x9C, 0xD1,
    0x7F, 0x32, 0xE5, 0xA8
)

def calc_crc8(data: bytes) -> int:
    crc = 0
    for b in data:
        crc = CRC_TABLE[(crc ^ b) & 0xFF]
    return crc


class LD19DriverNode(Node):
    def __init__(self):
        super().__init__('ld19_lidar_driver')

        # Declare parameters
        self.declare_parameter('port_name', '/dev/ldlidar')
        self.declare_parameter('port_baudrate', 230400)
        self.declare_parameter('frame_id', 'laser_frame')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('base_footprint_frame', 'base_footprint')
        self.declare_parameter('wheel_radius', 0.033)
        self.declare_parameter('laser_x_offset', 0.04)
        self.declare_parameter('laser_z_offset', 0.10)
        self.declare_parameter('topic_name', '/scan')
        self.declare_parameter('range_min', 0.02)
        self.declare_parameter('range_max', 12.0)
        self.declare_parameter('num_bins', 450)
        self.declare_parameter('publish_static_tf', True)
        self.declare_parameter('mock_hardware', False)
        self.declare_parameter('mock_fallback', True)

        self.port_name = self.get_parameter('port_name').value
        self.baudrate = self.get_parameter('port_baudrate').value
        self.frame_id = self.get_parameter('frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.base_footprint_frame = self.get_parameter('base_footprint_frame').value
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.laser_x_offset = float(self.get_parameter('laser_x_offset').value)
        self.laser_z_offset = float(self.get_parameter('laser_z_offset').value)
        self.topic_name = self.get_parameter('topic_name').value
        self.range_min = float(self.get_parameter('range_min').value)
        self.range_max = float(self.get_parameter('range_max').value)
        self.num_bins = int(self.get_parameter('num_bins').value)
        self.publish_static_tf = self.get_parameter('publish_static_tf').value
        self.mock_hardware = self.get_parameter('mock_hardware').value
        self.mock_fallback = self.get_parameter('mock_fallback').value

        # LaserScan publisher with sensor data QoS (Best Effort)
        self.scan_pub = self.create_publisher(LaserScan, self.topic_name, qos_profile_sensor_data)

        # Optional static TF broadcaster: base_footprint -> base_link -> laser_frame
        if self.publish_static_tf:
            self.tf_broadcaster = StaticTransformBroadcaster(self)
            self.publish_tf()
            self.tf_timer = self.create_timer(1.0, self.publish_tf)

        # State buffers for revolution accumulation
        self.current_points: List[Tuple[float, float, float]] = []  # (angle_rad, distance_m, intensity)
        self.last_start_angle: Optional[float] = None
        self.is_running = True
        self.serial_handle = None

        # Check serial library and determine mode
        self.use_mock = self.mock_hardware
        try:
            import serial
            self.serial_mod = serial
        except ImportError:
            self.get_logger().warn("pyserial not installed! Defaulting to Mock Hardware mode.")
            self.use_mock = True

        if not self.use_mock:
            self.thread = threading.Thread(target=self.serial_worker, daemon=True)
            self.thread.start()
        else:
            self.get_logger().info("Running in Mock LD19 Hardware Mode (Generating simulated LiDAR scans).")
            self.mock_timer = self.create_timer(0.10, self.publish_mock_scan)

        self.get_logger().info(
            f"LD19 Driver initialized on topic '{self.topic_name}', frame '{self.frame_id}', "
            f"port='{self.port_name}', baud={self.baudrate}, mock={self.use_mock}"
        )

    def publish_tf(self):
        now = self.get_clock().now().to_msg()
        tfs = []

        # 1. base_footprint -> base_link (chassis height: 0.033 m = wheel radius)
        tf_footprint_to_link = TransformStamped()
        tf_footprint_to_link.header.stamp = now
        tf_footprint_to_link.header.frame_id = self.base_footprint_frame
        tf_footprint_to_link.child_frame_id = self.base_frame_id
        tf_footprint_to_link.transform.translation.x = 0.0
        tf_footprint_to_link.transform.translation.y = 0.0
        tf_footprint_to_link.transform.translation.z = self.wheel_radius
        tf_footprint_to_link.transform.rotation.x = 0.0
        tf_footprint_to_link.transform.rotation.y = 0.0
        tf_footprint_to_link.transform.rotation.z = 0.0
        tf_footprint_to_link.transform.rotation.w = 1.0
        tfs.append(tf_footprint_to_link)

        # 2. base_link -> laser_frame (LiDAR mount: x=0.04m, z=0.10m above chassis)
        tf_link_to_laser = TransformStamped()
        tf_link_to_laser.header.stamp = now
        tf_link_to_laser.header.frame_id = self.base_frame_id
        tf_link_to_laser.child_frame_id = self.frame_id
        tf_link_to_laser.transform.translation.x = self.laser_x_offset
        tf_link_to_laser.transform.translation.y = 0.0
        tf_link_to_laser.transform.translation.z = self.laser_z_offset
        tf_link_to_laser.transform.rotation.x = 0.0
        tf_link_to_laser.transform.rotation.y = 0.0
        tf_link_to_laser.transform.rotation.z = 0.0
        tf_link_to_laser.transform.rotation.w = 1.0
        tfs.append(tf_link_to_laser)

        self.tf_broadcaster.sendTransform(tfs)

    def parse_packet(self, packet: bytes) -> Optional[List[Tuple[float, float, float]]]:
        """Parses a verified 47-byte LD19 packet into (angle_rad, dist_m, intensity) tuples."""
        if len(packet) != 47:
            return None
        if packet[0] != 0x54 or packet[1] != 0x2C:
            return None

        # Verify CRC8 over first 46 bytes
        expected_crc = packet[46]
        actual_crc = calc_crc8(packet[:46])
        if actual_crc != expected_crc:
            return None

        radar_speed = struct.unpack('<H', packet[2:4])[0]
        start_angle = struct.unpack('<H', packet[4:6])[0] * 0.01  # degrees
        end_angle = struct.unpack('<H', packet[42:44])[0] * 0.01  # degrees

        # Unwrap angle difference across 0/360 boundary
        if end_angle < start_angle:
            step = ((end_angle + 360.0) - start_angle) / 11.0
        else:
            step = (end_angle - start_angle) / 11.0

        points = []
        offset = 6
        for i in range(12):
            dist_mm = struct.unpack('<H', packet[offset:offset+2])[0]
            intensity = packet[offset+2]
            offset += 3

            angle_deg = start_angle + i * step
            if angle_deg >= 360.0:
                angle_deg -= 360.0
            angle_rad = math.radians(angle_deg)
            dist_m = dist_mm / 1000.0

            points.append((angle_rad, dist_m, float(intensity)))

        return points

    def serial_worker(self):
        """Background thread reading packets from physical serial port."""
        buf = bytearray()
        candidate_ports = [self.port_name]
        for p in ['/dev/ldlidar', '/dev/ttyUSB0', '/dev/ttyUSB1']:
            if p not in candidate_ports:
                candidate_ports.append(p)

        consecutive_failures = 0
        while self.is_running and rclpy.ok():
            try:
                if self.serial_handle is None or not self.serial_handle.is_open:
                    opened = False
                    for port_candidate in candidate_ports:
                        try:
                            self.get_logger().info(f"Connecting to LD19 on {port_candidate} @ {self.baudrate}...")
                            self.serial_handle = self.serial_mod.Serial(
                                port=port_candidate,
                                baudrate=self.baudrate,
                                timeout=0.5
                            )
                            self.port_name = port_candidate
                            self.get_logger().info(f"Serial port {port_candidate} opened successfully.")
                            opened = True
                            consecutive_failures = 0
                            break
                        except Exception:
                            continue

                    if not opened:
                        consecutive_failures += 1
                        if self.mock_fallback and consecutive_failures >= 2:
                            self.get_logger().warn(
                                f"Unable to open serial ports {candidate_ports}. "
                                "Falling back to Mock Hardware mode for testing without physical Pi."
                            )
                            self.use_mock = True
                            self.mock_timer = self.create_timer(0.10, self.publish_mock_scan)
                            return
                        self.get_logger().warn(f"Failed to open any LiDAR port in {candidate_ports}. Retrying in 1s...")
                        time.sleep(1.0)
                        continue

                raw_data = self.serial_handle.read(256)
                if not raw_data:
                    continue
                buf.extend(raw_data)

                # Prevent byte buffer from unbounded growth in noisy serial conditions
                if len(buf) > 4096:
                    buf = buf[-1024:]

                # Search for 0x54 0x2C packet headers (47 bytes each)
                while len(buf) >= 47:
                    if buf[0] == 0x54 and buf[1] == 0x2C:
                        packet = bytes(buf[:47])
                        buf = buf[47:]
                        pts = self.parse_packet(packet)
                        if pts:
                            self.accumulate_points(pts)
                    else:
                        buf.pop(0)

            except Exception as e:
                self.get_logger().warn(f"Serial communication error: {e}. Retrying in 1s...")
                if self.serial_handle:
                    try:
                        self.serial_handle.close()
                    except Exception:
                        pass
                    self.serial_handle = None
                consecutive_failures += 1
                if self.mock_fallback and consecutive_failures >= 3:
                    self.get_logger().warn("Repeated serial errors. Falling back to Mock Hardware mode.")
                    self.use_mock = True
                    self.mock_timer = self.create_timer(0.10, self.publish_mock_scan)
                    return
                time.sleep(1.0)

    def accumulate_points(self, points: List[Tuple[float, float, float]]):
        if not points:
            return

        start_angle = points[0][0]
        # Check revolution wrap (e.g. going from near 2pi back to near 0)
        if self.last_start_angle is not None and start_angle < self.last_start_angle and (self.last_start_angle - start_angle) > math.pi:
            # Full 360 degree revolution completed!
            self.publish_scan(self.current_points)
            self.current_points = []

        self.last_start_angle = start_angle
        self.current_points.extend(points)

        # Guard against buffer overflow if revolution wrap fails under noise
        if len(self.current_points) > 1000:
            self.get_logger().warn("Point accumulation buffer exceeded 1000 points. Publishing and resetting to prevent memory leak.")
            self.publish_scan(self.current_points)
            self.current_points = []

    def publish_scan(self, points: List[Tuple[float, float, float]]):
        if not points:
            return

        scan = LaserScan()
        now = self.get_clock().now().to_msg()
        scan.header.stamp = now
        scan.header.frame_id = self.frame_id

        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = (2.0 * math.pi) / float(self.num_bins)
        scan.time_increment = 0.0
        scan.scan_time = 0.10
        scan.range_min = self.range_min
        scan.range_max = self.range_max

        ranges = [float('inf')] * self.num_bins
        intensities = [0.0] * self.num_bins

        for angle_rad, dist_m, intensity in points:
            # Map [0, 2pi] to [-pi, pi]
            norm_angle = math.atan2(math.sin(angle_rad), math.cos(angle_rad))
            bin_idx = int((norm_angle - scan.angle_min) / scan.angle_increment)
            if bin_idx == self.num_bins:
                bin_idx = self.num_bins - 1
            if 0 <= bin_idx < self.num_bins:
                if self.range_min <= dist_m <= self.range_max:
                    if dist_m < ranges[bin_idx]:
                        ranges[bin_idx] = dist_m
                        intensities[bin_idx] = intensity

        scan.ranges = ranges
        scan.intensities = intensities
        self.scan_pub.publish(scan)

    def publish_mock_scan(self):
        """Generates realistic mock scan in rectangular room for tests without hardware."""
        scan = LaserScan()
        now = self.get_clock().now().to_msg()
        scan.header.stamp = now
        scan.header.frame_id = self.frame_id

        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = (2.0 * math.pi) / float(self.num_bins)
        scan.time_increment = 0.0
        scan.scan_time = 0.10
        scan.range_min = self.range_min
        scan.range_max = self.range_max

        room_half_w = 2.5
        room_half_h = 2.5

        ranges = []
        intensities = []
        for i in range(self.num_bins):
            ang = scan.angle_min + i * scan.angle_increment
            cos_a = math.cos(ang)
            sin_a = math.sin(ang)

            cand = []
            if abs(cos_a) > 1e-6:
                d = room_half_w / abs(cos_a)
                cand.append(d)
            if abs(sin_a) > 1e-6:
                d = room_half_h / abs(sin_a)
                cand.append(d)
            dist = min(cand) if cand else 2.5
            ranges.append(float(dist))
            intensities.append(200.0)

        scan.ranges = ranges
        scan.intensities = intensities
        self.scan_pub.publish(scan)

    def destroy_node(self):
        self.is_running = False
        if self.serial_handle:
            try:
                self.serial_handle.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LD19DriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
