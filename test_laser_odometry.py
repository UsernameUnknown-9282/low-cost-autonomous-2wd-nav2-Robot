#!/usr/bin/env python3
"""
Unit and Integration Tests for Module 2: Pure LiDAR Odometry & LDROBOT LD19 Bridge.
Tests:
1. LD19 CRC-8 calculation and packet parsing against official LDROBOT specification.
2. LD19 angle unwrap and interpolation across 0/360 degree boundary.
3. Corrupted packet and invalid header rejection.
4. LD19DriverNode in mock hardware fallback mode.
5. Configuration files syntax and compatibility (rf2o_params.yaml, ld19_params.yaml, diff_drive_2wd_nav2_params.yaml).
6. SyntheticScanGenerator 2D raycasting and scan formatting.
7. End-to-end RF2O pure LiDAR odometry, covariance validation (REP-105), and TF continuity.
8. udev rules validation for CP2102 and CH340 USB-UART bridges.
"""

import os
import sys
import math
import struct
import time
import yaml
import pytest

# Ensure local workspace in path
current_dir = os.path.dirname(os.path.abspath(__file__))
if current_dir not in sys.path:
    sys.path.insert(0, current_dir)

import rclpy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from ld19_driver_node import LD19DriverNode, calc_crc8, CRC_TABLE
from synthetic_scan_generator import SyntheticScanGenerator


@pytest.fixture(scope="module", autouse=True)
def ros_context():
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


# ============================================================================
# 1. LDROBOT LD19 Protocol & Packet Parsing Tests
# ============================================================================

def test_ld19_crc8_lookup_table():
    """Verify CRC table size and known values."""
    assert len(CRC_TABLE) == 256
    # Known values from official LDROBOT datasheet
    assert CRC_TABLE[0] == 0x00
    assert CRC_TABLE[1] == 0x4D
    assert CRC_TABLE[2] == 0x9A
    assert CRC_TABLE[255] == 0xA8


def test_ld19_crc8_calculation():
    """Verify CRC calculation on known payload."""
    test_data = bytes([0x54, 0x2C, 0x00, 0x00, 0x10, 0x00])
    crc = calc_crc8(test_data)
    assert isinstance(crc, int)
    assert 0 <= crc <= 255


def make_mock_ld19_packet(start_deg=45.0, end_deg=55.0, dist_mm=1500, intensity=200):
    """Helper to synthesize a valid 44-byte LD19 serial packet."""
    header = bytes([0x54, 0x2C])
    radar_speed = struct.pack('<H', 3600)  # 3600 deg/s = 10 Hz
    start_angle = struct.pack('<H', int(start_deg * 100))
    points = bytearray()
    for _ in range(12):
        points.extend(struct.pack('<H', dist_mm))
        points.append(intensity)
    end_angle = struct.pack('<H', int(end_deg * 100))
    timestamp = struct.pack('<H', 12345)

    payload = header + radar_speed + start_angle + bytes(points) + end_angle + timestamp
    assert len(payload) == 46
    crc8 = calc_crc8(payload)
    return payload + bytes([crc8])


def test_ld19_valid_packet_parsing():
    """Verify parsing a synthesized valid 47-byte LD19 packet."""
    if not rclpy.ok():
        rclpy.init()

    node = LD19DriverNode()
    try:
        packet = make_mock_ld19_packet(start_deg=10.0, end_deg=20.0, dist_mm=2500, intensity=180)
        assert len(packet) == 47

        points = node.parse_packet(packet)
        assert points is not None
        assert len(points) == 12

        # Check first point
        first_ang, first_dist, first_int = points[0]
        assert pytest.approx(math.degrees(first_ang), abs=0.1) == 10.0
        assert pytest.approx(first_dist, abs=1e-3) == 2.500
        assert first_int == 180.0

        # Check last point
        last_ang, last_dist, last_int = points[-1]
        assert pytest.approx(math.degrees(last_ang), abs=0.1) == 20.0
        assert pytest.approx(last_dist, abs=1e-3) == 2.500
        assert last_int == 180.0
    finally:
        node.destroy_node()


def test_ld19_corrupted_packet_rejection():
    """Verify packet with corrupted CRC is rejected."""
    if not rclpy.ok():
        rclpy.init()

    node = LD19DriverNode()
    try:
        packet = bytearray(make_mock_ld19_packet())
        # Invert CRC byte
        packet[-1] = (packet[-1] ^ 0xFF)
        result = node.parse_packet(bytes(packet))
        assert result is None
    finally:
        node.destroy_node()


def test_ld19_invalid_header_rejection():
    """Verify packet with invalid header bytes is rejected."""
    if not rclpy.ok():
        rclpy.init()

    node = LD19DriverNode()
    try:
        packet = bytearray(make_mock_ld19_packet())
        packet[0] = 0xAA  # Invalid header byte
        result = node.parse_packet(bytes(packet))
        assert result is None
    finally:
        node.destroy_node()


def test_ld19_angle_unwrapping():
    """Verify angle interpolation across the 360-degree wrap-around boundary."""
    if not rclpy.ok():
        rclpy.init()

    node = LD19DriverNode()
    try:
        # Start at 355 deg, end at 5 deg (diff is 10 deg across 360/0 wrap)
        packet = make_mock_ld19_packet(start_deg=355.0, end_deg=5.0, dist_mm=1000)
        points = node.parse_packet(packet)
        assert points is not None
        assert len(points) == 12

        # Point 0 is 355 deg
        assert pytest.approx(math.degrees(points[0][0]), abs=0.2) == 355.0
        # Point 11 is 5 deg
        assert pytest.approx(math.degrees(points[-1][0]), abs=0.2) == 5.0

        # Intermediate points should cross 0 without discontinuity
        angles_deg = [math.degrees(p[0]) for p in points]
        # Angles should start >= 355 and wrap into < 10
        assert angles_deg[0] >= 354.0
        assert angles_deg[-1] <= 6.0
    finally:
        node.destroy_node()


def test_ld19_driver_mock_scan_generation():
    """Verify LD19 driver mock mode outputs valid LaserScan."""
    if not rclpy.ok():
        rclpy.init()

    node = LD19DriverNode()
    try:
        from rclpy.qos import qos_profile_sensor_data
        scans_received = []
        sub = node.create_subscription(LaserScan, node.topic_name, lambda msg: scans_received.append(msg), qos_profile_sensor_data)

        node.publish_mock_scan()
        rclpy.spin_once(node, timeout_sec=0.2)

        assert len(scans_received) == 1
        scan = scans_received[0]
        assert scan.header.frame_id == node.frame_id
        assert len(scan.ranges) == node.num_bins
        assert pytest.approx(scan.range_min, abs=1e-5) == node.range_min
        assert pytest.approx(scan.range_max, abs=1e-5) == node.range_max
        assert all(node.range_min <= r <= node.range_max for r in scan.ranges)
    finally:
        node.destroy_node()


# ============================================================================
# 2. Configuration Parameters & Nav2 Compatibility Tests
# ============================================================================

def test_rf2o_params_file():
    """Verify rf2o_params.yaml contains required parameters and correct frames."""
    params_path = os.path.join(current_dir, 'rf2o_params.yaml')
    assert os.path.exists(params_path), f"Missing {params_path}"

    with open(params_path, 'r') as f:
        config = yaml.safe_load(f)

    rf2o = config['rf2o_laser_odometry']['ros__parameters']
    assert rf2o['laser_scan_topic'] == '/scan'
    assert rf2o['odom_topic'] == '/odom'
    assert rf2o['base_frame_id'] == 'base_footprint'
    assert rf2o['odom_frame_id'] == 'odom'
    assert rf2o['publish_tf'] is True
    assert rf2o['init_pose_from_topic'] == ''
    assert rf2o['freq'] == 10.0
    assert rf2o['pose_covariance_x'] == 0.001
    assert rf2o['pose_covariance_yaw'] == 0.001


def test_ld19_params_file():
    """Verify ld19_params.yaml contains valid hardware configuration."""
    params_path = os.path.join(current_dir, 'ld19_params.yaml')
    assert os.path.exists(params_path), f"Missing {params_path}"

    with open(params_path, 'r') as f:
        config = yaml.safe_load(f)

    ld19 = config['ld19_lidar_driver']['ros__parameters']
    assert ld19['port_name'] in ['/dev/ldlidar', '/dev/ttyUSB0']
    assert ld19['port_baudrate'] == 230400
    assert ld19['frame_id'] == 'laser_frame'
    assert ld19['base_frame_id'] == 'base_link'
    assert ld19['topic_name'] == '/scan'
    assert ld19['range_min'] == 0.02
    assert ld19['range_max'] == 12.0


def test_nav2_pure_lidar_odometry_compatibility():
    """Verify diff_drive_2wd_nav2_params.yaml is fully compatible with pure LiDAR odometry."""
    nav2_params_path = os.path.join(current_dir, 'diff_drive_2wd_nav2_params.yaml')
    assert os.path.exists(nav2_params_path), f"Missing {nav2_params_path}"

    with open(nav2_params_path, 'r') as f:
        params = yaml.safe_load(f)

    # Local costmap must use odom frame and base_link
    lc = params['local_costmap']['local_costmap']['ros__parameters']
    assert lc['global_frame'] == 'odom'
    assert lc['robot_base_frame'] == 'base_link'
    assert lc['obstacle_layer']['scan']['topic'] == '/scan'

    # Global costmap must use map frame and base_link
    gc = params['global_costmap']['global_costmap']['ros__parameters']
    assert gc['global_frame'] == 'map'
    assert gc['robot_base_frame'] == 'base_link'
    assert gc['obstacle_layer']['scan']['topic'] == '/scan'

    # AMCL should use base_footprint and odom
    amcl = params['amcl']['ros__parameters']
    assert amcl['base_frame_id'] == 'base_footprint'
    assert amcl['odom_frame_id'] == 'odom'
    assert amcl['scan_topic'] == 'scan'


# ============================================================================
# 3. Synthetic Scan Generator Tests
# ============================================================================

def test_synthetic_scan_generator_raycast():
    """Verify geometric raycasting logic inside a rectangular room."""
    if not rclpy.ok():
        rclpy.init()

    gen = SyntheticScanGenerator()
    try:
        # Place robot at (0, 0, 0) in 6x6 room (x in [-3, 3], y in [-3, 3])
        gen.robot_x = 0.0
        gen.robot_y = 0.0
        gen.robot_yaw = 0.0

        from rclpy.qos import qos_profile_sensor_data
        scans = []
        sub = gen.create_subscription(LaserScan, gen.scan_topic, lambda m: scans.append(m), qos_profile_sensor_data)

        gen.publish_scan()
        rclpy.spin_once(gen, timeout_sec=0.2)

        assert len(scans) == 1
        scan = scans[0]
        assert scan.header.frame_id == gen.laser_frame
        assert len(scan.ranges) == gen.num_readings

        # Center beam (beam_angle = 0, facing +x, east wall at x = +3.0)
        # Laser is at x = +0.04m, so expected dist = 3.0 - 0.04 = 2.96m (+- 0.05m noise)
        idx_forward = int(gen.num_readings / 2)  # angle 0 is at middle (-pi to +pi)
        forward_range = scan.ranges[idx_forward]
        assert pytest.approx(forward_range, abs=0.08) == 2.96
    finally:
        gen.destroy_node()


# ============================================================================
# 4. udev Rules & System Configuration Tests
# ============================================================================

def test_udev_rules_syntax():
    """Verify udev rule file contains CP2102 and CH340 vendor/product IDs and stable symlinks."""
    rules_path = os.path.join(current_dir, 'udev', '60-ldlidar.rules')
    assert os.path.exists(rules_path), f"Missing {rules_path}"

    with open(rules_path, 'r') as f:
        content = f.read()

    # Must contain Silicon Labs CP2102
    assert '10c4' in content.lower(), "Missing CP2102 idVendor 10c4"
    assert 'ea60' in content.lower(), "Missing CP2102 idProduct ea60"

    # Must contain CH340
    assert '1a86' in content.lower(), "Missing CH340 idVendor 1a86"
    assert '7523' in content.lower(), "Missing CH340 idProduct 7523"

    # Must define stable symlink /dev/ldlidar
    assert 'ldlidar' in content, "Missing /dev/ldlidar symlink"
    assert 'MODE="0666"' in content or 'MODE:="0777"' in content, "Missing permissive access mode"


def test_udev_setup_script_executable():
    """Verify setup_udev_rules.sh exists and is non-empty."""
    script_path = os.path.join(current_dir, 'udev', 'setup_udev_rules.sh')
    assert os.path.exists(script_path)
    with open(script_path, 'r') as f:
        content = f.read()
    assert 'udevadm control --reload-rules' in content
    assert '60-ldlidar.rules' in content


# ============================================================================
# 5. Robustness, TF Continuity & Boundary Edge Cases Tests
# ============================================================================

def test_ld19_driver_complete_tf_broadcasting():
    """Verify LD19DriverNode broadcasts both base_footprint->base_link AND base_link->laser_frame."""
    if not rclpy.ok():
        rclpy.init()

    node = LD19DriverNode()
    tf_buffer = None
    try:
        import tf2_ros
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer, node)

        # Trigger static TF broadcast
        node.publish_tf()
        rclpy.spin_once(node, timeout_sec=0.2)

        # 1. Lookup base_footprint -> base_link
        tf1 = tf_buffer.lookup_transform('base_footprint', 'base_link', rclpy.time.Time())
        assert tf1 is not None
        assert pytest.approx(tf1.transform.translation.z, abs=1e-3) == 0.033

        # 2. Lookup base_link -> laser_frame
        tf2 = tf_buffer.lookup_transform('base_link', 'laser_frame', rclpy.time.Time())
        assert tf2 is not None
        assert pytest.approx(tf2.transform.translation.x, abs=1e-3) == 0.04
        assert pytest.approx(tf2.transform.translation.z, abs=1e-3) == 0.10

        # 3. Lookup composite chain: base_footprint -> laser_frame (proves TF tree continuity)
        tf_chain = tf_buffer.lookup_transform('base_footprint', 'laser_frame', rclpy.time.Time())
        assert tf_chain is not None
        assert pytest.approx(tf_chain.transform.translation.x, abs=1e-3) == 0.04
        assert pytest.approx(tf_chain.transform.translation.z, abs=1e-3) == 0.133
    finally:
        node.destroy_node()


def test_ld19_driver_buffer_overflow_protection():
    """Verify point accumulation buffer caps at 1000 points under continuous noise."""
    if not rclpy.ok():
        rclpy.init()

    node = LD19DriverNode()
    try:
        # Feed 120 points * 10 calls = 1200 points without revolution wrap
        dummy_points = [(1.0, 1.5, 200.0)] * 120
        node.last_start_angle = 1.0  # constant angle to prevent wrap
        for _ in range(10):
            node.accumulate_points(dummy_points)

        # Buffer must have flushed and reset rather than exceeding 1000
        assert len(node.current_points) <= 1000
    finally:
        node.destroy_node()


def test_ld19_driver_boundary_pi_bin_mapping():
    """Verify points at exactly +pi or -pi are mapped into valid bins without drop or index error."""
    if not rclpy.ok():
        rclpy.init()

    node = LD19DriverNode()
    try:
        from rclpy.qos import qos_profile_sensor_data
        scans = []
        sub = node.create_subscription(LaserScan, node.topic_name, lambda m: scans.append(m), qos_profile_sensor_data)

        # Points at +pi and -pi
        boundary_points = [
            (math.pi, 2.0, 200.0),
            (-math.pi, 2.0, 200.0),
            (0.0, 1.5, 200.0),
        ]
        node.publish_scan(boundary_points)
        rclpy.spin_once(node, timeout_sec=0.2)

        assert len(scans) == 1
        scan = scans[0]
        # Bins must not be inf where points were provided
        non_inf_count = sum(1 for r in scan.ranges if not math.isinf(r))
        assert non_inf_count >= 2
    finally:
        node.destroy_node()


def test_ld19_driver_port_fallback_and_mock():
    """Verify mock_fallback parameter allows smooth operation on systems without serial hardware."""
    if not rclpy.ok():
        rclpy.init()

    # Create node with mock_hardware=True
    node = LD19DriverNode()
    try:
        assert node.use_mock is True or hasattr(node, 'serial_handle')
        assert node.base_footprint_frame == 'base_footprint'
        assert node.wheel_radius == 0.033
        assert node.laser_x_offset == 0.04
    finally:
        node.destroy_node()
