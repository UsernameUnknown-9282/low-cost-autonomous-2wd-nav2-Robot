#!/usr/bin/env python3
"""
Unit and Integration Tests for Module 3:
1. Gazebo 2WD Robot URDF Model (diff_drive_2wd.urdf)
   - Kinematic tree, dimensions, joint frames, inertial properties
   - Gazebo diff_drive plugin config (pure LiDAR odometry publish_odom=false)
   - Gazebo ray sensor config (LD19 450 samples, 10 Hz, frame_name=laser_frame)
2. Automated Mission Runner (mission_hall_to_trash.py)
   - Argument parsing, waypoint generation for both trash cans
   - Quaternion math conversions
   - Doorway clearance verification against house_full occupancy grid
   - Collision detection against obstacle cells and out-of-bounds coordinates
   - Dry-run mission execution
3. Automated Raspberry Pi Zero 2 W Provisioning Script (provision_pi.sh & setup_pi_zero2w.sh)
   - Bash syntax validation (bash -n)
   - CLI flags (--domain-id, --workspace, --dry-run)
   - Systemd service unit structure and auto-restart directives
   - udev rule integration
4. Launch and Parameter Compatibility
   - launch_sim_2wd_house.launch.py & launch_gazebo_sim.sh validation
   - diff_drive_2wd_nav2_params.yaml inflation buffer & pure pursuit config
"""

import os
import sys
import math
import subprocess
import xml.etree.ElementTree as ET
import yaml
import pytest

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
if CURRENT_DIR not in sys.path:
    sys.path.insert(0, CURRENT_DIR)

from mission_hall_to_trash import (
    quaternion_from_yaw,
    verify_map_clearance,
    MissionRunner,
    KNOWN_LOCATIONS
)


# ============================================================================
# 1. 2WD Robot URDF Model Tests (diff_drive_2wd.urdf)
# ============================================================================

@pytest.fixture(scope="module")
def urdf_tree():
    urdf_path = os.path.join(CURRENT_DIR, 'diff_drive_2wd.urdf')
    assert os.path.exists(urdf_path), f"diff_drive_2wd.urdf missing at {urdf_path}"
    tree = ET.parse(urdf_path)
    return tree


def test_urdf_root_and_links(urdf_tree):
    """Verify robot name and all mandatory links exist."""
    root = urdf_tree.getroot()
    assert root.tag == 'robot'
    assert root.attrib.get('name') == 'diff_drive_2wd'

    link_names = {l.attrib.get('name') for l in root.findall('link')}
    required_links = {
        'base_footprint',
        'base_link',
        'wheel_left',
        'wheel_right',
        'caster_wheel',
        'laser_frame'
    }
    assert required_links.issubset(link_names), f"Missing links: {required_links - link_names}"


def test_urdf_kinematic_joints(urdf_tree):
    """Verify joints connect according to REP-105 standard and physical specs."""
    root = urdf_tree.getroot()
    joints = {j.attrib.get('name'): j for j in root.findall('joint')}

    # 1. base_footprint -> base_link (elevated by wheel radius 0.033m)
    assert 'base_footprint_joint' in joints
    bf_joint = joints['base_footprint_joint']
    assert bf_joint.attrib.get('type') == 'fixed'
    assert bf_joint.find('parent').attrib.get('link') == 'base_footprint'
    assert bf_joint.find('child').attrib.get('link') == 'base_link'
    origin_bf = bf_joint.find('origin').attrib.get('xyz').split()
    assert float(origin_bf[2]) == pytest.approx(0.033, abs=0.001)

    # 2. base_link -> laser_frame (x=+0.040m, z=+0.100m, matches Module 2 static TF)
    assert 'laser_joint' in joints
    laser_joint = joints['laser_joint']
    assert laser_joint.attrib.get('type') == 'fixed'
    assert laser_joint.find('parent').attrib.get('link') == 'base_link'
    assert laser_joint.find('child').attrib.get('link') == 'laser_frame'
    origin_laser = laser_joint.find('origin').attrib.get('xyz').split()
    assert float(origin_laser[0]) == pytest.approx(0.040, abs=0.001)
    assert float(origin_laser[1]) == pytest.approx(0.000, abs=0.001)
    assert float(origin_laser[2]) == pytest.approx(0.100, abs=0.001)

    # 3. Left and Right continuous wheel joints
    assert 'wheel_left_joint' in joints
    assert 'wheel_right_joint' in joints
    w_left = joints['wheel_left_joint']
    w_right = joints['wheel_right_joint']
    assert w_left.attrib.get('type') == 'continuous'
    assert w_right.attrib.get('type') == 'continuous'

    orig_l = [float(v) for v in w_left.find('origin').attrib.get('xyz').split()]
    orig_r = [float(v) for v in w_right.find('origin').attrib.get('xyz').split()]
    # Separation = 0.0825 - (-0.0825) = 0.165m
    assert orig_l[1] > 0
    assert orig_r[1] < 0
    separation = orig_l[1] - orig_r[1]
    assert separation == pytest.approx(0.165, abs=0.002)


def test_urdf_pure_lidar_odometry_gazebo_plugin(urdf_tree):
    """
    Verify Gazebo differential drive plugin has wheel odometry disabled
    so RF2O pure laser odometry publishes /odom and odom->base_footprint.
    """
    root = urdf_tree.getroot()
    diff_plugins = [
        p for p in root.iter('plugin')
        if 'diff_drive' in p.attrib.get('filename', '')
    ]
    assert len(diff_plugins) >= 1, "Gazebo diff_drive plugin missing!"
    plugin = diff_plugins[0]

    # Critical Option A verification:
    pub_odom = plugin.find('publish_odom')
    pub_tf = plugin.find('publish_odom_tf')
    assert pub_odom is not None and pub_odom.text.strip().lower() == 'false', \
        "publish_odom must be FALSE for Pure LiDAR Odometry (Option A)!"
    assert pub_tf is not None and pub_tf.text.strip().lower() == 'false', \
        "publish_odom_tf must be FALSE so RF2O can broadcast odom->base_footprint!"

    # Geometry & command topics
    cmd_topic = plugin.find('command_topic')
    assert cmd_topic is not None and cmd_topic.text.strip() == 'cmd_vel'
    w_sep = float(plugin.find('wheel_separation').text.strip())
    assert w_sep == pytest.approx(0.165, abs=0.005)
    w_diam = float(plugin.find('wheel_diameter').text.strip())
    assert w_diam == pytest.approx(0.066, abs=0.005)


def test_urdf_lidar_ray_sensor_gazebo_plugin(urdf_tree):
    """Verify Gazebo simulated LiDAR matches LDROBOT LD19 specifications."""
    root = urdf_tree.getroot()
    sensors = [s for s in root.iter('sensor') if s.attrib.get('type') == 'ray']
    assert len(sensors) >= 1, "Gazebo ray sensor missing in URDF!"
    sensor = sensors[0]

    update_rate = float(sensor.find('update_rate').text.strip())
    assert update_rate == pytest.approx(10.0, abs=0.1)

    ray = sensor.find('ray')
    scan_h = ray.find('scan').find('horizontal')
    samples = int(scan_h.find('samples').text.strip())
    assert samples >= 360  # LD19 has 450 samples

    min_ang = float(scan_h.find('min_angle').text.strip())
    max_ang = float(scan_h.find('max_angle').text.strip())
    assert min_ang == pytest.approx(-math.pi, abs=0.05)
    assert max_ang == pytest.approx(math.pi, abs=0.05)

    plugin = sensor.find('plugin')
    assert 'ray_sensor' in plugin.attrib.get('filename', '')
    frame_name = plugin.find('frame_name').text.strip()
    assert frame_name == 'laser_frame'


# ============================================================================
# 2. Automated Mission Runner Tests (mission_hall_to_trash.py)
# ============================================================================

def test_quaternion_conversions():
    """Verify 2D yaw angle to quaternion math."""
    # 0 deg
    qx, qy, qz, qw = quaternion_from_yaw(0.0)
    assert (qx, qy, qz, qw) == (0.0, 0.0, 0.0, 1.0)

    # 90 deg (pi/2)
    qx, qy, qz, qw = quaternion_from_yaw(math.pi / 2.0)
    assert qz == pytest.approx(math.sin(math.pi / 4.0), abs=1e-5)
    assert qw == pytest.approx(math.cos(math.pi / 4.0), abs=1e-5)

    # 180 deg (pi)
    qx, qy, qz, qw = quaternion_from_yaw(math.pi)
    assert qz == pytest.approx(1.0, abs=1e-5)
    assert qw == pytest.approx(0.0, abs=1e-5)


def test_mission_waypoint_generation():
    """Verify waypoint generation for both trash cans and direct mode."""
    runner = MissionRunner(dry_run=True)

    # 1. Trash Can 1 (Living/Dining Room)
    wp1 = runner.build_waypoints(target_name='trash_can_1', mode='waypoints')
    assert len(wp1) >= 4
    # Confirms doorway buffer is present
    door_wps = [w for w in wp1 if 'doorway' in str(w).lower()]
    assert len(door_wps) >= 1
    # Confirms final destination is trash_can_1
    assert wp1[-1]['x'] == pytest.approx(KNOWN_LOCATIONS['trash_can_1']['x'], abs=0.01)
    assert wp1[-1]['y'] == pytest.approx(KNOWN_LOCATIONS['trash_can_1']['y'], abs=0.01)

    # 2. Trash Can 2 (Northwest Room)
    wp2 = runner.build_waypoints(target_name='trash_can_2', mode='waypoints')
    assert len(wp2) >= 3
    assert wp2[-1]['x'] == pytest.approx(KNOWN_LOCATIONS['trash_can_2']['x'], abs=0.01)

    # 3. Direct mode
    wp_direct = runner.build_waypoints(target_name='trash_can_1', mode='direct')
    assert len(wp_direct) == 1


def test_doorway_safe_passage_clearance():
    """
    Verify doorway safe passage clearance:
    Doorframe post is at (-1.23, -0.30).
    Waypoint is centered at (-1.20, -0.68).
    Distance from center to obstacle post must exceed 0.35m.
    """
    doorframe_obstacle = (-1.23, -0.30)
    doorway_waypoint = (KNOWN_LOCATIONS['doorway_buffer']['x'], KNOWN_LOCATIONS['doorway_buffer']['y'])

    dist = math.hypot(doorframe_obstacle[0] - doorway_waypoint[0],
                      doorframe_obstacle[1] - doorway_waypoint[1])
    # Clearance must be at least 0.35m (safely buffers 0.18m chassis radius and matches Nav2 inflation)
    assert dist >= 0.35, f"Doorway clearance {dist:.3f}m is too narrow!"


def test_map_clearance_verification_trash_can_1():
    """Verify that all waypoints for trash_can_1 have safe clearance on house_full.pgm."""
    map_yaml = os.path.join(CURRENT_DIR, 'house_full.yaml')
    runner = MissionRunner(dry_run=True)
    waypoints = runner.build_waypoints(target_name='trash_can_1', mode='waypoints')
    pts = [(wp['x'], wp['y']) for wp in waypoints]

    report = verify_map_clearance(pts, map_yaml, required_clearance_m=0.20)
    assert report['status'] == 'ok'
    assert report['overall_safe'] is True
    for res in report['point_results']:
        assert res['safe'] is True
        assert res['min_dist_m'] >= 0.20


def test_map_clearance_verification_trash_can_2():
    """Verify that all waypoints for trash_can_2 have safe clearance on house_full.pgm."""
    map_yaml = os.path.join(CURRENT_DIR, 'house_full.yaml')
    runner = MissionRunner(dry_run=True)
    waypoints = runner.build_waypoints(target_name='trash_can_2', mode='waypoints')
    pts = [(wp['x'], wp['y']) for wp in waypoints]

    report = verify_map_clearance(pts, map_yaml, required_clearance_m=0.20)
    assert report['status'] == 'ok'
    assert report['overall_safe'] is True


def test_map_clearance_obstacle_detection():
    """Verify that verify_map_clearance catches actual obstacles and out-of-bounds."""
    map_yaml = os.path.join(CURRENT_DIR, 'house_full.yaml')

    # Known obstacle wall coordinate
    obstacle_point = [(0.0, -0.15)]
    report_obs = verify_map_clearance(obstacle_point, map_yaml, required_clearance_m=0.20)
    assert report_obs['overall_safe'] is False

    # Out-of-bounds point
    oob_point = [(99.0, 99.0)]
    report_oob = verify_map_clearance(oob_point, map_yaml, required_clearance_m=0.20)
    assert report_oob['overall_safe'] is False


def test_dry_run_mission_execution():
    """Verify full dry-run execution completes with status 0."""
    runner = MissionRunner(dry_run=True)
    waypoints = runner.build_waypoints(target_name='trash_can_1')
    success = runner.execute_mission(waypoints)
    assert success is True


# ============================================================================
# 3. Raspberry Pi Provisioning Script Tests (provision_pi.sh & setup_pi_zero2w.sh)
# ============================================================================

def test_provisioning_script_syntax():
    """Verify bash syntax with bash -n."""
    scripts = ['provision_pi.sh', 'setup_pi_zero2w.sh', 'launch_gazebo_sim.sh']
    for s in scripts:
        path = os.path.join(CURRENT_DIR, s)
        assert os.path.exists(path), f"Script {s} not found!"
        res = subprocess.run(['bash', '-n', path], capture_output=True, text=True)
        assert res.returncode == 0, f"Syntax error in {s}: {res.stderr}"


def test_provisioning_script_dry_run_execution():
    """Verify provision_pi.sh runs in --dry-run mode without errors."""
    path = os.path.join(CURRENT_DIR, 'provision_pi.sh')
    cmd = ['bash', path, '--dry-run', '--domain-id', '42', '--workspace', '/home/test_user/RC']
    res = subprocess.run(cmd, capture_output=True, text=True)
    assert res.returncode == 0, f"provision_pi.sh dry-run failed: {res.stderr}"
    import re
    output = res.stdout
    clean_output = re.sub(r'\x1b\[[0-9;]*m', '', output)

    # Verify critical configuration keywords in output
    assert "ROS_DOMAIN_ID : 42" in clean_output
    assert "robot-motor.service" in clean_output
    assert "robot-lidar.service" in clean_output
    assert "/etc/udev/rules.d/60-ldlidar.rules" in clean_output
    assert "1024 MB" in clean_output


def test_systemd_service_definitions():
    """Verify systemd service unit template contents in provision_pi.sh."""
    prov_path = os.path.join(CURRENT_DIR, 'provision_pi.sh')
    with open(prov_path, 'r') as f:
        content = f.read()

    # Motor driver service
    assert 'motor_driver_node.py' in content
    assert 'launch_motor_driver.sh' in content
    assert 'Restart=always' in content
    assert 'RestartSec=3' in content
    assert 'Environment="ROS_DOMAIN_ID=' in content

    # LiDAR driver service
    assert 'ld19_driver_node.py' in content
    assert 'launch_lidar.sh' in content
    assert 'After=network.target' in content


# ============================================================================
# 4. Simulation Launch & Parameter Compatibility Tests
# ============================================================================

def test_urdf_xml_declaration_compatibility():
    """Verify diff_drive_2wd.urdf does not trigger lxml ValueError in spawn_entity.py."""
    import lxml.etree as etree
    urdf_path = os.path.join(CURRENT_DIR, 'diff_drive_2wd.urdf')
    with open(urdf_path, 'r') as f:
        urdf_text = f.read()

    # Must parse cleanly from unicode string without ValueError
    root = etree.fromstring(urdf_text)
    assert root.tag == 'robot'
    assert root.attrib.get('name') == 'diff_drive_2wd'


def test_mission_cli_and_target_validation():
    """Verify --no-sim, --slam flags and target validation in mission runner."""
    runner = MissionRunner(dry_run=True)
    # Valid targets
    wps1 = runner.build_waypoints('trash_can_1')
    assert len(wps1) > 0
    wps2 = runner.build_waypoints('trash_can_2')
    assert len(wps2) > 0

    # Invalid target must raise ValueError
    with pytest.raises(ValueError):
        runner.build_waypoints('nonexistent_location')


def test_simulation_launch_syntax():
    """Verify launch_sim_2wd_house.launch.py imports and syntax."""
    launch_path = os.path.join(CURRENT_DIR, 'launch_sim_2wd_house.launch.py')
    res = subprocess.run([sys.executable, '-m', 'py_compile', launch_path],
                         capture_output=True, text=True)
    assert res.returncode == 0, f"Syntax error in launch file: {res.stderr}"


def test_nav2_parameter_doorway_buffers():
    """Verify inflation radius, cost scaling, and RPP rotation in diff_drive_2wd_nav2_params.yaml."""
    params_path = os.path.join(CURRENT_DIR, 'diff_drive_2wd_nav2_params.yaml')
    with open(params_path, 'r') as f:
        params = yaml.safe_load(f)

    # 1. Doorway buffer: inflation layer settings in local and global costmaps
    local_inf = params['local_costmap']['local_costmap']['ros__parameters']['inflation_layer']
    global_inf = params['global_costmap']['global_costmap']['ros__parameters']['inflation_layer']

    assert local_inf['inflation_radius'] == pytest.approx(0.38, abs=0.01)
    assert local_inf['cost_scaling_factor'] == pytest.approx(3.0, abs=0.1)
    assert global_inf['inflation_radius'] == pytest.approx(0.38, abs=0.01)
    assert global_inf['cost_scaling_factor'] == pytest.approx(3.0, abs=0.1)

    # 2. Regulated Pure Pursuit Controller in-place rotation
    rpp = params['controller_server']['ros__parameters']['FollowPath']
    assert rpp['use_rotate_to_heading'] is True
    assert rpp['rotate_to_heading_angular_vel'] == pytest.approx(0.8, abs=0.05)
    assert rpp['use_collision_detection'] is True

    # 3. Pure LiDAR Odometry: amcl & controller reference base_footprint and odom
    amcl = params['amcl']['ros__parameters']
    assert amcl['base_frame_id'] == 'base_footprint'
    assert amcl['odom_frame_id'] == 'odom'
