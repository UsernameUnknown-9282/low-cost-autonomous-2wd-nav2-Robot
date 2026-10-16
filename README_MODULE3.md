# Module 3: Pre-Deployment Simulation, Mission Testing & Turnkey Pi Provisioning

This module delivers the simulation, mission automation, and turnkey deployment pipeline for the low-cost 2WD differential drive robot (Acrylic chassis, 2x Yellow TT DC motors, TB6612FNG driver, Raspberry Pi Zero 2 W, LDROBOT LD19 LiDAR) operating under **Option A: Pure LiDAR Odometry (RF2O)**.

---

## 1. System Architecture & Pre-Deployment Design

```
                     WSL2 / PC GAZEBO PRE-DEPLOYMENT SIMULATION
  +--------------------------------------------------------------------------------+
  |  Gazebo 3D World (turtlebot3_house.world)                                      |
  |  - Hallway Start: (-2.0m, -0.5m)                                               |
  |  - Spawns diff_drive_2wd.urdf (Acrylic Chassis + 2x TT Motors + LD19)         |
  |                                                                                |
  |  +-----------------------------+          +---------------------------------+  |
  |  | Gazebo Ray Sensor Plugin    |  /scan   | RF2O Laser Odometry Node        |  |
  |  | (LD19 450 samples @ 10 Hz)  |=========>| Subscribes: /scan               |  |
  |  +-----------------------------+          | Publishes:  /odom               |  |
  |                                           | Broadcasts: odom->base_footprint|  |
  |  +-----------------------------+          +---------------------------------+  |
  |  | Gazebo Diff Drive Plugin    |                           |                   |
  |  | (Wheel actuation ONLY)      |          +---------------------------------+  |
  |  | * publish_odom: FALSE       |          | Nav2 Regulated Pure Pursuit     |  |
  |  | * publish_odom_tf: FALSE    |          | - In-place rotation (0.8 rad/s) |  |
  |  | Subscribes: /cmd_vel        |<=========| - Doorway buffer (0.38m infl.)  |  |
  |  +-----------------------------+ /cmd_vel | - Collision lookahead braking   |  |
  |                                           +---------------------------------+  |
  +--------------------------------------------------------------------------------+
                                       ^
                                       | Nav2 Goals & Telemetry
  +--------------------------------------------------------------------------------+
  |  Automated Mission Runner (mission_hall_to_trash.py)                           |
  |  Hallway (-2.0, -0.5) -> Doorway (-1.20, -0.55) -> Trash Can Goal (1.45, 1.65)   |
  +--------------------------------------------------------------------------------+
```

### Pure LiDAR Odometry in Simulation (Option A)
In physical hardware, the yellow TT gearbox DC motors do not possess optical wheel encoders. Continuous odometry is generated strictly from planar laser scan flow (RF2O).
To accurately mirror reality in Gazebo:
1. The **Gazebo Diff Drive Plugin** inside `diff_drive_2wd.urdf` has `publish_odom` and `publish_odom_tf` explicitly disabled (`false`).
2. The simulated LD19 ray sensor outputs range measurements to `/scan` at 10 Hz.
3. `rf2o_laser_odometry` consumes `/scan` and publishes `/odom` and the `odom -> base_footprint` TF transform.
4. Nav2 and AMCL localize and plan paths using this pure LiDAR odometry stream.

### Doorway Clearance Buffers
The doorway between the hallway and living room contains a protruding frame post at `x = -1.23m, y = -0.30m`.
- Robot radius: $0.18\text{ m}$ (footprint diameter $0.36\text{ m}$).
- `diff_drive_2wd_nav2_params.yaml` configures an **inflation radius of $0.38\text{ m}$** with cost scaling factor **$3.0$**.
- This pushes the global planner away from doorframes and prevents corner clipping while preserving clearance through the $1.98\text{ m}$ total door opening.

---

## 2. Gazebo Pre-Deployment Simulation in WSL2

### Prerequisites (Intel Core i7 WSL2 Host)
Ensure ROS 2 Humble desktop and Gazebo are sourced:
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=42
export OGRE_RTT_MODE=Copy
```

### Launch Complete Simulation Stack
Run the turnkey launch script to open Gazebo with the house world, spawn the 2WD robot, start RF2O pure laser odometry, and bring up Nav2:
```bash
cd ~/RC
bash launch_gazebo_sim.sh
```

#### Optional Launch Flags
- **Headless Mode (CI / Low resource)**:
  ```bash
  bash launch_gazebo_sim.sh gui:=false
  ```
- **Launch RViz2 Visualization**:
  ```bash
  bash launch_gazebo_sim.sh rviz:=true
  ```
- **Live SLAM Mapping Mode**:
  ```bash
  bash launch_gazebo_sim.sh slam:=true
  ```

---

## 3. Automated Mission Runner: Hallway to Trash Can

`mission_hall_to_trash.py` coordinates the autonomous traversal from the hallway starting point through the narrow doorway threshold to the destination trash can.

### Target Destinations
| Target Name | Description | World Pose $(x, y, \text{yaw})$ |
| :--- | :--- | :--- |
| `trash_can_1` *(Default)* | Living/Dining Room Trash Can | $(1.45\text{ m}, 1.65\text{ m}, 45^\circ)$ |
| `trash_can_2` | Northwest Bedroom Trash Can | $(-4.00\text{ m}, 4.20\text{ m}, 135^\circ)$ |

### Waypoint Safe Sequence (`trash_can_1`)
1. **Hallway Start**: $(-2.00, -0.50, 0^\circ)$
2. **Corridor Alignment**: $(-1.50, -0.65, 0^\circ)$
3. **Doorway Safe Centerline**: $(-1.20, -0.68, 0^\circ)$ — *Clearance $> 0.38\text{ m}$ from frame post at $y = -0.30\text{ m}$, aligning with Nav2 $0.38\text{ m}$ inflation buffer*
4. **Hallway Junction**: $(0.10, -0.90, -14^\circ)$
5. **Living Room Corridor Entry**: $(1.30, 0.10, 40^\circ)$
6. **East Corridor Bypass**: $(2.70, 1.80, 90^\circ)$
7. **Dining Area Approach**: $(1.60, 2.50, -90^\circ)$
8. **Final Arrival**: $(1.45, 1.65, 45^\circ)$ — *Robot faces trash can in free space*

### Executing the Mission
- **Run Map Clearance & Route Dry-Run Validation (No ROS needed)**:
  ```bash
  python3 mission_hall_to_trash.py --dry-run
  ```
- **Execute Mission in Live Simulation (Gazebo / AMCL)**:
  ```bash
  python3 mission_hall_to_trash.py --target trash_can_1
  ```
- **Execute Mission in SLAM Mapping Mode**:
  ```bash
  python3 mission_hall_to_trash.py --target trash_can_1 --slam
  ```
- **Execute Mission on Physical Hardware (Disable sim clock)**:
  ```bash
  python3 mission_hall_to_trash.py --target trash_can_1 --no-sim
  ```
- **Execute Mission to Northwest Trash Can**:
  ```bash
  python3 mission_hall_to_trash.py --target trash_can_2
  ```

### Live Status Telemetry
While navigating, `mission_hall_to_trash.py` streams live telemetry to stdout:
```
[NAV2 STATUS] Time:  12s | Dist Remaining:  4.12m | Poses Left: 5
[NAV2 STATUS] Time:  24s | Dist Remaining:  2.85m | Poses Left: 3
[NAV2 STATUS] Time:  38s | Dist Remaining:  0.42m | Poses Left: 1

=================================================================
>>> MISSION ACCOMPLISHED: ROBOT REACHED TRASH CAN GOAL! <<<
=================================================================
```

---

## 4. Turnkey Raspberry Pi Zero 2 W Provisioning Guide

When the Raspberry Pi Zero 2 W hardware arrives, use the automated provisioning script for zero-touch configuration.

### Step 1: Flash SD Card
1. Download and run **Raspberry Pi Imager** on PC.
2. Select OS: **Raspberry Pi OS Lite (64-bit)** (Debian Bookworm or Bullseye) or **Ubuntu 22.04 LTS Server (64-bit)**.
3. Click the gear icon (Settings) and configure:
   - Hostname: `pi-robot`
   - Username: `pi` (or your preferred user) and password
   - Configure wireless LAN: Enter SSID and Wi-Fi password
   - Enable SSH (Password authentication or public key)
4. Flash the micro-SD card (16GB or 32GB recommended) and insert it into the Pi Zero 2 W.

### Step 2: Transfer Robot Workspace to Pi
From your PC terminal:
```bash
# Copy workspace repository over Wi-Fi
rsync -avz ~/RC pi@pi-robot.local:/home/pi/RC
```

### Step 3: Run Turnkey Provisioning Script
SSH into the Pi Zero 2 W and run:
```bash
ssh pi@pi-robot.local
cd ~/RC
sudo bash provision_pi.sh
```

### What `provision_pi.sh` Automates
1. **Memory & Swap Optimization**: Configures swap file to **1024 MB** (`CONF_SWAPSIZE=1024`). The Pi Zero 2 W has 512MB RAM; this prevents out-of-memory kernel panics during ROS communications.
2. **Package Installation**: Automatically installs `python3-pip`, `python3-serial`, `python3-yaml`, `python3-numpy`, `libgpiod-dev`, and GPIO drivers.
3. **ROS Environment**: Configures `ROS_DOMAIN_ID=42` in `/etc/environment` and `~/.bashrc`.
4. **Persistent LiDAR Rules**: Installs `/etc/udev/rules.d/60-ldlidar.rules` mapping CP2102 and CH340 adapters to `/dev/ldlidar` with read/write permissions.
5. **Hardware Interfaces**: Disables serial debug console on `/dev/ttyAMA0` and enables I2C/SPI via `raspi-config nonint`.
6. **Zero-Touch Systemd Services**:
   - `robot-motor.service`: Runs `/bin/bash launch_motor_driver.sh` on boot, sourcing ROS 2 environment and loading `motor_params.yaml` (300ms watchdog, TT motor deadband calibration).
   - `robot-lidar.service`: Runs `/bin/bash launch_lidar.sh` on boot, sourcing ROS 2 environment, loading `ld19_params.yaml`, and streaming `/scan` with static TF.
   - Configures `Restart=always` and `RestartSec=3` for fault tolerance.

### Step 4: Verification & Service Management on Pi
```bash
# Check service statuses
sudo systemctl status robot-motor.service
sudo systemctl status robot-lidar.service

# Stream live service logs
journalctl -u robot-motor.service -f
journalctl -u robot-lidar.service -f

# Verify USB LiDAR device symlink
ls -l /dev/ldlidar
# Expected: lrwxrwxrwx 1 root root 7 ... /dev/ldlidar -> ttyUSB0

# Verify LiDAR scan topic over Wi-Fi
ros2 topic echo /scan --once
```

---

## 5. Automated Verification & Test Suite

All 56 unit, integration, and geometry verification tests run headless without requiring physical hardware:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
python3 -m pytest -v test_laser_odometry.py test_motor_driver.py test_module3_simulation_mission_provisioning.py
```

### Test Coverage Summary (56/56 Tests Passing)
- **Motor Subsystem (Module 1 - 23 tests)**: Kinematics (straight forward/reverse, pure spin, curving turns), deadband step jump and linear scaling, 300ms watchdog failsafe, mock hardware fallback.
- **LiDAR Bridge & Laser Odometry (Module 2 - 17 tests)**: LD19 packet parser, CRC-8 lookup and calculation, angle unwrap across 0/360, synthetic scan raycasting, udev rules validation, REP-105 covariances.
- **Simulation, Missions & Provisioning (Module 3 - 16 tests)**:
  - `diff_drive_2wd.urdf` kinematic joints, inertial tags, Gazebo diff_drive `publish_odom=false` pure LiDAR odometry config, simulated LD19 ray sensor.
  - `mission_hall_to_trash.py` waypoints generation, doorway safe clearance calculation, obstacle detection on `house_full.pgm`, dry-run mission completion.
  - `provision_pi.sh` & `setup_pi_zero2w.sh` bash syntax checks, dry-run output assertions, systemd unit definitions.
  - `diff_drive_2wd_nav2_params.yaml` doorway inflation radius (0.38m) and RPP in-place rotation parameters.
