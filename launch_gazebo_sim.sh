#!/bin/bash
# =============================================================================
# Launch Gazebo Simulation with Pure LiDAR Odometry (RF2O) and Nav2
# 2WD Differential Drive Robot in turtlebot3_house world
# =============================================================================
set -e

# Source ROS 2 Humble and local workspace
source /opt/ros/humble/setup.bash
if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then
    source "$HOME/ros2_ws/install/setup.bash"
elif [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
fi

export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}
export OGRE_RTT_MODE=Copy
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:/opt/ros/humble/share/turtlebot3_gazebo/models

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Clean any lingering Gazebo processes
echo "Checking and cleaning previous Gazebo instances..."
killall -9 gzserver gzclient 2>/dev/null || true

echo "=========================================================================="
echo "=== 2WD Differential Drive Robot - Pre-Deployment Gazebo Simulation    ==="
echo "=========================================================================="
echo "ROS_DOMAIN_ID : $ROS_DOMAIN_ID"
echo "Chassis Model : diff_drive_2wd.urdf (Acrylic + 2x TT DC Motors)"
echo "Odometry Mode : Option A - Pure LiDAR Odometry (RF2O from /scan)"
echo "World         : turtlebot3_house.world"
echo "Navigation    : Nav2 Regulated Pure Pursuit with In-Place Rotation"
echo "Doorway Buffer: Inflation radius 0.38m, Cost scaling 3.0"
echo "Spawn Pose    : x=-2.0m, y=-0.5m (Hallway start)"
echo "=========================================================================="

# Pass CLI arguments down to launch file (e.g. gui:=false, rviz:=true, slam:=true)
ros2 launch "$SCRIPT_DIR/launch_sim_2wd_house.launch.py" "$@"
