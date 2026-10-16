#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then
    source "$HOME/ros2_ws/install/setup.bash"
elif [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
fi

# Ensure ROS_DOMAIN_ID matches robot Pi (default: 42)
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}
export OGRE_RTT_MODE=Copy

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "================================================================="
echo "=== Launching Pure LiDAR Odometry Navigation Pipeline (PC/WSL) ==="
echo "================================================================="
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "TF Chain: map -> odom (SLAM) -> base_footprint (RF2O) -> base_link -> laser_frame"
echo ""

# 1. Start RF2O Laser Odometry Node
echo "-> Starting RF2O Laser Odometry..."
ros2 launch "$SCRIPT_DIR/launch_rf2o_odometry.py" &
PID_RF2O=$!

trap "echo 'Shutting down navigation pipeline...'; kill -9 $PID_RF2O 2>/dev/null || true" EXIT

sleep 2

# 2. Start Nav2 with SLAM Toolbox and 2WD Pure Pursuit parameters
echo "-> Starting Nav2 with SLAM Toolbox & 2WD Differential Drive..."
ros2 launch nav2_bringup bringup_launch.py \
    slam:=True \
    map:="$SCRIPT_DIR/house_full.yaml" \
    use_sim_time:=False \
    params_file:="$SCRIPT_DIR/diff_drive_2wd_nav2_params.yaml" \
    autostart:=True
