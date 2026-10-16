#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then
    source "$HOME/ros2_ws/install/setup.bash"
elif [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Set default ROS_DOMAIN_ID to match robot network (default: 42)
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}

echo "=== Launching RF2O Pure LiDAR Odometry Node ==="
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "Subscribed topic: /scan"
echo "Published topic:  /odom"
echo "Published TF:     odom -> base_footprint"

ros2 launch "$SCRIPT_DIR/launch_rf2o_odometry.py"
