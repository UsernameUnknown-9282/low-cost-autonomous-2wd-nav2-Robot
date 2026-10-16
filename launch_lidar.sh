#!/bin/bash
set -e

# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Ensure ROS_DOMAIN_ID matches between Pi Zero 2 W and PC (default: 42)
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}

echo "=========================================================="
echo "  Launching LDROBOT LD19 LiDAR Node on Raspberry Pi Zero 2 W"
echo "  ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "  Serial Device: /dev/ldlidar (or /dev/ttyUSB0)"
echo "  Baud Rate:     230400"
echo "  Published:     /scan (LaserScan @ ~10 Hz)"
echo "=========================================================="

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
python3 "$SCRIPT_DIR/ld19_driver_node.py" --ros-args --params-file "$SCRIPT_DIR/ld19_params.yaml"
