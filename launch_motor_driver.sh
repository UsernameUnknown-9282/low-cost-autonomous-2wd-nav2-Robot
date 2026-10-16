#!/bin/bash
# Convenient runner for the 2WD robot motor driver node
set -e

# Source ROS 2 Humble
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "=== Starting 2WD Motor Driver Node ==="
echo "Node script: ${SCRIPT_DIR}/motor_driver_node.py"
echo "Parameters:  ${SCRIPT_DIR}/motor_params.yaml"

python3 "${SCRIPT_DIR}/motor_driver_node.py" --ros-args --params-file "${SCRIPT_DIR}/motor_params.yaml"
