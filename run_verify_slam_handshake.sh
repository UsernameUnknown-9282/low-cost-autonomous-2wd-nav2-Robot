#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then
    source "$HOME/ros2_ws/install/setup.bash"
elif [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}

echo "=== 1. Starting Synthetic Scan Generator & Static TF ==="
python3 "$SCRIPT_DIR/synthetic_scan_generator.py" &
PID_SCAN=$!

sleep 1

echo "=== 2. Starting RF2O Laser Odometry Node ==="
ros2 launch "$SCRIPT_DIR/launch_rf2o_odometry.py" &
PID_RF2O=$!

sleep 1

echo "=== 3. Starting SLAM Toolbox ==="
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=False &
PID_SLAM=$!

trap "kill -9 $PID_SCAN $PID_RF2O $PID_SLAM 2>/dev/null || true" EXIT

sleep 2

echo "=== 4. Running SLAM Toolbox & TF Handshake Verifier ==="
python3 "$SCRIPT_DIR/verify_slam_nav2_handshake.py"
VERIFY_EXIT=$?

echo "SLAM Handshake finished with exit code: $VERIFY_EXIT"
exit $VERIFY_EXIT
