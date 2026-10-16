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

echo "=== Starting Synthetic Scan Generator & TF Broadcaster (Domain: $ROS_DOMAIN_ID) ==="
python3 "$SCRIPT_DIR/synthetic_scan_generator.py" &
PID_SCAN=$!

sleep 1

echo "=== Starting RF2O Laser Odometry Node ==="
ros2 launch "$SCRIPT_DIR/launch_rf2o_odometry.py" &
PID_RF2O=$!

trap "kill -9 $PID_SCAN $PID_RF2O 2>/dev/null || true" EXIT

sleep 2

echo "=== Running Live Verifier ==="
python3 "$SCRIPT_DIR/verify_rf2o_live.py"
VERIFY_EXIT=$?

echo "Verification finished with exit code: $VERIFY_EXIT"
exit $VERIFY_EXIT
