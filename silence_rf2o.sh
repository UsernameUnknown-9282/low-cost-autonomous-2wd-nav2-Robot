#!/bin/bash
set -e
FILE="/home/user_not_found__404/ros2_ws/src/rf2o_laser_odometry/src/CLaserOdometry2D.cpp"
if [ -f "$FILE" ]; then
    sed -i 's/RCLCPP_INFO(get_logger(), "execution time/RCLCPP_DEBUG(get_logger(), "execution time/g' "$FILE"
    sed -i 's/RCLCPP_INFO(get_logger(), "Laser odom/RCLCPP_DEBUG(get_logger(), "Laser odom/g' "$FILE"
    sed -i 's/RCLCPP_INFO(get_logger(), "Robot-base odom/RCLCPP_DEBUG(get_logger(), "Robot-base odom/g' "$FILE"
    echo "Patched CLaserOdometry2D.cpp to use DEBUG logging."
    cd /home/user_not_found__404/ros2_ws
    source /opt/ros/humble/setup.bash
    colcon build --packages-select rf2o_laser_odometry
    echo "Rebuild complete!"
fi
