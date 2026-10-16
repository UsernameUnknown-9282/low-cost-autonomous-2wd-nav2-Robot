#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:/opt/ros/humble/share/turtlebot3_gazebo/models
export OGRE_RTT_MODE=Copy

# Kill any dangling instances
killall -9 gzserver gzclient 2>/dev/null || true

echo "=== Launching Gazebo TurtleBot3 House Simulation ==="
echo "Model: $TURTLEBOT3_MODEL"
echo "A Gazebo 3D simulation window will open on your Windows desktop."
echo "World: turtlebot3_house.world"

ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
