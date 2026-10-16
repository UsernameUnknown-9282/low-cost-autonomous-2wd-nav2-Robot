#!/bin/bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle
export OGRE_RTT_MODE=Copy

ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True
