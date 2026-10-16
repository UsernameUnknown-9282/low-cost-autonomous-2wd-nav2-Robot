#!/bin/bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle
export OGRE_RTT_MODE=Copy

rviz2 -d /opt/ros/humble/share/turtlebot3_navigation2/rviz/tb3_navigation2.rviz --ros-args -p use_sim_time:=true
