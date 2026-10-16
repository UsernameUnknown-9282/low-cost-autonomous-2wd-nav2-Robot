#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle
export OGRE_RTT_MODE=Copy

echo "=== Launching Nav2 for 2WD Differential Drive Robot (NavFn + Pure Pursuit with In-Place Rotation) ==="

ros2 launch nav2_bringup bringup_launch.py \
    slam:=True \
    map:=/mnt/c/Users/Jaya/Downloads/RC/house_full.yaml \
    use_sim_time:=True \
    params_file:=/mnt/c/Users/Jaya/Downloads/RC/diff_drive_2wd_nav2_params.yaml \
    autostart:=True
