#!/usr/bin/env python3
"""
ROS 2 Launch file for Pure LiDAR Odometry (rf2o_laser_odometry).
Subscribes to /scan, publishes /odom (nav_msgs/Odometry) and broadcasts odom -> base_footprint TF.
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    params_file = os.path.join(
        os.path.dirname(__file__),
        'rf2o_params.yaml'
    )

    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[params_file],
    )

    return LaunchDescription([rf2o_node])
