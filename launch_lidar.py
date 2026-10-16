#!/usr/bin/env python3
"""
ROS 2 Launch file for LDROBOT LD19 LiDAR on Raspberry Pi Zero 2 W (or PC test).
Starts ld19_driver_node with configured serial port, baud rate, and static transform.
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        os.path.dirname(__file__),
        'ld19_params.yaml'
    )

    lidar_node = Node(
        package=None,
        executable='python3',
        arguments=[os.path.join(os.path.dirname(__file__), 'ld19_driver_node.py'), '--ros-args', '--params-file', params_file],
        name='ld19_lidar_driver',
        output='screen'
    )

    return LaunchDescription([
        lidar_node
    ])
