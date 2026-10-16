#!/usr/bin/env python3
"""
Gazebo Pre-Deployment Simulation Launch with Pure LiDAR Odometry (Option A).
Links:
1. Gazebo 3D simulation with turtlebot3_house.world
2. 2WD Differential Drive Robot URDF (diff_drive_2wd.urdf)
3. Simulated LD19 LiDAR sensor publishing /scan @ 10 Hz
4. Pure LiDAR Odometry (RF2O) computing /odom and broadcasting odom -> base_footprint TF
5. Nav2 Navigation Stack with 2WD Regulated Pure Pursuit & In-Place Rotation
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    RegisterEventHandler,
    SetEnvironmentVariable
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    workspace_dir = os.path.dirname(os.path.abspath(__file__))
    urdf_file = os.path.join(workspace_dir, 'diff_drive_2wd.urdf')
    nav2_params_file = os.path.join(workspace_dir, 'diff_drive_2wd_nav2_params.yaml')
    rf2o_params_file = os.path.join(workspace_dir, 'rf2o_params.yaml')
    map_file = os.path.join(workspace_dir, 'house_full.yaml')

    # Launch Configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gui = LaunchConfiguration('gui', default='true')
    headless = LaunchConfiguration('headless', default='false')
    autostart = LaunchConfiguration('autostart', default='true')
    use_slam = LaunchConfiguration('slam', default='False')
    rviz = LaunchConfiguration('rviz', default='false')
    
    # Normalize slam configuration for Nav2 PythonExpression compatibility ('True' or 'False')
    slam_normalized = PythonExpression(["'True' if '", use_slam, "'.lower() in ['true', '1'] else 'False'"])
    
    # Spawn pose in hallway
    x_pose = LaunchConfiguration('x_pose', default='-2.0')
    y_pose = LaunchConfiguration('y_pose', default='-0.5')
    z_pose = LaunchConfiguration('z_pose', default='0.01')
    yaw_pose = LaunchConfiguration('yaw_pose', default='0.0')

    # Default world
    try:
        tb3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
        default_world = os.path.join(tb3_gazebo_dir, 'worlds', 'turtlebot3_house.world')
    except Exception:
        default_world = os.path.join(workspace_dir, 'turtlebot3_house.world')

    world_path = LaunchConfiguration('world', default=default_world)

    # 1. Package directories
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    # Read URDF contents
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    # Declare Launch Arguments
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')
    declare_gui = DeclareLaunchArgument('gui', default_value='true', description='Open Gazebo GUI')
    declare_world = DeclareLaunchArgument('world', default_value=default_world)
    declare_slam = DeclareLaunchArgument('slam', default_value='False', description='Run SLAM Toolbox vs AMCL')
    declare_rviz = DeclareLaunchArgument('rviz', default_value='false', description='Launch RViz2')
    declare_x = DeclareLaunchArgument('x_pose', default_value='-2.0')
    declare_y = DeclareLaunchArgument('y_pose', default_value='-0.5')

    # 2. Gazebo Server & Client
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_path}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        ),
        condition=IfCondition(gui)
    )

    # 3. Robot State Publisher (Publishes diff_drive_2wd TF tree)
    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_desc
        }]
    )

    # 4. Spawn 2WD Differential Drive Robot into Gazebo
    spawn_robot_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_diff_drive_2wd',
        arguments=[
            '-entity', 'diff_drive_2wd',
            '-topic', 'robot_description',
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
            '-Y', yaw_pose
        ],
        output='screen'
    )

    # 5. RF2O Laser Odometry Node (Pure LiDAR Odometry)
    # Computes /odom from /scan and broadcasts odom -> base_footprint
    rf2o_cmd = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[
            rf2o_params_file,
            {'use_sim_time': use_sim_time}
        ]
    )

    # 6. Nav2 Bringup (Controller, Costmaps, Planner, Recoveries, AMCL/SLAM)
    nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
            'slam': slam_normalized,
            'autostart': autostart
        }.items()
    )

    # 7. Optional RViz2
    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_nav2_bringup, 'rviz', 'nav2_default_view.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz),
        output='screen'
    )

    ld = LaunchDescription()

    # Environment
    ld.add_action(SetEnvironmentVariable('ROS_DOMAIN_ID', '42'))
    ld.add_action(SetEnvironmentVariable('OGRE_RTT_MODE', 'Copy'))

    # Arguments
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_gui)
    ld.add_action(declare_world)
    ld.add_action(declare_slam)
    ld.add_action(declare_rviz)
    ld.add_action(declare_x)
    ld.add_action(declare_y)

    # Nodes and Launches
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_robot_cmd)
    ld.add_action(rf2o_cmd)
    ld.add_action(nav2_cmd)
    ld.add_action(rviz_cmd)

    return ld
