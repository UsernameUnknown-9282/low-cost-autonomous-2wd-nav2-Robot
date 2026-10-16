import os
import sys
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    current_dir = os.path.dirname(os.path.abspath(__file__))
    script_path = os.path.join(current_dir, 'motor_driver_node.py')
    params_file = os.path.join(current_dir, 'motor_params.yaml')

    node_process = ExecuteProcess(
        cmd=[
            sys.executable,
            script_path,
            '--ros-args',
            '--params-file',
            params_file
        ],
        output='screen'
    )

    return LaunchDescription([
        node_process
    ])
