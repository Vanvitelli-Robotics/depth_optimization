import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

import os

def generate_launch_description():
    # Get the path to the parameters.yaml file
    config_file_opt = os.path.join(get_package_share_directory('depth_optimization'), 'config', 'parameters.yaml')
    #bash_script = os.path.join(get_package_share_directory('depth_optimization'), 'bash_script', 'run_depth_opt.sh')

    # Declare the log_level argument
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level'
    )

    # Launch the dope_node node
    dope_depth_optimizer_server = Node(
        package='depth_optimization',
        executable='dope_depth_optimizer_server',
        name='dope_depth_optimizer_server',
        output='screen',
        emulate_tty=True,
        parameters=[config_file_opt],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
    )
    return LaunchDescription([
        log_level_arg,
        dope_depth_optimizer_server
    ])
