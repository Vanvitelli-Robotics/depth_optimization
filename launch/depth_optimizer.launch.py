import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    # Get the path to the parameters.yaml file
    config_file = os.path.join(get_package_share_directory('depth_optimization'), 'config', 'parameters.yaml')
    print(config_file)
    #DeclareLaunchArgument('config_file', default_value=config_file, description='Path to the YAML config file')
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=config_file,
        description='Path to the YAML config file'
    )


    # Declare the log_level argument
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level'
    )

    # Launch the dope_node node
    depth_optimizer_server = Node(
        package='depth_optimization',
        executable='depth_optimizer_server',
        name='depth_optimizer_server',
        output='screen',
        emulate_tty=True,
        parameters=[LaunchConfiguration('config_file')],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
    )

    return LaunchDescription([
        config_file_arg,
        log_level_arg,
        depth_optimizer_server
    ])
