from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    share = get_package_share_directory('eng612_wall_following')
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='single', choices=['single', 'dual']),
        DeclareLaunchArgument('wall_side', default_value='right', choices=['right', 'left']),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('enabled', default_value='false'),
        DeclareLaunchArgument('log_path', default_value=''),
        DeclareLaunchArgument('base_speed', default_value='0.10'),
        DeclareLaunchArgument('stamped_cmd_vel', default_value='false'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(share, 'config', 'controller.yaml')),
        Node(package='eng612_wall_following', executable='wall_follower',
             name='wall_follower', output='screen', parameters=[
                 LaunchConfiguration('params_file'),
                 {'mode': LaunchConfiguration('mode'),
                  'wall_side': LaunchConfiguration('wall_side'),
                  'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                  'enabled': ParameterValue(LaunchConfiguration('enabled'), value_type=bool),
                  'base_speed': ParameterValue(LaunchConfiguration('base_speed'), value_type=float),
                  'stamped_cmd_vel': ParameterValue(LaunchConfiguration('stamped_cmd_vel'), value_type=bool),
                  'log_path': LaunchConfiguration('log_path')}]),
    ])
