"""Controller for a real Burger; run robot bringup separately on its SBC."""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share = get_package_share_directory('eng612_wall_following')
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='dual', choices=['single', 'dual']),
        DeclareLaunchArgument('wall_side', default_value='right', choices=['right', 'left']),
        DeclareLaunchArgument('base_speed', default_value='0.05'),
        DeclareLaunchArgument('log_path', default_value=''),
        DeclareLaunchArgument('stamped_cmd_vel', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'control.launch.py')),
            launch_arguments={
                'mode': LaunchConfiguration('mode'),
                'wall_side': LaunchConfiguration('wall_side'),
                'base_speed': LaunchConfiguration('base_speed'),
                'log_path': LaunchConfiguration('log_path'),
                'stamped_cmd_vel': LaunchConfiguration('stamped_cmd_vel'),
                'params_file': os.path.join(share, 'config', 'controller_real.yaml'),
                'use_sim_time': 'false',
                'enabled': 'false',
            }.items()),
    ])
