"""Humble/Gazebo Classic only. Starts arena and Burger, not the controller."""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    project = get_package_share_directory('eng612_wall_following')
    gazebo = get_package_share_directory('gazebo_ros')
    tb3 = get_package_share_directory('turtlebot3_gazebo')
    description = get_package_share_directory('turtlebot3_description')
    with open(os.path.join(description, 'urdf', 'turtlebot3_burger.urdf')) as handle:
        robot_description = handle.read()
    models = os.path.join(tb3, 'models')
    model_path = os.pathsep.join(filter(None, [models, os.environ.get('GAZEBO_MODEL_PATH', '')]))
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=os.path.join(project, 'worlds', 'arena.world')),
        DeclareLaunchArgument('x', default_value='-0.4'),
        DeclareLaunchArgument('y', default_value='-0.7'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger'),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_path),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo, 'launch', 'gazebo.launch.py')),
            launch_arguments={'world': LaunchConfiguration('world'), 'verbose': 'false'}.items()),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'use_sim_time': True, 'robot_description': robot_description}], output='screen'),
        Node(package='gazebo_ros', executable='spawn_entity.py', output='screen',
             arguments=['-entity', 'burger', '-file',
                        os.path.join(models, 'turtlebot3_burger', 'model.sdf'),
                        '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'),
                        '-z', '0.01', '-Y', LaunchConfiguration('yaw'), '-timeout', '90.0']),
    ])
