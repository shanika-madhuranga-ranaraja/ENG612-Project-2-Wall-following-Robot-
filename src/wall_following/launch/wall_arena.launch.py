
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Project 2 custom world
    package_dir = get_package_share_directory('wall_following')

    world_file = os.path.join(
        package_dir,
        'worlds',
        'wall_arena.world'
    )

    # Gazebo ROS launch file
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        ),
        launch_arguments={
            'world': world_file
        }.items()
    )

    # TurtleBot3 Burger model
    robot_model = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models',
        'turtlebot3_burger',
        'model.sdf'
    )

    # Spawn at bottom-left, facing +Y
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'burger',
            '-file', robot_model,
            '-x', '-0.70',
            '-y', '-0.70',
            '-z', '0.01',
            '-Y', '1.5708'
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        TimerAction(
            period=5.0,
            actions=[spawn_robot]
        )
    ])

