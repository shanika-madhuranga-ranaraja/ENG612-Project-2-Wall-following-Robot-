
import os
from glob import glob
from setuptools import setup

package_name = 'wall_following'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')
        ),
        (
            os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.world')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shan',
    maintainer_email='shan@example.com',
    description='ENG612 Project 2 TurtleBot3 PID wall following',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_subscriber = wall_following.lidar_subscriber:main',
            'wall_following_pid = wall_following.wall_following_pid:main',
        ],
    },
)

