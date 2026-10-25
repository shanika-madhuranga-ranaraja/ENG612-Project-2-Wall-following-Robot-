from glob import glob
from setuptools import find_packages, setup

package_name = 'eng612_wall_following'
setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/worlds', glob('worlds/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ENG612 project group',
    maintainer_email='student@example.com',
    description='TurtleBot3 single/dual PID wall following',
    license='Apache-2.0',
    entry_points={'console_scripts': ['wall_follower = eng612_wall_following.node:main']},
)
