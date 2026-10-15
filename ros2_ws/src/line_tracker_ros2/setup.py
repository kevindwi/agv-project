from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'line_tracker_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='karlindokusuma',
    maintainer_email='karlindokusuma@todo.todo',
    description='Node ROS 2 line tracking: kamera -> PID -> serial ke firmware AGV',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'line_tracker_node = line_tracker_ros2.line_tracker_node:main',
        ],
    },
)
