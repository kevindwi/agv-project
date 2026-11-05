from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'agv_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kevin',
    maintainer_email='kevin@todo.todo',
    description='Hardware interface and core launch files for AGV',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = agv_bringup.camera_node:main',
            'serial_bridge_node = agv_bringup.serial_bridge_node:main',
        ],
    },
)
