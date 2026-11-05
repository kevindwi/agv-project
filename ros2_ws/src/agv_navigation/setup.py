from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'agv_navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kevin',
    maintainer_email='kevin@todo.todo',
    description='Line follower and odometry navigation package for AGV',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'line_follower_node = agv_navigation.line_follower_node:main',
            'odometry_node = agv_navigation.odometry_node:main',
        ],
    },
)
