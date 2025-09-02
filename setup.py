import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'godirect_ros2'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.xml')),
    ],
    install_requires=['setuptools', 'godirect'],
    zip_safe=True,
    maintainer='tomislav',
    maintainer_email='tbazina@gmail.com',
    description='ROS2 interface to Vernier Go Direct Hand Dynamometer',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'grip_force_publisher = godirect_ros2.nodes.grip_force_publisher:main',
            'grip_force_calibration = godirect_ros2.nodes.grip_force_calibration_experiment:main',
        ],
    },
)
