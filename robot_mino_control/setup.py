import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'robot_mino_control'

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
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dzawkan',
    maintainer_email='dzawkan@example.com',
    description='Bringup ros2_control untuk robot_mino menggunakan topic_based_ros2_control',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_bridge_node = robot_mino_control.serial_bridge_node:main',
            'lidar_node.py = robot_mino_control.lidar_node:main',
        ],
    },
)
