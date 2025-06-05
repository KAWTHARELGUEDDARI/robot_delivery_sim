from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_delivery_sim'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='kawthar.elgueddari@usmba.ac.ma',
    description='A ROS 2 package for simulating a delivery robot environment.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'env_node = robot_delivery_sim.env_node:main',
            'robot_controller = robot_delivery_sim.robot_controller:main',
        ],
    },
)
