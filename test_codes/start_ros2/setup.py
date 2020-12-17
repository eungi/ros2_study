#!/usr/bin/env python3
# Authors: Eungi Cho

import glob
import os

from setuptools import setup


package_name = 'start_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            glob.glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eungi',
    maintainer_email='ceg@robotis.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'topic_publisher = start_ros2.topic_publisher:main',
            'custom_topic_publisher = start_ros2.custom_topic_publisher:main',
            'custom_topic_subscriber = start_ros2.custom_topic_subscriber:main'
        ],
    },
)
