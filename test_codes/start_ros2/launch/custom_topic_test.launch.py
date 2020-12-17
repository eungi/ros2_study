#!/usr/bin/env python3
# Authors: Eungi Cho

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='start_ros2',
            node_namespace='test1',
            node_executable='custom_topic_publisher',
            # output='screen'
        ),
        Node(
            package='start_ros2',
            node_namespace='test1',
            node_executable='custom_topic_subscriber',
            # output='screen'
        )
    ])
