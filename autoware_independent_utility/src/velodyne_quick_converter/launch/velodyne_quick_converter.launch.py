#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='velodyne_quick_converter',
            executable='velodyne_quick_converter_node',
            name='velodyne_quick_converter',
            output='screen',
        ),
    ])
