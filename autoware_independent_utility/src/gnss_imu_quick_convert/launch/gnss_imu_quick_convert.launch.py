#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gnss_imu_quick_convert',
            executable='gnss_imu_quick_convert_node',
            name='gnss_imu_quick_convert',
            output='screen',
        ),
    ])
