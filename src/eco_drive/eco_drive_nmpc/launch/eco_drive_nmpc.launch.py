from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    config_file = os.path.join(
        get_package_share_directory('eco_drive_nmpc'),
        'config',
        'eco_drive_nmpc.yaml'
    )

    eco_drive_node = Node(
        package='eco_drive_nmpc',
        executable='eco_drive_nmpc',
        name='eco_drive_nmpc',
        output='screen',
        parameters=[config_file]
    )

    return LaunchDescription([
        eco_drive_node
    ])