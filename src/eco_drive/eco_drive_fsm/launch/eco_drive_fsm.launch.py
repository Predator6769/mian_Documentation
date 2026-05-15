from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    eco_drive_fsm_node = Node(
        package='eco_drive_fsm',
        executable='eco_drive_fsm',
        name='eco_drive_fsm',
        output='screen'
    )

    return LaunchDescription([
        eco_drive_fsm_node
    ])