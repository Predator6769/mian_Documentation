#!/usr/bin/env python3
"""
Launch file for Simple Proportional Controller
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    # Declare launch arguments
    dt_arg = DeclareLaunchArgument(
        'dt',
        default_value='0.1',
        description='Control loop time step (seconds)'
    )
    
    window_arg = DeclareLaunchArgument(
        'window_seconds',
        default_value='2.0',
        description='Moving average window (seconds)'
    )
    
    Kp_arg = DeclareLaunchArgument(
        'Kp',
        default_value='0.1',
        description='Proportional gain'
    )
    
    use_idm_arg = DeclareLaunchArgument(
        'use_idm',
        default_value='true',
        description='Use IDM baseline model'
    )
    
    debug_arg = DeclareLaunchArgument(
        'debug_mode',
        default_value='true',
        description='Enable debug output'
    )
    
    # Controller node
    controller_node = Node(
        package='simple_proportional_controller',
        executable='simple_proportional_controller',
        name='simple_proportional_controller',
        output='screen',
        parameters=[{
            'dt': LaunchConfiguration('dt'),
            'window_seconds': LaunchConfiguration('window_seconds'),
            'Kp': LaunchConfiguration('Kp'),
            'use_idm': LaunchConfiguration('use_idm'),
            'debug_mode': LaunchConfiguration('debug_mode'),
            'max_accel': 2.0,
            'max_decel': -3.0,
            'vehicle_length': 5.0
        }],
        remappings=[
            ('/ego_odom', '/localization/kinematic_state'),
            ('/leader_speed', '/acc/perception/velocity'),
            ('/leader_position', '/perception/leader_position'),
            ('/gap', '/acc/perception/gap'),
            ('/control_command', '/acc/target_vel'),
        ]
    )

    install_prefix = os.environ['COLCON_PREFIX_PATH'].split(':')[0]

    workspace_root = os.path.abspath(os.path.join(install_prefix, ".."))

    plots_dir = os.path.join(workspace_root, "acc_debug_plots_3_31_2026_traffic_smoother_test_3_vel_acc_667_final")

    plot_node = Node(
        package='simple_proportional_controller',
        executable='live_acc_plotter',
        name='live_acc_plotter',
        output='screen',
        parameters=[{
            'max_buffer_sec': 300.0,
            'save_dir': plots_dir,
        }],
        condition=IfCondition(LaunchConfiguration('debug_mode'))
    )

    
    # RViz for visualization (optional)
    rviz_config = os.path.join(
        get_package_share_directory('simple_proportional_controller'),
        'rviz',
        'controller.rviz'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('debug_mode'))
    )
    
    return LaunchDescription([
        dt_arg,
        window_arg,
        Kp_arg,
        use_idm_arg,
        debug_arg,
        controller_node,
        plot_node,
        rviz_node
    ])
