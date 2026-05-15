#!/usr/bin/env python3
"""
Launch file for EWMA Controller with Safety Features
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    # Declare launch arguments for easily tunable parameters
    dt_arg = DeclareLaunchArgument(
        'dt',
        default_value='0.1',
        description='Control loop time step (seconds)'
    )
    
    alpha_arg = DeclareLaunchArgument(
        'alpha',
        default_value='0.02',
        description='EWMA smoothing factor'
    )
    
    Kp_arg = DeclareLaunchArgument(
        'Kp',
        default_value='2.0',
        description='Proportional gain'
    )
    
    use_idm_arg = DeclareLaunchArgument(
        'use_idm',
        default_value='true',
        description='Use IDM baseline model'
    )
    
    debug_arg = DeclareLaunchArgument(
        'debug_mode',
        default_value='false',
        description='Enable debug output and plotting'
    )

    use_raw_inputs_arg = DeclareLaunchArgument(
        'use_raw_float_inputs',
        default_value='false',
        description='Use raw Float64 instead of Float64Stamped for inputs'
    )
    
    # Main EWMA Controller node
    controller_node = Node(
        package='simple_proportional_ewma_safety_features',
        executable='simple_proportional_ewma_safety_features',
        name='ewma_controller',
        output='screen',
        parameters=[{
            'dt': LaunchConfiguration('dt'),
            'alpha': LaunchConfiguration('alpha'),
            'Kp': LaunchConfiguration('Kp'),
            'use_idm': LaunchConfiguration('use_idm'),
            'debug_mode': LaunchConfiguration('debug_mode'),
            'use_raw_float_inputs': LaunchConfiguration('use_raw_inputs_arg'),
            'vehicle_length': 5.0,
            'max_accel': 2.0,
            'max_decel': -3.0,
            'max_odom_age_sec': 0.2,
            'max_lead_age_sec': 0.2
        }],
        remappings=[
            ('/ego_odom', '/localization/kinematic_state'),
            ('/leader_speed', '/acc/perception/velocity'),
            ('/leader_position', '/perception/leader_position'),
            ('/gap', '/acc/perception/gap'),
            ('/control_command', '/acc/target_vel'),
        ]
    )

    # Setup directories for plotting
    install_prefix = os.environ['COLCON_PREFIX_PATH'].split(':')[0]
    workspace_root = os.path.abspath(os.path.join(install_prefix, ".."))
    plots_dir = os.path.join(workspace_root, "acc_debug_plots")

    # Live plotter node
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
        get_package_share_directory('simple_proportional_ewma_safety_features'),
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
        alpha_arg,
        Kp_arg,
        use_idm_arg,
        debug_arg,
        use_raw_inputs_arg,
        controller_node,
        plot_node,
        rviz_node
    ])