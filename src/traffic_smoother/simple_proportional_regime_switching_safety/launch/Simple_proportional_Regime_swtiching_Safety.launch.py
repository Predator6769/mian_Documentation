#!/usr/bin/env python3
"""
Launch file for Regime Switching Controller with Safety Features
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    # ==========================================
    # Declare Launch Arguments for ALL Parameters
    # ==========================================
    dt_arg = DeclareLaunchArgument(
        'dt',
        default_value='0.1',
        description='Control loop time step (seconds)'
    )
    
    cruising_window_arg = DeclareLaunchArgument(
        'cruising_window',
        default_value='30.0',
        description='Cruising window (seconds)'
    )

    accel_trend_threshold_arg = DeclareLaunchArgument(
        'accel_trend_threshold',
        default_value='0.1',
        description='Acceleration detection threshold'
    )

    decel_trend_threshold_arg = DeclareLaunchArgument(
        'decel_trend_threshold',
        default_value='-0.8',
        description='Deceleration detection threshold'
    )
    
    Kp_arg = DeclareLaunchArgument(
        'Kp',
        default_value='2.0',
        description='Proportional gain'
    )

    vehicle_length_arg = DeclareLaunchArgument(
        'vehicle_length',
        default_value='5.0',
        description='Vehicle length'
    )
    
    use_idm_arg = DeclareLaunchArgument(
        'use_idm',
        default_value='true',
        description='Use IDM baseline model'
    )

    max_accel_arg = DeclareLaunchArgument(
        'max_accel',
        default_value='2.0',
        description='Maximum acceleration limit'
    )

    max_decel_arg = DeclareLaunchArgument(
        'max_decel',
        default_value='-3.0',
        description='Maximum deceleration limit'
    )
    
    debug_mode_arg = DeclareLaunchArgument(
        'debug_mode',
        default_value='false',
        description='Enable debug output and plotting'
    )

    use_raw_inputs_arg = DeclareLaunchArgument(
        'use_raw_float_inputs',
        default_value='false',
        description='Use raw Float64 instead of Float64Stamped for inputs'
    )

    max_odom_age_sec_arg = DeclareLaunchArgument(
        'max_odom_age_sec',
        default_value='0.2',
        description='Max allowed age for odometry messages'
    )

    max_lead_age_sec_arg = DeclareLaunchArgument(
        'max_lead_age_sec',
        default_value='0.2',
        description='Max allowed age for leader speed/position messages'
    )
    
    # ==========================================
    # Main Regime Switching Controller Node
    # ==========================================
    controller_node = Node(
        package='simple_proportional_regime_switching_safety',
        executable='simple_proportional_regime_switching_safety',
        name='regime_switching_controller',
        output='screen',
        parameters=[{
            'dt': LaunchConfiguration('dt'),
            'cruising_window': LaunchConfiguration('cruising_window'),
            'accel_trend_threshold': LaunchConfiguration('accel_trend_threshold'),
            'decel_trend_threshold': LaunchConfiguration('decel_trend_threshold'),
            'Kp': LaunchConfiguration('Kp'),
            'vehicle_length': LaunchConfiguration('vehicle_length'),
            'use_idm': LaunchConfiguration('use_idm'),
            'max_accel': LaunchConfiguration('max_accel'),
            'max_decel': LaunchConfiguration('max_decel'),
            'debug_mode': LaunchConfiguration('debug_mode'),
            'use_raw_float_inputs': LaunchConfiguration('use_raw_float_inputs'),
            'max_odom_age_sec': LaunchConfiguration('max_odom_age_sec'),
            'max_lead_age_sec': LaunchConfiguration('max_lead_age_sec')
        }],
        remappings=[
            ('/ego_odom', '/localization/kinematic_state'),
            ('/leader_speed', '/acc/perception/velocity'),
            ('/leader_position', '/perception/leader_position'),
            ('/gap', '/acc/perception/gap'),
            ('/control_command', '/acc/target_vel'),
        ]
    )

    # ==========================================
    # Setup directories for plotting
    # ==========================================
    install_prefix = os.environ['COLCON_PREFIX_PATH'].split(':')[0]
    workspace_root = os.path.abspath(os.path.join(install_prefix, ".."))
    plots_dir = os.path.join(workspace_root, "acc_debug_plots")

    # Live plotter node (Strictly targeting simple_proportional_controller package)
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
    
    # ==========================================
    # RViz for visualization (optional)
    # ==========================================
    rviz_config = os.path.join(
        get_package_share_directory('simple_proportional_regime_switching_safety'),
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
        # Launch Arguments
        dt_arg,
        cruising_window_arg,
        accel_trend_threshold_arg,
        decel_trend_threshold_arg,
        Kp_arg,
        vehicle_length_arg,
        use_idm_arg,
        max_accel_arg,
        max_decel_arg,
        debug_mode_arg,
        use_raw_inputs_arg,
        max_odom_age_sec_arg,
        max_lead_age_sec_arg,
        
        # Nodes
        controller_node,
        plot_node,
        rviz_node
    ])