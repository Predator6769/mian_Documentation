#!/usr/bin/env python3
"""
Simple Proportional Controller ROS2 Node
For traffic smoothing using moving average equilibrium estimation
"""

import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import numpy as np
from collections import deque

# ROS2 message types
from std_msgs.msg import Float64, Bool, String
from geometry_msgs.msg import Twist, TwistStamped, Point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from autoware_auto_vehicle_msgs.msg import VelocityReport
from tier4_debug_msgs.msg import Float64Stamped

# Custom message (if using, otherwise comment out)
# from your_package.msg import ControlCommand

class SimpleProportionalController(Node):
    """
    ROS2 Node for Simple Proportional Controller
    
    Subscribes to:
    - /ego_odom: Odometry of ego vehicle (for current speed)
    - /leader_speed: Speed of leader vehicle (Float64Stamped)
    - /leader_position: Position of leader (Point or Float64 for longitudinal distance)
    - /gap: Distance to leader (Float64Stamped)
    
    Publishes:
    - /control_command: Desired acceleration/speed (TwistStamped or custom)
    - /equilibrium_speed: Estimated equilibrium speed (Float64)
    - /controller_debug: Debug information (String)
    - /controller_markers: Visualization markers (MarkerArray)
    """
    
    def __init__(self):
        super().__init__('simple_proportional_controller')
        
        # Declare parameters with default values
        self.declare_parameter('dt', 0.1)
        self.declare_parameter('window_seconds', 3.0)
        self.declare_parameter('Kp', 0.1)
        self.declare_parameter('vehicle_length', 5.0)
        self.declare_parameter('use_idm', True)
        self.declare_parameter('max_accel', 2.0)
        self.declare_parameter('max_decel', -3.0)
        self.declare_parameter('debug_mode', False)
        self.declare_parameter('debug_log_path', '/tmp/simple_proportional_controller_debug.log')
        
        # Get parameters
        self.dt = self.get_parameter('dt').value
        self.window_seconds = self.get_parameter('window_seconds').value
        self.Kp = self.get_parameter('Kp').value
        self.vehicle_length = self.get_parameter('vehicle_length').value
        self.use_idm = self.get_parameter('use_idm').value
        self.max_accel = self.get_parameter('max_accel').value
        self.max_decel = self.get_parameter('max_decel').value
        self.debug_mode = self.get_parameter('debug_mode').value
        self.debug_log_path = self.get_parameter('debug_log_path').value
        
        # Calculate window samples
        self.window_samples = int(np.ceil(self.window_seconds / self.dt))
        
        # Initialize buffers
        self.leader_speed_buffer = deque(maxlen=self.window_samples)
        
        # State variables
        self.current_speed = 0.0
        self.current_position = 0.0
        self.leader_speed = 0.0
        self.leader_position = 0.0
        self.gap = 10.0  # initial large gap
        self.equilibrium_speed = 0.0
        self.desired_acceleration = 0.0
        self.desired_speed = 0.0
        
        # Safety flags
        self.leader_detected = False
        self.emergency_brake = False
        self.debug_log_file = None
        
        # IDM parameters (if using)
        # self.idm_params = {
        #     'v0': 6.705,      # Desired speed (m/s)
        #     'T': 1.5,          # Safe time headway (s)
        #     's0': 15.0,         # Minimum spacing (m)
        #     'delta': 4.0,      # Acceleration exponent
        #     'a': 2.0,          # Maximum acceleration (m/s²)
        #     'b': 2.5          # Comfortable deceleration (m/s²)
        # }

        self.idm_params = {
            'v0': 10.00,
            'T': 1.0,
            's0': 2.5,
            'delta': 4.0,
            'a': 1.0,
            'b': 0.5
        }
        
        # Setup QoS profiles
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/ego_odom',
            self.position_callback,
            qos_profile
        )

        self.ego_velocity_sub = self.create_subscription(
            VelocityReport,
            '/vehicle/status/velocity_status',
            self.velocity_status_callback,
            qos_profile
        )
        
        self.leader_speed_sub = self.create_subscription(
            Float64Stamped,
            '/leader_speed',
            self.leader_speed_callback,
            qos_profile
        )
        
        self.leader_position_sub = self.create_subscription(
            Point,
            '/leader_position',
            self.leader_position_callback,
            qos_profile
        )
        
        self.gap_sub = self.create_subscription(
            Float64Stamped,
            '/gap',
            self.gap_callback,
            qos_profile
        )
        
        # Publishers
        self.control_pub = self.create_publisher(
            TwistStamped,
            '/control_command',
            10
        )
        
        self.equilibrium_pub = self.create_publisher(
            Float64,
            '/equilibrium_speed',
            10
        )
        
        self.debug_pub = self.create_publisher(
            String,
            '/controller_debug',
            10
        )
        
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/controller_markers',
            10
        )
        
        # Control timer (runs at dt frequency)
        self.control_timer = self.create_timer(self.dt, self.control_loop)
        
        # Status timer (for logging)
        self.status_timer = self.create_timer(1.0, self.status_callback)

        self.setup_debug_log_file()
        
        self.get_logger().info(
            f"Simple Proportional Controller initialized:\n"
            f"  dt: {self.dt}s, window: {self.window_seconds}s ({self.window_samples} samples)\n"
            f"  Kp: {self.Kp}, use_idm: {self.use_idm}\n"
            f"  max_accel: {self.max_accel}, max_decel: {self.max_decel}\n"
            f"  debug_log_path: {self.debug_log_path}"
        )

    def setup_debug_log_file(self):
        """Open a line-buffered text file for debug output."""
        log_dir = os.path.dirname(self.debug_log_path)
        if log_dir:
            os.makedirs(log_dir, exist_ok=True)

        self.debug_log_file = open(self.debug_log_path, 'a', buffering=1, encoding='utf-8')
        self.debug_log_file.write("# Simple Proportional Controller debug log\n")

    def close_debug_log_file(self):
        """Close the debug log file if it is open."""
        if self.debug_log_file is not None and not self.debug_log_file.closed:
            self.debug_log_file.close()
        self.debug_log_file = None
    
    def position_callback(self, msg):
        """Callback for odometry messages used only for position."""
        self.current_position = msg.pose.pose.position.x

        if self.debug_mode:
            self.get_logger().debug(f"Odometry position: pos={self.current_position:.2f}")

    def velocity_status_callback(self, msg):
        """Callback for ego velocity from vehicle status."""
        self.current_speed = msg.longitudinal_velocity

        if self.debug_mode:
            self.get_logger().debug(f"Velocity status: speed={self.current_speed:.2f}")
    
    def leader_speed_callback(self, msg):
        """Callback for leader speed messages"""
        self.leader_speed = msg.data
        self.leader_detected = True
        
        # Add to buffer for moving average
        self.leader_speed_buffer.append(self.leader_speed)
        
        if self.debug_mode:
            self.get_logger().debug(f"Leader speed: {self.leader_speed:.2f}")
    
    def leader_position_callback(self, msg):
        """Callback for leader position messages"""
        self.leader_position = msg.x  # Assuming x is longitudinal coordinate
        self.update_gap_from_positions()
    
    def gap_callback(self, msg):
        """Callback for direct gap measurements (e.g., from radar)"""
        self.gap = msg.data
        if self.debug_mode:
            self.get_logger().debug(f"Gap measurement: {self.gap:.2f}m")
    
    def update_gap_from_positions(self):
        """Calculate gap from ego and leader positions"""
        if hasattr(self, 'current_position') and hasattr(self, 'leader_position'):
            self.gap = self.leader_position - self.current_position - self.vehicle_length
            if self.gap < 0:
                self.get_logger().warn(f"Negative gap detected: {self.gap:.2f}m")
                self.emergency_brake = True
            else:
                self.emergency_brake = False
    
    def estimate_equilibrium(self):
        """
        Estimate equilibrium speed using moving average of leader speed
        """
        if len(self.leader_speed_buffer) == 0:
            # No data yet, use current speed as estimate
            return self.current_speed
        
        # Moving average
        equilibrium = np.mean(list(self.leader_speed_buffer))
        return equilibrium
    
    def idm_acceleration(self, speed, lead_speed, gap):
        """
        Intelligent Driver Model acceleration
        """
        v = speed
        v_lead = lead_speed
        s = gap
        delta_v = v - v_lead
        
        # Extract IDM parameters
        v0 = self.idm_params['v0']
        T = self.idm_params['T']
        s0 = self.idm_params['s0']
        delta = self.idm_params['delta']
        a = self.idm_params['a']
        b = self.idm_params['b']
        
        # Desired spacing
        if s > 0 and v > 0:
            s_star = s0 + max(0, v * T + (v * delta_v) / (2 * np.sqrt(a * b)))
        else:
            s_star = s0 + v * T
        
        # IDM acceleration formula
        if s > 0 and s_star > 0:
            accel = a * (1 - (v / v0)**delta - (s_star / s)**2)
        else:
            accel = a * (1 - (v / v0)**delta)
        
        # Limit
        accel = np.clip(accel, -b, a)
        
        return accel
    
    def control_loop(self):
        """
        Main control loop - runs at dt frequency
        """
        # Check if we have necessary data
        if not self.leader_detected:
            self.get_logger().warn("No leader detected, cannot compute control", throttle_duration_sec=2.0)
            return
        
        # Step 1: Estimate equilibrium speed
        self.equilibrium_speed = self.estimate_equilibrium()
        
        # Step 2: Compute baseline acceleration
        if self.use_idm:
            idm_accel = self.idm_acceleration(
                self.current_speed, 
                self.leader_speed, 
                self.gap
            )
        else:
            idm_accel = 0.0
        
        # Step 3: Compute proportional control term
        speed_error = self.equilibrium_speed - self.current_speed
        control_term = self.Kp * speed_error
        
        # Step 4: Total desired acceleration
        desired_accel = idm_accel + control_term
        
        # Step 5: Apply acceleration limits
        desired_accel = np.clip(desired_accel, self.max_decel, self.max_accel)
        
        # Step 6: Emergency brake if gap too small
        if self.gap < 2.0:  # 2 meters minimum safety distance
            desired_accel = self.max_decel
            self.desired_speed = 0.0
            self.get_logger().warn(f"Emergency braking! Gap: {self.gap:.2f}m")
        else:
        # Step 7: Compute desired speed for logging
            self.desired_speed = self.current_speed + desired_accel * self.dt
            self.desired_acceleration = desired_accel
        
        # Step 8: Publish control command
        self.publish_control(desired_accel)
        
        # Step 9: Publish equilibrium speed
        eq_msg = Float64()
        eq_msg.data = self.equilibrium_speed
        self.equilibrium_pub.publish(eq_msg)
        
        # Step 10: Publish debug info
        if self.debug_mode:
            self.publish_debug(idm_accel, control_term, speed_error)
        
        # Step 11: Publish visualization markers
        self.publish_markers()
    
    def publish_control(self, acceleration):
        """
        Publish control command as TwistStamped message

        Using Twist linear.x for desired acceleration
        (or you can use linear.x for desired speed and angular.z for acceleration)
        """
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        
        # Option A: Send desired acceleration
        cmd.twist.linear.z = acceleration  # m/s²
        
        # Option B: Send desired speed (uncomment if your vehicle uses speed commands)
        cmd.twist.linear.x = self.desired_speed  # m/s
        
        # Add timestamp or other info in angular (optional)
        cmd.twist.angular.y = self.equilibrium_speed  # just for debugging
        
        self.control_pub.publish(cmd)
    
    def publish_debug(self, idm_accel, control_term, speed_error):
        """
        Publish debug information
        """
        debug_msg = String()
        debug_msg.data = (
            f"t={self.get_clock().now().nanoseconds/1e9:.2f}, "
            f"v_ego={self.current_speed:.2f}, v_leader={self.leader_speed:.2f}, "
            f"U_eq={self.equilibrium_speed:.2f}, error={speed_error:.2f}, "
            f"gap={self.gap:.2f}, idm={idm_accel:.3f}, control={control_term:.3f}, "
            f"accel={self.desired_acceleration:.3f}"
        )
        self.debug_pub.publish(debug_msg)
        if self.debug_log_file is not None:
            self.debug_log_file.write(debug_msg.data + "\n")
        self.get_logger().debug(debug_msg.data)
    
    def publish_markers(self):
        """
        Publish visualization markers for RViz
        """
        marker_array = MarkerArray()
        
        # Text marker for debug info
        text_marker = Marker()
        text_marker.header.frame_id = "map"
        text_marker.header.stamp = self.get_clock().now().to_msg()
        text_marker.ns = "controller_info"
        text_marker.id = 0
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position.x = self.current_position
        text_marker.pose.position.y = 2.0
        text_marker.pose.position.z = 2.0
        text_marker.scale.z = 1.0
        text_marker.color.a = 1.0
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.text = (
            f"U_eq: {self.equilibrium_speed:.1f}\n"
            f"a_des: {self.desired_acceleration:.2f}"
        )
        
        marker_array.markers.append(text_marker)
        self.marker_pub.publish(marker_array)
    
    def status_callback(self):
        """
        Periodic status update (every 1 second)
        """
        self.get_logger().info(
            f"Status - Speed: {self.current_speed:.2f}, "
            f"Desired Speed: {self.desired_speed:.2f}, "
            f"Leader: {self.leader_speed:.2f}, "
            f"Eq: {self.equilibrium_speed:.2f}, "
            f"Gap: {self.gap:.2f}, "
            f"Accel: {self.desired_acceleration:.3f}"
        )
    
    def reset(self):
        """Reset controller state"""
        self.leader_speed_buffer.clear()
        self.equilibrium_speed = 0.0
        self.desired_acceleration = 0.0
        self.get_logger().info("Controller reset")

    def destroy_node(self):
        self.close_debug_log_file()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = SimpleProportionalController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Controller stopped by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
