#!/usr/bin/env python3
"""
EWMA Controller ROS2 Node
For traffic smoothing using EWMA equilibrium estimation
WITH SAFETY FEATURES: Disables control when gap < 15m, enforces min gap 10m
"""

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
try:
    from tier4_debug_msgs.msg import Float64Stamped
except ImportError:
    Float64Stamped = None


class EWMAController(Node):
    """
    ROS2 Node for EWMA Controller
    
    SAFETY FEATURES:
    - Control disabled when gap < SAFE_GAP_THRESHOLD (17m)
    - Minimum gap enforced: cannot go below MIN_SAFE_GAP (10m)
    - Emergency braking when gap becomes negative
    - Max deceleration limited to IDM b parameter (-0.5 m/s²)
    """
    
    def __init__(self):
        super().__init__('ewma_controller')
        
        # ==================== SAFETY PARAMETERS ====================
        self.SAFE_GAP_THRESHOLD = 17.0   # Gap below which control is disabled (meters)
        self.MIN_SAFE_GAP = 10.0         # Minimum allowed gap (meters)
        # ===========================================================
        
        # Declare parameters with default values
        self.declare_parameter('dt', 0.1)
        self.declare_parameter('alpha', 0.02)      # EWMA smoothing factor
        self.declare_parameter('Kp', 2.0)          # Proportional gain
        self.declare_parameter('vehicle_length', 5.0)
        self.declare_parameter('use_idm', True)
        self.declare_parameter('max_accel', 2.0)
        self.declare_parameter('max_decel', -3.0)
        self.declare_parameter('debug_mode', False)
        self.declare_parameter('use_raw_float_inputs', False)
        self.declare_parameter('max_odom_age_sec', 0.2)
        self.declare_parameter('max_lead_age_sec', 0.2)
        
        # Get parameters
        self.dt = self.get_parameter('dt').value
        self.alpha = self.get_parameter('alpha').value      # EWMA smoothing factor
        self.Kp = self.get_parameter('Kp').value
        self.vehicle_length = self.get_parameter('vehicle_length').value
        self.use_idm = self.get_parameter('use_idm').value
        self.max_accel = self.get_parameter('max_accel').value
        self.max_decel = self.get_parameter('max_decel').value
        self.debug_mode = self.get_parameter('debug_mode').value
        self.use_raw_float_inputs = self.get_parameter('use_raw_float_inputs').value
        self.max_odom_age_sec = self.get_parameter('max_odom_age_sec').value
        self.max_lead_age_sec = self.get_parameter('max_lead_age_sec').value
        
        # State variables
        self.current_speed = 0.0
        self.current_position = 0.0
        self.leader_speed = 0.0
        self.leader_position = 0.0
        self.gap = 10.0  # initial large gap
        self.equilibrium_speed = 0.0
        self.desired_acceleration = 0.0
        self.desired_speed = 0.0
        
        # EWMA state - stores previous equilibrium estimate
        self.prev_equilibrium = None
        
        # Message timestamps (seconds)
        self.current_speed_stamp = None
        self.leader_speed_stamp = None
        self.gap_stamp = None
        
        # Safety flags
        self.leader_detected = False
        self.emergency_brake = False
        self.control_disabled = False  # Flag when control is disabled due to small gap
        
        # Safety counters for logging
        self.safety_violation_count = 0
        self.control_disabled_count = 0
        
        # IDM parameters (from your simulation)
        # AV parameters: [v0, T, s0, delta, a, b]
        # v0 = 10.0 m/s (36 km/h), T = 1.00 s, s0 = 2.5 m, delta = 4.0, a = 1.00 m/s², b = 0.5 m/s²
        self.idm_params = {
            'v0': 10.0,      # Desired speed (m/s) - 36 km/h
            'T': 1.00,       # Safe time headway (s) - AV aggressive
            's0': 2.5,       # Minimum spacing (m)
            'delta': 4.0,    # Acceleration exponent
            'a': 1.00,       # Maximum acceleration (m/s²)
            'b': 0.5         # Comfortable deceleration (m/s²) - Max decel limit
        }
        
        # Log safety parameters
        self.get_logger().info(
            f"SAFETY CONFIGURATION:\n"
            f"  Control disabled when gap < {self.SAFE_GAP_THRESHOLD} m\n"
            f"  Minimum allowed gap: {self.MIN_SAFE_GAP} m\n"
            f"  Max deceleration (IDM b): {self.idm_params['b']} m/s²"
        )
        
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
            self.odom_callback,
            qos_profile
        )
        
        leader_msg_type = Float64 if (self.use_raw_float_inputs or Float64Stamped is None) else Float64Stamped
        gap_msg_type = Float64 if (self.use_raw_float_inputs or Float64Stamped is None) else Float64Stamped

        if Float64Stamped is None and not self.use_raw_float_inputs:
            self.get_logger().warn(
                "tier4_debug_msgs is unavailable; falling back to raw Float64 inputs for leader speed and gap."
            )
            self.use_raw_float_inputs = True

        self.leader_speed_sub = self.create_subscription(
            leader_msg_type,
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
            gap_msg_type,
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
        
        self.get_logger().info(
            f"EWMA Controller initialized:\n"
            f"  dt: {self.dt}s, alpha: {self.alpha}\n"
            f"  Kp: {self.Kp}, use_idm: {self.use_idm}\n"
            f"  SAFETY: control_off_gap<{self.SAFE_GAP_THRESHOLD}m, min_gap={self.MIN_SAFE_GAP}m\n"
            f"  IDM: v0={self.idm_params['v0']}, T={self.idm_params['T']}, b={self.idm_params['b']}"
        )

    @staticmethod
    def stamp_to_sec(stamp):
        return float(stamp.sec) + (float(stamp.nanosec) * 1e-9)

    def get_msg_time_s(self, msg):
        if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
            stamp = msg.header.stamp
            if stamp.sec != 0 or stamp.nanosec != 0:
                return self.stamp_to_sec(stamp)

        if hasattr(msg, 'stamp'):
            stamp = msg.stamp
            if hasattr(stamp, 'sec') and hasattr(stamp, 'nanosec'):
                if stamp.sec != 0 or stamp.nanosec != 0:
                    return self.stamp_to_sec(stamp)

        return float(self.get_clock().now().nanoseconds) * 1e-9

    def latest_safe_accel(self):
        """Return safe deceleration when data is stale"""
        return min(0.0, self.desired_acceleration)

    def validate_state_freshness(self):
        """Validate that incoming data is not stale"""
        if (self.current_speed_stamp is None or
            self.leader_speed_stamp is None or
            self.gap_stamp is None):
            return False, "waiting for stamped odom/leader/gap messages"

        now_sec = float(self.get_clock().now().nanoseconds) * 1e-9
        odom_age = now_sec - self.current_speed_stamp
        leader_age = now_sec - self.leader_speed_stamp
        gap_age = now_sec - self.gap_stamp

        if odom_age > self.max_odom_age_sec:
            return False, f"stale odom ({odom_age:.2f}s old)"

        if leader_age > self.max_lead_age_sec:
            return False, f"stale leader speed ({leader_age:.2f}s old)"

        if gap_age > self.max_lead_age_sec:
            return False, f"stale gap ({gap_age:.2f}s old)"

        return True, ""
    
    def odom_callback(self, msg):
        """Callback for odometry messages"""
        self.current_speed = msg.twist.twist.linear.x
        self.current_position = msg.pose.pose.position.x
        self.current_speed_stamp = self.get_msg_time_s(msg)
        
        if self.debug_mode:
            self.get_logger().debug(f"Odometry: speed={self.current_speed:.2f}, pos={self.current_position:.2f}")
    
    def leader_speed_callback(self, msg):
        """Callback for leader speed messages"""
        self.leader_speed = msg.data
        self.leader_speed_stamp = self.get_msg_time_s(msg)
        self.leader_detected = True
        
        if self.debug_mode:
            self.get_logger().debug(f"Leader speed: {self.leader_speed:.2f}")
    
    def leader_position_callback(self, msg):
        """Callback for leader position messages"""
        self.leader_position = msg.x
        self.gap_stamp = self.get_msg_time_s(msg)
        self.update_gap_from_positions()
    
    def gap_callback(self, msg):
        """Callback for direct gap measurements (e.g., from radar)"""
        self.gap = msg.data
        self.gap_stamp = self.get_msg_time_s(msg)
        if self.debug_mode:
            self.get_logger().debug(f"Gap measurement: {self.gap:.2f}m")
    
    def update_gap_from_positions(self):
        """Calculate gap from ego and leader positions with safety enforcement"""
        if hasattr(self, 'current_position') and hasattr(self, 'leader_position'):
            raw_gap = self.leader_position - self.current_position - self.vehicle_length
            
            # SAFETY: Enforce minimum gap
            if raw_gap < self.MIN_SAFE_GAP:
                self.safety_violation_count += 1
                self.gap = self.MIN_SAFE_GAP
                if self.debug_mode:
                    self.get_logger().warn(f"Gap enforcement: {raw_gap:.2f}m -> {self.MIN_SAFE_GAP}m")
            else:
                self.gap = raw_gap
            
            # Emergency braking if gap becomes negative
            if self.gap < 0:
                self.get_logger().error(f"NEGATIVE GAP DETECTED: {self.gap:.2f}m - EMERGENCY BRAKING")
                self.emergency_brake = True
            else:
                self.emergency_brake = False
    
    def estimate_equilibrium_ewma(self):
        """
        Estimate equilibrium speed using EWMA (Exponentially Weighted Moving Average)
        Formula: U_t = alpha * v_t + (1 - alpha) * U_{t-1}
        """
        if self.prev_equilibrium is None:
            # First sample: use current leader speed as initial estimate
            self.prev_equilibrium = self.leader_speed
            return self.leader_speed
        
        # EWMA formula
        equilibrium = self.alpha * self.leader_speed + (1 - self.alpha) * self.prev_equilibrium
        self.prev_equilibrium = equilibrium
        return equilibrium
    
    def idm_acceleration(self, speed, lead_speed, gap):
        """
        Intelligent Driver Model acceleration
        Using AV parameters from simulation
        """
        v = speed
        v_lead = lead_speed
        s = gap
        delta_v = v - v_lead
        
        # Extract IDM parameters (AV aggressive parameters)
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
        
        # Limit deceleration to IDM b parameter (-0.5 m/s²)
        accel = np.clip(accel, -b, a)
        
        return accel
    
    def control_loop(self):
        """
        Main control loop - runs at dt frequency
        WITH SAFETY FEATURES:
        1. Control disabled when gap < SAFE_GAP_THRESHOLD (17m)
        2. Minimum gap enforced (10m)
        3. Emergency braking for negative gap
        """
        # Check if we have necessary data
        if not self.leader_detected:
            self.get_logger().warn("No leader detected, cannot compute control", throttle_duration_sec=2.0)
            return

        data_fresh, freshness_reason = self.validate_state_freshness()
        if not data_fresh:
            desired_accel = self.latest_safe_accel()
            self.desired_acceleration = desired_accel
            self.desired_speed = max(0.0, self.current_speed + desired_accel * self.dt)
            self.publish_control(desired_accel)
            self.get_logger().warn(
                f"Using safe fallback command due to stale data: {freshness_reason}",
                throttle_duration_sec=2.0
            )
            return
        
        # Step 1: Estimate equilibrium speed using EWMA
        self.equilibrium_speed = self.estimate_equilibrium_ewma()
        
        # Step 2: Compute baseline acceleration using IDM
        if self.use_idm:
            idm_accel = self.idm_acceleration(
                self.current_speed,
                self.leader_speed,
                self.gap
            )
        else:
            idm_accel = 0.0
        
        # SAFETY: Check if control should be disabled
        self.control_disabled = (self.gap < self.SAFE_GAP_THRESHOLD)
        
        if self.control_disabled:
            self.control_disabled_count += 1
            # Use only IDM (no control term)
            desired_accel = idm_accel
            if self.debug_mode and self.control_disabled_count % 100 == 0:
                self.get_logger().warn(
                    f"Control DISABLED - Gap: {self.gap:.2f}m < {self.SAFE_GAP_THRESHOLD}m"
                )
        else:
            # Step 3: Compute proportional control term
            speed_error = self.equilibrium_speed - self.current_speed
            control_term = self.Kp * speed_error
            
            # Step 4: Total desired acceleration
            desired_accel = idm_accel + control_term
        
        # SAFETY: Emergency braking if gap becomes negative or emergency brake triggered
        if self.emergency_brake or self.gap < 0:
            desired_accel = -self.idm_params['b']  # Use IDM b parameter (-0.5 m/s²)
            self.desired_speed = 0.0
            self.get_logger().error(f"EMERGENCY BRAKING - Gap: {self.gap:.2f}m")
        else:
            # Step 5: Apply acceleration limits (max_accel to max_decel)
            desired_accel = np.clip(desired_accel, self.max_decel, self.max_accel)
            # Also respect IDM b as maximum deceleration
            desired_accel = max(desired_accel, -self.idm_params['b'])
            # Compute desired speed for logging
            self.desired_speed = self.current_speed + desired_accel * self.dt
        
        self.desired_acceleration = desired_accel
        
        # Step 6: Publish control command
        self.publish_control(desired_accel)
        
        # Step 7: Publish equilibrium speed
        eq_msg = Float64()
        eq_msg.data = self.equilibrium_speed
        self.equilibrium_pub.publish(eq_msg)
        
        # Step 8: Publish debug info
        if self.debug_mode:
            self.publish_debug(idm_accel, control_term if not self.control_disabled else 0, 
                              self.equilibrium_speed - self.current_speed)
        
        # Step 9: Publish visualization markers
        self.publish_markers()
    
    def publish_control(self, acceleration):
        """
        Publish control command as TwistStamped message
        """
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        
        # Send desired acceleration
        cmd.twist.linear.z = acceleration  # m/s²
        
        # Send desired speed (for reference)
        cmd.twist.linear.x = self.desired_speed  # m/s
        
        # Add equilibrium speed for debugging
        cmd.twist.angular.y = self.equilibrium_speed
        
        self.control_pub.publish(cmd)
    
    def publish_debug(self, idm_accel, control_term, speed_error):
        """
        Publish debug information
        """
        now_sec = float(self.get_clock().now().nanoseconds) * 1e-9
        odom_age = now_sec - self.current_speed_stamp if self.current_speed_stamp is not None else float('nan')
        leader_age = now_sec - self.leader_speed_stamp if self.leader_speed_stamp is not None else float('nan')
        gap_age = now_sec - self.gap_stamp if self.gap_stamp is not None else float('nan')

        debug_msg = String()
        debug_msg.data = (
            f"t={self.get_clock().now().nanoseconds/1e9:.2f}, "
            f"v_ego={self.current_speed:.2f}, v_leader={self.leader_speed:.2f}, "
            f"U_eq={self.equilibrium_speed:.2f}, error={speed_error:.2f}, "
            f"gap={self.gap:.2f}, idm={idm_accel:.3f}, control={control_term:.3f}, "
            f"control_enabled={not self.control_disabled}, "
            f"accel={self.desired_acceleration:.3f}, "
            f"odom_age={odom_age:.2f}, lead_age={leader_age:.2f}, gap_age={gap_age:.2f}"
        )
        self.debug_pub.publish(debug_msg)
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
        
        status = "ACTIVE" if not self.control_disabled else "DISABLED (gap<15m)"
        text_marker.text = (
            f"EWMA Controller\n"
            f"U_eq: {self.equilibrium_speed:.1f}\n"
            f"a_des: {self.desired_acceleration:.2f}\n"
            f"Gap: {self.gap:.1f}m\n"
            f"Status: {status}"
        )
        
        marker_array.markers.append(text_marker)
        self.marker_pub.publish(marker_array)
    
    def status_callback(self):
        """
        Periodic status update (every 1 second)
        """
        self.get_logger().info(
            f"Status - Speed: {self.current_speed:.2f}, "
            f"Desired: {self.desired_speed:.2f}, "
            f"Leader: {self.leader_speed:.2f}, "
            f"Eq: {self.equilibrium_speed:.2f}, "
            f"Gap: {self.gap:.2f}, "
            f"Accel: {self.desired_acceleration:.3f}, "
            f"Control: {'OFF' if self.control_disabled else 'ON'}, "
            f"Safety violations: {self.safety_violation_count}"
        )
    
    def reset(self):
        """Reset controller state"""
        self.prev_equilibrium = None
        self.equilibrium_speed = 0.0
        self.desired_acceleration = 0.0
        self.control_disabled = False
        self.get_logger().info("Controller reset")


def main(args=None):
    rclpy.init(args=args)
    
    node = EWMAController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Controller stopped by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()