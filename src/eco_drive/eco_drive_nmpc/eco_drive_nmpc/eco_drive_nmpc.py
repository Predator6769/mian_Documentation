#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped


class EcoDriveNMPC(Node):

    def __init__(self):
        super().__init__('eco_drive_nmpc')

        # Parameters loaded from YAML
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('velocity', 5.0)          # target velocity
        self.declare_parameter('max_accel', 2.0)
        self.declare_parameter('max_decel', -3.0)

        self.publish_rate = self.get_parameter('publish_rate').value
        self.v_target = self.get_parameter('velocity').value
        self.max_accel = self.get_parameter('max_accel').value
        self.max_decel = self.get_parameter('max_decel').value

        self.dt = 1.0 / self.publish_rate

        # State
        self.current_velocity = 0.0
        self.acceleration = 0.0

        # Subscriber (ego velocity)
        self.create_subscription(
            Odometry,
            '/localization/kinematic_state',
            self.odom_callback,
            10
        )

        # Publisher
        self.cmd_pub = self.create_publisher(
            TwistStamped,
            '/acc_unit/eco_drive_velocity',
            10
        )

        # Timer
        self.timer = self.create_timer(self.dt, self.publish_command)

        self.get_logger().info(
            f'Eco Drive NMPC (feedback tracker) started\n'
            f'Target Velocity: {self.v_target:.2f} m/s\n'
            f'Publish Rate: {self.publish_rate:.2f} Hz'
        )

    def odom_callback(self, msg):
        self.current_velocity = msg.twist.twist.linear.x

    def compute_acceleration(self):
        # core equation: (target - current) / dt
        a = (self.v_target - self.current_velocity) / self.dt

        # clamp for safety
        a = max(self.max_decel, min(self.max_accel, a))

        return a

    def publish_command(self):

        self.acceleration = self.compute_acceleration()

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

        # publish current velocity (for FSM comparison/debug)
        msg.twist.linear.x = self.current_velocity

        # computed acceleration toward target
        msg.twist.linear.z = self.acceleration

        self.cmd_pub.publish(msg)

        self.get_logger().info(
            f'v={self.current_velocity:.2f} | '
            f'v_ref={self.v_target:.2f} | '
            f'a={self.acceleration:.2f}'
        )


def main(args=None):

    rclpy.init(args=args)
    node = EcoDriveNMPC()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()