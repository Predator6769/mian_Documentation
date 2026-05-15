#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from tier4_debug_msgs.msg import Float64Stamped


class EcoDriveFSM(Node):

    def __init__(self):
        super().__init__('eco_drive_fsm')

        # Subscribers
        self.idm_sub = self.create_subscription(
            TwistStamped,
            '/acc/target_vel',
            self.idm_callback,
            10
        )

        self.eco_sub = self.create_subscription(
            TwistStamped,
            '/acc_unit/eco_drive_velocity',
            self.eco_callback,
            10
        )

        self.gap_sub = self.create_subscription(
            Float64Stamped,
            '/acc/perception/gap',
            self.gap_callback,
            10
        )

        # Publisher
        self.final_pub = self.create_publisher(
            TwistStamped,
            '/acc_unit/final_velocity',
            10
        )

        # Store latest commands
        self.idm_msg = None
        self.eco_msg = None
        self.gap_msg = None

        # Timer
        self.timer = self.create_timer(0.1, self.publish_final_command)

        self.get_logger().info("Eco Drive FSM Started")

    def idm_callback(self, msg):
        self.idm_msg = msg

    def eco_callback(self, msg):
        self.eco_msg = msg

    def gap_callback(self, msg):
        self.gap_msg = msg

    def publish_final_command(self):

        # Wait until both messages are available
        if self.idm_msg is None or self.eco_msg is None:
            return

        # Extract IDM values
        idm_velocity = self.idm_msg.twist.linear.x
        idm_acceleration = self.idm_msg.twist.linear.z

        # Extract EcoDrive values
        eco_velocity = self.eco_msg.twist.linear.x
        eco_acceleration = self.eco_msg.twist.linear.z

        # FSM Logic
        #
        # If eco velocity exceeds IDM safe velocity,
        # choose IDM.
        #
        # Else choose EcoDrive.

        if self.gap_msg is not None and self.gap_msg.data < 5.0:

            selected_mode = "EMERGENCY_BRAKE"
            final_velocity = 0.0
            final_acceleration = -3.0

        elif eco_velocity > idm_velocity:

            selected_mode = "IDM"

            final_velocity = idm_velocity
            final_acceleration = idm_acceleration

        else:

            selected_mode = "ECO_DRIVE"

            final_velocity = eco_velocity
            final_acceleration = eco_acceleration

        # Publish selected command
        output_msg = TwistStamped()

        output_msg.header.stamp = self.get_clock().now().to_msg()
        output_msg.header.frame_id = "base_link"

        output_msg.twist.linear.x = final_velocity
        output_msg.twist.linear.z = final_acceleration

        self.final_pub.publish(output_msg)

        self.get_logger().info(
            f'Mode: {selected_mode} | '
            f'Final Velocity: {final_velocity:.2f} m/s | '
            f'Final Acceleration: {final_acceleration:.2f} m/s²',
            # throttle_duration_sec=1.0
        )


def main(args=None):

    rclpy.init(args=args)

    node = EcoDriveFSM()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
