#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Header
from rclpy.clock import Clock

class TeleopToStamped(Node):
    def __init__(self):
        super().__init__('teleop_to_stamped')

        # Subscribe to the teleop Twist message
        self.twist_sub = self.create_subscription(
            Twist,
            '/cmd_vel',  # Default topic for teleop_twist_keyboard
            self.twist_callback,
            10)

        # Publisher for the TwistStamped message
        self.stamped_pub = self.create_publisher(
            TwistStamped,
            '/hospital_bot_controller/cmd_vel',  # Your desired topic
            10)

        self.get_logger().info('Started teleop_to_stamped node')

    def twist_callback(self, twist_msg):
        # Create a TwistStamped message
        twist_stamped_msg = TwistStamped()

        # Fill in the current time for the header
        twist_stamped_msg.header = Header()
        twist_stamped_msg.header.stamp = Clock().now().to_msg()  # Set the timestamp
        twist_stamped_msg.header.frame_id = 'base_link'  # Set frame_id (can be changed if needed)

        # Copy the linear and angular velocity from the Twist message
        twist_stamped_msg.twist.linear = twist_msg.linear
        twist_stamped_msg.twist.angular = twist_msg.angular

        # Publish the TwistStamped message
        self.stamped_pub.publish(twist_stamped_msg)

        self.get_logger().info(f'Publishing: {twist_stamped_msg}')

def main(args=None):
    rclpy.init(args=args)
    teleop_to_stamped = TeleopToStamped()
    rclpy.spin(teleop_to_stamped)

    teleop_to_stamped.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
