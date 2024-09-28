#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelRelay(Node):

    def __init__(self):
        super().__init__('cmd_vel_relay')
        
        # Subscribe to /cmd_vel
        self.cmd_vel_subscriber = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Publish to /hospital_bot_controller/cmd_vel
        self.hospital_cmd_vel_publisher = self.create_publisher(
            Twist,
            '/hospital_bot_controller/cmd_vel',
            10
        )

    def cmd_vel_callback(self, msg):
        # Simply relay the received message to the other topic
        self.hospital_cmd_vel_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
