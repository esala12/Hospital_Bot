#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler
import math
import time

class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('odometry_publisher')

        # Initialize the subscriber to /robot_velocity_feedback
        self.subscription = self.create_subscription(
            Twist,
            '/robot_velocity_feedback',
            self.velocity_callback,
            10
        )

        # Initialize the publisher for odometry messages
        self.publisher = self.create_publisher(Odometry, '/odometry/filtered', 10)

        # Robot's state variables
        self.x = 0.0  # Position in x direction
        self.y = 0.0  # Position in y direction (optional, not used here)
        self.theta = 0.0  # Heading angle (yaw)

        # Velocities
        self.linear_x = 0.0
        self.angular_z = 0.0

        # Last update time for integration
        self.last_time = self.get_clock().now()

    def velocity_callback(self, msg: Twist):
        # Extract linear and angular velocities
        self.linear_x = msg.linear.x
        self.angular_z = msg.angular.z

        # Calculate time elapsed since last update
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9  # Convert ns to seconds
        self.last_time = current_time

        # Integrate velocities to update position and heading
        self.x += self.linear_x * math.cos(self.theta) * dt
        self.y += self.linear_x * math.sin(self.theta) * dt
        self.theta += self.angular_z * dt

        # Normalize theta to the range [-pi, pi]
        self.theta = (self.theta + math.pi) % (2 * math.pi) - math.pi

        # Publish the odometry message
        self.publish_odometry(current_time)

    def publish_odometry(self, current_time):
        odom_msg = Odometry()

        # Set the header information
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'

        # Set the position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        # Set the orientation using quaternion
        quat = quaternion_from_euler(0, 0, self.theta)
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]

        # Set the linear and angular velocities
        odom_msg.twist.twist.linear.x = self.linear_x
        odom_msg.twist.twist.angular.z = self.angular_z

        # Publish the odometry message
        self.publisher.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    node = OdometryPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
