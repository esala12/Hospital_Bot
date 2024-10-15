#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
from tf_transformations import quaternion_from_euler

class TFPublisherFromOdometry(Node):
    def __init__(self):
        super().__init__('tf_from_odometry')

        # Create a subscription to the /odometry/filtered topic
        self.subscription = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odometry_callback,
            10  # QoS profile
        )

        # Create a TF broadcaster to broadcast the transformation
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

    def odometry_callback(self, msg: Odometry):
        # Extract the position and orientation from the odometry message
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z

        quat = msg.pose.pose.orientation  # Quaternion (x, y, z, w)

        # Create a TransformStamped message
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = msg.header.frame_id  # Typically 'odom'
        t.child_frame_id = msg.child_frame_id  # Typically 'base_footprint'

        # Set the translation (from odometry position)
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z

        # Set the rotation (from odometry orientation)
        t.transform.rotation.x = quat.x
        t.transform.rotation.y = quat.y
        t.transform.rotation.z = quat.z
        t.transform.rotation.w = quat.w

        # Broadcast the transformation
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)

    # Create the node
    node = TFPublisherFromOdometry()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
