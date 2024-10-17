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

        # Create a TF broadcaster to broadcast the transformations
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

    def odometry_callback(self, msg: Odometry):
        # Extract the position and orientation from the odometry message
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z

        quat = msg.pose.pose.orientation  # Quaternion (x, y, z, w)

        # Create a TransformStamped message for the base footprint
        base_t = TransformStamped()
        base_t.header.stamp = self.get_clock().now().to_msg()
        base_t.header.frame_id = msg.header.frame_id  # Typically 'odom'
        base_t.child_frame_id = 'base_footprint'

        # Set the translation (from odometry position)
        base_t.transform.translation.x = x
        base_t.transform.translation.y = y
        base_t.transform.translation.z = z

        # Set the rotation (from odometry orientation)
        base_t.transform.rotation.x = quat.x
        base_t.transform.rotation.y = quat.y
        base_t.transform.rotation.z = quat.z
        base_t.transform.rotation.w = quat.w

        # Broadcast the transformation for base_footprint
        self.tf_broadcaster.sendTransform(base_t)

        # Create a TransformStamped message for the laser link
        laser_t = TransformStamped()
        laser_t.header.stamp = self.get_clock().now().to_msg()
        laser_t.header.frame_id = 'base_footprint'  # Laser is relative to base_footprint
        laser_t.child_frame_id = 'laser'  # Laser frame ID

        # Set the translation: 15 cm above the base footprint in the z direction
        laser_t.transform.translation.x = 0.09
        laser_t.transform.translation.y = 0.0
        laser_t.transform.translation.z = 0.15  # 15 cm above

        # Laser is aligned with base_footprint, so no rotation
        laser_t.transform.rotation.x = 0.0
        laser_t.transform.rotation.y = 0.0
        laser_t.transform.rotation.z = 0.0
        laser_t.transform.rotation.w = 1.0

        # Broadcast the transformation for laser_link
        self.tf_broadcaster.sendTransform(laser_t)

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
