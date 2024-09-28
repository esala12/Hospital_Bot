#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import numpy as np
from rclpy.time import Time
from rclpy.constants import S_TO_NS
import math
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster

class SimpleController(Node):
    def __init__(self):
        super().__init__("simple_controller")

        # Declare parameters for wheel radius and separation
        self.declare_parameter("wheel_radius", 0.033)
        self.declare_parameter("wheel_separation", 0.17)

        self.wheel_radius_ = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_separation_ = self.get_parameter("wheel_separation").get_parameter_value().double_value  

        self.get_logger().info("Using wheel_radius %f" % self.wheel_radius_)
        self.get_logger().info("Using wheel_separation %f" % self.wheel_separation_) 

        # Initialize previous positions and time
        self.left_wheel_prev_pos_ = 0.0
        self.right_wheel_prev_pos_ = 0.0
        self.prev_time_ = self.get_clock().now()

        # Initialize odometry state
        self.x_ = 0.0
        self.y_ = 0.0
        self.theta_ = 0.0

        # Publishers and subscribers
        self.wheel_cmd_pub_ = self.create_publisher(Float64MultiArray, "simple_velocity_controller/commands", 10)
        self.vel_sub_ = self.create_subscription(Twist, "hospital_bot_controller/cmd_vel", self.velCallback, 10)
        self.joint_sub_ = self.create_subscription(JointState, "joint_states", self.jointCallback, 10)
        self.odom_pub_ = self.create_publisher(Odometry, "hospital_bot_controller/odom", 10)

        # Speed conversion matrix for wheel control
        self.speed_conversion_ = np.array([[self.wheel_radius_/2, self.wheel_radius_/2],
                                           [self.wheel_radius_/self.wheel_separation_, -self.wheel_radius_/self.wheel_separation_]])  
        
        # Prepare Odometry message
        self.odom_msgs_ = Odometry()
        self.odom_msgs_.header.frame_id = "odom"
        self.odom_msgs_.child_frame_id = "base_footprint"
        self.odom_msgs_.pose.pose.orientation.x = 0.0
        self.odom_msgs_.pose.pose.orientation.y = 0.0
        self.odom_msgs_.pose.pose.orientation.z = 0.0
        self.odom_msgs_.pose.pose.orientation.w = 1.0

        # TF broadcaster for transforms
        self.br_ = TransformBroadcaster(self)

        # Base_footprint to odom transform
        self.transform_stamped_ = TransformStamped()
        self.transform_stamped_.header.frame_id = "odom"
        self.transform_stamped_.child_frame_id = "base_footprint"

        # Laser_link to base_footprint transform
        self.base_link_transform = TransformStamped()
        self.base_link_transform.header.frame_id = "base_footprint"
        self.base_link_transform.child_frame_id = "base_link"
        self.base_link_transform.transform.translation.x = 0.0
        self.base_link_transform.transform.translation.y = 0.0
        self.base_link_transform.transform.translation.z = 0.033
        self.base_link_transform.transform.rotation.x = 0.0
        self.base_link_transform.transform.rotation.y = 0.0
        self.base_link_transform.transform.rotation.z = 0.0
        self.base_link_transform.transform.rotation.w = 1.0

    def velCallback(self, msg):
        # Compute wheel velocities from Twist message
        robot_speed = np.array([[msg.linear.x], [msg.angular.z]])
        wheel_speed = np.matmul(np.linalg.inv(self.speed_conversion_), robot_speed)

        # Publish wheel commands
        wheel_speed_msg = Float64MultiArray()
        wheel_speed_msg.data = [wheel_speed[1, 0], wheel_speed[0, 0]]
        self.wheel_cmd_pub_.publish(wheel_speed_msg)

    def jointCallback(self, msg):
        # Calculate time difference and wheel displacements
        dp_left = msg.position[1] - self.left_wheel_prev_pos_
        dp_right = msg.position[0] - self.right_wheel_prev_pos_
        dt = Time.from_msg(msg.header.stamp) - self.prev_time_

        # Update previous positions and time
        self.left_wheel_prev_pos_ = msg.position[1]
        self.right_wheel_prev_pos_ = msg.position[0]
        self.prev_time_ = Time.from_msg(msg.header.stamp)

        # Compute wheel angular velocities
        fi_left = dp_left / (dt.nanoseconds / S_TO_NS)
        fi_right = dp_right / (dt.nanoseconds / S_TO_NS)

        # Calculate robot's linear and angular velocities
        linear = (self.wheel_radius_ * fi_right + self.wheel_radius_ * fi_left) / 2
        angular = (self.wheel_radius_ * fi_right - self.wheel_radius_ * fi_left) / self.wheel_separation_

        # Update robot pose (x, y, theta)
        d_s = (self.wheel_radius_ * dp_right + self.wheel_radius_ * dp_left) / 2
        d_theta = (self.wheel_radius_ * dp_right - self.wheel_radius_ * dp_left) / self.wheel_separation_
        self.theta_ += d_theta
        self.x_ += d_s * math.cos(self.theta_)
        self.y_ += d_s * math.sin(self.theta_)

        # Convert to quaternion for orientation
        q = quaternion_from_euler(0, 0, self.theta_)

        # Use joint message timestamp to ensure synchronized transforms
        current_time = msg.header.stamp

        # Populate Odometry message
        self.odom_msgs_.pose.pose.orientation.x = q[0]
        self.odom_msgs_.pose.pose.orientation.y = q[1]
        self.odom_msgs_.pose.pose.orientation.z = q[2]
        self.odom_msgs_.pose.pose.orientation.w = q[3]
        self.odom_msgs_.header.stamp = current_time
        self.odom_msgs_.pose.pose.position.x = self.x_
        self.odom_msgs_.pose.pose.position.y = self.y_
        self.odom_msgs_.twist.twist.linear.x = linear
        self.odom_msgs_.twist.twist.angular.z = angular

        # Publish odometry message
        self.odom_pub_.publish(self.odom_msgs_)

        # Broadcast odom->base_footprint transform
        self.transform_stamped_.transform.translation.x = self.x_
        self.transform_stamped_.transform.translation.y = self.y_
        self.transform_stamped_.transform.rotation.x = q[0]
        self.transform_stamped_.transform.rotation.y = q[1]
        self.transform_stamped_.transform.rotation.z = q[2]
        self.transform_stamped_.transform.rotation.w = q[3]
        self.transform_stamped_.header.stamp = current_time
        self.br_.sendTransform(self.transform_stamped_)

        # Broadcast base_footprint->laser_link transform
        self.base_link_transform.header.stamp = current_time
        self.br_.sendTransform(self.base_link_transform)


def main():
    rclpy.init()
    simple_controller = SimpleController()
    rclpy.spin(simple_controller)
    simple_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()