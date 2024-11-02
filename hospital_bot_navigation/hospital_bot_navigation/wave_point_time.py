#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import time
from datetime import datetime

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        
        # Initialize the navigator
        self.navigator = BasicNavigator()

    def wait_until_time(self, target_hour, target_minute):
        # Keep checking the current time until the specified hour and minute
        while rclpy.ok():
            now = datetime.now()
            if now.hour == target_hour and now.minute == target_minute:
                self.get_logger().info(f"Starting navigation at {target_hour:02d}:{target_minute:02d}")
                break
            time.sleep(1)  # Check every second

    def set_initial_pose(self, initial_pose):
        # Set the initial pose of the robot
        initial_pose_msg = PoseStamped()
        initial_pose_msg.header.frame_id = 'map'
        initial_pose_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Set the position and orientation for the initial pose
        initial_pose_msg.pose.position.x = initial_pose[0]
        initial_pose_msg.pose.position.y = initial_pose[1]
        initial_pose_msg.pose.position.z = 0.0  # Assuming a 2D navigation plane
        
        initial_pose_msg.pose.orientation.z = initial_pose[2]
        initial_pose_msg.pose.orientation.w = initial_pose[3]
        
        self.navigator.setInitialPose(initial_pose_msg)
        self.get_logger().info("Initial pose set.")

    def go_to_waypoints(self, waypoints):
        # Wait for the Navigation server to be ready
        self.navigator.waitUntilNav2Active()

        # Go through each waypoint
        for waypoint in waypoints:
            self.get_logger().info(f"Navigating to waypoint: {waypoint}")
            
            # Create a PoseStamped goal for each waypoint
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'  # Reference frame for navigation
            goal_pose.header.stamp = self.get_clock().now().to_msg()

            # Set the goal position
            goal_pose.pose.position.x = waypoint[0]
            goal_pose.pose.position.y = waypoint[1]
            goal_pose.pose.orientation.z = waypoint[2]
            goal_pose.pose.orientation.w = waypoint[3]

            # Navigate to the goal
            self.navigator.goToPose(goal_pose)

            # Monitor the navigation status
            while not self.navigator.isTaskComplete():
                feedback = self.navigator.getFeedback()
                if feedback:
                    self.get_logger().info(f"Distance to goal: {feedback.distance_remaining:.2f}")

            # Check if navigation succeeded
            result = self.navigator.getResult()
            if result:
                self.get_logger().info("Successfully reached the waypoint!")
            else:
                self.get_logger().info("Failed to reach the waypoint!")
                break

            # Sleep between waypoints (optional)
            time.sleep(1)

        # The navigator lifecycle will remain active; no shutdown

def main(args=None):
    rclpy.init(args=args)
    waypoint_navigator = WaypointNavigator()

    # Wait until 3:00 PM
    waypoint_navigator.wait_until_time(15, 0)

    # Set the initial pose
    initial_pose = (0.542567253112793, -0.028166532516479492, 0.0030893283324285804, 0.9999952280138413)  # (x, y, orientation z, orientation w)
    waypoint_navigator.set_initial_pose(initial_pose)

    # Define the specific goal positions from the /goal_pose topic
    waypoints = [
        (3.412578582763672, -0.010426491498947144, 0.014212131535267307, 0.9998990025583705),  # Waypoint 1
        (3.5233993530273438, -2.744586706161499, -0.6972588084887705, 0.7168194709861193),     # Waypoint 2
        (6.6157402992248535, 0.18376551568508148, 0.003028785785491401, 0.9999954132178135),    # Waypoint 3
        (0.542567253112793, -0.028166532516479492, 0.0030893283324285804, 0.9999952280138413)   # Waypoint 4
    ]

    # Navigate through the waypoints
    waypoint_navigator.go_to_waypoints(waypoints)

    # Do not shut down anything after completing the waypoints

    # Keep the node active to maintain tf and other services
    rclpy.spin(waypoint_navigator)

    # Shutdown when manually interrupted (Ctrl+C)
    waypoint_navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
