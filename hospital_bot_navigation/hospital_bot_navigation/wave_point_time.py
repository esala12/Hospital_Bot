#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import time
from datetime import datetime, timedelta

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        
        # Initialize the navigator
        self.navigator = BasicNavigator()

    def wait_until_time(self, target_hour, target_minute):
        # Calculate the target time today
        now = datetime.now()
        target_time = now.replace(hour=target_hour, minute=target_minute, second=0, microsecond=0)

        # If the target time has already passed today, assume it's for the next day
        if target_time <= now:
            target_time += timedelta(days=1)

        # Keep checking the current time until the specified hour and minute
        while rclpy.ok():
            now = datetime.now()
            remaining_time = target_time - now

            # Print remaining time in HH:MM:SS format
            self.get_logger().info(f"Time until start: {str(remaining_time).split('.')[0]}")

            # Break the loop when the target time is reached
            if remaining_time.total_seconds() <= 0:
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

    def go_to_waypoints(self, waypoints, wait_time=5):
        
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

            # Wait at the waypoint
            self.get_logger().info(f"Waiting for {wait_time} seconds at the waypoint.")
            time.sleep(wait_time)

def main(args=None):
    rclpy.init(args=args)
    waypoint_navigator = WaypointNavigator()

    # Wait until 6:35 AM
    waypoint_navigator.wait_until_time(18, 53)

    # Set the initial pose
    initial_pose = (0.0, 0.0, 0.002339403188309809, 0.9999972635926173)  # (x, y, orientation z, orientation w)
    waypoint_navigator.set_initial_pose(initial_pose)

    # Define the specific goal positions from the /goal_pose topic
    waypoints = [
        (5.179640769958496, -2.0815277099609375, 0.02013735309341808, 0.9997972229459287),  # Waypoint 1
        (0.5983695983886719, -4.00203800201416, -0.9999972238405127, 0.0023563342860463436),  # Waypoint 2
        (-5.9140729904174805, -2.9068262577056885, -0.9994980968003662, 0.03167892820859885),   # Waypoint 3
        (0.0, 0.0, 0.0, 1.0)
    ]

    # Navigate through the waypoints and wait for 10 seconds at each waypoint
    waypoint_navigator.go_to_waypoints(waypoints, wait_time=10)

    # Do not shut down anything after completing the waypoints

    # Keep the node active to maintain tf and other services
    rclpy.spin(waypoint_navigator)

    # Shutdown when manually interrupted (Ctrl+C)
    waypoint_navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
