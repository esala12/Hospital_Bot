#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

class LaserScanFilter(Node):
    def __init__(self):
        super().__init__('laser_scan_filter')
        
        # Subscribe to the /scan topic
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Publisher for the filtered scan data
        self.filtered_scan_publisher = self.create_publisher(
            LaserScan,
            '/filtered_scan',
            10
        )

        # Define the angle ranges to filter out (in radians)
        self.filter_ranges = [
            (np.deg2rad(40), np.deg2rad(50)),    # 40 to 50 degrees
            (np.deg2rad(125), np.deg2rad(135))   # 125 to 135 degrees
        ]

    def scan_callback(self, msg):
        # Filter out specific angle ranges
        filtered_ranges = list(msg.ranges)  # Start with the original ranges

        angle_increment = msg.angle_increment
        current_angle = msg.angle_min

        for i in range(len(msg.ranges)):
            distance = msg.ranges[i]
            # Check if the current angle falls within any of the specified ranges
            if any(start <= current_angle <= end for start, end in self.filter_ranges):
                # Set the distance to infinity for angles we want to filter out
                filtered_ranges[i] = float('inf')
            current_angle += angle_increment

        # Create a new LaserScan message for the filtered data
        filtered_scan = LaserScan()
        filtered_scan.header = msg.header
        filtered_scan.angle_min = msg.angle_min
        filtered_scan.angle_max = msg.angle_max
        filtered_scan.angle_increment = msg.angle_increment
        filtered_scan.time_increment = msg.time_increment
        filtered_scan.scan_time = msg.scan_time
        filtered_scan.range_min = msg.range_min
        filtered_scan.range_max = msg.range_max
        filtered_scan.ranges = filtered_ranges

        # Publish the filtered scan data
        self.filtered_scan_publisher.publish(filtered_scan)
        self.get_logger().info("Published filtered scan data")

def main(args=None):
    rclpy.init(args=args)
    laser_scan_filter = LaserScanFilter()
    rclpy.spin(laser_scan_filter)
    laser_scan_filter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
