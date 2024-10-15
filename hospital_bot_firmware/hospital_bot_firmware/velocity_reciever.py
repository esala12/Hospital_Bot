#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class MotorFeedbackReceiver(Node):
    def __init__(self):
        super().__init__("motor_feedback_receiver")
        self.pub_ = self.create_publisher(Twist, "motor_feedback", 10)

        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 115200)
        self.port_ = self.get_parameter("port").value
        self.baudrate_ = self.get_parameter("baudrate").value

        try:
            self.arduino_ = serial.Serial(self.port_, self.baudrate_, timeout=1)
            self.get_logger().info(f"Connected to Arduino on {self.port_}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect: {e}")
            rclpy.shutdown()

        self.timer_ = self.create_timer(0.1, self.timer_callback)  # 10 Hz

    def timer_callback(self):
        if self.arduino_.is_open:
            try:
                data = self.arduino_.readline().decode("utf-8").strip()
                if data:
                    linear_x, angular_z = map(float, data.split(","))
                    msg = Twist()
                    msg.linear.x = linear_x
                    msg.angular.z = angular_z
                    self.pub_.publish(msg)
                    self.get_logger().info(f"Published: {msg}")
            except ValueError as e:
                self.get_logger().warn(f"Invalid data received: {data} - {e}")

def main():
    rclpy.init()
    motor_feedback_receiver = MotorFeedbackReceiver()
    rclpy.spin(motor_feedback_receiver)
    motor_feedback_receiver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
