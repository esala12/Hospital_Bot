#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time
import threading
import sys

class MotorControlTransmitter(Node):
    def __init__(self):
        super().__init__('motor_control_transmitter')

        # Subscribe to the /cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10
        )

        # Publisher for /robot_velocity_feedback
        self.feedback_publisher = self.create_publisher(Twist, '/robot_velocity_feedback', 10)

        # Initialize the serial connection with retries
        self.serial_port = self.connect_serial()

        if not self.wait_for_arduino_ready():
            self.get_logger().error("Arduino did not send 'READY' signal. Shutting down.")
            self.cleanup()
            sys.exit(1)

        self.get_logger().info('Arduino is ready for communication')

        # Start a thread to read feedback from Arduino
        self.feedback_thread = threading.Thread(target=self.read_feedback, daemon=True)
        self.feedback_thread.start()

    def connect_serial(self):
        """Attempt to connect to the Arduino serial port with retries."""
        static_serial_port = '/dev/ttyACM0'  # Set your static serial port here
        max_retries = 5
        for attempt in range(max_retries):
            try:
                return serial.Serial(static_serial_port, 115200, timeout=1)
            except serial.SerialException as e:
                self.get_logger().warn(f"Failed to open {static_serial_port}: {e}")
                time.sleep(2)  # Wait before retrying

        self.get_logger().error("Unable to connect to Arduino after multiple attempts.")
        sys.exit(1)

    def wait_for_arduino_ready(self, max_retries=10, delay=0.5):
        """Wait for Arduino to send a 'READY' signal with retries."""
        for _ in range(max_retries):
            if self.serial_port.in_waiting > 0:
                try:
                    line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
                    if line == "READY":
                        return True
                except UnicodeDecodeError as e:
                    self.get_logger().warn(f"Decode error: {e}")
            time.sleep(delay)
        return False

    def listener_callback(self, msg):
        """Send motor commands based on /cmd_vel topic."""
        vb_x = msg.linear.x  # Linear velocity in m/s
        wb_z = msg.angular.z  # Angular velocity in rad/s

        # Calculate wheel velocities
        v_left_linear = vb_x - (wb_z * 0.35 / 2)
        v_right_linear = vb_x + (wb_z * 0.35 / 2)

        vt1 = self.rad_per_sec_to_rpm(v_left_linear / 0.04)
        vt2 = self.rad_per_sec_to_rpm(v_right_linear / 0.04)

        # Send RPM values to Arduino
        command = f'{vt1},{vt2}\n'
        self.serial_port.write(command.encode())

    def read_feedback(self):
        """Continuously read and publish feedback from Arduino."""
        buffer = ""
        while True:
            if self.serial_port.in_waiting > 0:
                try:
                    data = self.serial_port.read(self.serial_port.in_waiting).decode('utf-8', errors='ignore')
                    buffer += data

                    # Process complete lines from the buffer
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        self.process_feedback(line.strip())
                except UnicodeDecodeError as e:
                    self.get_logger().warn(f"Decode error: {e}")

    def process_feedback(self, line):
        """Process and publish valid feedback data."""
        try:
            if line.startswith('<') and line.endswith('>'):
                line = line[1:-1]  # Remove markers

            if ',' not in line:
                raise ValueError("Invalid format: missing comma")

            linear_x, angular_z = map(float, line.split(','))
            msg = Twist()
            msg.linear.x = linear_x
            msg.angular.z = angular_z
            self.feedback_publisher.publish(msg)

        except ValueError as e:
            self.get_logger().warn(f"Invalid feedback: {e} | Raw Data: '{line}'")

    def rad_per_sec_to_rpm(self, rad_per_sec):
        """Convert radians per second to RPM."""
        return (rad_per_sec * 60) / (2 * 3.14159)

    def cleanup(self):
        """Close the serial port and shutdown the node."""
        if hasattr(self, 'serial_port') and self.serial_port.is_open:
            self.serial_port.close()
        self.destroy_node()

def main(args=None):
    try:
        rclpy.init(args=args)
        node = MotorControlTransmitter()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down due to KeyboardInterrupt...")
    finally:
        if 'node' in locals():
            node.cleanup()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
