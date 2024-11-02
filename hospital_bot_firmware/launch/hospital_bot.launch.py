from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hospital_bot_firmware',
            executable='odometry_publish.py',
            name='odometry_publisher',
            output='screen'
        ),
        Node(
            package='hospital_bot_firmware',
            executable='tf_publisher.py',
            name='tf_publisher',
            output='screen'
        )
    ])
