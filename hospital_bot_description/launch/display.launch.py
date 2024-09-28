from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration


def generate_launch_description():
    # Update the model argument to use the absolute path to the URDF file
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value="/home/esala/hospital_bot/src/hospital_bot_description/urdf/hospital_bot.urdf.xacro",
        description="Absolute path to robot URDF file"
    )

    # Set up the robot description parameter with the updated model argument
    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]), value_type=str)

    # Node for the robot state publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    # Node for the joint state publisher GUI
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )

    # Node for RViz with the configuration file path
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(get_package_share_directory("hospital_bot_description"), "rviz", "display.rviz")]
    )

    # Return the launch description with all nodes and arguments
    return LaunchDescription([
        model_arg,
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz_node
    ])
