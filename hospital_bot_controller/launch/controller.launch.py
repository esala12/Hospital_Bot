from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():

    # Declare launch arguments
    use_python_arg = DeclareLaunchArgument(
        "use_python",
        default_value="False"
    )

    wheel_radius_arg = DeclareLaunchArgument(
        "wheel_radius",
        default_value="0.033"
    )

    wheel_separation_arg = DeclareLaunchArgument(
        "wheel_separation",
        default_value="0.17"
    )

    use_simple_controller_arg = DeclareLaunchArgument(
        "use_simple_controller",
        default_value="True"
    )

    wheel_radius_error_arg = DeclareLaunchArgument(
        "wheel_radius_error",
        default_value="0.005"
    )

    wheel_separation_error_arg = DeclareLaunchArgument(
        "wheel_separation_error",
        default_value="0.02"
    )

    # Load launch configurations
    use_python = LaunchConfiguration("use_python")
    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_separation = LaunchConfiguration("wheel_separation")
    use_simple_controller = LaunchConfiguration("use_simple_controller")
    wheel_radius_error = LaunchConfiguration("wheel_radius_error")
    wheel_separation_error = LaunchConfiguration("wheel_separation_error")

    # Joint state broadcaster spawner
    joint_state_broadcaster_spawn = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager"
        ]
    )

    # Simple controller setup
    simple_controller = GroupAction(
        condition=IfCondition(use_simple_controller),
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "simple_velocity_controller",
                    "--controller-manager",
                    "/controller_manager"
                ]
            ),
            Node(
                package="hospital_bot_controller",
                executable="simple_controller.py",
                parameters=[{
                    "wheel_radius": wheel_radius,
                    "wheel_separation": wheel_separation
                }],
                condition=IfCondition(use_python)
            ),
            Node(
                package="hospital_bot_controller",
                executable="simple_controller",
                parameters=[{
                    "wheel_radius": wheel_radius,
                    "wheel_separation": wheel_separation
                }],
                condition=UnlessCondition(use_python)
            )
        ]
    )

    # Noisy controller setup
    noisy_controller_node = Node(
        package="hospital_bot_controller",
        executable="noisy_controller.py",
        parameters=[{
            "wheel_radius": wheel_radius,
            "wheel_radius_error": wheel_radius_error,
            "wheel_separation": wheel_separation,
            "wheel_separation_error": wheel_separation_error
        }]
    )

    return LaunchDescription([
        use_python_arg,
        wheel_radius_arg,
        wheel_separation_arg,
        use_simple_controller_arg,
        wheel_radius_error_arg,
        wheel_separation_error_arg,
        joint_state_broadcaster_spawn,
        simple_controller,
        noisy_controller_node  # Add the noisy controller node directly
    ])
