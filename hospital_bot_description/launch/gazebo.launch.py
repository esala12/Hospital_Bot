import os
from os import pathsep
from ament_index_python.packages import get_package_share_directory, get_package_prefix

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
     
    hospital_bot_description = get_package_share_directory("hospital_bot_description")
    hospital_bot_description_prefix = get_package_prefix("hospital_bot_description")

    model_path = os.path.join(hospital_bot_description, "models")
    model_path += pathsep + os.path.join(hospital_bot_description_prefix, "share")

    env_variable = SetEnvironmentVariable("GAZEBO_MODEL_PATH", model_path)

    # Declare simulation time argument
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use simulation (Gazebo) clock if true"
    )

    model_arg = DeclareLaunchArgument(name="model", default_value=os.path.join(
                                        hospital_bot_description, "urdf", "hospital_bot.urdf.xacro"),
                                        description="Absolute path to robot urdf file")
     
    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]),
                                        value_type=str)
     
    world_name_arg = DeclareLaunchArgument(name="world_name", default_value="empty")

    world_path = PathJoinSubstitution([
        hospital_bot_description,
        "worlds",
        PythonExpression(expression=["'", LaunchConfiguration("world_name"), "'", " + '.world'"])
    ])

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description,
                     "use_sim_time": LaunchConfiguration("use_sim_time")}]
    )

    # Gazebo Server with use_sim_time
    start_gazebo_server = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory("gazebo_ros"), "launch", "gzserver.launch.py"
    )),
        launch_arguments={
            "world": world_path,
            "use_sim_time": LaunchConfiguration("use_sim_time")
        }.items())

    # Gazebo Client with use_sim_time
    start_gazebo_client = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory("gazebo_ros"), "launch", "gzclient.launch.py"
    )),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time")
        }.items())

    # Spawning robot in Gazebo
    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "hospital_bot", "-topic", "robot_description"],
        output="screen",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}]
    )

    # Including the second launch file (controller.launch.py)
    start_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("hospital_bot_controller"),
            "launch",
            "controller.launch.py"
        )),
        launch_arguments={
            "use_python": "true",
            "use_simple_controller": "true",
            "use_sim_time": LaunchConfiguration("use_sim_time")
        }.items()
    )

    return LaunchDescription([
        use_sim_time_arg,
        env_variable,
        model_arg,
        world_name_arg,
        robot_state_publisher_node,
        start_gazebo_server,
        start_gazebo_client,
        spawn_robot,
        start_controller_launch  # Including the second launch file
    ])
