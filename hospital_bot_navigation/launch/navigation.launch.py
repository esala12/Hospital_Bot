import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare Launch Arguments
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true"
    )

    map_name_arg = DeclareLaunchArgument(
        "map_name",
        default_value="small_house"
    )

    amcl_config_arg = DeclareLaunchArgument(
        "amcl_config",
        default_value=os.path.join(
            get_package_share_directory("hospital_bot_navigation"),
            "param",
            "nav2_params.yaml"  # Adjusted to reflect the correct file in the param folder
        )
    )

    # Launch Configurations
    map_name = LaunchConfiguration("map_name")
    use_sim_time = LaunchConfiguration("use_sim_time")
    amcl_config = LaunchConfiguration("amcl_config")

    # Paths and Directories
    map_dir = PathJoinSubstitution([
        get_package_share_directory('hospital_bot_mapping'),
        'maps',
        map_name,
        'map.yaml'
    ])

    param_file_name = 'nav2_params.yaml'  # This is now referencing the correct file
    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            get_package_share_directory('hospital_bot_navigation'),
            'param',
            param_file_name))

    nav2_launch_file_dir = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch')

    # Add all the necessary lifecycle nodes for navigation
    lifecycle_nodes = [
        "map_server",           # Provides the map to the navigation system
        "amcl",                 # Adaptive Monte Carlo Localization
        "planner_server",       # Global path planner
        "controller_server",    # Local path controller
        "recoveries_server",    # Manages recovery behaviors
        "bt_navigator",         # Behavior Tree navigator
        "waypoint_follower"     # Optional, for waypoint following
    ]

    # Nodes
    nav2_map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        output="screen",
        parameters=[
            {"yaml_filename": map_dir},
            {"use_sim_time": use_sim_time}
        ]
    )

    nav2_amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[
            amcl_config,
            {"use_sim_time": use_sim_time}
        ]
    )

    nav2_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[
            {"node_names": lifecycle_nodes},
            {"use_sim_time": use_sim_time},
            {"autostart": True}
        ]
    )

    # RViz configuration
    rviz_config_file = os.path.join(
        get_package_share_directory('hospital_bot_navigation'),
        'rviz',
        'navigation.rviz'  # Path relative to the package
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    # Add cmd_vel_relay node
    cmd_vel_relay_node = Node(
        package='hospital_bot_navigation',  # Replace with your actual package name
        executable='cmd_vel_relay.py',  # The name of the Python script without the .py extension
        output='screen'
    )

    # Include Nav2 bringup
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
        launch_arguments={
            'map': map_dir,
            'use_sim_time': use_sim_time,
            'params_file': param_dir}.items(),
    )

    # Return Launch Description
    return LaunchDescription([
        use_sim_time_arg,
        map_name_arg,
        amcl_config_arg,
        nav2_map_server,
        nav2_amcl,
        nav2_lifecycle_manager,
        rviz_node,
        nav2_bringup,
        cmd_vel_relay_node  # Add the cmd_vel_relay node here
    ])
