cmake_minimum_required(VERSION 3.8)
project(hospital_bot_navigation)

# Set the standard to C++14 or C++17 for ROS 2 compatibility
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)  # Add this if you're using Python (rclpy)
find_package(rclcpp REQUIRED)  # ROS 2 C++ library
find_package(nav2_bringup REQUIRED)  # Navigation2 bringup
find_package(nav2_common REQUIRED)  # Common utilities in Nav2
find_package(nav2_core REQUIRED)  # Core interfaces for Navigation2
find_package(nav2_util REQUIRED)  # Utility libraries for Nav2
find_package(nav2_msgs REQUIRED)  # Messages for Nav2
find_package(tf2_ros REQUIRED)  # For using transforms in ROS 2
find_package(sensor_msgs REQUIRED)  # For handling sensor messages like LaserScan
find_package(std_msgs REQUIRED)  # Standard message types
find_package(geometry_msgs REQUIRED)  # To work with robot geometry messages
find_package(nav_msgs REQUIRED)  # To work with navigation-specific messages like paths and maps

# Install Python packages
ament_python_install_package(${PROJECT_NAME})

# Install Python scripts
install(PROGRAMS
  ${PROJECT_NAME}/cmd_vel_relay.py
  ${PROJECT_NAME}/wave_point_follower.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories (launch files, params, etc.)
install(DIRECTORY launch param rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # Enable linting checks like cppcheck, uncrustify, etc.
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
