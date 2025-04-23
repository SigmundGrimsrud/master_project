cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
project(vrx_nav2)

# Find ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav2_bringup REQUIRED)
find_package(robot_localization REQUIRED)
find_package(vrx_gazebo REQUIRED)

# Install Python scripts
install(PROGRAMS
  scripts/odometry_node.py
  scripts/set_datum.py
  scripts/dual_usv_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install configuration files
install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install Python package (if you have Python modules)
ament_python_install_package(
  vrx_nav2
  PACKAGE_DIR src/vrx_nav2
)

# Declare ament package
ament_package()
