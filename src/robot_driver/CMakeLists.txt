cmake_minimum_required(VERSION 3.8)
project(robot_driver)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)  # Required for Python-based nodes

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
