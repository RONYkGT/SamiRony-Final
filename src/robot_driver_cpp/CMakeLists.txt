cmake_minimum_required(VERSION 3.5)
project(robot_driver_cpp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(bot_behavior_interfaces REQUIRED)

include_directories(include)

# Add executable for all action servers
add_executable(turn_action_servers
  src/main.cpp
  src/robot_driver_cpp/turn_left_action_server.cpp
  src/robot_driver_cpp/turn_right_action_server.cpp
  src/robot_driver_cpp/move_forward_action_server.cpp
)

ament_target_dependencies(turn_action_servers
  rclcpp
  rclcpp_action
  geometry_msgs
  bot_behavior_interfaces
)

# Install the executable
install(TARGETS
  turn_action_servers
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)
ament_package()
