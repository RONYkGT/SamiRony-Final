cmake_minimum_required(VERSION 3.8)
project(bot_behavior_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  "action/TurnRight.action"
  "action/TurnLeft.action"
)

ament_export_dependencies(rosidl_default_runtime)

# Install the header files
install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
