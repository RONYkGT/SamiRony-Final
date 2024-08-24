cmake_minimum_required(VERSION 3.5)
project(behavior_tree)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(bot_behavior_interfaces REQUIRED) 

set(dependencies
  behaviortree_cpp_v3
  ament_index_cpp
  rclcpp
  rclcpp_action
  bot_behavior_interfaces 
)

include_directories(include)
include_directories(${bot_behavior_interfaces_INCLUDE_DIRS})  # Include the directory for constants

# Add libraries for your nodes
add_library(${PROJECT_NAME} SHARED
  src/behavior_tree/FindCan.cpp
  src/behavior_tree/DropCan.cpp
  src/behavior_tree/FindQR.cpp
  src/behavior_tree/MoveForward.cpp
  src/behavior_tree/PickCan.cpp
  src/behavior_tree/CanClose.cpp
  src/behavior_tree/CanInView.cpp
  src/behavior_tree/CanPicked.cpp
  src/behavior_tree/QRinView.cpp
  src/behavior_tree/OnBlackStrip.cpp
)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})

# Add the main executable
add_executable(trashbot_main
  src/trashbot_main.cpp
)
target_link_libraries(trashbot_main ${PROJECT_NAME})

# Install include directory
install(DIRECTORY include/
  DESTINATION include/
)

# Install XML files for behavior trees
install(DIRECTORY behavior_trees_xml)

# Install targets
install(TARGETS
  ${PROJECT_NAME}
  trashbot_main
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include ${bot_behavior_interfaces_INCLUDE_DIRS})  
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_package()
