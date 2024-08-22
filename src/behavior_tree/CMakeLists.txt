cmake_minimum_required(VERSION 3.8)
project(behavior_tree)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find the required packagess
find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)

ament_package()
