cmake_minimum_required(VERSION 3.8)
project(path_planner_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set C++20 standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(foxglove_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# ROS2 Node
add_executable(a_star_node src/a_star_node.cpp)

ament_target_dependencies(a_star_node
  rclcpp
  foxglove_msgs
  geometry_msgs
)

install(
  TARGETS a_star_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
