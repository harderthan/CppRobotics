cmake_minimum_required(VERSION 3.8)
project(grid_based_search)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set C++20 standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(rclcpp REQUIRED)
find_package(foxglove_msgs REQUIRED)

# Add a_star subdirectory
add_subdirectory(a_star)

# ROS2 Node
add_executable(a_star_node ros2/a_star_node.cpp)

ament_target_dependencies(a_star_node
  rclcpp
  foxglove_msgs
)

# Include directories for a_star headers
target_include_directories(a_star_node
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
)

# Link a_star library
target_link_libraries(a_star_node
  a_star
)

install(
  TARGETS a_star_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY foxglove
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
