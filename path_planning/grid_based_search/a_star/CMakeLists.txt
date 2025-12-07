cmake_minimum_required(VERSION 3.8)
project(a_star)

# Set C++20 standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Create library
add_library(a_star
  a_star.cpp
)

# Include directories
target_include_directories(a_star
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include>
)

# Install library
install(
  TARGETS a_star
  EXPORT a_star_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN "*.hpp"
)

# Install CMake export
install(
  EXPORT a_star_targets
  FILE a_star_targets.cmake
  NAMESPACE a_star::
  DESTINATION share/a_star/cmake
)

# Tests
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(a_star_test
    a_star_test.cpp
  )

  target_include_directories(a_star_test
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/..
  )

  target_link_libraries(a_star_test
    a_star
  )
endif()

