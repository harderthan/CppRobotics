cmake_minimum_required(VERSION 3.15)
project(cpp_robotics)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find matplotlib_cpp
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
find_package(MatplotlibCpp REQUIRED)
