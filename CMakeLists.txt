cmake_minimum_required(VERSION 3.15)
project(cpp_robotics)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Run cpplint using execute_process
find_program(CPPLINT_EXEC cpplint)
if (CPPLINT_EXEC)
    execute_process(
        COMMAND ${CPPLINT_EXEC} --extensions=cc,cpp,h,hpp --recursive ./ --quiet
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        RESULT_VARIABLE cpplint_result
        OUTPUT_VARIABLE cpplint_output
        ERROR_VARIABLE cpplint_error
    )
    if (cpplint_result)
        message(FATAL_ERROR "cpplint found errors:\n${cpplint_error}")
    else()
        message(STATUS "cpplint completed successfully.")
    endif()
else()
    message(WARNING "cpplint not found. Please install cpplint to enable linting.")
endif()

# Find matplotlib_cpp
find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
find_package(MatplotlibCpp REQUIRED)

add_subdirectory(path_planning)
