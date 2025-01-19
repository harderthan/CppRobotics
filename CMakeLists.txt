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



# Get the Numpy include directory dynamically
find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)
execute_process(
    COMMAND ${Python3_EXECUTABLE} -c "import numpy; print(numpy.get_include())"
    OUTPUT_VARIABLE NumPy_INCLUDE_DIRS
    OUTPUT_STRIP_TRAILING_WHITESPACE
)
if (NOT NumPy_INCLUDE_DIRS)
    message(FATAL_ERROR "Failed to find Numpy include directory. Ensure Numpy is installed.")
else()
    message(STATUS "Numpy include directory: ${NumPy_INCLUDE_DIRS}")
    include_directories(${NumPy_INCLUDE_DIRS})
endif()

# Find MatplotlibCpp
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
find_package(MatplotlibCpp REQUIRED)

# Build the subdirectories
add_subdirectory(path_planning)
