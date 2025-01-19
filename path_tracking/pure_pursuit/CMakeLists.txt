set(PURE_PURSUIT pure_pursuit)
add_executable(${PURE_PURSUIT} main.cc pure_pursuit.cc)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(${PURE_PURSUIT} MatplotlibCpp Python3::Python)
