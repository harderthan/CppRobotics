set(A_STAR a_star)
add_executable(${A_STAR} main.cc a_star.cc)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(${A_STAR} MatplotlibCpp Python3::Python)
