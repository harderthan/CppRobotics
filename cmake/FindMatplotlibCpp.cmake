find_path(MatplotlibCpp_INCLUDE_DIRS "matplotlibcpp.h"
  PATHS ${CMAKE_CURRENT_LIST_DIR}/../third_party/matplotlib-cpp
)

if (MatplotlibCpp_INCLUDE_DIRS)
  add_library(MatplotlibCpp INTERFACE)
  target_include_directories(MatplotlibCpp INTERFACE ${MatplotlibCpp_INCLUDE_DIRS})
  message(STATUS "Found MatplotlibCpp: ${MatplotlibCpp_INCLUDE_DIRS}")
else()
  message(FATAL_ERROR "Could not find MatplotlibCpp")
endif()
