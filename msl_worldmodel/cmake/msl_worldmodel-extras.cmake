
## Required for CGAL
find_package(CGAL REQUIRED COMPONENTS Core)
include(${CGAL_USE_FILE})
include_directories(${CGAL_INCLUDE_DIRS})

## Required for this library
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
