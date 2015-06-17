
## Required for CGAL
find_package(CGAL REQUIRED COMPONENTS Core)
include(${CGAL_USE_FILE})
include_directories(${CGAL_INCLUDE_DIRS})

## Required for this library
set(CGAL_DONT_OVERRIDE_CMAKE_FLAGS TRUE CACHE BOOL "Don't override flags")
set(CMAKE_CXX_FLAGS "-lCGAL -lCGAL_Core -std=c++11 -frounding-math ${CMAKE_CXX_FLAGS}")
