cmake_minimum_required(VERSION 2.8.3)
project(msl_base)

## Use c++ 11x std
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
## Enable exception handling for segfaults
set(CMAKE_CXX_FLAGS "-rdynamic -g -fnon-call-exceptions -ggdb ${CMAKE_CXX_FLAGS}")
## Used for cgal
set(CGAL_DONT_OVERRIDE_CMAKE_FLAGS TRUE CACHE BOOL "Don't override flags")
set(CMAKE_CXX_FLAGS "-lCGAL -lCGAL_Core -frounding-math ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED roscpp system_config system_util fsystem autodiff event_handling alica_engine alica_ros_proxy msl_expressions msl_worldmodel)
find_package(CGAL REQUIRED COMPONENTS Core)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES alica_engine msl_expressions
  CATKIN_DEPENDS alica_engine alica_ros_proxy msl_expressions msl_simulator msl_worldmodel
#  DEPENDS tinyxml
  DEPENDS cgal
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS} ${CGAL_INCLUDE_DIRS})
include(${CGAL_USE_FILE})

## Declare a cpp library
# add_library(msl_base
#   src/${PROJECT_NAME}/msl_base.cpp
# )

## Declare a cpp executable
add_executable(msl_base src/Base.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(msl_base msl_base_generate_messages_cpp alica_engine msl_expressions msl_worldmodel alica_ros_proxy CGAL)

## Specify libraries to link a library or executable target against
 target_link_libraries(msl_base ${catkin_LIBRARIES} alica_engine msl_expressions msl_worldmodel alica_ros_proxy CGAL)


