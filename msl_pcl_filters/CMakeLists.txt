cmake_minimum_required(VERSION 2.8.3)
project(msl_pcl_filters)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)
find_package(PCL 1.7 REQUIRED)

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
  LIBRARIES msl_pcl_filters
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)
 
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

## Declare a C++ library
add_library(msl_pcl_filters
  src/FilterVoxelGrid.cpp
)

## Add cmake target dependencies of the executable
## same as for the library above
#add_dependencies(msl_pcl_filters
#  ${catkin_LIBRARIES}
#  ${PCL_LIBRARIES}
#)

## Specify libraries to link a library or executable target against
target_link_libraries(msl_pcl_filters
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

