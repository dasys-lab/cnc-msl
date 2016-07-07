cmake_minimum_required(VERSION 2.8.3)
project(msl_astra_pro_driver)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED roscpp sensor_msgs geometry_msgs)

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
#  LIBRARIES msl_astra_pro_driver
  CATKIN_DEPENDS sensor_msgs geometry_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
	${catkin_INCLUDE_DIRS}
	${CMAKE_CURRENT_SOURCE_DIR}/OpenNI-Linux-x64-2.2/Include
	${CMAKE_CURRENT_SOURCE_DIR}/OpenNI-Linux-x64-2.2/Samples/Common
)
link_directories(
	 ${CMAKE_CURRENT_SOURCE_DIR}/OpenNI-Linux-x64-2.2/Redist
)

## Declare a C++ executable
add_executable(msl_astra_pro_driver
	src/msl_astra_pro_driver_node.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(msl_astra_pro_driver
	${catkin_LIBRARIES}
	OpenNI2
	rt
)
