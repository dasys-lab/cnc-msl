cmake_minimum_required(VERSION 2.8.3)
project(msl_directed_depth_vision)

## Use c++ 11x std
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
set(MRPT_DONT_USE_DBG_LIBS 1)
 
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED system_config nodelet roscpp pcl_conversions msl_pcl_filters)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake CACHE INTERNAL "" FORCE)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(MRPT REQUIRED base maps)
find_package(PCL 1.7  REQUIRED)

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
#  LIBRARIES
  CATKIN_DEPENDS system_config msl_pcl_filters
#  DEPENDS 
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${MRPT_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/OpenNI-Linux-x64-2.2/Include
	${CMAKE_CURRENT_SOURCE_DIR}/OpenNI-Linux-x64-2.2/Samples/Common
)


link_directories(
    ${PCL_LIBRARY_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/OpenNI-Linux-x64-2.2/Redist
)
add_definitions(${PCL_DEFINITIONS})

add_executable(depth_vision
        src/DirectedDepthVisionNode.cpp
        src/DepthVision.cpp
)
#add_dependencies(depth_vision
#  ${catkin_LIBRARIES}
#  ${Boost_LIBRARIES}
#  ${PCL_LIBRARIES}
#  ${PCL_CONVERSIONS_LIBRARiIES}
#  ${MRPT_LIBS}
#)
## Specify libraries to link a library or executable target against
target_link_libraries(depth_vision
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PCL_LIBRARIES}
  ${PCL_CONVERSIONS_LIBRARIES}
  ${MRPT_LIBS}
  OpenNI2
  rt
)
