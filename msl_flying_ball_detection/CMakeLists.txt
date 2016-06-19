cmake_minimum_required(VERSION 2.8.3)
project(msl_flying_ball_detection)

## Use c++ 11x std
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
if (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")
	set(CMAKE_CXX_COMPILER_ARG1 "-std=c++11" CACHE STRING "C++ version for eclipse" FORCE)
	set(CMAKE_ECLIPSE_VERSION "4.5" CACHE STRING "Eclipse version" FORCE)
	add_definitions (-DCMAKE_ECLIPSE_GENERATE_SOURCE_PROJECT=TRUE)
endif (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")

find_package(catkin REQUIRED roscpp sensor_msgs geometry_msgs pcl_conversions)
find_package(PCL 1.7  REQUIRED)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES msl_astra_pro_driver
  CATKIN_DEPENDS sensor_msgs geometry_msgs
#  DEPENDS system_lib
)

include_directories(include
	${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
	${CMAKE_CURRENT_SOURCE_DIR}/OpenNI-Linux-x64-2.2/Include
	${CMAKE_CURRENT_SOURCE_DIR}/OpenNI-Linux-x64-2.2/Samples/Common
)
add_definitions(${PCL_DEFINITIONS})

link_directories(
	 ${CMAKE_CURRENT_SOURCE_DIR}/OpenNI-Linux-x64-2.2/Redist
	 ${PCL_LIBRARY_DIRS}
)

add_executable(msl_flying_ball_detection
	src/FlyingBallDetection.cpp
	src/FrameListener.cpp
)

#add_dependencies(msl_flying_ball_detection
#	${catkin_LIBRARIES}
#)

target_link_libraries(msl_flying_ball_detection
	${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${PCL_CONVERSIONS_LIBRARIES}
	OpenNI2
	rt
)
