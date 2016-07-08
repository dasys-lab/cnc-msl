cmake_minimum_required(VERSION 2.8.3)
project(msl_flying_ball_detection)

find_package(catkin REQUIRED roscpp sensor_msgs geometry_msgs pcl_conversions msl_actuator_msgs msl_msgs)
find_package(PCL 1.7  REQUIRED)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES msl_astra_pro_driver
  CATKIN_DEPENDS sensor_msgs geometry_msgs cnc_geometry msl_actuator_msgs
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

add_dependencies(msl_flying_ball_detection
	msl_msgs_generate_messages_cpp
	msl_actuator_msgs_generate_messages_cpp
)

target_link_libraries(msl_flying_ball_detection
	${catkin_LIBRARIES}
	${PCL_LIBRARIES}
	${PCL_CONVERSIONS_LIBRARIES}
	OpenNI2
	rt
)
