cmake_minimum_required(VERSION 2.8.3)
project(goal_detection)

## Use c++ 11x std
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
if (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")
	set(CMAKE_CXX_COMPILER_ARG1 "-std=c++11" CACHE STRING "C++ version for eclipse" FORCE)
	set(CMAKE_ECLIPSE_VERSION "4.5" CACHE STRING "Eclipse version" FORCE)
	add_definitions (-DCMAKE_ECLIPSE_GENERATE_SOURCE_PROJECT=TRUE)
endif (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  msl_msgs
  geometry_msgs
  sensor_msgs
  tf_conversions
  system_config
)
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES goal_detection
   CATKIN_DEPENDS roscpp rospy std_msgs msl_msgs geometry_msgs sensor_msgs  tf_conversions system_config
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(goal_detection 
	src/GoalDetection.cpp
	src/LaserScanListener.cpp
	)
add_dependencies(goal_detection ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(goal_detection
  ${catkin_LIBRARIES}
)
