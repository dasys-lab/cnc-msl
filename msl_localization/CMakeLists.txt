cmake_minimum_required(VERSION 2.8.3)
project(msl_localization)

## Use c++ 11x std
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
if (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")
	set(CMAKE_CXX_COMPILER_ARG1 "-std=c++11" CACHE STRING "C++ version for eclipse" FORCE)
	set(CMAKE_ECLIPSE_VERSION "4.5" CACHE STRING "Eclipse version" FORCE)
	add_definitions (-DCMAKE_ECLIPSE_GENERATE_SOURCE_PROJECT=TRUE)
endif (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")

find_package(catkin REQUIRED msl_sensor_msgs system_config msl_msgs tf msl_vision)

set(CMAKE_CXX_FLAGS "-rdynamic -fnon-call-exceptions -ggdb ${CMAKE_CXX_FLAGS}")

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES msl_localization
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(msl_localization
 src/CNLocalization.cpp
 src/EgoMotionEstimator.cpp
 src/MapHelper.cpp
 src/msl_localization.cpp
 src/RandomHelper.cpp
 src/RosMsgReceiver.cpp
 src/Rprop.cpp
)

add_dependencies (msl_localization 
	msl_sensor_msgs_generate_messages_cpp  
	msl_actuator_msgs_generate_messages_cpp 
)

target_link_libraries(msl_localization ${catkin_LIBRARIES})
