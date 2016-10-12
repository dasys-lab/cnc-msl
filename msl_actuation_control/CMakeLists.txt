cmake_minimum_required(VERSION 2.8.3)
project(msl_actuation_control)

## Use c++ 11x std
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
if (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")
	set(CMAKE_CXX_COMPILER_ARG1 "-std=c++11" CACHE STRING "C++ version for eclipse" FORCE)
	set(CMAKE_ECLIPSE_VERSION "4.5" CACHE STRING "Eclipse version" FORCE)
	add_definitions (-DCMAKE_ECLIPSE_GENERATE_SOURCE_PROJECT=TRUE)
endif (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")

find_package(catkin REQUIRED system_config roscpp can_connection msl_actuator_msgs msl_sensor_msgs)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES msl_actuation_control
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
add_executable(msl_actuation_control src/ActuationControl.cpp)
target_link_libraries(msl_actuation_control ${Boost_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(msl_actuation_control msl_actuator_msgs_generate_messages_cpp msl_sensor_msgs_generate_messages_cpp)


