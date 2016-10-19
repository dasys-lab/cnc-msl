cmake_minimum_required(VERSION 2.8.3)
project(msl_beagle_board_black_udp)

## Use c++ 11x std
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -ggdb")
if (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")
	set(CMAKE_CXX_COMPILER_ARG1 "-std=c++11" CACHE STRING "C++ version for eclipse" FORCE)
	set(CMAKE_ECLIPSE_VERSION "4.5" CACHE STRING "Eclipse version" FORCE)
	add_definitions (-DCMAKE_ECLIPSE_GENERATE_SOURCE_PROJECT=TRUE)
endif (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cnc_geometry
  msl_actuator_msgs
  msl_bbb_api
  msl_black_lib
  system_config
  process_manager
  cnc_geometry
  can_connection_bbb
)


catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES msl_beagle_board_black
#  CATKIN_DEPENDS msl_black_lib
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(msl_beagle_board_black_udp 
	src/actuator.cpp
	src/ballhandle.cpp
	src/imu.cpp
	src/lightbarrier.cpp
	src/motor.cpp
	src/opticalflow.cpp
	src/sensor.cpp
	src/shovelselect.cpp
	src/sensor.cpp
	src/Spline.cpp
)
target_link_libraries(msl_beagle_board_black_udp ${catkin_LIBRARIES})
add_dependencies(msl_beagle_board_black_udp msl_actuator_msgs_generate_messages_cpp)

add_executable(debug_udp src/debug.cpp)
target_link_libraries(debug_udp ${catkin_LIBRARIES})
add_dependencies(debug_udp msl_actuator_msgs_generate_messages_cpp)
