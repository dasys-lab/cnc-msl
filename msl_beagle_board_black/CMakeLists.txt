cmake_minimum_required(VERSION 2.8.3)
project(msl_beagle_board_black)

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
  msl_actuator_msgs
  msl_black_lib
  system_config
  process_manager
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES msl_beagle_board_black
#  CATKIN_DEPENDS msl_black_lib
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(msl_beagle_board_black 
	src/actuator.cpp
	src/ballhandle.cpp
	src/imu.cpp
	src/lightbarrier.cpp
	src/opticalflow.cpp
	src/shovelselect.cpp
)
add_dependencies(msl_beagle_board_black msl_actuator_msgs_generate_messages_cpp)
target_link_libraries(msl_beagle_board_black ${catkin_LIBRARIES})

add_executable(debug src/debug.cpp)
add_dependencies(debug msl_actuator_msgs_generate_messages_cpp)
target_link_libraries(debug ${catkin_LIBRARIES})

