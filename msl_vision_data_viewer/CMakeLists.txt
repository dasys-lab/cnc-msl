cmake_minimum_required(VERSION 2.8.3)
project(msl_vision_data_viewer)

## Use c++ 11x std
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
if (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")
	set(CMAKE_CXX_COMPILER_ARG1 "-std=c++11" CACHE STRING "C++ version for eclipse" FORCE)
	set(CMAKE_ECLIPSE_VERSION "4.5" CACHE STRING "Eclipse version" FORCE)
	add_definitions (-DCMAKE_ECLIPSE_GENERATE_SOURCE_PROJECT=TRUE)
endif (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")

find_package(catkin REQUIRED roscpp msl_sensor_msgs system_config msl_robot_id)

find_package(Boost REQUIRED COMPONENTS system iostreams)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES msl_vision_data_viewer
  CATKIN_DEPENDS roscpp msl_sensor_msgs system_config msl_robot_id
  DEPENDS ${Boost_LIBRARIES} m util
)

include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

FILE(GLOB SRCFILES_CPP src/helpers/*.cpp)

add_executable(msl_vision_data_viewer src/DataView.cpp ${SRCFILES_CPP})
set_target_properties(msl_vision_data_viewer PROPERTIES COMPILE_FLAGS "-O3 -fpermissive")
target_link_libraries(msl_vision_data_viewer m util ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(msl_vision_data_viewer msl_sensor_msgs_generate_messages_cpp)

