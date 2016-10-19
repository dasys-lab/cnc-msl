cmake_minimum_required(VERSION 2.8.3)
project(carpet_calibrator)

## Use c++ 11x std
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
if (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")
	set(CMAKE_CXX_COMPILER_ARG1 "-std=c++11" CACHE STRING "C++ version for eclipse" FORCE)
	set(CMAKE_ECLIPSE_VERSION "4.5" CACHE STRING "Eclipse version" FORCE)
	add_definitions (-DCMAKE_ECLIPSE_GENERATE_SOURCE_PROJECT=TRUE)
endif (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")

find_package(catkin REQUIRED system_config nodelet roscpp image_transport cv_bridge msl_actuator_msgs msl_vision alica_ros_proxy)

find_package(Boost REQUIRED COMPONENTS system thread)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES carpet_calibrator
  CATKIN_DEPENDS system_config 
  msl_actuator_msgs
  msl_vision
  alica_ros_proxy
  #DEPENDS ${Boost_LIBRARIES} 
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_library(carpet_calibrator
   src/CarpetCalibratorNodelet.cpp
)

add_executable(carpet_calibrator_line_points
   src/FilterLinePointsCalib.cpp
   src/FilterLinePointsNode.cpp
)

add_dependencies(carpet_calibrator msl_actuator_msgs_generate_messages_cpp)
add_dependencies(carpet_calibrator_line_points msl_actuator_msgs_generate_messages_cpp)

target_link_libraries(carpet_calibrator
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  Xv
)
target_link_libraries(carpet_calibrator_line_points
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  Xv
)
