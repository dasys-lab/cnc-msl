cmake_minimum_required(VERSION 2.8.3)
project(msl_bbb_api)

find_package(catkin REQUIRED COMPONENTS)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES msl_bbb_api
  #CATKIN_DEPENDS roscpp rospy std_msgs
  #DEPENDS system_lib
)

include_directories(include)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(msl_bbb_api
  src/BeagleGPIO.cpp
  src/BeaglePins.cpp
  src/BeaglePWM.cpp
)

