cmake_minimum_required(VERSION 2.8.3)
project(msl_motion)

## Use c++ 11x std
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
if (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")
	set(CMAKE_CXX_COMPILER_ARG1 "-std=c++11" CACHE STRING "C++ version for eclipse" FORCE)
	set(CMAKE_ECLIPSE_VERSION "4.5" CACHE STRING "Eclipse version" FORCE)
	add_definitions (-DCMAKE_ECLIPSE_GENERATE_SOURCE_PROJECT=TRUE)
endif (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  can_connection
  system_config
  msl_actuator_msgs
  msl_msgs
  message_generation
  roscpp
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES msl_motion
  CATKIN_DEPENDS message_runtime can_connection msl_actuator_msgs msl_msgs
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})

## msl_motion
add_executable(msl_motion
  src/RosHelper.cpp
  src/settings.cpp
  src/driver/controller.cpp
  src/driver/eposcan.cpp
  src/logging.cpp
  src/gonzales.cpp
  src/EposGonzales.cpp
)
target_link_libraries(msl_motion pthread m ${catkin_LIBRARIES} ${Boost_LIBRARIES})
set_target_properties(msl_motion PROPERTIES COMPILE_FLAGS "-O3 -Wall -Werror")
add_dependencies(msl_motion 
  msl_actuator_msgs_generate_messages_cpp 
  msl_msgs_generate_messages_cpp
)

## msl_motion_lifetest
add_executable(msl_motion_lifetest
  src/RosHelper.cpp
  src/settings.cpp
  src/driver/controller.cpp
  src/driver/eposcan.cpp
  src/logging.cpp
  src/gonzales.cpp
  src/epostest.cpp
)
target_link_libraries(msl_motion_lifetest pthread m ${catkin_LIBRARIES} ${Boost_LIBRARIES})
set_target_properties(msl_motion_lifetest PROPERTIES COMPILE_FLAGS "-O3 -Wall -Werror")
add_dependencies(msl_motion_lifetest 
  msl_actuator_msgs_generate_messages_cpp
  msl_msgs_generate_messages_cpp
)

