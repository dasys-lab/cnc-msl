cmake_minimum_required(VERSION 2.8.3)
project(msl_goalie_kicker)

## Use c++ 11x std
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
if (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")
	set(CMAKE_CXX_COMPILER_ARG1 "-std=c++11" CACHE STRING "C++ version for eclipse" FORCE)
	set(CMAKE_ECLIPSE_VERSION "4.5" CACHE STRING "Eclipse version" FORCE)
	add_definitions (-DCMAKE_ECLIPSE_GENERATE_SOURCE_PROJECT=TRUE)
endif (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")

find_package(catkin REQUIRED COMPONENTS msl_actuator_msgs system_config cnc_geometry serial) 

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
  INCLUDE_DIRS include
#  LIBRARIES msl_goalie_kicker
  CATKIN_DEPENDS msl_actuator_msgs system_config cnc_geometry serial
#  DEPENDS system_lib 
)

###########
## Build ##
###########

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} 
  src/Kicker.cpp 
)


target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} msl_actuator_msgs_generate_messages_cpp)
