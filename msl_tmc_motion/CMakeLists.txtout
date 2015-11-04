cmake_minimum_required(VERSION 2.8.3)
project(msl_tmc_motion)

## Use c++ 11x std
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS msl_actuator_msgs system_config cnc_geometry) 

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
#  LIBRARIES msl_tmc_motion
  CATKIN_DEPENDS msl_actuator_msgs system_config cnc_geometry
#  DEPENDS system_lib 
)

###########
## Build ##
###########

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
)
file(GLOB_RECURSE tmc_src "src/*.cpp")
add_executable(${PROJECT_NAME} ${tmc_src})
message ("<<<< tmc: ${catkin_LIBRARIES}")
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} msl_actuator_msgs_generate_messages_cpp)
