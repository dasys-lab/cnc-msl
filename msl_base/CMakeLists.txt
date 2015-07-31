cmake_minimum_required(VERSION 2.8.3)
project(msl_base)
 
## Use c++ 11x std
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")


## Enable exception handling for segfaults
set(CMAKE_CXX_FLAGS "-rdynamic -g -fnon-call-exceptions -ggdb ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp 
  system_config
  autodiff 
  alica_engine 
  alica_ros_proxy 
  msl_expressions 
  msl_worldmodel
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS 
    roscpp 
    system_config
    autodiff  
    alica_engine 
    alica_ros_proxy 
    msl_expressions 
    msl_worldmodel
)

###########
## Build ##
###########


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})

## Declare a cpp executable
add_executable(msl_base src/Base.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(msl_base ${catkin_LIBRARIES})

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(msl_base ${catkin_LIBRARIES})
