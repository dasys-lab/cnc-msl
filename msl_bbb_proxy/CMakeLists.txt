cmake_minimum_required(VERSION 2.8.3)
project(msl_bbb_proxy)


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
find_package(catkin REQUIRED system_config roscpp roslib msl_actuator_msgs msl_msgs std_msgs udp_proxy_generator process_manager)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system filesystem regex)

catkin_package(
# INCLUDE_DIRS include
# LIBRARIES 
 CATKIN_DEPENDS system_config roscpp roslib msl_actuator_msgs msl_msgs std_msgs
 DEPENDS system filesystem regex
)

include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

# Triggers the udp_proxy_generator to create the serialization source
add_custom_command(
    OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/proxy_gen/ros2udpProxy.cpp
    COMMAND rosrun udp_proxy_generator udp_proxy_generator ${PROJECT_NAME}
    #COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/../../devel/lib/udp_proxy_generator/udp_proxy_generator ${PROJECT_NAME}
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/relayMsgs.conf ${udp_proxy_generator_LIBRARIES}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "${PROJECT_NAME}: Generating Code ..."
 )

## Declare a cpp executable
add_executable(${PROJECT_NAME}
  proxy_gen/ros2udpProxy.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} msl_actuator_msgs_gencpp)

