cmake_minimum_required(VERSION 2.8.3)
project(msl_udp_proxy)


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
find_package(catkin REQUIRED system_config roscpp roslib sensor_msgs msl_sensor_msgs process_manager alica_ros_proxy udp_proxy_generator robot_control msl_actuator_msgs msl_helper_msgs)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system filesystem regex)


catkin_package(
# INCLUDE_DIRS include
# LIBRARIES 
 CATKIN_DEPENDS system_config roscpp roslib sensor_msgs msl_sensor_msgs process_manager msl_actuator_msgs msl_helper_msgs
 DEPENDS system filesystem regex
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

# Please note, that the following command does not work, if you compile your workspace the first time, because rosrun does not know anything about udp_proxy_generator in that case.
# We could exchange the command with something more ROS-Independent, like calling the binary without rosrun.
add_custom_command(
    OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/proxy_gen/ros2udpProxy.cpp
    #COMMAND rosrun udp_proxy_generator udp_proxy_generator ${PROJECT_NAME}
    COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/../../../devel/lib/udp_proxy_generator/udp_proxy_generator ${PROJECT_NAME}
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
add_dependencies(${PROJECT_NAME} 
	msl_sensor_msgs_generate_messages_cpp 
	msl_msgs_generate_messages_cpp 
	process_manager_generate_messages_cpp 
	robot_control_generate_messages_cpp
)

