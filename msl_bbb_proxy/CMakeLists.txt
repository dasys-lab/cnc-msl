cmake_minimum_required(VERSION 2.8.3)
project(msl_bbb_proxy)

## Use c++ 11x std
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
if (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")
	set(CMAKE_CXX_COMPILER_ARG1 "-std=c++11" CACHE STRING "C++ version for eclipse" FORCE)
	set(CMAKE_ECLIPSE_VERSION "4.5" CACHE STRING "Eclipse version" FORCE)
	add_definitions (-DCMAKE_ECLIPSE_GENERATE_SOURCE_PROJECT=TRUE)
endif (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")

find_package(catkin REQUIRED system_config roscpp roslib msl_actuator_msgs msl_msgs std_msgs udp_proxy_generator process_manager)

find_package(Boost REQUIRED COMPONENTS system filesystem regex)

catkin_package(
# INCLUDE_DIRS include
# LIBRARIES 
 CATKIN_DEPENDS system_config roscpp roslib msl_actuator_msgs msl_msgs std_msgs
 DEPENDS system filesystem regex
)

include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_custom_command(
    OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/proxy_gen/ros2udpProxy.cpp
    COMMAND rosrun udp_proxy_generator udp_proxy_generator ${PROJECT_NAME}
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/relayMsgs.conf ${udp_proxy_generator_LIBRARIES}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "${PROJECT_NAME}: Generating Code ..."
 )

add_executable(${PROJECT_NAME}
  proxy_gen/ros2udpProxy.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(${PROJECT_NAME} msl_actuator_msgs_generate_messages_cpp)

