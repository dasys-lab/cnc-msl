cmake_minimum_required(VERSION 2.8.3)
project(can_connection)

find_package(catkin REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES can_connection
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(include)

add_library(can_connection 
  src/usbcanconnection.cpp
  src/udpcanconnection.cpp
)
set_target_properties(can_connection PROPERTIES COMPILE_FLAGS "-O3 -Wall -Werror -fPIC -lpthread")

target_link_libraries(can_connection  ${catkin_LIBRARIES} pthread)

