cmake_minimum_required(VERSION 2.8.3)
project(usb_can_proxy)

find_package(catkin REQUIRED
  can_connection
  system_config
  roscpp
  msl_actuator_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES usb_can_proxy
  CATKIN_DEPENDS system_config can_connection roscpp message_runtime msl_actuator_msgs
#  DEPENDS system_lib
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(usb_can_proxy src/UsbCanProxy.cpp)
target_link_libraries(usb_can_proxy ${catkin_LIBRARIES})
set_target_properties(usb_can_proxy PROPERTIES COMPILE_FLAGS "-O3 -Wall -Werror")

add_executable(usb_can_console src/UsbCanConsole.cpp)
target_link_libraries(usb_can_console ${catkin_LIBRARIES})
set_target_properties(usb_can_console PROPERTIES COMPILE_FLAGS "-O3 -Wall -Werror -fPIC -pthread")
