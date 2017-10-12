cmake_minimum_required(VERSION 2.8.3)
project(msl_helper_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  msl_msgs
  agent_id
  message_generation
)

add_message_files(
  FILES
  DebugMsg.msg
  DebugPoint.msg
  Edge.msg
  PassMsg.msg
  WatchBallMsg.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  msl_msgs
  agent_id
)

catkin_package(
 CATKIN_DEPENDS roscpp std_msgs agent_id message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

