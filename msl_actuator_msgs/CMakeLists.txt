cmake_minimum_required(VERSION 2.8.3)
project(msl_actuator_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  msl_msgs
  agent_id
  message_generation
)

add_message_files(
  FILES
  BallCatchCmd.msg
  BallHandleCmd.msg
  BallHandleMode.msg
  HaveBallInfo.msg
  JoystickControl.msg
  CanMsg.msg
  JoystickRobotIDs.msg
  KickControl.msg
  KickerStatInfo.msg
  KickTime.msg
  MotionBurst.msg
  MotionControl.msg
  MotionLight.msg
  MotionStatInfo.msg
  OdometryInfo.msg
  PWMInfo.msg
  RawOdometryInfo.msg
  ShovelSelectCmd.msg
  VisionRelocTrigger.msg
  IMUData.msg
  CalibrationCoefficient.msg
)

generate_messages(
  DEPENDENCIES
  agent_id
  std_msgs
  msl_msgs
)

catkin_package(
# INCLUDE_DIRS include
#  LIBRARIES msl_actuator_msgs
 CATKIN_DEPENDS roscpp std_msgs msl_msgs message_runtime
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

