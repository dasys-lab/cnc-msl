cmake_minimum_required(VERSION 2.8.3)
project(msl_sensor_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  msl_msgs
  agent_id
  message_generation
)

add_message_files(
  FILES
  BallInfo.msg
  BallInfoCovMatrix.msg
  CameraSettings.msg
  CameraSettingsRequest.msg
  CompassInfo.msg
  CorrectedOdometryCovInfo.msg
  CorrectedOdometryInfo.msg
  CovMatrix3D.msg
  DistanceScanInfo.msg
  LocalizationType.msg
  LinePointList.msg
  ObstacleInfo.msg
  SharedBallInfo.msg
  SharedWorldInfo.msg
  TrackedOdometryInfo.msg
  VisionControl.msg
  VisionDebug.msg
  VisionGameState.msg
  VisionImage.msg
  WorldModelData.msg
  SimulatorWorldModelData.msg
  BallHypothesis.msg
  BallHypothesisList.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  agent_id
  msl_msgs
)

catkin_package(
# INCLUDE_DIRS include
#  LIBRARIES msl_actuator_msgs
 CATKIN_DEPENDS roscpp std_msgs msl_msgs message_runtime agent_id
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)