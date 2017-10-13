cmake_minimum_required(VERSION 2.8.3)
project(msl_msgs)    
       
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  agent_id
  message_generation
)

add_message_files(
  FILES
  KickConfig.msg
  MotionInfo.msg
  Point2dInfo.msg
  Point3dInfo.msg
  PositionInfo.msg
  Timestamp.msg
  Velocity3dInfo.msg
  VelocityInfo.msg
  JoystickCommand.msg
  GameStage.msg
  RefBoxCommand.msg
  RefBoxState.msg
  PathPlanner.msg
  VoronoiNetInfo.msg
  CorridorCheck.msg
  Pose2dStamped.msg
)
 
generate_messages(
  DEPENDENCIES
  std_msgs
  agent_id
  geometry_msgs
)

catkin_package(
 CATKIN_DEPENDS roscpp std_msgs message_runtime geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

