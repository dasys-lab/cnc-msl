cmake_minimum_required(VERSION 2.8.3)
project(msl_worldmodel) 
  
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED cnc_geometry system_config msl_actuator_msgs msl_sensor_msgs event_handling alica_engine rqt_robot_control constraintsolver) 
find_package(CGAL REQUIRED COMPONENTS Core)
 include(${CGAL_USE_FILE})

## Use c++ 11x std
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

## Enable exception handling for segfaults using SigFault.h
set(CMAKE_CXX_FLAGS "-rdynamic -fnon-call-exceptions -ggdb ${CMAKE_CXX_FLAGS}")
 
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES msl_worldmodel
  CATKIN_DEPENDS 
    cnc_geometry 
    system_config 
    msl_actuator_msgs 
    msl_sensor_msgs 
    event_handling 
    alica_engine
    constraintsolver
  DEPENDS cgal
  CFG_EXTRAS ${PROJECT_NAME}-extras.cmake
)

###########
## Build ##
###########

 include_directories(include ${catkin_INCLUDE_DIRS} ${CGAL_INCLUDE_DIRS})

## Declare a cpp library
 add_library(msl_worldmodel
   src/MSLConstraintBuilder.cpp
   src/MSLWorldModel.cpp
   src/MSLFootballField.cpp
   src/robotmovement/RobotMovement.cpp
   src/sharedworldmodel/MSLSharedWorldModel.cpp
   src/RawSensorData.cpp
   src/Robots.cpp
   src/Ball.cpp
   src/Game.cpp
   src/Kicker.cpp
   src/KickCurve.cpp
   src/WhiteBoard.cpp
   src/Rules.cpp
   src/pathplanner/PathPlanner.cpp
   src/pathplanner/SearchNode.cpp
   src/pathplanner/VoronoiNet.cpp
   src/pathplanner/PathProxy.cpp
   src/pathplanner/evaluator/PathEvaluator.cpp
   src/pathplanner/evaluator/Evaluator.cpp
   src/pathplanner/evaluator/PrefDirEvaluator.cpp
   src/ballTracking/BallIntegrator.cpp  
   src/ballTracking/BallZTracker.cpp  
   src/ballTracking/EgoMotionEstimator.cpp  
   src/ballTracking/ObjectContainer.cpp  
   src/ballTracking/ObjectTracker.cpp  
   src/ballTracking/SharedMemoryHelper.cpp  
   src/ballTracking/TimeHelper.cpp
 )

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(msl_worldmodel 
  ${catkin_LIBRARIES} 
  msl_worldmodel_generate_messages_cpp 
  msl_sensor_msgs_generate_messages_cpp  
  msl_actuator_msgs_generate_messages_cpp 
  geometry_msgs_generate_messages_cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(msl_worldmodel ${catkin_LIBRARIES})

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
catkin_add_gtest(${PROJECT_NAME}-test src/pathplanner/test/test_path_planner.cpp)
if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} ${catkin_LIBRARIES})
endif()

