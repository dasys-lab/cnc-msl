cmake_minimum_required(VERSION 2.8.3)
project(msl_robot)
 
## Use c++ 11x std
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
if (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")
	set(CMAKE_CXX_COMPILER_ARG1 "-std=c++11" CACHE STRING "C++ version for eclipse" FORCE)
	set(CMAKE_ECLIPSE_VERSION "4.5" CACHE STRING "Eclipse version" FORCE)
	add_definitions (-DCMAKE_ECLIPSE_GENERATE_SOURCE_PROJECT=TRUE)
endif (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")

find_package(catkin REQUIRED COMPONENTS msl_worldmodel cnc_geometry system_config)

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES msl_robot
  CATKIN_DEPENDS msl_worldmodel cnc_geometry system_config
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(msl_robot
  src/MSLRobot.cpp
  src/kicker/KickCurve.cpp
  src/kicker/Kicker.cpp
  src/robotmovement/AlloSearchArea.cpp
  src/robotmovement/EgoSearchArea.cpp
  src/robotmovement/MovementQuery.cpp
  src/robotmovement/RobotMovement.cpp
  src/robotmovement/SearchArea.cpp
  src/dribbleCalibration/container/DribbleCalibrationContainer.cpp
  src/dribbleCalibration/container/DribbleCalibrationQuery.cpp
  src/dribbleCalibration/container/MovementContainer.cpp
  src/dribbleCalibration/behaviours/DribbleBackward.cpp
  src/dribbleCalibration/behaviours/DribbleForward.cpp
  src/dribbleCalibration/behaviours/DribbleRotateLeft.cpp
  src/dribbleCalibration/behaviours/DribbleRotateRight.cpp
)

#message ("msl_robot - catkin_LIBRARIES: ${catkin_LIBRARIES}")

target_link_libraries(msl_robot
  ${catkin_LIBRARIES}
)
