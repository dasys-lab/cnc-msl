cmake_minimum_required(VERSION 2.8.3)
project(msl_motion_calibration_laser)

## Use c++ 11x std
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
if (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")
	set(CMAKE_CXX_COMPILER_ARG1 "-std=c++11" CACHE STRING "C++ version for eclipse" FORCE)
	set(CMAKE_ECLIPSE_VERSION "4.5" CACHE STRING "Eclipse version" FORCE)
	add_definitions (-DCMAKE_ECLIPSE_GENERATE_SOURCE_PROJECT=TRUE)
endif (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs cnc_geometry)

find_package(Boost REQUIRED COMPONENTS system filesystem regex)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES 
 CATKIN_DEPENDS roscpp cnc_geometry
 DEPENDS 
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(msl_motion_calibration_laser
  src/MotionCalibrationLaser.cpp
  src/LaserPointGroup.cpp
  src/LaserPoint.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

