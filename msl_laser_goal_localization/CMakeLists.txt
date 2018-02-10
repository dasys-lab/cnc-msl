cmake_minimum_required(VERSION 2.8.3)
project(msl_laser_goal_localization)

## Use c++ 11x std
set(CMAKE_CXX_FLAGS "-std=c++11  ${CMAKE_CXX_FLAGS}")
if (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")
	set(CMAKE_CXX_COMPILER_ARG1 "-std=c++11" CACHE STRING "C++ version for eclipse" FORCE)
	set(CMAKE_ECLIPSE_VERSION "4.5" CACHE STRING "Eclipse version" FORCE)
	add_definitions (-DCMAKE_ECLIPSE_GENERATE_SOURCE_PROJECT=TRUE)
endif (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")

find_package(catkin REQUIRED roscpp sensor_msgs pcl_conversions)
find_package(PCL 1.7  REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES msl_laser_goal_localization
  CATKIN_DEPENDS roscpp sensor_msgs
)

include_directories( include
     ${catkin_INCLUDE_DIRS}
     ${PCL_INCLUDE_DIRS}
)
add_definitions(${PCL_DEFINITIONS})

link_directories(
	 ${PCL_LIBRARY_DIRS}
)

# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(msl_laser_goal_localization
	src/LaserGoalDetection.cpp
)

# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(msl_laser_goal_localization
	${catkin_LIBRARIES}
	${PCL_LIBRARIES}
	${PCL_CONVERSIONS_LIBRARIES}
	rt
)
