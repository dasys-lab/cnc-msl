cmake_minimum_required(VERSION 2.8.3)
project(analyse_kick)

find_package(catkin REQUIRED roscpp msl_msgs sensor_msgs agent_id)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES analyse_kick
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(analyse_kick src/analyse.cpp)

add_dependencies(analyse_kick ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} msl_msgs_generate_messages_cpp agent_id)

target_link_libraries(analyse_kick
  ${catkin_LIBRARIES}
)
