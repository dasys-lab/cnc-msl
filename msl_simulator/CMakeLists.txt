cmake_minimum_required(VERSION 2.8.3)
project(msl_simulator)



## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  msl_sensor_msgs 
  msl_actuator_msgs
)
## System dependencies are found with CMake's conventions
#find_package(Boost REQUIRED COMPONENTS system)

# some utils and helper vars
string(TOLOWER ${CMAKE_PROJECT_NAME} CMAKE_PROJECT_NAME_LOWER)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/modules)
include(${PROJECT_SOURCE_DIR}/cmake/Utils.cmake)
standard_config()
standard_paths(${PROJECT_SOURCE_DIR} bin lib)

# definitions for knowing the OS from the code
add_definitions(-DHAVE_UNIX)
add_definitions(-DHAVE_LINUX)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
   DIRECTORY msg FILES
   log_frame.msg  
   messages_robocup_ssl_refbox_log.msg 
   sim_commands.msg
   sim_robot_command.msg      
   ssl_detection_robot.msg
   messages_robocup_ssl_detection.msg  
   messages_robocup_ssl_wrapper.msg     
   sim_packet.msg       
   sim_robot_replacement.msg  
   ssl_geometry_camera_calibration.msg
   messages_robocup_ssl_geometry.msg   
   sim_ball_replacement.msg             
   sim_replacement.msg  
   ssl_detection_ball.msg     
   ssl_geometry_field_size.msg;
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
 )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES msl_simulator
  CATKIN_DEPENDS roscpp message_runtime 
 #DEPENDS 
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)

# we will append all libs to this var
set(libs)

# OpenGL
find_package(OpenGL REQUIRED)
include_directories(${OPENGL_INCLUDE_DIR})
list(APPEND libs ${OPENGL_LIBRARIES})
#find_package(GLUT REQUIRED)
#include_directories(${GLUT_INCLUDE_DIR})

# Qt
set(QT_USE_QTNETWORK true)
set(QT_USE_QTOPENGL true)
find_package(Qt4 REQUIRED)
include(${QT_USE_FILE})
list(APPEND libs ${QT_LIBRARIES})

# ODE
find_package(ODE REQUIRED)
include_directories(${ODE_INCLUDE_DIRS})
list(APPEND libs ${ODE_LIBRARIES})
#TODO: make this an option
option(DOUBLE_PRECISION "Use double precision? If not single precision will be used." ON)
if(DOUBLE_PRECISION)
    add_definitions(-DdDOUBLE)
else()
    add_definitions(-DdSINGLE)
endif()

# VarTypes
find_package(VarTypes REQUIRED)
include_directories(${VARTYPES_INCLUDE_DIRS})
list(APPEND libs ${VARTYPES_LIBRARIES})

# Protobuf
#find_package(Protobuf REQUIRED)
#include_directories(${PROTOBUF_INCLUDE_DIRS})
#list(APPEND libs ${PROTOBUF_LIBRARIES})

#protobuf_generate_cpp(PROTO_CPP PROTO_H
#    src/proto/messages_robocup_ssl_detection.proto
#    src/proto/messages_robocup_ssl_geometry.proto
#    src/proto/messages_robocup_ssl_wrapper.proto
#    src/proto/messages_robocup_ssl_refbox_log.proto
#    src/proto/grSim_Replacement.proto
#    src/proto/grSim_Commands.proto
#    src/proto/grSim_Packet.proto
#)


qt4_add_resources(msl_simulator_RESOURCES
    resources/textures.qrc
)

set(msl_simulator_RESOURCES
    ${msl_simulator_RESOURCES}
    resources/grsim.rc
)

## Declare a cpp library
# add_library(msl_simulator
#   src/${PROJECT_NAME}/msl_simulator.cpp
# )

set(msl_simulator_SOURCES src/main.cpp
    src/mainwindow.cpp
    src/glwidget.cpp
    src/graphics.cpp
    src/physics/pworld.cpp
    src/physics/pobject.cpp
    src/physics/pball.cpp
    src/physics/pground.cpp
    src/physics/pfixedbox.cpp
    src/physics/pcylinder.cpp
    src/physics/pbox.cpp
    src/physics/pray.cpp
    #src/net/robocup_ssl_server.cpp
    src/net/MSLSimulatorROSCommunication.cpp
    #src/net/robocup_ssl_client.cpp
    src/sslworld.cpp
    src/robot.cpp
    src/configwidget.cpp
    src/statuswidget.cpp
    src/logger.cpp
    src/robotwidget.cpp
    src/getpositionwidget.cpp)

set(msl_simulator_HEADERS
    include/mainwindow.h
    include/glwidget.h
    include/graphics.h
    include/physics/pworld.h
    include/physics/pobject.h
    include/physics/pball.h
    include/physics/pground.h
    include/physics/pfixedbox.h
    include/physics/pcylinder.h
    include/physics/pbox.h
    include/physics/pray.h
    #include/net/robocup_ssl_server.h
    #include/net/robocup_ssl_client.h
    include/net/MSLSimulatorROSCommunication.h
    include/sslworld.h
    include/robot.h
    include/configwidget.h
    include/statuswidget.h
    include/logger.h
    include/robotwidget.h
    include/getpositionwidget.h
    include/common.h
    include/config.h;
)

file(GLOB msl_simulator_CONFIG_FILES "config/*.ini")
set_source_files_properties(${msl_simulator_CONFIG_FILES}  PROPERTIES MACOSX_PACKAGE_LOCATION "config")



## Declare a cpp executable
add_executable(msl_simulator     
    ${msl_simulator_SOURCES}
    ${msl_simulator_CONFIG_FILES}
    #${PROTO_CPP}
    #${PROTO_H}
    ${msl_simulator_HEADERS}
    ${msl_simulator_RESOURCES};
)

target_link_libraries(msl_simulator ${libs} ${catkin_LIBRARIES} )
add_dependencies(msl_simulator ${PROJECT_NAME}_generate_messages_cpp msl_sensor_msgs msl_actuator_msgs )

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(msl_simulator_node msl_simulator_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(msl_simulator_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS msl_simulator msl_simulator_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_msl_simulator.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
