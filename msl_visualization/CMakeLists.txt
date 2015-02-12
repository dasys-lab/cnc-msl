cmake_minimum_required(VERSION 2.8.3)
project(msl_visualization)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

SET( BASE_DIR ${CMAKE_SOURCE_DIR} )
SET( CAMBADA_CONFIG_DIR ${BASE_DIR}/config )
SET( CAMBADA_SRC_DIR ${BASE_DIR}/src )

# Define where to find modules
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules CACHE INTERNAL "" FORCE)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  alica_engine
  system_config
  roscpp
  message_generation
  msl_msgs)
find_package(XSD REQUIRED)
find_package(XERCESC REQUIRED)
find_package(EIGEN3 REQUIRED)
find_package(VTK REQUIRED)


# by default only QtCore and QtGui modules are enabled
# other modules must be enabled like this:
#SET( QT_USE_QT3SUPPORT TRUE )   
SET( QT_USE_QTXML TRUE )
find_package( Qt4               REQUIRED COMPONENTS QtCore QtNetwork QtOpenGL QtGui)
INCLUDE( ${QT_USE_FILE} ) # adds some useful macros and variables, that contains a path to CMake script
SET( QT_WRAP_CPP        TRUE )
SET( QT_USE_QTXML       TRUE )

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#find_package(Doxygen)
#IF(DOXYGEN_FOUND)
#  ADD_CUSTOM_TARGET(doc ${DOXYGEN_EXECUTABLE} ${CMAKE_SOURCE_DIR}/Doxyfile
#    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
#    COMMENT "Generating documentation (Doxygen)...")
#ENDIF(DOXYGEN_FOUND)

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
#add_message_files(
#    DIRECTORY msg FILES
#    RefereeBoxControlBody.msg
#    RefereeBoxInfoBody.msg
#    RefereeBoxSate.msg
# )

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
#generate_messages(
#   DEPENDENCIES
#   msl_msgs  # Or other packages containing msgs
# )

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
#  INCLUDE_DIRS include
#  LIBRARIES msl_visualization
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)


INCLUDE_DIRECTORIES(
        .
        
        # Workaround for hxx/cxx files
        ${CAMBADA_CONFIG_DIR}
	src
        # Libraries
	src/libs/libtcod-1.5.1/include
        src/libs/pman
        #src/libs/rtdb

        src/comm                                    # Comm
        src/util                                    # Utilities
        src/geom                                    # Geometry
        
        # Agent
        src/agent/
        src/agent/loc                               # loc
        src/agent/world                             # worldstate
        src/agent/behaviours                        # behaviours
        src/agent/roles                             # roles
        src/agent/strategy                          # strategy

        # Integrator
        src/agent/integrator/localization           # subdirectory localization
        src/agent/integrator/filters                # subdirectory filters
        src/agent/integrator                        # integrator
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

## Declare a cpp library
# add_library(msl_visualization
#   src/${PROJECT_NAME}/msl_visualization.cpp
# )

## Declare a cpp executable
# add_executable(msl_visualization_node src/msl_visualization_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes

## Specify libraries to link a library or executable target against
# target_link_libraries(msl_visualization_node
#   ${catkin_LIBRARIES}
# )

SET( pman_SRC # this is a variable
        src/libs/pman/pman.c
        src/libs/pman/sem_utils.c
)

ADD_LIBRARY( pman ${pman_SRC} )
TARGET_LINK_LIBRARIES( pman cambada_util )
set_target_properties( pman PROPERTIES COMPILE_FLAGS "-fPIC" )


#ADD_LIBRARY( rtdb src/libs/rtdb/rtdb_api.c )
##SET_TARGET_PROPERTIES( rtdb PROPERTIES LINKER_LANGUAGE C)
#SET_TARGET_PROPERTIES( rtdb PROPERTIES COMPILE_FLAGS "-fPIC" )

#SET( RTDB_USER_H_FILE src/libs/rtdb/rtdb_user.h )

## For XRTDB CFLAGS: Include all directories inside source dir AND in include_directories
#SET( RTDB_INCLUDE_DIRS )
#GET_PROPERTY(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
#STRING(LENGTH "${CMAKE_SOURCE_DIR}" srcdir_len) # get cmake source dir string length
#FOREACH(dir ${dirs})
#        STRING( LENGTH "${dir}" dir_len)
#        IF( ${dir_len} GREATER ${srcdir_len} OR ${dir_len} EQUAL ${srcdir_len} )
#                STRING(SUBSTRING ${dir} 0 ${srcdir_len} cmpare_str)
#                IF( ${cmpare_str} STREQUAL ${CMAKE_SOURCE_DIR} )
#                        SET( RTDB_INCLUDE_DIRS "${RTDB_INCLUDE_DIRS} -I${dir}")
#                ENDIF()
#        ENDIF()
#ENDFOREACH()

#CONFIGURE_FILE(
#        src/libs/rtdb/parser/rtdb_configuration.h.in
#        ${CMAKE_CURRENT_BINARY_DIR}/rtdb_configuration.h
#)
#
#SET( rtdbparser_SRC
#        src/libs/rtdb/parser/xrtdb.c
#        src/libs/rtdb/parser/rtdb_errors.c
#        src/libs/rtdb/parser/rtdb_functions.c
 #       src/libs/rtdb/parser/rtdb_user_creator.c
#        src/libs/rtdb/parser/rtdb_ini_creator.c
#        
#        src/libs/rtdb/parser/xrtdb.tab.c
#)

#include_directories( ${CMAKE_CURRENT_BINARY_DIR} )

#ADD_EXECUTABLE ( xrtdb ${rtdbparser_SRC} )


# Create cambada.conf.cxx/hxx
add_custom_command(
        OUTPUT cambada.conf.cxx
        OUTPUT cambada.conf.hxx
        COMMAND ${XSD_EXECUTABLE}
        ARGS cxx-tree --generate-serialization ${CAMBADA_CONFIG_DIR}/cambada.conf.xsd
        VERBATIM
)

# Create SetPieces.cxx/hxx
add_custom_command(
        OUTPUT SetPieces.cxx
        OUTPUT SetPieces.hxx
        COMMAND ${XSD_EXECUTABLE}
        ARGS cxx-tree --generate-serialization ${CAMBADA_CONFIG_DIR}/SetPieces.xsd
        VERBATIM
)

# sources for utils library
SET( util_SRC
        #src/util/cambada.conf.cxx
        #src/util/SetPieces.cxx
        src/util/PID.cpp
        src/util/Clock.cpp
        #src/util/ConfigXML.cpp
        src/util/LinRegression.cpp
        src/util/Param.cpp
        src/util/SharedTimer.cpp
        src/util/SlidingWindow.cpp
        src/util/Timer.cpp
        src/util/KickerConf.cpp
        #src/util/HeightMap.cpp
        src/util/ClippedRamp.cpp
        
        # Utilities for WorldState
        src/util/EgoMotionEstimator.cpp
        src/util/BallPositionKalman.cpp
        src/util/BallPositionParticle.cpp
        src/util/VelocityRegression.cpp
        src/util/LinearRegression.cpp
        src/util/StuckRobotSensor.cpp
)

ADD_LIBRARY( cambada_util ${util_SRC} )
set_target_properties( cambada_util PROPERTIES COMPILE_FLAGS "-fPIC" )




SET( geom_SRC
        src/geom/Angle.cc
        src/geom/Vec.cc
        src/geom/geometry.cc
        src/geom/Frame2D.cc
)

ADD_LIBRARY( geom ${geom_SRC} )
set_target_properties( geom PROPERTIES COMPILE_FLAGS "-fPIC" )


#ADD_LIBRARY( comm src/comm/MulticastSocket.cpp )
#SET_TARGET_PROPERTIES( comm PROPERTIES COMPILE_FLAGS "-fPIC" )
#
#
#SET( comm_SRC
#        src/comm/multicast.cpp
#        src/comm/comm.cpp
#)
#
#set ( comm_OBJ cambadaComm )
#ADD_EXECUTABLE ( ${comm_OBJ} ${comm_SRC} )
#TARGET_LINK_LIBRARIES( ${comm_OBJ} rtdb pthread comm util )
#SET_TARGET_PROPERTIES( ${comm_OBJ} PROPERTIES OUTPUT_NAME comm )


include(${QT_USE_FILE})
include(${VTK_USE_FILE})

INCLUDE_DIRECTORIES(
        .
        src/tools/basestation/FieldWidget
        src/tools/basestation/FullInfoWindow
        src/tools/basestation/FullWindow
        src/tools/basestation/LogWidget
        src/tools/basestation/MainWindow
        src/tools/basestation/RefBoxWidget
        src/tools/basestation/RobotInfoWidget
        src/tools/basestation/RobotWidget
        src/tools/basestation/UpdateWidget
)

INCLUDE_DIRECTORIES( ${CMAKE_CURRENT_BINARY_DIR} )

SET( BASESTATION_SRCS 
        src/tools/basestation/FieldWidget/FieldWidget.cpp
        src/tools/basestation/FieldWidget/FieldWidget3D.cpp
        src/tools/basestation/FieldWidget/RobotInfo.cpp
        src/tools/basestation/RobotVisualization/RobotVisualization.cpp
        src/tools/basestation/FullInfoWindow/FullInfoWindow.cpp
        src/tools/basestation/FullWindow/FullWindow.cpp
        src/tools/basestation/LogWidget/LogWidget.cpp
        src/tools/basestation/MainWindow/MainWindow.cpp
        src/tools/basestation/RefBoxWidget/RefBoxDialog.cpp
        src/tools/basestation/RefBoxWidget/RefBoxWidget.cpp
#       src/tools/basestation/RefBoxWidget/RefBoxXML.cpp
        src/tools/basestation/RobotInfoWidget/RobotInfoWidget.cpp
        src/tools/basestation/RobotWidget/RobotDialog.cpp
        src/tools/basestation/RobotWidget/RobotWidget.cpp
        src/tools/basestation/UpdateWidget/UpdateWidget.cpp
        src/tools/basestation/main.cpp
)
# another list, this time it includes all header files that should be treated with moc
SET( BASESTATION_MOC_HDRS
        src/tools/basestation/FieldWidget/FieldWidget.h
        src/tools/basestation/FieldWidget/FieldWidget3D.h
        src/tools/basestation/FieldWidget/RobotInfo.h
        src/tools/basestation/RobotVisualization/RobotVisualization.h
        src/tools/basestation/FullInfoWindow/FullInfoWindow.h
        src/tools/basestation/FullWindow/FullWindow.h
        src/tools/basestation/LogWidget/LogWidget.h
        src/tools/basestation/MainWindow/MainWindow.h
        src/tools/basestation/RefBoxWidget/RefBoxDialog.h
        src/tools/basestation/RefBoxWidget/RefBoxWidget.h
#       src/tools/basestation/RefBoxWidget/RefBoxXML.h
        src/tools/basestation/RobotInfoWidget/RobotInfoWidget.h
        src/tools/basestation/RobotWidget/RobotDialog.h
        src/tools/basestation/RobotWidget/RobotWidget.h
        src/tools/basestation/UpdateWidget/UpdateWidget.h
)

# some .ui files
SET( BASESTATION_UIS
        src/tools/basestation/FullInfoWindow/FullInfoWindow_WG.ui
        src/tools/basestation/FullWindow/FullWindow.ui
        src/tools/basestation/LogWidget/LogWidget.ui
        src/tools/basestation/MainWindow/MainWindow.ui
        src/tools/basestation/RefBoxWidget/RefBoxDialog.ui
        src/tools/basestation/RefBoxWidget/RefBoxWG.ui
        src/tools/basestation/RobotInfoWidget/RobotInfoWG.ui
        src/tools/basestation/RobotWidget/RobotDialog.ui
        src/tools/basestation/RobotWidget/robotwidget.ui
)

# and finally an resource file
SET( BASESTATION_RCS
        src/tools/basestation/basestation.qrc
)

QT4_WRAP_UI( BASESTATION_UI_HDRS ${BASESTATION_UIS} ) #Generate headers using QT UI files
QT4_WRAP_CPP( BASESTATION_MOC_SRCS ${BASESTATION_MOC_HDRS} ) #Generate CPP files from moh headers
QT4_ADD_RESOURCES( BASESTATION_RC_SRCS ${BASESTATION_RCS} )

#ADD_EXECUTABLE( basestation EXCLUDE_FROM_ALL
ADD_EXECUTABLE( basestation 
 ${BASESTATION_SRCS} 
 ${BASESTATION_MOC_SRCS} 
 ${BASESTATION_RC_SRCS} 
 ${BASESTATION_UI_HDRS} 
)

TARGET_LINK_LIBRARIES( basestation 
 ${QT_LIBRARIES} 
 ${catkin_LIBRARIES}
# ${LIBXMLPP_LIBRARY} 
 cambada_util 
 #rtdb 
 worldstate 
 geom 
 xerces-c 
 #comm 
 pthread
 glut
 GL
 z
 QVTK
)

add_dependencies(basestation msl_msgs_generate_messages_cpp)

ADD_CUSTOM_TARGET( tools DEPENDS
 basestation
)

SET( worldstate_SRC # this is a variable
src/agent/world/Field.cpp
src/agent/world/Obstacle.cpp
src/agent/world/Robot.cpp
src/agent/world/Sonar.cpp
src/agent/world/Ball.cpp
src/agent/world/VisionInfo.cpp
src/agent/world/LowLevelInfo.cpp
src/agent/world/WorldState.cpp
src/agent/world/Compass.cpp
src/agent/world/Zones.cpp
src/agent/world/CoachInfo.cpp
src/agent/world/SystemInfo.cpp
)
ADD_LIBRARY( worldstate ${worldstate_SRC} )
TARGET_LINK_LIBRARIES( worldstate cambada_util geom )
set_target_properties( worldstate PROPERTIES COMPILE_FLAGS "-fPIC" )

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
# install(TARGETS msl_visualization msl_visualization_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_msl_visualization.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
