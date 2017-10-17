cmake_minimum_required(VERSION 2.8.3)
project(msl_visualization)
  
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
if (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")
	set(CMAKE_CXX_COMPILER_ARG1 "-std=c++11" CACHE STRING "C++ version for eclipse" FORCE)
	set(CMAKE_ECLIPSE_VERSION "4.5" CACHE STRING "Eclipse version" FORCE)
	add_definitions (-DCMAKE_ECLIPSE_GENERATE_SOURCE_PROJECT=TRUE)
endif (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")

SET( BASE_DIR ${CMAKE_SOURCE_DIR} )
SET( CAMBADA_CONFIG_DIR ${BASE_DIR}/config )
SET( CAMBADA_SRC_DIR ${BASE_DIR}/src )

# Define where to find modules
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules CACHE INTERNAL "" FORCE)

find_package(catkin REQUIRED COMPONENTS
  alica_engine
  system_config
  roscpp
  message_generation
  msl_msgs
  msl_helper_msgs
  msl_sensor_msgs
  msl_robot_id)
  
find_package(XSD REQUIRED)
find_package(XERCESC REQUIRED)
find_package(EIGEN3 REQUIRED)
find_package(VTK REQUIRED)

# by default only QtCore and QtGui modules are enabled
# other modules must be enabled like this:
#SET( QT_USE_QT3SUPPORT TRUE )   
SET( QT_USE_QTXML TRUE )
#find_package( Qt4 REQUIRED COMPONENTS QtCore QtNetwork QtOpenGL QtGui)
find_package(Qt5Gui REQUIRED)
find_package(Qt5Network REQUIRED)
find_package(Qt5OpenGL REQUIRED)
find_package(Qt5Core REQUIRED)
#INCLUDE( ${QT_USE_FILE} ) # adds some useful macros and variables, that contains a path to CMake script
SET( QT_WRAP_CPP        TRUE )
SET( QT_USE_QTXML       TRUE )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES msl_visualization
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)


INCLUDE_DIRECTORIES(
        .
        ${catkin_INCLUDE_DIRS}
        # Workaround for hxx/cxx files
        ${CAMBADA_CONFIG_DIR}
	src
        # Libraries
	src/libs/libtcod-1.5.1/include
        src/libs/pman
 )

SET( pman_SRC # this is a variable
        src/libs/pman/pman.c
        src/libs/pman/sem_utils.c
)

ADD_LIBRARY( pman ${pman_SRC} )
TARGET_LINK_LIBRARIES( pman )
set_target_properties( pman PROPERTIES COMPILE_FLAGS "-fPIC" )


add_custom_command(
        OUTPUT SetPieces.cxx
        OUTPUT SetPieces.hxx
        COMMAND ${XSD_EXECUTABLE}
        ARGS cxx-tree --generate-serialization ${CAMBADA_CONFIG_DIR}/SetPieces.xsd
        VERBATIM
)


include(${VTK_USE_FILE})

INCLUDE_DIRECTORIES(
        .
        src/tools/basestation/FieldWidget
        src/tools/basestation/FullWindow
        src/tools/basestation/MainWindow
        src/tools/basestation/RefBoxWidget
        src/tools/basestation/RobotInfoWidget
        src/tools/basestation/RobotWidget
)

INCLUDE_DIRECTORIES( ${CMAKE_CURRENT_BINARY_DIR} )

SET( BASESTATION_SRCS 
        src/tools/basestation/FieldWidget/FieldWidget3D.cpp
        src/tools/basestation/FieldWidget/QVTKWidget3.cpp
        src/tools/basestation/FieldWidget/RobotInfo.cpp
        src/tools/basestation/RobotVisualization/RobotVisualization.cpp
        src/tools/basestation/FullWindow/FullWindow.cpp
        src/tools/basestation/MainWindow/MainWindow.cpp
        src/tools/basestation/RefBoxWidget/RefBoxWidget.cpp
        src/tools/basestation/RobotInfoWidget/RobotInfoWidget.cpp
        src/tools/basestation/RobotWidget/RobotWidget.cpp
        src/tools/basestation/main.cpp
)
# another list, this time it includes all header files that should be treated with moc
SET( BASESTATION_MOC_HDRS
        src/tools/basestation/FieldWidget/FieldWidget3D.h
        src/tools/basestation/FieldWidget/QVTKWidget3.h
        src/tools/basestation/FieldWidget/RobotInfo.h
        src/tools/basestation/RobotVisualization/RobotVisualization.h
        src/tools/basestation/FullWindow/FullWindow.h
        src/tools/basestation/MainWindow/MainWindow.h
        src/tools/basestation/RefBoxWidget/RefBoxWidget.h
#       src/tools/basestation/RefBoxWidget/RefBoxXML.h
        src/tools/basestation/RobotInfoWidget/RobotInfoWidget.h
        src/tools/basestation/RobotWidget/RobotWidget.h
)

# some .ui files
SET( BASESTATION_UIS
        src/tools/basestation/FullWindow/FullWindow.ui
        src/tools/basestation/MainWindow/MainWindow.ui
        src/tools/basestation/RefBoxWidget/RefBoxWG.ui
        src/tools/basestation/RobotInfoWidget/RobotInfoWG.ui
        src/tools/basestation/RobotWidget/robotwidget.ui
)

# and finally an resource file
SET( BASESTATION_RCS
        src/tools/basestation/basestation.qrc
)

QT5_WRAP_UI( BASESTATION_UI_HDRS ${BASESTATION_UIS} ) #Generate headers using QT UI files
QT5_WRAP_CPP( BASESTATION_MOC_SRCS ${BASESTATION_MOC_HDRS} ) #Generate CPP files from moh headers
QT5_ADD_RESOURCES( BASESTATION_RC_SRCS ${BASESTATION_RCS} )

#ADD_EXECUTABLE( basestation EXCLUDE_FROM_ALL
ADD_EXECUTABLE( basestation 
 ${BASESTATION_SRCS} 
 ${BASESTATION_MOC_SRCS} 
 ${BASESTATION_RC_SRCS} 
 ${BASESTATION_UI_HDRS} 
)

TARGET_LINK_LIBRARIES( basestation 
 ${QT_LIBRARIES} 
 ${Qt5Network_LIBRARIES}
 ${Qt5Core_LIBRARIES}
 ${Qt5Gui_LIBRARIES}
 ${Qt5OpenGL_LIBRARIES}
 ${catkin_LIBRARIES}
# ${LIBXMLPP_LIBRARY} 
 #rtdb 
 xerces-c 
 #comm 
 pthread
 glut
 GL
 z
# QVTK
${VTK_LIBRARIES}
)

add_dependencies(basestation msl_msgs_generate_messages_cpp)

ADD_CUSTOM_TARGET( tools DEPENDS
 basestation
)

