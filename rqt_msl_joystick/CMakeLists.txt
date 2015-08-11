cmake_minimum_required(VERSION 2.8.0)

project(rqt_msl_joystick)

## Use c++ 11x std
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS roscpp message_generation msl_msgs system_config)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
find_package(Qt4 4.4.3 COMPONENTS QtCore QtGui REQUIRED)

include(${QT_USE_FILE})

set(rqt_msl_joystick_SRCS
  src/rqt_msl_joystick/Joystick.cpp
)

set(rqt_msl_joystick_HDRS
  include/rqt_msl_joystick/Joystick.h
)

set(rqt_msl_joystick_UIS
  ui/rqt_msl_joystick/Joystick.ui
)

set(rqt_msl_joystick_INCLUDE_DIRECTORIES
  include
  ${CMAKE_CURRENT_BINARY_DIR}
)

catkin_package(
  INCLUDE_DIRS ${rqt_msl_joystick_INCLUDE_DIRECTORIES}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS rqt_gui rqt_gui_cpp system_config
)
catkin_python_setup()

qt4_wrap_cpp(rqt_msl_joystick_MOCS ${rqt_msl_joystick_HDRS})
qt4_wrap_ui(rqt_msl_joystick_UIS_H ${rqt_msl_joystick_UIS})

include_directories(${rqt_msl_joystick_INCLUDE_DIRECTORIES} ${catkin_INCLUDE_DIRS})
add_library(${PROJECT_NAME} ${rqt_msl_joystick_SRCS} ${rqt_msl_joystick_MOCS} ${rqt_msl_joystick_UIS_H})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})

find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME})

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(PROGRAMS scripts/rqt_msl_joystick
  DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(PROGRAMS scripts/rqt_msl_joystick
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
