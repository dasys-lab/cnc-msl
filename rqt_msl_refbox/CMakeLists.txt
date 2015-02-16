cmake_minimum_required(VERSION 2.8.3)

project(rqt_msl_refbox)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS rqt_gui rqt_gui_cpp)

find_package(Qt4 COMPONENTS QtCore QtGui QtNetwork REQUIRED)

include(${QT_USE_FILE})

set(rqt_msl_refbox_SRCS
  src/rqt_msl_refbox/RefBox.cpp
  src/rqt_msl_refbox/RefBoxCommunication.cpp
)

set(rqt_msl_refbox_HDRS
  include/rqt_msl_refbox/RefBox.h
  include/rqt_msl_refbox/RefBoxCommunication.h
)

set(rqt_msl_refbox_UIS
  src/rqt_msl_refbox/RefBox.ui
  src/rqt_msl_refbox/RefBoxCommunication.ui
)

set(rqt_msl_refbox_INCLUDE_DIRECTORIES
  include
  ${CMAKE_CURRENT_BINARY_DIR}
)

catkin_package(
  INCLUDE_DIRS ${rqt_msl_refbox_INCLUDE_DIRECTORIES}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS rqt_gui rqt_gui_cpp
)
catkin_python_setup()

qt4_wrap_cpp(rqt_msl_refbox_MOCS ${rqt_msl_refbox_HDRS})
qt4_wrap_ui(rqt_msl_refbox_UIS_H ${rqt_msl_refbox_UIS})

include_directories(${rqt_msl_refbox_INCLUDE_DIRECTORIES} ${catkin_INCLUDE_DIRS})
add_library(${PROJECT_NAME} ${rqt_msl_refbox_SRCS} ${rqt_msl_refbox_MOCS} ${rqt_msl_refbox_UIS_H})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY} ${QT_QTNETWORK_LIBRARY})

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

install(PROGRAMS scripts/rqt_msl_refbox
  DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(PROGRAMS scripts/rqt_msl_refbox
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
