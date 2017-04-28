cmake_minimum_required(VERSION 2.8.3)

project(msl_refbox)

## Use c++ 11x std
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
if (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")
	set(CMAKE_CXX_COMPILER_ARG1 "-std=c++11" CACHE STRING "C++ version for eclipse" FORCE)
	set(CMAKE_ECLIPSE_VERSION "4.5" CACHE STRING "Eclipse version" FORCE)
	add_definitions (-DCMAKE_ECLIPSE_GENERATE_SOURCE_PROJECT=TRUE)
endif (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS rqt_gui rqt_gui_cpp msl_msgs msl_sensor_msgs alica_ros_proxy)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(Qt5 REQUIRED Core Gui Network Widgets)
set(QT_LIBRARIES Qt5::Gui Qt5::Network Qt5::Widgets)

set(msl_refbox_SRCS
  src/msl_refbox/RefBox.cpp
  src/msl_refbox/GameData.cpp
  src/msl_refbox/tinyxml2.cpp
  src/msl_refbox/XMLProtocolParser.cpp
  )

set(msl_refbox_HDRS
  include/msl_refbox/RefBox.h
  include/msl_refbox/GameData.h
  include/msl_refbox/tinyxml2.h
  include/msl_refbox/XMLProtocolParser.h
)

set(msl_refbox_UIS
  src/msl_refbox/RefBox.ui
)

set(msl_refbox_INCLUDE_DIRECTORIES
  include
  ${CMAKE_CURRENT_BINARY_DIR}
)

catkin_package(
  INCLUDE_DIRS ${msl_refbox_INCLUDE_DIRECTORIES}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS rqt_gui rqt_gui_cpp
)
catkin_python_setup()

qt5_wrap_cpp(msl_refbox_MOCS ${msl_refbox_HDRS})
qt5_wrap_ui(msl_refbox_UIS_H ${msl_refbox_UIS})

include_directories(${msl_refbox_INCLUDE_DIRECTORIES} ${catkin_INCLUDE_DIRS})
add_library(${PROJECT_NAME} ${msl_refbox_SRCS} ${msl_refbox_MOCS} ${msl_refbox_UIS_H})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${QT_LIBRARIES})
add_dependencies(${PROJECT_NAME} msl_msgs_generate_messages_cpp)

qt5_use_modules(${PROJECT_NAME} Gui Network)

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

install(PROGRAMS scripts/msl_refbox
  DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(PROGRAMS scripts/msl_refbox
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Test stuff

#add_executable(SimpleTCPClientExample src/msl_refbox/SimpleTCPClientExample.cpp)
