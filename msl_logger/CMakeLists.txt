cmake_minimum_required(VERSION 2.8.3)
project(msl_logger)

## Use c++ 11x std
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
if (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")
	set(CMAKE_CXX_COMPILER_ARG1 "-std=c++11" CACHE STRING "C++ version for eclipse" FORCE)
	set(CMAKE_ECLIPSE_VERSION "4.5" CACHE STRING "Eclipse version" FORCE)
	add_definitions (-DCMAKE_ECLIPSE_GENERATE_SOURCE_PROJECT=TRUE)
endif (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")

find_package(catkin REQUIRED msl_worldmodel system_config)

set(CMAKE_CXX_FLAGS "-rdynamic -fnon-call-exceptions -ggdb ${CMAKE_CXX_FLAGS}")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES msl_logger
  CATKIN_DEPENDS msl_worldmodel system_config
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(msl_logger
 src/Logger.cpp
)

# not needed???
#add_dependencies (msl_logger
# msl_worldmodel
#)

add_executable(msl_logger_test
 src/main.cpp
)

target_link_libraries(msl_logger_test msl_logger)
target_link_libraries(msl_logger ${catkin_LIBRARIES})