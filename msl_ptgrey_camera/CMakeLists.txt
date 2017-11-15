cmake_minimum_required(VERSION 2.8.3)
project(msl_ptgrey_camera)

## Use c++ 11x std
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
if (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")
	set(CMAKE_CXX_COMPILER_ARG1 "-std=c++11" CACHE STRING "C++ version for eclipse" FORCE)
	set(CMAKE_ECLIPSE_VERSION "4.5" CACHE STRING "Eclipse version" FORCE)
	add_definitions (-DCMAKE_ECLIPSE_GENERATE_SOURCE_PROJECT=TRUE)
endif (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")


INCLUDE_DIRECTORIES("/usr/include/flycapture") ## TODO: write find module

find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS system_config)

#set(flycapture_DIR
	#/usr/include/flycapture
#)

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES msl_ptgrey_camera
  CATKIN_DEPENDS system_config flycapture
)


include_directories(include ${catkin_INCLUDE_DIRS})


#add_library(msl_ptgrey_camera
#  src/MSLPtGreyCamera.cpp
#  src/CamPropertySetter.cpp
#  src/CNImage.cpp
#  include/msl_ptgrey_camera/MSLPtGreyCamera.h
#)



#add_library(system_config
#  src/MSLPtGreyCamera.cpp
#  src/CamPropertySetter.cpp
#  src/CNImage.cpp
#  include/msl_ptgrey_camera/MSLPtGreyCamera.h
#)

set(sources
  src/MSLPtGreyCamera.cpp
 src/CamPropertySetter.cpp
 src/CNImage.cpp
src/Recorder.cpp

)


add_executable(msl_ptgrey_camera ${sources})

target_link_libraries(msl_ptgrey_camera
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  flycapture
)


