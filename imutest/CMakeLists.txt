cmake_minimum_required(VERSION 2.8.3)
project(imutest)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)


find_package(LibFTDI1 NO_MODULE REQUIRED)
include (${LIBFTDI_USE_FILE})

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
#  LIBRARIES imutest
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

## Your package locations should be listed before other locations
# include_directories(include)

## Declare a C++ executable
add_executable(imutest src/test.c)
target_link_libraries(imutest ${LIBFTDI_LIBRARIES})
