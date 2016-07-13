cmake_minimum_required(VERSION 2.8.3)
project(rt_imu_lib_test)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

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
  LIBRARIES RTIMULib
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

INCLUDE_DIRECTORIES(include)
INCLUDE(${CMAKE_CURRENT_SOURCE_DIR}/RTIMULibVersion.txt)

SET(LIBRTIMU_SRCS
    src/RTFusion.cpp
    src/RTFusionRTQF.cpp
    src/RTMath.cpp
    src/RTFusionKalman4.cpp
    src/RTIMUAccelCal.cpp
    src/RTIMUHal.cpp
    src/RTIMUMagCal.cpp
    src/RTIMUSettings.cpp
    src/IMUDrivers/RTIMU.cpp
    src/IMUDrivers/RTIMUGD20M303DLHC.cpp
    src/IMUDrivers/RTIMUGD20HM303DLHC.cpp
    src/IMUDrivers/RTIMUGD20HM303D.cpp
    src/IMUDrivers/RTIMULSM9DS0.cpp
    src/IMUDrivers/RTIMULSM9DS1.cpp
    src/IMUDrivers/RTIMUMPU9150.cpp
    src/IMUDrivers/RTIMUMPU9250.cpp
    src/IMUDrivers/RTIMUBMX055.cpp
    src/IMUDrivers/RTIMUBNO055.cpp
    src/IMUDrivers/RTIMUNull.cpp
    src/IMUDrivers/RTPressure.cpp
    src/IMUDrivers/RTPressureBMP180.cpp
    src/IMUDrivers/RTPressureLPS25H.cpp
    src/IMUDrivers/RTPressureMS5611.cpp
    src/IMUDrivers/RTPressureMS5637.cpp
    src/IMUDrivers/RTHumidity.cpp
    src/IMUDrivers/RTHumidityHTS221.cpp
    src/IMUDrivers/RTHumidityHTU21D.cpp
)

ADD_LIBRARY(RTIMULib SHARED ${LIBRTIMU_SRCS})
SET_PROPERTY(TARGET RTIMULib PROPERTY VERSION ${RTIMULIB_VERSION})
SET_PROPERTY(TARGET RTIMULib PROPERTY SOVERSION ${RTIMULIB_VERSION_MAJOR})
INSTALL(TARGETS RTIMULib DESTINATION lib)
INSTALL(DIRECTORY . DESTINATION include FILES_MATCHING PATTERN "*.h")

SET(DRIVE_SRCS src/RTIMULibDrive.cpp)
ADD_EXECUTABLE(RTIMULibDrive ${DRIVE_SRCS})
TARGET_LINK_LIBRARIES(RTIMULibDrive RTIMULib)
INSTALL(TARGETS RTIMULibDrive DESTINATION bin)

SET(CAL_SRCS src/RTIMULibCal.cpp)
ADD_EXECUTABLE(RTIMULibCal ${CAL_SRCS})
TARGET_LINK_LIBRARIES(RTIMULibCal RTIMULib)
INSTALL(TARGETS RTIMULibCal DESTINATION bin)

SET(CAL_SRCS src/RTIMULibSpeed.cpp)
ADD_EXECUTABLE(RTIMULibSpeed ${CAL_SRCS})
TARGET_LINK_LIBRARIES(RTIMULibSpeed RTIMULib)
INSTALL(TARGETS RTIMULibSpeed DESTINATION bin)