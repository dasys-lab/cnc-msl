cmake_minimum_required(VERSION 2.8.3)
project(msl_vision)
 
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake CACHE INTERNAL "" FORCE)

## Use c++ 11x std
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
if (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")
	set(CMAKE_CXX_COMPILER_ARG1 "-std=c++11" CACHE STRING "C++ version for eclipse" FORCE)
	set(CMAKE_ECLIPSE_VERSION "4.5" CACHE STRING "Eclipse version" FORCE)
	add_definitions (-DCMAKE_ECLIPSE_GENERATE_SOURCE_PROJECT=TRUE)
endif (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")

find_package(catkin REQUIRED std_msgs msl_actuator_msgs msl_sensor_msgs msl_msgs msl_helper_msgs system_config roscpp)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)
find_package(SSE)

catkin_package(
  INCLUDE_DIRS src src/filters src/helpers 
  LIBRARIES lib_msl_vision #lib_msl_vision_3d
  CATKIN_DEPENDS system_config msl_actuator_msgs msl_sensor_msgs msl_msgs msl_helper_msgs std_msgs 
  DEPENDS ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} pthread m Xv X11 Xext ICE SM dc1394 raw1394
)

include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

link_directories("/usr/X11R6/lib")

FILE(GLOB lib_msl_vision_SOURCES
	src/camControl/BasisAutoParam.cpp
	src/camControl/BasisControl.cpp
	src/camControl/Brightness.cpp
	src/camControl/GainControl.cpp
	src/camControl/PControl.cpp
	src/camControl/ReferenceArea.cpp
	src/camControl/Tools/ga.cpp
	src/camControl/Whitepoint.cpp
	src/CameraQuickCam.cpp 
	src/driver/dc1394camera.cpp 
	src/driver/flea2.cpp 
	src/driver/imagingsource.cpp 
	src/driver/sony.cpp
	src/filters/FilterAdaptiveROI.cpp 
	src/filters/FilterAdaptiveROIDirected.cpp 
	src/filters/FilterAddBallBlobsToSeg.cpp
	src/filters/FilterAdjustGain.cpp 
	src/filters/FilterAdjustGainDirected.cpp 
	src/filters/FilterAdjustGainNew.cpp 
	src/filters/FilterBox.cpp 
	src/filters/FilterCalibrationDirected.cpp 
	src/filters/Filter.cpp 
	src/filters/FilterDistanceProfile.cpp 
	src/filters/FilterDistanceProfileNew.cpp 
	src/filters/FilterDrawScanLines.cpp 
	src/filters/FilterExtractBlobs.cpp 
	src/filters/FilterExtractBlobsDirected.cpp 
#	src/filters/FilterExtractImages.cpp
#	src/filters/FilterExtractLineImage.cpp
	src/filters/FilterGrayToDarkSeg.cpp 
	src/filters/FilterHistoLin.cpp 
	src/filters/FilterHoughCalib.cpp 
	src/filters/FilterLinePointsCalib.cpp
	src/filters/FilterLinePoints.cpp 
#	src/filters/FilterLinePoints3D.cpp 
	src/filters/FilterLinePointsROI.cpp 
	src/filters/FilterLinePointsROIDirected.cpp 
	src/filters/FilterSegToRGB.cpp 
	src/filters/FilterSobelDir.cpp 
	src/filters/FilterSoftHDR.cpp 
	src/filters/FilterTemplateMatching.cpp 
	src/filters/FilterTemplateMatchingGoalie.cpp 
#	src/filters/FilterTemplateMatchingScanLines.cpp
#	src/filters/FilterTitanBraces.cpp
	src/filters/FilterYUVCountColoredDots.cpp 
	src/filters/FilterYUVExtractImages.cpp 
	src/filters/FilterYUVExtractSubImages.cpp 
	src/filters/FilterYUVQuickCamToYUV.cpp 
	src/filters/FilterYUVToGray.cpp 
#	src/filters/FilterYUV2RGB.cpp 
	src/filters/FilterYUVToRGB.cpp 
	src/filters/FilterYUVToYUVFull.cpp 
	src/helpers/BallClusterHelp.cpp 
	src/helpers/BallHelper.cpp 
	src/helpers/BallHelperDirected2.cpp 
	src/helpers/BallHelperDirectedGoalie.cpp 
#	src/helpers/BallHelperKinect.cpp
	src/helpers/BallIntegrator.cpp 
	src/helpers/BallZTracker.cpp
#	src/helpers/blobber.cpp
	src/helpers/CompassValueHelper.cpp 
	src/helpers/ConfigHelper.cpp 
	src/helpers/CorrectedOdometryHelper.cpp 
	src/helpers/CovarianceHelper.cpp 
	src/helpers/CoxLocalization.cpp 
#	src/helpers/Draw.cpp
	src/helpers/DistanceLookupHelper.cpp
#	src/helpers/Distance3DHelper.cpp
	src/helpers/EgoMotionEstimator.cpp 
	src/helpers/Environment.cpp 
	src/helpers/ErrorMinLocalization.cpp 
	src/helpers/ExtensionDirectedHelper.cpp 
	src/helpers/ExtensionHelper.cpp 
	src/helpers/FieldLut.cpp 
	src/helpers/FootballField.cpp 
	src/helpers/GoalHelperLocalization.cpp 
	src/helpers/ImageMaskHelper.cpp 
	src/helpers/KeyHelper.cpp 
	src/helpers/Line2D.cpp 
	src/helpers/LineDistanceHelper.cpp 
	src/helpers/LocalizeDebug.cpp 
	src/helpers/Logger.cpp 
	src/helpers/ObjectContainer.cpp 
	src/helpers/ObjectTracker.cpp 
	src/helpers/OpponentHelper.cpp 
	src/helpers/PacketHelper.cpp 
	src/helpers/ParticleFilter.cpp 
	src/helpers/ParticleFilterGoalie.cpp 
	src/helpers/PositionHelper.cpp 
	src/helpers/PositionHelperDirected.cpp 
	src/helpers/RandomHelper.cpp 
	src/helpers/RawOdometryHelper.cpp 
	src/helpers/Replayer.cpp 
	src/helpers/ROIHelperOmniCam.cpp 
#	src/helpers/ScanCircleHelper.cpp
#	src/helpers/ScanLineHelper3D.cpp 
	src/helpers/ScanLineHelperBall.cpp 
	src/helpers/ScanLineHelper.cpp
	src/helpers/ScanLineHelperDirected.cpp 
	src/helpers/SharedBallDirectedHelper.cpp 
	src/helpers/SharedBallHelper.cpp 
	src/helpers/SharedMemoryHelper.cpp 
	src/helpers/SpicaDirectedHelper.cpp
	src/helpers/SpicaHelper.cpp 
	src/helpers/TimeHelper.cpp
	src/helpers/ColorCalibrationHelper.cpp 
	src/helpers/CameraCalibrationHelper.cpp 
	src/XVDisplay.cpp 
)

## Vision 2D

add_library(lib_msl_vision ${lib_msl_vision_SOURCES})
target_link_libraries(lib_msl_vision stdc++ pthread m Xv X11 Xext ICE SM dc1394 raw1394 ${OpenCV_LIBS} ${catkin_LIBRARIES})
set_target_properties(lib_msl_vision PROPERTIES COMPILE_FLAGS "-DSWITCHLOC -ftree-vectorize ${SSE_FLAGS} -ffast-math -O3 -funroll-loops -fomit-frame-pointer -g -Wall -Wunused")
add_dependencies(lib_msl_vision msl_actuator_msgs_generate_messages_cpp msl_sensor_msgs_generate_messages_cpp msl_msgs_generate_messages_cpp msl_helper_msgs_generate_messages_cpp)

add_executable(msl_vision src/VisionPlayer.cpp)
target_link_libraries(msl_vision stdc++ pthread m Xv X11 Xext ICE SM dc1394 raw1394 lib_msl_vision ${Boost_LIBRARIES} ${catkin_LIBRARIES})
set_target_properties(msl_vision PROPERTIES COMPILE_FLAGS "-DSWITCHLOC -ftree-vectorize ${SSE_FLAGS} -ffast-math -O3 -funroll-loops -fomit-frame-pointer -g -Wall -Wunused")
add_dependencies(msl_vision msl_actuator_msgs_generate_messages_cpp msl_sensor_msgs_generate_messages_cpp msl_msgs_generate_messages_cpp msl_helper_msgs_generate_messages_cpp)

## Vision 3D
FILE(GLOB lib_msl_vision_3d_SOURCES
# 	src/filters/Filter.cpp
	src/filters/FilterExtractImages.cpp
	src/filters/FilterExtractLineImage.cpp
	src/filters/FilterExtractPanno.cpp
	src/filters/FilterLinePoints3D.cpp
# 	src/filters/FilterSobelDir.cpp
# 	src/filters/FilterTitanBraces.cpp
# 	src/filters/FilterYUVExtractImages.cpp
# 	src/filters/FilterYUV2RGB.cpp
# 	src/helpers/blobber.cpp
	src/helpers/Draw.cpp
	src/helpers/Distance3DHelper.cpp
# 	src/helpers/KeyHelper.cpp 
# 	src/helpers/ScanCircleHelper.cpp
	src/helpers/ScanLineHelper3D.cpp
	src/helpers/SpicaHelper.cpp
# 	src/helpers/TimeHelper.cpp
	src/XVDisplay.cpp 
)

#add_library(lib_msl_vision_3d ${lib_msl_vision_3d_SOURCES})
#target_link_libraries(lib_msl_vision_3d stdc++ pthread m Xv X11 Xext ICE SM dc1394 raw1394 ${OpenCV_LIBS} ${catkin_LIBRARIES})
#set_target_properties(lib_msl_vision_3d PROPERTIES COMPILE_FLAGS "-DSWITCHLOC -ftree-vectorize ${SSE_FLAGS} -ffast-math -O3 -funroll-loops -fomit-frame-pointer -g -Wall -Wunused")
#add_dependencies(lib_msl_vision_3d msl_actuator_msgs_generate_messages_cpp msl_sensor_msgs_generate_messages_cpp msl_msgs_generate_messages_cpp msl_helper_msgs_generate_messages_cpp)

#add_executable(msl_vision_3d src/VisionPlayer3D.cpp)
#target_link_libraries(msl_vision_3d stdc++ pthread m Xv X11 Xext ICE SM dc1394 raw1394 lib_msl_vision_3d ${Boost_LIBRARIES} ${catkin_LIBRARIES})
#set_target_properties(msl_vision_3d PROPERTIES COMPILE_FLAGS "-DSWITCHLOC -ftree-vectorize ${SSE_FLAGS} -ffast-math -O3 -funroll-loops -fomit-frame-pointer -g -Wall -Wunused")
#add_dependencies(msl_vision_3d msl_actuator_msgs_generate_messages_cpp msl_sensor_msgs_generate_messages_cpp msl_msgs_generate_messages_cpp msl_helper_msgs_generate_messages_cpp)

## Vision Calibration

add_executable(msl_vision_calib src/VisionPlayerCalib.cpp)
target_link_libraries(msl_vision_calib stdc++ pthread m Xv X11 Xext ICE SM dc1394 raw1394 lib_msl_vision ${Boost_LIBRARIES} ${catkin_LIBRARIES})
set_target_properties(msl_vision_calib PROPERTIES COMPILE_FLAGS "-DSWITCHLOC -ftree-vectorize ${SSE_FLAGS} -ffast-math -O3 -funroll-loops -fomit-frame-pointer -g -Wall -Wunused")

## Carpet Calculator
add_executable(msl_carpet_calculator src/helpers/ConfigHelper.cpp src/tools/CarpetCalculator.cpp)
target_link_libraries(msl_carpet_calculator pthread stdc++ m Xv X11 Xext ICE SM dc1394 raw1394 lib_msl_vision ${Boost_LIBRARIES} ${catkin_LIBRARIES})
set_target_properties(msl_carpet_calculator PROPERTIES COMPILE_FLAGS "-DSWITCHLOC -ftree-vectorize ${SSE_FLAGS} -ffast-math -O3 -funroll-loops -fomit-frame-pointer -g -Wall -Wunused")

## Carpe Noctem GA

add_executable(msl_cn_ga src/camControl/Tools/ga.cpp)
target_link_libraries(msl_cn_ga pthread m Xv X11 stdc++ Xext ICE SM dc1394 raw1394 lib_msl_vision ${Boost_LIBRARIES} ${catkin_LIBRARIES})
set_target_properties(msl_cn_ga PROPERTIES COMPILE_FLAGS "-DSWITCHLOC -ftree-vectorize ${SSE_FLAGS} -ffast-math -O3 -funroll-loops -fomit-frame-pointer -g -Wall -Wunused")

## Carpe Noctem CCT

add_executable(msl_cn_cct src/camControl/Tools/cct.cpp)
target_link_libraries(msl_cn_cct pthread m Xv X11 Xext ICE stdc++ SM dc1394 raw1394 lib_msl_vision ${Boost_LIBRARIES} ${catkin_LIBRARIES})
set_target_properties(msl_cn_cct PROPERTIES COMPILE_FLAGS "-DSWITCHLOC -ftree-vectorize ${SSE_FLAGS} -ffast-math -O3 -funroll-loops -fomit-frame-pointer -g -Wall -Wunused")

## Vision Goalie

#add_executable(msl_vision_goalie src/VisionPlayerGoalie.cpp)
#target_link_libraries(msl_vision_goalie pthread stdc++ Xv m X11 Xext ICE SM dc1394 raw1394 lib_msl_vision ${Boost_LIBRARIES} ${catkin_LIBRARIES})
#set_target_properties(msl_vision_goalie PROPERTIES COMPILE_FLAGS "-DSWITCHLOC -ftree-vectorize ${SSE_FLAGS} -ffast-math -O3 -funroll-loops -fomit-frame-pointer -g -Wall -Wunused")

## Vision Goalie 1394

#add_executable(msl_vision_goalie_1394 src/VisionPlayerGoalie1394.cpp)
#target_link_libraries(msl_vision_goalie_1394 stdc++ pthread Xv m X11 Xext ICE SM dc1394 raw1394 lib_msl_vision ${Boost_LIBRARIES} ${catkin_LIBRARIES})
#set_target_properties(msl_vision_goalie_1394 PROPERTIES COMPILE_FLAGS "-DSWITCHLOC -ftree-vectorize ${SSE_FLAGS} -ffast-math -O3 -funroll-loops -fomit-frame-pointer -g -Wall -Wunused")

## Vision Kinect

#add_executable(msl_vision_kinect src/VisionPlayerKinect.cpp)
#target_link_libraries(msl_vision_kinect pthread stdc++ m Xv X11 Xext ICE SM dc1394 raw1394 lib_msl_vision ${Boost_LIBRARIES} ${catkin_LIBRARIES})
#set_target_properties(msl_vision_kinect PROPERTIES COMPILE_FLAGS "-DSWITCHLOC -ftree-vectorize ${SSE_FLAGS} -ffast-math -O3 -funroll-loops -fomit-frame-pointer -g -Wall -Wunused")

## Camera Model Test

#add_executable(camera_model_test src/CameraModelTest.cpp)
#target_link_libraries(camera_model_test pthread stdc++ m X11 Xv Xext ICE SM dc1394 raw1394 lib_msl_vision ${Boost_LIBRARIES} ${catkin_LIBRARIES})
#set_target_properties(camera_model_test PROPERTIES COMPILE_FLAGS "-DSWITCHLOC  -ftree-vectorize ${SSE_FLAGS} -ffast-math -O3 -funroll-loops -fomit-frame-pointer -g -Wall -Wunused")

## Calculator

add_executable(msl_calculator src/helpers/ConfigHelper.cpp src/tools/Calculator.cpp src/tools/distcalc.cpp src/tools/Calculator.cpp)
target_link_libraries(msl_calculator pthread m X11 Xv Xext ICE stdc++ SM dc1394 raw1394 lib_msl_vision ${Boost_LIBRARIES} ${catkin_LIBRARIES})
set_target_properties(msl_calculator PROPERTIES COMPILE_FLAGS "-ftree-vectorize ${SSE_FLAGS} -ffast-math -O5 -funroll-loops -fomit-frame-pointer -g -Wall -Wunused")

## Ray Calculator

#add_executable(msl_ray_calculator src/tools/RayCalculator.cpp )
#target_link_libraries(msl_ray_calculator pthread m stdc++ X11 Xv Xext ICE SM dc1394 raw1394 lib_msl_vision ${Boost_LIBRARIES} ${catkin_LIBRARIES})
#set_target_properties(msl_ray_calculator PROPERTIES COMPILE_FLAGS "-ftree-vectorize ${SSE_FLAGS} -ffast-math -O5 -funroll-loops -fomit-frame-pointer -g -Wall -Wunused")


SET(ROBOT "$ENV{ROBOT}")
IF(ROBOT STREQUAL "")
	EXECUTE_PROCESS(COMMAND hostname
		OUTPUT_VARIABLE ROBOT
	        OUTPUT_STRIP_TRAILING_WHITESPACE)
ENDIF(ROBOT STREQUAL "")

IF(NOT ROBOT STREQUAL "mops")
	ADD_CUSTOM_COMMAND(
		OUTPUT DistanceLookup.dat
		COMMAND msl_calculator
#		WORKING_DIRECTORY ${EXECUTABLE_OUTPUT_PATH}
		DEPENDS msl_calculator
		COMMENT "Creating Lookuptable"
	)
	ADD_CUSTOM_TARGET(DistanceLookup ALL
	DEPENDS DistanceLookup.dat
	)

## This is only required for Tim Schlueter's 3d OmniVision
#	ADD_CUSTOM_COMMAND(
#		OUTPUT RayLookup.dat
#		COMMAND msl_ray_calculator
##		WORKING_DIRECTORY ${EXECUTABLE_OUTPUT_PATH}
#		DEPENDS msl_ray_calculator
#		COMMENT "Creating RayLookuptable"
#	)
#	ADD_CUSTOM_TARGET(RayLookup ALL
#		DEPENDS RayLookup.dat
#	)
ENDIF(NOT ROBOT STREQUAL "mops")

