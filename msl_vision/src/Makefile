GCC = g++
#gcc


#-m64
CFLAGS = -ftree-vectorize -msse3 -ffast-math -O3 -funroll-loops -fomit-frame-pointer -g -Wall -Wunused -I$(IPP_HOME)/include -I$(ES_ROOT)/autogen/cpp/ -I$(ES_ROOT)/Spica/Castor/ -I$(ES_ROOT)/Spica/Castor/geminga -I$(ES_ROOT)/Spica/Castor/geminga/autogen/cpp -I/usr/include/opencv 
CFLAGS += -I$(ES_ROOT)/supplementary/freenect/include 
#CFLAGS += -I/usr/include/boost/thread -I/usr/include/ImageMagick

LIBS = -lm -lraw1394 -ldc1394 -L/usr/X11R6/lib -lSM -lICE -lXv -lXext -lX11 -lpthread -L$(ES_ROOT)/lib -lcastor -lgeminga -lmessages
#LIBS += -lMagick++ -lboost_thread
LIBS += -lfreenect -lfreenect_sync

SOURCESDRV = driver/dc1394camera.cpp \
       	driver/imagingsource.cpp \
	driver/flea2.cpp \
	driver/sony.cpp


#DEFINES= -DOLDBALL
DEFINES= -DSWITCHLOC

#filters/FilterAdjustGainDirected.cpp \

SOURCES = $(SOURCESDRV) \
	CameraQuickCam.cpp \
	XVDisplay.cpp \
	filters/Filter.cpp \
	filters/FilterYUVToRGB.cpp \
	filters/FilterYUVToGray.cpp \
	filters/FilterYUVExtractSubImages.cpp \
	filters/FilterYUVToYUVFull.cpp \
	filters/FilterYUVQuickCamToYUV.cpp \
	filters/FilterYUVExtractImages.cpp \
	filters/FilterTemplateMatchingGoalie.cpp \
	filters/FilterLinePointsROIDirected.cpp \
	filters/FilterAdjustGainDirected.cpp \
	filters/FilterDrawScanLines.cpp \
	filters/FilterLinePoints.cpp \
	filters/FilterLinePointsROI.cpp \
	filters/FilterHoughCalib.cpp \
	filters/FilterBox.cpp \
	filters/FilterAdaptiveROI.cpp \
	filters/FilterAdaptiveROIDirected.cpp \
	filters/FilterExtractBlobs.cpp \
	filters/FilterExtractBlobsDirected.cpp \
	filters/FilterGrayToDarkSeg.cpp \
	filters/FilterDistanceProfile.cpp \
	filters/FilterDistanceProfileNew.cpp \
	filters/FilterAddBallBlobsToSeg.cpp \
	filters/FilterSegToRGB.cpp \
	filters/FilterAdjustGain.cpp \
	filters/FilterTemplateMatching.cpp \
	filters/FilterSobelDir.cpp \
	filters/FilterCalibrationDirected.cpp \
	filters/FilterHistoLin.cpp \
	filters/FilterYUVCountColoredDots.cpp \
	filters/FilterAdjustGainNew.cpp \
	filters/FilterSoftHDR.cpp \
	helpers/KeyHelper.cpp \
	helpers/ScanLineHelper.cpp \
	helpers/ScanLineHelperBall.cpp \
	helpers/FootballField.cpp \
	helpers/DistanceLookupHelper.cpp \
	helpers/RandomHelper.cpp \
	helpers/ParticleFilter.cpp \
	helpers/ParticleFilterGoalie.cpp \
	helpers/LocalizeDebug.cpp \
	helpers/ImageMaskHelper.cpp \
	helpers/LineDistanceHelper.cpp \
	helpers/GoalHelperLocalization.cpp \
	helpers/EgoMotionEstimator.cpp \
	helpers/BallHelper.cpp \
	helpers/SpicaHelper.cpp \
	helpers/SpicaDirectedHelper.cpp \
	helpers/ConfigHelper.cpp \
	helpers/PacketHelper.cpp \
	helpers/RawOdometryHelper.cpp \
	helpers/OpponentHelper.cpp \
	helpers/CompassValueHelper.cpp \
	helpers/ObjectTracker.cpp \
	helpers/BallZTracker.cpp \
	helpers/ObjectContainer.cpp \
	helpers/BallIntegrator.cpp \
	helpers/TimeHelper.cpp \
	helpers/Environment.cpp \
	helpers/Logger.cpp \
	helpers/Replayer.cpp \
	helpers/BallClusterHelp.cpp \
	helpers/PositionHelper.cpp \
	helpers/PositionHelperDirected.cpp \
	helpers/BallHelperDirected2.cpp \
	helpers/BallHelperDirectedGoalie.cpp \
	helpers/ScanLineHelperDirected.cpp \
	helpers/ROIHelperOmniCam.cpp \
	helpers/SharedMemoryHelper.cpp \
	helpers/SharedBallHelper.cpp \
	helpers/SharedBallDirectedHelper.cpp \
	helpers/CorrectedOdometryHelper.cpp \
	helpers/ExtensionHelper.cpp \
	helpers/ExtensionDirectedHelper.cpp \
	helpers/FieldLut.cpp \
	helpers/Line2D.cpp \
	helpers/CoxLocalization.cpp \
	helpers/ErrorMinLocalization.cpp \
	helpers/CovarianceHelper.cpp \
	helpers/BallHelperKinect.cpp
        #ImageHandler.cpp \

#EXECUTABLE = VisionPlayer VisionPlayer2 VisionPlayerQuickCam VisionImageLogger VisionImagePlayer VisionOfflineTest VisionImageConverter \
#	VisionLineDetect VisionPotentialField VisionHoughCalib VisionHolderCalib VisionImageWriter \
#	SharedMemoryTest VisionFilterTest

#EXECUTABLE = VisionPlayer VisionImageConverter VisionPlayerGoalie VisionPlayerGoalie1394
EXECUTABLE = VisionPlayer VisionPlayerGoalie VisionPlayerGoalie1394 CameraModelTest VisionPlayerKinect
#EXECUTABLE = VisionPlayer

TOOLS = Calculator CalculateImageMask


MYHOSTNAME := $(shell hostname)

ADD_DEP := $(shell ls $(ES_ROOT)/etc/$(MYHOSTNAME)/Vision.conf)

DISTANCELOOKUP_DEPS = $(ES_ROOT)/etc/Vision.conf
DISTANCELOOKUP_DEPS += $(ADD_DEP)


EXECUTABLE_SRC =  $(EXECUTABLE:=.cpp)

EXECUTABLE_OBJS = $(EXECUTABLE_SRC:.cpp=.o)

OBJS = $(SOURCES:.cpp=.o)

ADD_DEPS = DistanceLookup

all: $(EXECUTABLE) $(TOOLS) $(ADD_DEPS)
	@echo "target all"
$(EXECUTABLE): $(OBJS) $(EXECUTABLE_OBJS)
	$(GCC) $(LIBS) $(OBJS) $@.o -o $@

.cpp.o:
	$(GCC) $(CFLAGS) $(DEFINES) -c $< -o $@

Calculator: helpers/ConfigHelper.o tools/Calculator.cpp
	$(GCC) $(CFLAGS) -c tools/distcalc.cpp -o tools/distcalc.o
	$(GCC) $(CFLAGS) -c tools/Calculator.cpp -o tools/Calculator.o
	$(GCC) -lm -L$(ES_ROOT)/lib -lcastor -lgeminga -lmessages helpers/ConfigHelper.o tools/distcalc.o tools/Calculator.o -o Calculator

CalculateImageMask: helpers/ConfigHelper.o tools/CalculateImageMask.cpp
	$(GCC) $(CFLAGS) -c tools/CalculateImageMask.cpp -o tools/CalculateImageMask.o
	$(GCC) -lm -L$(ES_ROOT)/lib -lcastor -lgeminga -lmessages helpers/ConfigHelper.o tools/CalculateImageMask.o -o CalculateImageMask

DistanceLookup: $(DISTANCELOOKUP_DEPS) $(TOOLS)
	./Calculator
	./CalculateImageMask
	touch DistanceLookup


clean:
	rm -f *.o
	rm -f filters/*.o
	rm -f helpers/*.o
	rm -f helpers/Kinect/*.o
	rm -f driver/*.o
	rm -f tools/*.o

distclean: clean
	rm -f DistanceLookup.dat
	rm -f DistanceLookup $(EXECUTABLE) $(TOOLS)

run:
	gnuplot -persist localizeDebug.plot &
	./VisionPlayer --localizeDebug --false | grep Loc1

conf:
	vi ../../etc/$(MYHOSTNAME)/Vision.conf
