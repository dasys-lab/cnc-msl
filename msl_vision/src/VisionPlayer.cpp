/*
 * $Id: VisionPlayer.cpp 2214 2007-04-17 17:44:39Z cn $
 *
 *
 * Copyright 2005,2006 Carpe Noctem, Distributed Systems Group,
 * University of Kassel. All right reserved.
 *
 * The code is derived from the software contributed to Carpe Noctem by
 * the Carpe Noctem Team.
 *
 * The code is licensed under the Carpe Noctem Userfriendly BSD-Based
 * License (CNUBBL). Redistribution and use in source and binary forms,
 * with or without modification, are permitted provided that the
 * conditions of the CNUBBL are met.
 *
 * You should have received a copy of the CNUBBL along with this
 * software. The license is also available on our website:
 * http://carpenoctem.das-lab.net/license.txt
 *
 *
 * <description>
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <algorithm>

#include <sys/time.h>

#include "driver/imagingsource.h"

#include <time.h>
#include <sys/time.h>

#include "XVDisplay.h"

#include <byteswap.h>

//#include <ipp.h>

#include "global/Types.h"

#include "filters/FilterYUVExtractSubImages.h"
#include "filters/FilterYUVToRGB.h"
#include "filters/FilterYUVToGray.h"
#include "filters/FilterDrawScanLines.h"
#include "filters/FilterLinePoints.h"
#include "filters/FilterLinePointsROI.h"
#include "filters/FilterHoughCalib.h"
#include "filters/FilterBox.h"
#include "filters/FilterAdaptiveROI.h"
#include "filters/FilterExtractBlobs.h"
#include "filters/FilterGrayToDarkSeg.h"
#include "filters/FilterDistanceProfile.h"
#include "filters/FilterDistanceProfileNew.h"
#include "filters/FilterAddBallBlobsToSeg.h"
#include "filters/FilterSegToRGB.h"
#include "filters/FilterAdjustGain.h"
#include "filters/FilterSobelDir.h"
#include "filters/FilterTemplateMatching.h"
#include "filters/FilterSoftHDR.h"

#include "helpers/ScanLineHelper.h"
#include "helpers/ScanLineHelperBall.h"
#include "helpers/DistanceLookupHelper.h"
#include "helpers/RandomHelper.h"
#include "helpers/ParticleFilter.h"
#include "helpers/ParticleFilterGoalie.h"
#include "helpers/LocalizeDebug.h"
#include "helpers/ImageMaskHelper.h"
#include "helpers/LineDistanceHelper.h"
#include "helpers/Lookuptable.h"
#include "helpers/GoalHelperLocalization.h"
#include "helpers/ImageMaskHelper.h"
#include "helpers/BallHelper.h"
#include "helpers/TimeHelper.h"
#include "helpers/Environment.h"
#include "helpers/Logger.h"
#include "helpers/Replayer.h"
#include "helpers/ROIHelperOmniCam.h"
#include "helpers/BallClusterHelp.h"
#include "helpers/ErrorMinLocalization.h"
#include "helpers/KeyHelper.h"
#include "helpers/SpicaHelper.h"
#include "helpers/ColorCalibrationHelper.h"
#include "helpers/CameraCalibrationHelper.h"
#include "camControl/Whitepoint.h"
#include "camControl/Brightness.h"

//HACK: diagnostic output
/*
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/DiagnosticArray.h>
*/

#include <SystemConfig.h>
#include <DateTime.h>

//jpeg stream
//#include "ImageHandler.h"

#include <iostream>
#include "ros/ros.h"

//using CarpeNoctem.Messages;

struct IppiSize {
	int height;
	int width;
};


void drawROI(unsigned char * image, int width, int height, ROI roi){

	for(int i = roi.top; i <= roi.bottom; i++){

		if(i >= 0 && i < height && roi.left >= 0 && roi.left < width)
			image[i*width + roi.left] = 0;
		if(i >= 0 && i < height && roi.right >= 0 && roi.right < width)
			image[i*width + roi.right] = 0;

	}

	for(int i = roi.left; i <= roi.right; i++){

		if(i >= 0 && i < width && roi.top >= 0 && roi.top < height)
			image[roi.top*width + i] = 0;
		if(i >= 0 && i < width && roi.bottom >= 0 && roi.bottom < height)
			image[roi.bottom*width + i] = 0;

	}


};


int main(int argc,char *argv[]){

	IppiSize imagesize_;
	imagesize_.width = camera::ImagingSource::usImageWidth;
	imagesize_.height = camera::ImagingSource::usImageHeight;

	std::cout << "DEBUG: Camera Resolution: " << imagesize_.width << " x " << imagesize_.height << std::endl;

	SystemConfig* sc;

	try {

		sc = SystemConfig::getInstance();

		camera::ImagingSource* cam = NULL; // new Cam Driver

		bool sshImage = false;
		bool drawObstacles = false;
		bool display_frames = true;
		bool display_gray = false;
		bool offline = false;
		bool fixed_image = false;
		bool drawScanLines = false;
		bool help = false;
		bool drawCircle = false;
		bool drawRGB = false;
		bool drawSegmented = false;
		bool lines = false;
		bool log = false;
		bool extendedLogging = false;
		bool roi = false;
		bool drawBall = false;
		bool directions = false;
		bool drawROI = false;
		bool drawRoland=false;
		bool localizeDebugFlag = false;
		bool softhdr=false;
		bool input=false;
		bool isGoalie=false;
		bool streamMJpeg=false;
		bool sendROIImage=false, sendGrayImage=false;

		int fixed_number = 1;

		XVDisplay * xvDisplay = NULL;
		XVDisplay * xvDisplay2 = NULL;

		if(argc > 1){

			for(int i = 1; i < argc; i++){
				if(std::string(argv[i]) == "--false")
					display_frames = false;
				if(std::string(argv[i]) == "--goalie")
                                        isGoalie = true;
				if(std::string(argv[i]) == "--gray")
					display_gray = true;
				if(std::string(argv[i]) == "--offline")
					offline = true;
				if(std::string(argv[i]) == "--scanLines")
					drawScanLines = true;
				if(std::string(argv[i]) == "--drawCircle")
					drawCircle = true;
				if(std::string(argv[i]) == "--rgb")
					drawRGB = true;
				if(std::string(argv[i]) == "--help")
					help = true;
				if(std::string(argv[i]) == "--lines")
					lines = true;
				if(std::string(argv[i]) == "--localizeDebug")
					localizeDebugFlag = true;
				if(std::string(argv[i]) == "--log")
					log = true;
				if(std::string(argv[i]) == "--extendedLogging")
					extendedLogging = true;
				if(std::string(argv[i]) == "--sshImage")
					sshImage = true;
				if(std::string(argv[i]) == "--segmented")
					drawSegmented = true;
				if(std::string(argv[i]) == "--fixed"){
					fixed_image = true;
					fixed_number = atoi(argv[i+1]);
				}
				if(std::string(argv[i]) == "--begin"){
					fixed_number = atoi(argv[i+1]);
				}
				if(std::string(argv[i]) == "--roi")
					roi = true;
				if(std::string(argv[i]) == "--ball")
					drawBall = true;
				if(std::string(argv[i]) == "--dir")
					directions = true;
				if(std::string(argv[i]) == "--drawRoland")
					drawRoland = true;
				if(std::string(argv[i]) == "--drawROI")
					drawROI = true;
				if(std::string(argv[i]) == "--softHDR")
					softhdr = true;
				if(std::string(argv[i]) == "--input")
                                        input = true;
				if(std::string(argv[i]) == "--drawObstacles")
					drawObstacles = true;
				if(std::string(argv[i]) == "--mjpg")
					streamMJpeg = true;
			}


		}

		if(help){
			printf("\nOptions for VisionPlayer:\n\n");
			printf("--false: hides window\n");
			printf("--gray: displays gray_image with line points and writes localization debug info\n");
			printf("--offline: offline mode using images in $VISION_LOG\n");
			printf("--scanLines: draws scan lines, only available with --gray\n");
			printf("--drawROI: draws ROIs, only available on grayscale-images\n");
			printf("--ball: draws balls, only available on grayscale-images\n");
			printf("--roi: draws channel of interest for ROI-channel\n");
			printf("--dir: draws direction-image for Balldetection\n");
			printf("--fixed [imageNumber]: only the image [imageNumber] from the $VISION_LOG is used\n");
			printf("--drawCircle: draws camera circle, only available with --gray\n");
			printf("--rgb: displays RGB Image\n");
			printf("--segmented: segemented image\n");
			printf("--mjpg: stream camera over http \n");
			printf("--localizeDebug: log line point data. call gnuplot -persist localizeDebug.plot\n");
			printf("--input: key input for segmentation. use keys a and q \n");
			printf("--help: shows this help text\n\n");

			exit(0);

		}



		std::cout << "Initializing camera:" << std::endl;

		printf("Start capturing ...\n");

		printf("Framerate was set!\n");

		Configuration *vision = (*sc)["Vision"];

		std::string camera_vendor = vision->tryGet<std::string>("Imaging Source", "Vision", "Camera1394Settings", "CameraVendor", NULL);

        short mx = vision->get<short>("Vision", "CameraMX", NULL);
        short my = vision->get<short>("Vision", "CameraMY", NULL);
		short radius = vision->get<short>("Vision", "CameraRadius", NULL);

        int edgethresh = vision->get<int>("Vision", "BallEdgethres", NULL);
        int edgemaskthresh = vision->get<int>("Vision", "BallEdgemaskthres", NULL);
        int maskThresh = vision->get<int>("Vision", "BallTemplMaskThres", NULL);
            Configuration *kh = (*sc)["KickHelper"];
            int kickerCount = (int)kh->tryGet<int>(3, "KickConfiguration", "KickerCount", NULL);
		if(kickerCount>3) kickerCount=3;

		std::cout << "BallParams: [Edgethesh]=" << edgethresh << " [EdgeMaskThresh]=" << edgemaskthresh << " [TemplateThesh]=" << maskThresh << std::endl;

		int area = vision->get<int>("Vision", "ImageArea", NULL);

		if(display_frames){
			xvDisplay = new XVDisplay(imagesize_.width, imagesize_.height, XV_UYVY);
			xvDisplay2 = new XVDisplay(area, area, XV_UYVY);
        }
        int wb1 = vision->get<int>("Vision", "Camera1394Settings", "WB1", NULL);
        int wb2 = vision->get<int>("Vision", "Camera1394Settings", "WB2", NULL);

		bool enableGainFilter = vision->tryGet<bool>(false, "Vision", "Camera1394Settings", "EnableGainFilter", NULL);
		bool particleFilterAllowed = vision->tryGet<bool>(true, "Vision", "ParticleFilterAllowed", NULL);
		bool detectObstacles = vision->tryGet<bool>(true, "Vision", "DetectObstacles", NULL);


		camera::ImagingSource::white_balance_t wb;
		wb.bu = wb1;
		wb.rv = wb2;

		if(!offline) {
			std::cout << "DEBUG in offline" << std::endl;
			//cam = new camera::ImagingSource(0);
			cam = new camera::ImagingSource(camera_vendor.c_str());
			cam->setVideoMode(DC1394_VIDEO_MODE_640x480_YUV422);
			cam->init();
			std::cout << "Cam init" << std::endl;
			//Trigger?
			if(vision->tryGet<bool>(false, "Vision", "Camera1394Settings", "SetManSettingsMode", NULL)) {
				cam->setManualSettingModes();
			}
			//Gamma ... here unsigned char
			cam->setGamma(vision->get<int>("Vision", "Camera1394Settings", "Gamma", NULL));

			if (vision->tryGet<bool>(false, "Vision", "Camera1394Settings", "UseBrightness", NULL)) {
				std::cout << "Param setting Brightness" << std::endl;
				cam->setBrightness(vision->get<int>("Vision", "Camera1394Settings", "Brightness", NULL));
			}

			if (vision->get<bool>("Vision", "Camera1394Settings", "AutoGain", NULL)) {

				std::cout << "DEBUG Autogain on ... SOMETHING DIFFERENT" << std::endl;
				// some comments
				// and more commits ... commit = push

				cam->enableAutoGain(true);
			} else {
				std::cout << "DEBUG man Gain ... TEST CHANGE" << std::endl;
				cam->enableAutoGain(false);
				cam->setGain(vision->get<int>("Vision", "Camera1394Settings", "Gain", NULL));
			}

			//not yet implemented !!!  TODO !!!!!!

			cam->setHue(vision->get<int>("Vision", "Camera1394Settings", "Hue", NULL));
			cam->setExposure(vision->get<int>("Vision", "Camera1394Settings", "Exposure", NULL));
			cam->setSaturation(vision->get<int>("Vision", "Camera1394Settings", "Saturation", NULL));

			cam->disableAutoWhiteBalance(); //?

			if(vision->tryGet<bool>(false, "Vision", "Camera1394Settings", "SetManSettingsMode", NULL)) {
				cam->setManualSettingModes();
				cam->enableAutoShutter(false);
			}
			cam->setWhiteBalance(wb);

			cam->setShutter(vision->tryGet<int>(30, "Vision", "Camera1394Settings", "Shutter", NULL));
			//TODO DOMINIK: could not set framerate
			cam->setFramerate(DC1394_FRAMERATE_30);  //DC1394_FRAMERATE_15
			std::cout << "DEBUG Framerate 30 " << std::endl;
			cam->startCapture();
			std::cout << "DEBUG start Capture " << std::endl;
			std::cout << "DEBUG end if " << std::flush;

		}

		SpicaHelper::initialize();
        ColorCalibration::ColorCalibrationHelper::initialize();
        CameraCalibration::CameraCalibrationHelper::initialize();

		if(extendedLogging){

			std::cout << "DEBUG extended Logging" << std::endl;
			char filename[256] = "/extendedLogging.dat";
			char path[256];
			strcpy(path, getenv("VISION_LOG"));

			char * path_filename = strcat(path, filename);

			Environment::getInstance()->setLogFileName(std::string(path_filename));

		}



		printf("Capturing was started\n");

		unsigned char * currImage = NULL;
		unsigned char * currRealImage = NULL;
		unsigned char * prevImage = (unsigned char *) malloc(imagesize_.width*imagesize_.height*2);
		unsigned char * imageRGB = NULL;
		unsigned char * image_gray = NULL;
		unsigned char * image_uv = NULL;
		unsigned char * image_roi = NULL;
		unsigned char * image_roiRoland = NULL;
		unsigned char * saved_image = NULL;
		unsigned char * image_gray_circle = NULL;
		unsigned char * image_gray_seg = NULL;
		unsigned char * image_gray_saved = (unsigned char *) malloc(area*area);
		unsigned char * image_segmented = NULL;
		unsigned char * imageSDir = NULL;
		unsigned char * imBall = NULL;
		unsigned char * imbrightenedGray = NULL;


		ScanLineHelper scanHelper;
		ScanLineHelperBall scanHelperBall;
		DistanceLookupHelper distanceHelper(area);
		LineDistanceHelper lineDistHelper;
		RandomGaussHelper gaussHelper;
		GoalHelperLocalization goalHelper;
		ImageMaskHelper imageMaskHelper(area, area);
		BallHelper ballHelper(area);

		FilterYUVExtractSubImages filterYUVExtractSubImages(imagesize_.width, imagesize_.height, area);
        filterYUVExtractSubImages.loadAllLookupTables();
		FilterLinePoints filterLinePoints(area);
		FilterLinePointsROI filterLinePointsROI(area);
		FilterDrawScanLines filterDrawScanLines(area, area);
		FilterSoftHDR filterSoftHDR(imagesize_.width, imagesize_.height);

		FilterBox filterBox(area, area);
		FilterAdaptiveROI filterAdaptiveROI(imagesize_.width, imagesize_.height, area);
		FilterExtractBlobs filterExtractBlobs(area, area);
		FilterGrayToDarkSeg filterGrayToDarkSeg(area, area);

		FilterDistanceProfile filterDistanceProfile(area, area);
		FilterDistanceProfileNew filterDistanceProfileNew(area, area);

		FilterAddBallBlobsToSeg filterAddBallBlobsToSeg(area, area);
		FilterSegToRGB filterSegToRGB(area,area);


		FilterYUVToRGB filterYUVToRGB(imagesize_.width, imagesize_.height);
		FilterHoughCalib filterHoughCalib(imagesize_.width, imagesize_.height);
		FilterYUVToGray filterYUVToGray(imagesize_.width, imagesize_.height);

		//FilterHistoLin filterHistLin(area, area);
		FilterSobelDir filterSobelDir(area, area);
		FilterTemplateMatching filterTMatch(area, area);

		FilterAdjustGain filterAdjustGain(imagesize_.width, imagesize_.height);
		ParticleFilter particleFilter(1200);
		ParticleFilterGoalie particleFilterGoalie(1200);
		LocalizeDebug localizeDebug;

		std::vector<BlobBounds> potYellowGoalBlobs;
		std::vector<BlobBounds> potBlueGoalBlobs;

		//Auto gain
		Brightness *br;
        if(enableGainFilter && !offline) br = Brightness::getInstance(640, 480, cam);
		Whitepoint *wp;
        if(enableGainFilter && !offline) wp = Whitepoint::getInstance(640, 480, cam);

		//ImageHandler ih(imagesize_.width, imagesize_.height, 4);

/*
		if (streamMJpeg){
			//initialize
			std::cout << "DEBUG init ImageWriter" << std::endl;
			ih.initImageWriter();
		}
*/

		printf("After Start ISO Transformation\n");

		std::vector<LinePoint> linePoints;
		linePoints.clear();

		std::vector<LinePoint> linePointsROI;
		linePointsROI.clear();

		int imCounter = fixed_number;
		int counter = 0;
		int writeCounter = 1;

		int *balls;
		int ballCount = 0, clusterCount = 0;
		BallClusterHelp ballClusterHelp;
		ballCluster *cluster = new ballCluster[10000];
		std::vector<ROIData> roiData;
		ROIData curBallROI;

		//Error minimalization init
#ifdef SWITCHLOC
		redwolf::ErrorMinLocalization *newLoc = new redwolf::ErrorMinLocalization();
		Configuration *localization = (*sc)["Localization"];
		int localizationBegin = 0;
		Position pos;
		Position relocPos;
		WeightedPosition weightPosErrorMin;
		WeightedPosition weightPosParticleFilter;
		pos.x = localization->get<double>("Localization","InitialPosition", SystemConfig::getHostname().c_str(),"X",NULL);;
		pos.y= localization->get<double>("Localization","InitialPosition", SystemConfig::getHostname().c_str(),"Y",NULL);;
		pos.heading = localization->get<double>("Localization","InitialPosition", SystemConfig::getHostname().c_str(),"Heading",NULL);;
		relocPos.heading = pos.heading;
		relocPos.x = pos.x;
		relocPos.y = pos.y;
		const double cornerOffset = localization->get<double>("Localization", "CornerOffset", NULL);

		const double switchToParticle = localization->get<double>("Localization", "SwitchToParticle", NULL);
		const double switchToErrorMin = localization->get<double>("Localization", "SwitchToErrorMin", NULL);
		const int weightBufferSize = localization->get<double>("Localization", "WeightBufferSize", NULL);
		const double weightOfBufferValues = localization->get<double>("Localization", "WeightOfOldValue", NULL);
		const int nrOfPfIterations = localization->get<double>("Localization", "NrOfParticleFilterIteration", NULL);
		const int nrOfEmIterations = localization->get<double>("Localization", "NrOfErrorMinIteration", NULL);
		int emIterationCount = 0;
		int pfIterationCount = 0;
		bool useParticleFilter = localization->get<bool>("Localization","InitialPosition", SystemConfig::getHostname().c_str(), "InitialUseOfParticleFilter", NULL);

		double *weightBuffer = new double[weightBufferSize];
		for (int i=0;i<weightBufferSize; i++) weightBuffer[i] = -1.0;
		int weightBufferCount = 0;

		std::cout<<"NewLoc Iinitial Position"<<pos.x<<" "<<pos.y<<" "<<pos.heading<<std::endl;
#endif

		camera::Frame frame;
		if(!offline){
			printf("Stop ISO Transmission\n");

			cam->stopCapture();

			printf("Before Start ISO Transmission\n");
			cam->startCapture();
		}
		else
			saved_image = (unsigned char *) malloc(imagesize_.width*imagesize_.height*3);

/*
		//HACK: diagnostic output
		int diagCounter = 0;
		ros::Publisher diag_pub = SpicaHelper::visionNode->advertise<diagnostic_msgs::DiagnosticArray>("diagnostics", 1000);
*/

		while(ros::ok()){

			std::cout << "DEBUG after main while loop" << std::endl;
			std::vector<Goal> yellowGoals;
			std::vector<Goal> blueGoals;
			std::vector<CornerPost> cornerPosts;
			yellowGoals.clear();
			blueGoals.clear();
			cornerPosts.clear();

			unsigned long long visionTimeOmniCamLong;

            if(!offline){
                if (CameraCalibration::CameraCalibrationHelper::setSettings == true) {
                    CameraCalibration::CameraCalibrationHelper::setSettings = false;

                    cout << "CamCalib\tsetSettings" << endl;
                    CameraCalibration::Settings* settings = CameraCalibration::CameraCalibrationHelper::cameraSettings;
                    // BRIGHTNESS
                    if (settings->useBrightness == true) {
                        if (cam->getBrightness() != settings->brightness) {
                            cout << "CamCalib\tchanging brightness\tfrom: ";
                            cout << cam->getBrightness() << " to: ";
                            cout << settings->brightness;
                            cout << endl;

                            cam->setBrightness(settings->brightness);

                            CameraCalibration::CameraCalibrationHelper::settingsAreRequested = true;
                        }
                    }
                    // EXPOSURE
                    if (cam->getExposure() != settings->exposure) {
                        cout << "CamCalib\tchanging exposure\tfrom: ";
                        cout << cam->getExposure() << " to: ";
                        cout << settings->exposure;
                        cout << endl;

                        cam->setExposure(settings->exposure);

                        CameraCalibration::CameraCalibrationHelper::settingsAreRequested = true;
                    }
                    // WHITEBALANCE
                    if (settings->autoWhiteBalance == false) {
                        bool setWB = false;
                        camera::ImagingSource::white_balance_t wb = cam->getWhiteBalance();
                        if (wb.bu != settings->whiteBalance1) {
                            cout << "CamCalib\tchanging whiteBalance1 (bu)\tfrom: ";
                            cout << wb.bu << " to: ";
                            cout << settings->whiteBalance1;
                            cout << endl;

                            wb.bu = settings->whiteBalance1;

                            setWB = true;
                        }
                        if (wb.rv != settings->whiteBalance2) {
                            cout << "CamCalib\tchanging whiteBalance2 (rv)\tfrom: ";
                            cout << wb.rv << " to: ";
                            cout << settings->whiteBalance2;
                            cout << endl;

                            wb.rv = settings->whiteBalance2;

                            setWB = true;
                        }
                        if (setWB) {
                            cam->setWhiteBalance(wb);

                            CameraCalibration::CameraCalibrationHelper::settingsAreRequested = true;
                        }
                    }
                    // HUE
                    if (cam->getHue() != settings->hue) {
                        cout << "CamCalib\tchanging hue\tfrom: ";
                        cout << cam->getHue() << " to: ";
                        cout << settings->hue;
                        cout << endl;

                        cam->setHue(settings->hue);

                        CameraCalibration::CameraCalibrationHelper::settingsAreRequested = true;
                    }
                    // SATURATION
                    if (cam->getSaturation() != settings->saturation) {
                        cout << "CamCalib\tchanging saturation\tfrom: ";
                        cout << cam->getSaturation() << " to: ";
                        cout << settings->saturation;
                        cout << endl;

                        cam->setSaturation(settings->saturation);

                        CameraCalibration::CameraCalibrationHelper::settingsAreRequested = true;
                    }
                    // ENABLE GAMMA
                    if (cam->isGamma() != settings->enabledGamma) {
                        cout << "CamCalib\tchanging enabledGamma\tfrom: ";
                        cout << (cam->isGamma() ? "true" : "false") << " to: ";
                        cout << (settings->enabledGamma ? "true" : "false");
                        cout << endl;

                        cam->enableGamma(settings->enabledGamma);

                        CameraCalibration::CameraCalibrationHelper::settingsAreRequested = true;
                    }
                    // GAMMA
                    if (cam->isGamma() &&
                            cam->getGamma() != settings->gamma) {
                        cout << "CamCalib\tchanging gamma\tfrom: ";
                        cout << cam->getGamma() << " to: ";
                        cout << settings->gamma;
                        cout << endl;

                        cam->setGamma(settings->gamma);

                        CameraCalibration::CameraCalibrationHelper::settingsAreRequested = true;
                    }
                    // AUTOSHUTTER
                    if (cam->isAutoShutter() != settings->autoShutter) {
                        cout << "CamCalib\tchanging autoShutter\tfrom: ";
                        cout << (cam->isAutoShutter() ? "true" : "false") << " to: ";
                        cout << (settings->autoShutter ? "true" : "false");
                        cout << endl;

                        cam->enableAutoShutter(settings->autoShutter);

                        CameraCalibration::CameraCalibrationHelper::settingsAreRequested = true;
                    }
                    // SHUTTER
                    if (!cam->isAutoShutter() &&
                            cam->getShutter() != settings->shutter) {
                        cout << "CamCalib\tchanging shutter\tfrom: ";
                        cout << cam->getShutter() << " to: ";
                        cout << settings->shutter;
                        cout << endl;

                        cam->setShutter(settings->shutter);

                        CameraCalibration::CameraCalibrationHelper::settingsAreRequested = true;
                    }
                    // AUTOGAIN
                    if (cam->isAutoGain() != settings->autoGain) {
                        cout << "CamCalib\tchanging autoGain\tfrom: ";
                        cout << (cam->isAutoGain() ? "true" : "false") << " to: ";
                        cout << (settings->autoGain ? "true" : "false");
                        cout << endl;

                        cam->enableAutoGain(settings->autoGain);

                        CameraCalibration::CameraCalibrationHelper::settingsAreRequested = true;
                    }
                    // GAIN
                    if (!cam->isAutoGain() &&
                            cam->getGain() != settings->gain) {
                        cout << "CamCalib\tchanging gain\tfrom: ";
                        cout << cam->getGain() << " to: ";
                        cout << settings->gain;
                        cout << endl;

                        cam->setGain(settings->gain);

                        CameraCalibration::CameraCalibrationHelper::settingsAreRequested = true;
                    }
                }

				std::cout << "DEBUG start Capture" << std::endl;

				if (!cam->getFrame(frame))
				{
					std::cerr << "Error while capturing image, aborting!" << std::endl;
					std::cout << "Error while capturing image, aborting!" << std::endl;
                    exit(1);
				}

                if (CameraCalibration::CameraCalibrationHelper::settingsAreRequested == true) {
                    CameraCalibration::CameraCalibrationHelper::settingsAreRequested = false;

                    cout << "CamCalib\tsettingsAreRequested" << endl;
                    CameraCalibration::Settings* settings = new CameraCalibration::Settings();
                    settings->brightness = cam->getBrightness();
                    settings->exposure = cam->getExposure();
                    settings->autoWhiteBalance = false;
                    camera::ImagingSource::white_balance_t wb = cam->getWhiteBalance();
                    settings->whiteBalance1 = wb.bu;
                    settings->whiteBalance2 = wb.rv;
                    settings->hue = cam->getHue();
                    settings->saturation = cam->getSaturation();
                    settings->gamma = cam->getGamma();
                    settings->autoShutter = cam->isAutoShutter();
                    settings->shutter = cam->getShutter();
                    settings->autoGain = cam->isAutoGain();
                    settings->gain = cam->getGain();

                    CameraCalibration::CameraCalibrationHelper::sendSettings(settings);

                    delete settings;
                }

//				cam->setWhiteBalance(wb);



				if(drawRGB)
				{
					std::cout << "DEBUG filterYUVtoRGB" << std::endl;
					imageRGB = filterYUVToRGB.process((unsigned char *) frame.getImagePtr(), imagesize_.width * imagesize_.height * 2);
				}
				currImage = (unsigned char *) frame.getImagePtr();
/*
				if(streamMJpeg){
					imageRGB = filterYUVToRGB.process(currImage, imagesize_.width * imagesize_.height * 2);
					//writeImage(unsigned char[] image data , bool isGray)
					std::cout << "DEBUG stream write image" << std::endl;
					ih.writeImage(imageRGB, true);
				}
*/
				visionTimeOmniCamLong = supplementary::DateTime::getUtcNowC();

				counter++;

				if(log && counter >= 200 && counter % 10 >= 0){

					char filename[256];
					char path[256];
					strcpy(path, getenv("VISION_LOG"));

					sprintf(filename, "/log-image-%04d.raw", writeCounter);
					char * path_filename = strcat(path, filename);

					FILE * logfile = fopen(path_filename, "w");

					fwrite(currImage, sizeof(char), imagesize_.width*imagesize_.height*3, logfile);

					fclose(logfile);



					if(extendedLogging){
						Environment::getInstance()->setLoggingMode(true);
						Logger::getInstance()->setWriteCounter(writeCounter);
						Logger::getInstance()->logImageInfo(writeCounter, visionTimeOmniCamLong);
					}

					writeCounter++;

				}

/*
				//HACK: diagnostic output hack
                                if(diagCounter > 9){

                                          ROS_WARN("HACK: some test diagnostic output");
                                          diagnostic_msgs::DiagnosticStatus diagMsg;
                                          diagMsg.name = "CNVision";
                                          diagMsg.message = "some daig msg";
                                          diagMsg.level = diagnostic_msgs::DiagnosticStatus::OK;

                                          std::vector<diagnostic_msgs::KeyValue> kvs;

                                          diagnostic_msgs::KeyValue kv3;
                                          kv3.key = "internal_state_val";
                                          kv3.value =  boost::lexical_cast<std::string>("4");
                                          kvs.push_back(kv3);

                                          diagnostic_msgs::KeyValue kv;
                                          kv.key = "Sup_Vol";
                                          kv.value = "5";
                                          kvs.push_back(kv);

                                          diagnostic_msgs::KeyValue kv2;
                                          kv2.key = "Scan_Value";
                                          kv2.value = "0";
                                          kvs.push_back(kv2);

                                          diagMsg.values = kvs;

                                          diagnostic_msgs::DiagnosticArray diagArray;
                                          diagArray.status.push_back(diagMsg);
                                          diagArray.header.stamp = ros::Time::now();

                                          ROS_INFO("DEBUG: CNVisionPlayer: send diag msg with level %d", diagMsg.level);

                                          diag_pub.publish(diagArray);
                                        //BUG !!! JUST FOR TESTING !!!
                                        diagCounter = 0;

                                } else {
                                        diagCounter++;
                                }
*/


			}
			else {
				char filename[256];
				char filename2[256];
				char path[256];
				char path2[256];
				strcpy(path, getenv("VISION_LOG"));
				strcpy(path2, getenv("VISION_LOG"));

				sprintf(filename, "/log-image-%04d.raw", imCounter);
				char * path_filename = strcat(path, filename);

				if(!fixed_image)
					imCounter++;
				counter++;

				printf("Processing LogFile: %s\n", path_filename);

				FILE * logfile = fopen(path_filename, "r");

				if(logfile != NULL){

					fread(saved_image, sizeof(char), imagesize_.width*imagesize_.height*3, logfile);
					fclose(logfile);
				}
				else{

					printf("Log file not found ... Restarting\n");
					imCounter = 1;
					counter = 0;

					sprintf(filename2, "/log-image-%04d.raw", imCounter);
					char * path_filename2 = strcat(path2, filename2);

					logfile = fopen(path_filename2, "r");
					if(logfile == NULL){
						printf("Log file not found\n");
						break;
					}

				}

				//currImage = filterYUVFullToYUV.process((unsigned char *) saved_image, imagesize_.width* imagesize_.height*3);
				currImage = saved_image;
				if(drawRGB)
					imageRGB = filterYUVToRGB.process(currImage, imagesize_.width* imagesize_.height*2);

				if(extendedLogging){
					visionTimeOmniCamLong = Replayer::getInstance()->replay(imCounter);


				}
				else {
					visionTimeOmniCamLong = supplementary::DateTime::getUtcNowC();
				}
			}

			struct timeval tv_before;
			gettimeofday(&tv_before, NULL);

			std::vector<ROI> rois;
			rois.clear();
			std::vector<BlobBounds> ballBlobs;
			ballBlobs.clear();

			visionTimeOmniCamLong -= 700000;

			TimeHelper::getInstance()->setVisionTimeOmniCam(visionTimeOmniCamLong);

			printf("Stage 1: Image precomputation\n");


			if(softhdr) {
				currRealImage = currImage;
				if(counter>0) {
					currImage = filterSoftHDR.process(currRealImage, prevImage, imagesize_.width, imagesize_.height);
					if(drawRGB)
						imageRGB = filterYUVToRGB.process(currImage, imagesize_.width* imagesize_.height*2);
				}
			}





			// Filters for calibrating camera middle
			if(drawCircle){
				image_gray_circle = filterYUVToGray.process(currImage, imagesize_.width*imagesize_.height*2);
				filterHoughCalib.DrawCircle(mx, my, radius, (unsigned char *) image_gray_circle, imagesize_.width, imagesize_.height);
				filterHoughCalib.DrawCircle(mx, my, 10, (unsigned char *) image_gray_circle, imagesize_.width, imagesize_.height);
			}


			//Extract Gray and UV Subimages (area x area) from the camera image
			//image_gray not modified
			//image_uv modified with trhesholds
			filterYUVExtractSubImages.process(currImage, imagesize_.width, imagesize_.height, mx, my, image_gray, image_uv, image_roi, image_roiRoland, imbrightenedGray);
			printf("Stage 2: Copy gray scale Image\n");
			memcpy(image_gray_saved, image_gray, area*area);
			printf("Stage 3: Compute Image for Obstacles\n");
			//Segmentation for collision avoidance and opponent detection
			image_gray_seg = filterGrayToDarkSeg.process(image_gray, image_uv, area, area, imageMaskHelper);

#ifdef OLDBALL
			//Gauss (Box) Filtering of UV_Image
			filterBox.process(image_roiRoland, area, area);
#endif

			printf("Stage 4: Detect Linepoints\n");
			//Get LinePoints for Self-Localization
			image_gray = filterLinePoints.process((unsigned char *) image_gray, area, area, linePoints, distanceHelper, scanHelper);
			//Send linepointlist here:
			SpicaHelper::sendLinePoints(linePoints);


			printf("Stage 5: Detect ROIs\n");
			roiData = filterLinePointsROI.process((unsigned char *) image_uv, area, area, linePointsROI, distanceHelper, scanHelperBall);

			// rio Data contains which info???
			printf("Stage 6: Insert Tracked ROI into ROI List\n");
			roiData.insert(roiData.end()-kickerCount, curBallROI);

			//New Ball
			//does what?
			printf("Stage 7: Compute Sobel on ROIs\n");
			imageSDir = filterSobelDir.process(image_uv, image_uv, roiData, area, area, edgethresh, edgemaskthresh);

			printf("Stage 8: Apply Templatematching\n");
			imBall = filterTMatch.process(imageSDir, balls, ballCount, image_uv, roiData, maskThresh, area, area, 4, 19, 6, image_roi);
			printf("Stage 9:\n");
			clusterCount = ballClusterHelp.clusterBalls(balls, ballCount, cluster, 200);

			//ballClusterHelp.clusterStdOut(cluster, clusterCount, mx, my, false);

#ifdef OLDBALL
			//Get ROIs for Ball Detection, ROIs are segmented
			filterAdaptiveROI.process(filterBox.getResult());

			//Extract potential BallBlobs from ROI segemented image
			filterExtractBlobs.process(filterAdaptiveROI.getResult(), area, area, rois, 255, ballBlobs, distanceHelper);

			//Add red segmented regions to dark segmented image, to deal with shadows in front of the ball
			filterAddBallBlobsToSeg.process(filterAdaptiveROI.getResult(), image_gray_seg, ballBlobs, area, area, 255);
#endif


			// filter for converting the segmented image to an rgb representation
			if(drawSegmented)
				image_segmented = filterSegToRGB.process(image_gray_seg, area*area);

			// Filters for holder calibration
			if(drawScanLines)
				image_gray = filterDrawScanLines.process((unsigned char *) image_gray, area, area, scanHelper, true);


			// Particle filter using the line points and some helpers + empty yellowGoal, blueGoal and corner posts blobs

			printf("MovingObject imageNumber %d\n", imCounter);

#ifdef SWITCHLOC
			if (useParticleFilter) {
			    if(!isGoalie)
				particleFilter.iterate(linePoints, lineDistHelper, gaussHelper, yellowGoals, blueGoals, cornerPosts);
			    else
				particleFilterGoalie.iterate(linePoints, lineDistHelper, gaussHelper, yellowGoals, blueGoals, cornerPosts);
			}
#endif
#ifndef SWITCHLOC
			if(!isGoalie)
			    particleFilter.iterate(linePoints, lineDistHelper, gaussHelper, yellowGoals, blueGoals, cornerPosts);
			else
			    particleFilterGoalie.iterate(linePoints, lineDistHelper, gaussHelper, yellowGoals, blueGoals, cornerPosts);
#endif
			//Old Ball detektion
#ifdef OLDBALL
			// Get Ball Position from Extracted Blobs and using the localization (max Particle)
			printf("Number of PotBallBlobs: %d\n", ballBlobs.size());
			//ballHelper.getBallFromBlobs(ballBlobs, &maxParticle);
			//
			//ROI ballROI = ROIHelperOmniCam::getInstance()->getROIForObject(ballHelper.getBallPosition().x, ballHelper.getBallPosition().y, 120.0, 120.0, distanceHelper);
#endif

#ifdef SWITCHLOC
			printf("Stage 10: Localization\n");
			if (localizationBegin == 100)
			{
				double weightBufferValue = 0.0;
				int i;
				for (i = 0;i<weightBufferSize; i++) {
					if (weightBuffer[i] == -1.0) break;
					weightBufferValue += weightBuffer[i];

				}
				weightBufferValue = weightBufferValue/i;
				printf("VisionPlayer NewLoc weightBufferValue %f\n",weightBufferValue);

				weightPosErrorMin = newLoc->estimatedPosition(pos, linePoints, lineDistHelper);
				if (useParticleFilter) {
				    if(!isGoalie)
					  weightPosParticleFilter = particleFilter.getEstimatedPosition();
				    else
					  weightPosParticleFilter = particleFilterGoalie.getEstimatedPosition();
				}
				bool isInCorner = false;
				// Changed from 'isInCorner == true' to real assignment
				if (fabs(pos.x) > (FootballField::FieldLength/2.0 - cornerOffset) && fabs(pos.y) > (FootballField::FieldWidth/2.0 - cornerOffset)) { printf("NewLoc INCORNER!!!!"); isInCorner = true; }
				double pfWeight = (weightBufferValue*weightOfBufferValues + weightPosParticleFilter.weight*(1-weightOfBufferValues));
				double emWeight = (weightBufferValue*weightOfBufferValues + weightPosErrorMin.weight*(1-weightOfBufferValues));
				cout << "LOC DEBUG: " << emWeight << "\t" << pfWeight << endl;
//				printf("NewLoc ParticleFilter: %f: %f %f %f\n",weightPosParticleFilter.weight, weightPosParticleFilter.x, weightPosParticleFilter.y,weightPosParticleFilter.heading);
//				printf("NewLoc em: %f: %f %f %f\n",weightPosErrorMin.weight, weightPosErrorMin.x, weightPosErrorMin.y, weightPosErrorMin.heading);
				if (particleFilterAllowed &&
					(!(isInCorner && (weightBufferValue > 0.9|| weightBufferValue <0.8))
					&&
						 (
							(
								useParticleFilter &&
								(pfIterationCount < nrOfPfIterations || pfWeight < switchToErrorMin)
							)
						||
							(emWeight < switchToParticle )
						)
					)
				)
				{

					if (useParticleFilter == false ) {
						useParticleFilter = true;
						if(!isGoalie) particleFilter.iterate(linePoints, lineDistHelper, gaussHelper, yellowGoals, blueGoals, cornerPosts);
						else particleFilterGoalie.iterate(linePoints, lineDistHelper, gaussHelper, yellowGoals, blueGoals, cornerPosts);
					}
					pos.x = weightPosParticleFilter.x;
					pos.y = weightPosParticleFilter.y;
					pos.heading = weightPosParticleFilter.heading;
					weightBuffer[weightBufferCount] = weightPosParticleFilter.weight;
					//check for corner (only use error min)
					printf("VisionPlayer Loc Particlefilter: %f: %f %f %f\n", weightPosParticleFilter.weight, pos.x, pos.y, pos.heading);
					if(!isGoalie) particleFilter.writeCoi();
					else particleFilterGoalie.writeCoi();

					pfIterationCount++;
				}
				else
				{
					useParticleFilter = false;
					pos.x = weightPosErrorMin.x;
                                        pos.y = weightPosErrorMin.y;
                                        pos.heading = weightPosErrorMin.heading;
					weightBuffer[weightBufferCount] = weightPosErrorMin.weight;

					printf("VisionPlayer Loc ErrorMin: %f: %f %f %f\n",weightPosErrorMin.weight, pos.x, pos.y, pos.heading);
					newLoc->writeCoi();
					pfIterationCount = 0;
				}
				weightBufferCount = (weightBufferCount + 1) % weightBufferSize;
			}
			else localizationBegin ++;
			// get max Particle for Ball Detection
			//
			Particle maxParticle;// = new Particle();
			if (particleFilterAllowed && useParticleFilter)
			{
				if(!isGoalie) maxParticle = particleFilter.getMaxParticle();
				else maxParticle = particleFilterGoalie.getMaxParticle();
			}
			else
			{
				maxParticle.posx = pos.x;
				maxParticle.posy = pos.y;
				maxParticle.heading = pos.heading;
				maxParticle.weight = weightPosErrorMin.weight;
			}
#endif
#ifndef SWITCHLOC
			WeightedPosition wpos;
			if(isGoalie) wpos = particleFilter.getEstimatedPosition();
			else wpos = particleFilterGoalie.getEstimatedPosition();
			Position pos;
			pos.x = wpos.x;
			pos.y = wpos.y;
			pos.heading = wpos.heading;
			Particle maxParticle
			if(!isGoalie) maxParticle = particleFilter.getMaxParticle();
			else maxParticle = particleFilterGoalie.getMaxParticle();
#endif
			printf("Stage 11: Cluster available Balls\n");
			Point p = ballHelper.getBallFromBlobs(cluster, clusterCount, roiData, ballBlobs, &maxParticle);
			if(roiData.size() > 0)
				curBallROI = roiData[0];


			printf("Stage 12: Compute Goal positions\n");
			// Calculate Goals from the estimated position
			goalHelper.getGoalsFromPosition(pos);

			//calculate distance profile for collision avoidance
			printf("Stage 13: Obstacle avoidance\n");
            if(detectObstacles) filterDistanceProfileNew.process(image_gray_saved, area, area, scanHelper, distanceHelper);


			struct timeval tv_after;
			gettimeofday(&tv_after, NULL);

			long timediff = tv_after.tv_usec - tv_before.tv_usec;
			if(timediff < 0)
				timediff += 1000000;

			printf("\n\nTime for FilterChain: %ld\n\n", timediff);

			// reloc trigger
			if(SpicaHelper::reloc == true){
				SpicaHelper::reloc = false;
				pos.x = relocPos.x;
				pos.y = relocPos.y;
				pos.heading = relocPos.heading;
			}

            // ColorCalibration
            ColorCalibration::ColorCalibrationHelper::sendImage(currImage, imagesize_.width, imagesize_.height);

			int lpCounter = 0;
			for(unsigned int i = 0; i < linePoints.size(); i++){
				if(linePoints[i].x*linePoints[i].x + linePoints[i].y*linePoints[i].y < 800.0*800.0) //800???
					lpCounter++;


			}
			printf("LinePoints with low Distance %d\n", lpCounter);

			if(localizeDebugFlag && counter >= 0 && maxParticle.weight > 0.0){
				FILE * fd = fopen("LinePoints.txt", "w");
				SpicaHelper::vdd->list.clear();
				for(unsigned int i = 0; i < linePoints.size(); i++){
					fprintf(fd, "%f %f %f\n", linePoints[i].x, linePoints[i].y, pos.heading);
					msl_msgs::Point2dInfo tmp;
					tmp.x = linePoints[i].x;
					tmp.y = linePoints[i].y;
					SpicaHelper::vdd->list.push_back(tmp);
					msl_msgs::PositionInfo pi;
					pi.x = pos.x;
					pi.y = pos.y;
					pi.angle = pos.heading;
					pi.certainty = maxParticle.weight;
					SpicaHelper::vdd->position = pi;
					SpicaHelper::vdd->distanceScan = SpicaHelper::wm->distanceScan;
					SpicaHelper::vdd->ball = SpicaHelper::wm->ball;
					SpicaHelper::vdd->locType.type = SpicaHelper::wm->odometry.locType.type;
					SpicaHelper::vdd->obstacles = SpicaHelper::wm->obstacles;
					SpicaHelper::vdd->senderID = supplementary::SystemConfig::getOwnRobotID();
				}
				SpicaHelper::sendDebugMsg();
				fclose(fd);

				FILE * fd2 = fopen("MaxParticle.txt", "w");

				fprintf(fd2, "%f %f\n", maxParticle.posx, maxParticle.posy);
				fclose(fd2);

				Particle drawParticle;
				drawParticle.posx = pos.x;
				drawParticle.posy = pos.y;
				drawParticle.heading = pos.heading;
				drawParticle.weight = 0.9;
				localizeDebug.drawFieldForParticle(drawParticle, 0);
				localizeDebug.drawParticlesOnField(particleFilter);
			}

			if(softhdr) {
			  memcpy(prevImage, currRealImage, imagesize_.width*imagesize_.height*2);
			}

			printf("Stage 14: Show Image & Adjust Gain\n");
			// manage display option
			if(xvDisplay != NULL && !(counter%33!=0 && sshImage)){
				if(drawCircle)
					xvDisplay->displayFrameGRAY((char *) image_gray_circle);
				else if(drawRGB)
					xvDisplay->displayFrameRGB((char *) imageRGB);
				else if(drawScanLines)
					xvDisplay2->displayFrameGRAY((char *) image_gray);
				else if(drawRoland)
					xvDisplay2->displayFrameGRAY((char *) image_roiRoland);
                else if(drawObstacles)
                    xvDisplay2->displayFrameGRAY((char *) imbrightenedGray);

				else if(directions) {
					if(drawBall)
						ballClusterHelp.visualizeCluster(imageSDir, area, area, cluster, clusterCount);
					xvDisplay2->displayFrameGRAY((char *) imageSDir);
				}
				else if(display_gray) {
					if(drawBall)
						ballClusterHelp.visualizeCluster(image_gray, area, area, cluster, clusterCount);
					ballHelper.visualizeBall(image_gray, area, p, 10);
					if(drawROI)
						filterLinePointsROI.visualizeROIs(image_gray, roiData, area, area);
					xvDisplay2->displayFrameGRAY((char *) image_gray);
				}
				else if(roi) {
					if(drawBall)
						ballClusterHelp.visualizeCluster(image_roi, area, area, cluster, clusterCount);
					if(drawROI)
						filterLinePointsROI.visualizeROIs(image_roi, roiData, area, area);
					xvDisplay2->displayFrameGRAY((char *) image_roi);
				}
				else if(drawSegmented)
					xvDisplay2->displayFrameRGB((char *) image_segmented);
				else {
					if(drawBall)
						ballClusterHelp.visualizeCluster(image_uv, area, area, cluster, clusterCount);
					if(drawROI)
						filterLinePointsROI.visualizeROIs(image_uv, roiData, area, area);
					xvDisplay2->displayFrameGRAY((char *) image_uv);
				}
			}

			if(KeyHelper::checkKey('L')) sendGrayImage=true;
			if(KeyHelper::checkKey('l')) sendGrayImage=false;
			if(KeyHelper::checkKey('O')) sendROIImage=true;
			if(KeyHelper::checkKey('o')) sendROIImage=false;


			if(sendGrayImage) {
				ballClusterHelp.visualizeCluster(image_gray, area, area, cluster, clusterCount);
				ballHelper.visualizeBall(image_gray, area, p, 10);
				filterLinePointsROI.visualizeROIs(image_gray, roiData, area, area);
				SpicaHelper::streamGreyMJPEG(image_gray, area, area);
			} else if (sendROIImage) {
				SpicaHelper::streamGreyMJPEG(image_roi, area, area);
			}

/*
			if(streamMJpeg){
				//imageRGB = filterYUVToRGB.process(currImage, imagesize_.width * imagesize_.height * 2);
				//image_gray_circle = filterYUVToGray.process(currImage, imagesize_.width*imagesize_.height*2);
				//writeImage(unsigned char[] image data , bool isGray)
				std::cout << "DEBUG stream write image" << std::endl;
				ih.writeImage(image_roi, true, area, area);
//				ih.writeImage(imageRGB, true);
			}
*/

			SpicaHelper::send();
            SpicaHelper::sendGameState();
			if(input) KeyHelper::checkKeyPress();
			KeyHelper::checkRemoteKey();
			if(KeyHelper::checkKey('P')) localizeDebugFlag=true;
			else if (KeyHelper::checkKey('p')) localizeDebugFlag=false;
			if(!offline){

				if(KeyHelper::checkKey('s')){
					cam->setSaturation(cam->getSaturation()-5);
					printf("Parameter Adjustmen - Saturation: %d\n", cam->getSaturation());
				}
				if(KeyHelper::checkKey('S')) {
					cam->setSaturation(cam->getSaturation()+5);
					printf("Parameter Adjustmen - Saturation: %d\n", cam->getSaturation());

				}
				if(KeyHelper::checkKey('w')) {
				}
				if(KeyHelper::checkKey('W')) {
				}
				if(KeyHelper::checkKey('e')) {
				}
				if(KeyHelper::checkKey('E')) {
				}


				printf("Stage 15:\n");
				if(softhdr && !offline) {
					/*int currGain = 670; //camera->getGain();
					if(counter % 2 == 0)
						currGain += 150;
					else
						currGain -= 150;
						cam->setGain(currGain);
				} else {
                                        if(enableGainFilter) filterAdjustGain.process(*cam,filterDistanceProfileNew, linePoints, maxParticle);*/

				}
				//Auto-Gain
				if(enableGainFilter){
					br->process(frame.getImagePtr(), counter);
					if(KeyHelper::checkKey('X')) {br->addBrightness(0.5);}
					if(KeyHelper::checkKey('x')) {br->addBrightness(-0.5);}
				}
				//wp->process(frame.getImagePtr(), counter);

			}
			else {
				// pause offline player
				usleep(15000);
			}

		} //end while
	} // end big try block
	catch (camera::CameraException e) { //(Camera1394Exception e) {
		std::cout << "Exception: " << e.what() << std::endl;
		exit(1);
	}
	exit(0);
}
