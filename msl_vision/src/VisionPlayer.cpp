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

#include <iostream>
#include "ros/ros.h"
#include <SystemConfig.h>
#include <DateTime.h>


int main(int argc,char *argv[]){
	SystemConfig* sc;

	try {
		sc = SystemConfig::getInstance();
		camera::ImagingSource* cam = NULL;
		bool sshImage = false, drawObstacles = false, display_frames = true, display_gray = false,
				offline = false, fixed_image = false, drawScanLines = false, help = false, drawCircle = false,
				drawRGB = false, drawSegmented = false, lines = false, log = false, roi = false,
				drawBall = false, directions = false, drawROI = false, drawRoland=false, localizeDebugFlag = false,
				softhdr=false, input=false, isGoalie=false, streamMJpeg=false, sendROIImage=false, sendGrayImage=false,
				localize=true;
		int fixed_number = 1;
		XVDisplay * xvDisplay = NULL;
		XVDisplay * xvDisplay2 = NULL;

		for(int i = 1; i < argc; i++){
			if(std::string(argv[i]) == "--noneLocalize")
				localize = false;
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

		if(help){
			printf("\nOptions for VisionPlayer:\n\n");
			printf("--false: hides window\n");
			printf("--noneLocalize: disables localization (transmission of CorrectedOdometryInfo otherwise Linepoints, Obstacles and Ballhypothesis are executed)\n");
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

		Configuration *vision = (*sc)["Vision"];
		std::string camera_vendor = vision->tryGet<std::string>("Imaging Source", "Vision", "Camera1394Settings", "CameraVendor", NULL);
        short mx = vision->get<short>("Vision", "CameraMX", NULL);
        short my = vision->get<short>("Vision", "CameraMY", NULL);
		short radius = vision->get<short>("Vision", "CameraRadius", NULL);
		short imageWidth = 640;
		short imageHeight = 480;

        int edgethresh = vision->get<int>("Vision", "BallEdgethres", NULL);
        int edgemaskthresh = vision->get<int>("Vision", "BallEdgemaskthres", NULL);
        int maskThresh = vision->get<int>("Vision", "BallTemplMaskThres", NULL);
            Configuration *kh = (*sc)["KickHelper"];
            int kickerCount = (int)kh->tryGet<int>(3, "KickConfiguration", "KickerCount", NULL);
		if(kickerCount>3) kickerCount=3;
		int area = vision->get<int>("Vision", "ImageArea", NULL);

		if(display_frames){
			xvDisplay = new XVDisplay(imageWidth, imageHeight, XV_UYVY);
			xvDisplay2 = new XVDisplay(area, area, XV_UYVY);
        }
		camera::ImagingSource::white_balance_t wb;
        wb.bu = vision->get<int>("Vision", "Camera1394Settings", "WB1", NULL);
        wb.rv = vision->get<int>("Vision", "Camera1394Settings", "WB2", NULL);
		bool enableGainFilter = vision->tryGet<bool>(false, "Vision", "Camera1394Settings", "EnableGainFilter", NULL);
		bool particleFilterAllowed = vision->tryGet<bool>(true, "Vision", "ParticleFilterAllowed", NULL);
		bool detectObstacles = vision->tryGet<bool>(true, "Vision", "DetectObstacles", NULL);

		if(!offline) {
			cam = new camera::ImagingSource(camera_vendor.c_str());
			cam->setVideoMode(DC1394_VIDEO_MODE_640x480_YUV422);
			cam->init();
			std::cout << "Cam init" << std::endl;
			if(isGoalie) cam->setManualSettingModesGoalie();
			if(vision->tryGet<bool>(false, "Vision", "Camera1394Settings", "SetManSettingsMode", NULL)) {
				cam->setManualSettingModes();
			}
			cam->setGamma(vision->get<int>("Vision", "Camera1394Settings", "Gamma", NULL));
			if (vision->tryGet<bool>(false, "Vision", "Camera1394Settings", "UseBrightness", NULL)) {
				cam->setBrightness(vision->get<int>("Vision", "Camera1394Settings", "Brightness", NULL));
			}

			if (vision->get<bool>("Vision", "Camera1394Settings", "AutoGain", NULL)) {

				cam->enableAutoGain(true);
				std::cout << "A tänchen: Autogain is true!!!!" << std::endl;
			} else {
				cam->enableAutoGain(false);
				cam->setGain(vision->get<int>("Vision", "Camera1394Settings", "Gain", NULL));
			}

			cam->setHue(vision->get<int>("Vision", "Camera1394Settings", "Hue", NULL));
			cam->setExposure(vision->get<int>("Vision", "Camera1394Settings", "Exposure", NULL));
			cam->setSaturation(vision->get<int>("Vision", "Camera1394Settings", "Saturation", NULL));
			cam->disableAutoWhiteBalance();

			if(vision->tryGet<bool>(false, "Vision", "Camera1394Settings", "SetManSettingsMode", NULL)) {
				cam->setManualSettingModes();
				cam->enableAutoShutter(false);
			}
			cam->setWhiteBalance(wb);

			cam->setShutter(vision->tryGet<int>(30, "Vision", "Camera1394Settings", "Shutter", NULL));
			cam->setFramerate(DC1394_FRAMERATE_30);
			cam->startCapture();
		}

		SpicaHelper::initialize();
        ColorCalibration::ColorCalibrationHelper::initialize();
        CameraCalibration::CameraCalibrationHelper::initialize();

		unsigned char * currImage = NULL, *currRealImage = NULL, *imageRGB = NULL, *image_gray = NULL, *image_uv = NULL,
				*image_roi = NULL, *image_roiRoland = NULL, *saved_image = NULL, *image_gray_circle = NULL, *image_gray_seg = NULL,
				*image_segmented = NULL, *imageSDir = NULL, *imBall = NULL, *imbrightenedGray = NULL;
		unsigned char * image_gray_saved = (unsigned char *) malloc(area*area);
		unsigned char * prevImage = (unsigned char *) malloc(imageWidth*imageHeight*2);


		ScanLineHelper scanHelper;
		ScanLineHelperBall scanHelperBall;
		DistanceLookupHelper distanceHelper(area);
		LineDistanceHelper lineDistHelper;
		RandomGaussHelper gaussHelper;
		GoalHelperLocalization goalHelper;
		ImageMaskHelper imageMaskHelper(area, area);
		BallHelper ballHelper(area);

		FilterYUVExtractSubImages filterYUVExtractSubImages(imageWidth, imageHeight, area);
        filterYUVExtractSubImages.loadAllLookupTables();
		FilterLinePoints filterLinePoints(area);
		FilterLinePointsROI filterLinePointsROI(area);
		FilterDrawScanLines filterDrawScanLines(area, area);
		FilterSoftHDR filterSoftHDR(imageWidth, imageHeight);

		FilterBox filterBox(area, area);
		FilterAdaptiveROI filterAdaptiveROI(imageWidth, imageHeight, area);
		FilterExtractBlobs filterExtractBlobs(area, area);
		FilterGrayToDarkSeg filterGrayToDarkSeg(area, area);

		FilterDistanceProfile filterDistanceProfile(area, area);
		FilterDistanceProfileNew filterDistanceProfileNew(area, area);

		FilterAddBallBlobsToSeg filterAddBallBlobsToSeg(area, area);
		FilterSegToRGB filterSegToRGB(area,area);


		FilterYUVToRGB filterYUVToRGB(imageWidth, imageHeight);
		FilterHoughCalib filterHoughCalib(imageWidth, imageHeight);
		FilterYUVToGray filterYUVToGray(imageWidth, imageHeight);

		//FilterHistoLin filterHistLin(area, area);
		FilterSobelDir filterSobelDir(area, area);
		FilterTemplateMatching filterTMatch(area, area);

		FilterAdjustGain filterAdjustGain(imageWidth, imageHeight);
		ParticleFilter particleFilter(1200);
		ParticleFilterGoalie particleFilterGoalie(1200);
		LocalizeDebug localizeDebug;

		std::vector<BlobBounds> potYellowGoalBlobs;
		std::vector<BlobBounds> potBlueGoalBlobs;

		//Auto gain
		Brightness *br;
        if(enableGainFilter && !offline) br = Brightness::getInstance(imageWidth, imageHeight, cam);
		Whitepoint *wp;
        if(enableGainFilter && !offline) wp = Whitepoint::getInstance(imageWidth, imageHeight, cam);

		std::vector<LinePoint> linePoints;
		std::vector<LinePoint> linePointsROI;
		std::vector<Goal> yellowGoals;
		std::vector<Goal> blueGoals;
		std::vector<CornerPost> cornerPosts;
		std::vector<ROI> rois;
		std::vector<BlobBounds> ballBlobs;

		unsigned long long visionTimeOmniCamLong;
		int imCounter = fixed_number, counter = 0, writeCounter = 1;

		int *balls;
		int ballCount = 0, clusterCount = 0;
		BallClusterHelp ballClusterHelp;
		ballCluster *cluster = new ballCluster[10000];
		std::vector<ROIData> roiData;
		ROIData curBallROI;

		redwolf::ErrorMinLocalization *newLoc = new redwolf::ErrorMinLocalization();
		Configuration *localization = (*sc)["Localization"];
		int localizationBegin = 0;
		Position pos;
		Position relocPos;
		WeightedPosition weightPosErrorMin;
		pos.x = localization->get<double>("Localization","InitialPosition", SystemConfig::getHostname().c_str(),"X",NULL);;
		pos.y= localization->get<double>("Localization","InitialPosition", SystemConfig::getHostname().c_str(),"Y",NULL);;
		pos.heading = localization->get<double>("Localization","InitialPosition", SystemConfig::getHostname().c_str(),"Heading",NULL);;
		relocPos.heading = pos.heading;
		relocPos.x = pos.x;
		relocPos.y = pos.y;
		const double cornerOffset = localization->get<double>("Localization", "CornerOffset", NULL);
		bool useParticleFilter = localization->get<bool>("Localization","InitialPosition", SystemConfig::getHostname().c_str(), "InitialUseOfParticleFilter", NULL);



		camera::Frame frame;
		if(!offline){
			cam->stopCapture();
			cam->startCapture();
		}
		else
			saved_image = (unsigned char *) malloc(imageWidth*imageHeight*3);


		while(ros::ok()) {
			std::cout << "DEBUG after main while loop" << std::endl;
			yellowGoals.clear();
			blueGoals.clear();
			cornerPosts.clear();
			rois.clear();
			ballBlobs.clear();

            if(!offline){
                if (CameraCalibration::CameraCalibrationHelper::setSettings == true) {
                    CameraCalibration::CameraCalibrationHelper::setSettings = false;
                    CameraCalibration::CameraCalibrationHelper::setCameraSettings(cam);
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

				if(drawRGB)
				{
					std::cout << "DEBUG filterYUVtoRGB" << std::endl;
					imageRGB = filterYUVToRGB.process((unsigned char *) frame.getImagePtr(), imageWidth*imageHeight * 2);
				}
				currImage = (unsigned char *) frame.getImagePtr();
				visionTimeOmniCamLong = supplementary::DateTime::getUtcNowC();
				counter++;

				if(log && counter >= 200 && counter % 10 >= 0){

					char filename[256];
					char path[256];
					strcpy(path, getenv("VISION_LOG"));
					sprintf(filename, "/log-image-%04d.raw", writeCounter);
					char * path_filename = strcat(path, filename);

					FILE * logfile = fopen(path_filename, "w");
					fwrite(currImage, sizeof(char), imageWidth*imageHeight*3, logfile);
					fclose(logfile);
					writeCounter++;
				}
			}
			else {
				char filename[256], filename2[256];
				char path[256], path2[256];
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
					fread(saved_image, sizeof(char), imageWidth*imageHeight*3, logfile);
					fclose(logfile);
				}
				else{
					printf("Log file not found ... Restarting\n");
					imCounter = 1; counter = 0;
					sprintf(filename2, "/log-image-%04d.raw", imCounter);
					char * path_filename2 = strcat(path2, filename2);

					logfile = fopen(path_filename2, "r");
					if(logfile == NULL){
						printf("Log file not found\n");
						break;
					}
				}

				currImage = saved_image;
				if(drawRGB)
					imageRGB = filterYUVToRGB.process(currImage, imageWidth*imageHeight*2);

				visionTimeOmniCamLong = supplementary::DateTime::getUtcNowC();
			}
			struct timeval tv_before;
			gettimeofday(&tv_before, NULL);
			visionTimeOmniCamLong -= 700000;
			TimeHelper::getInstance()->setVisionTimeOmniCam(visionTimeOmniCamLong);

			printf("Stage 1: Image precomputation\n");
			if(softhdr) {
				currRealImage = currImage;
				if(counter>0) {
					currImage = filterSoftHDR.process(currRealImage, prevImage, imageWidth,imageHeight);
					if(drawRGB)
						imageRGB = filterYUVToRGB.process(currImage, imageWidth*imageHeight*2);
				}
			}

			// Filters for calibrating camera middle
			if(drawCircle){
				image_gray_circle = filterYUVToGray.process(currImage, imageWidth*imageHeight*2);
				filterHoughCalib.DrawCircle(mx, my, radius, (unsigned char *) image_gray_circle, imageWidth, imageHeight);
				filterHoughCalib.DrawCircle(mx, my, 10, (unsigned char *) image_gray_circle, imageWidth, imageHeight);
			}

			//Extract Gray and UV Subimages (area x area) from the camera image
			//image_gray not modified
			//image_uv modified with trhesholds
			filterYUVExtractSubImages.process(currImage, imageWidth, imageHeight, mx, my, image_gray, image_uv, image_roi, image_roiRoland, imbrightenedGray);
			printf("Stage 2: Copy gray scale Image\n");
			memcpy(image_gray_saved, image_gray, area*area);
			printf("Stage 3: Compute Image for Obstacles\n");
			//Segmentation for collision avoidance and opponent detection
			image_gray_seg = filterGrayToDarkSeg.process(image_gray, image_uv, area, area, imageMaskHelper);

			printf("Stage 4: Detect Linepoints\n");
			//Get LinePoints for Self-Localization
			image_gray = filterLinePoints.process((unsigned char *) image_gray, area, area, linePoints, distanceHelper, scanHelper);
			//Send linepointlist here:
			SpicaHelper::sendLinePoints(linePoints, visionTimeOmniCamLong);


			printf("Stage 5: Detect ROIs\n");
			roiData = filterLinePointsROI.process((unsigned char *) image_uv, area, area, linePointsROI, distanceHelper, scanHelperBall);

			// rio Data contains which info???
			printf("Stage 6: Insert Tracked ROI into ROI List\n");
			roiData.insert(roiData.end()-kickerCount, curBallROI);

			printf("Stage 7: Compute Sobel on ROIs\n");
			imageSDir = filterSobelDir.process(image_uv, image_uv, roiData, area, area, edgethresh, edgemaskthresh);

			printf("Stage 8: Apply Templatematching\n");
			imBall = filterTMatch.process(imageSDir, balls, ballCount, image_uv, roiData, maskThresh, area, area, 4, 19, 6, image_roi);
			printf("Stage 9: Clustering Ball Hypothesis\n");
			clusterCount = ballClusterHelp.clusterBalls(balls, ballCount, cluster, 200);
			//ballClusterHelp.clusterStdOut(cluster, clusterCount, mx, my, false);

			// filter for converting the segmented image to an rgb representation
			if(drawSegmented)
				image_segmented = filterSegToRGB.process(image_gray_seg, area*area);

			// Filters for holder calibration
			if(drawScanLines)
				image_gray = filterDrawScanLines.process((unsigned char *) image_gray, area, area, scanHelper, true);

			Particle maxParticle;
			if (localize && localizationBegin++ > 50)
			{
				printf("Stage 10: Localization\n");
				if (useParticleFilter) {
					if(!isGoalie)
					particleFilter.iterate(linePoints, lineDistHelper, gaussHelper, yellowGoals, blueGoals, cornerPosts);
					else
					particleFilterGoalie.iterate(linePoints, lineDistHelper, gaussHelper, yellowGoals, blueGoals, cornerPosts);

					WeightedPosition wpos;
					if(isGoalie) wpos = particleFilter.getEstimatedPosition();
					else wpos = particleFilterGoalie.getEstimatedPosition();
					Position pos;
					pos.x = wpos.x;
					pos.y = wpos.y;
					pos.heading = wpos.heading;
					printf("VisionPlayer LocalizationType: ParticleFilter: %f %f %f\n", pos.x, pos.y, pos.heading);
					Particle maxParticle;
					if(!isGoalie) maxParticle = particleFilter.getMaxParticle();
					else maxParticle = particleFilterGoalie.getMaxParticle();
				}
				else {
					weightPosErrorMin = newLoc->estimatedPosition(pos, linePoints, lineDistHelper);
					pos.x = weightPosErrorMin.x;
					pos.y = weightPosErrorMin.y;
					pos.heading = weightPosErrorMin.heading;
					printf("VisionPlayer LocalizationType: ErrorMin: %f: %f %f %f\n",weightPosErrorMin.weight, pos.x, pos.y, pos.heading);
					newLoc->writeCoi();

					maxParticle.posx = pos.x;
					maxParticle.posy = pos.y;
					maxParticle.heading = pos.heading;
					maxParticle.weight = weightPosErrorMin.weight;

					// reloc trigger
					if(SpicaHelper::reloc == true){
						SpicaHelper::reloc = false;
						pos.x = relocPos.x;
						pos.y = relocPos.y;
						pos.heading = relocPos.heading;
					}
				}
			}

			Point p;
			if(localize) {
				printf("Stage 11: Compute Ballhypothesis Balls\n");
				p = ballHelper.getBallFromBlobs(cluster, clusterCount, roiData, &maxParticle);
				printf("Stage 12: Compute Goal positions\n");
				goalHelper.getGoalsFromPosition(pos);
			} else {
				printf("Stage 11: Send Ballhypothesis\n");
				ballHelper.sendBallHypotesis(cluster, clusterCount, roiData);
			}
			if(roiData.size() > 0)
				curBallROI = roiData[0];

			printf("Stage 13: Obstacle avoidance\n");
            if(detectObstacles) filterDistanceProfileNew.process(image_gray_saved, area, area, scanHelper, distanceHelper);


			struct timeval tv_after;
			gettimeofday(&tv_after, NULL);
			long timediff = tv_after.tv_usec - tv_before.tv_usec;
			if(timediff < 0)
				timediff += 1000000;
			printf("\n\nTime for FilterChain: %ld\n\n", timediff);

            // ColorCalibration
            ColorCalibration::ColorCalibrationHelper::sendImage(currImage, imageWidth, imageHeight);

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

				Particle drawParticle;
				drawParticle.posx = pos.x;
				drawParticle.posy = pos.y;
				drawParticle.heading = pos.heading;
				drawParticle.weight = 0.9;
				localizeDebug.drawFieldForParticle(drawParticle, 0);
				localizeDebug.drawParticlesOnField(particleFilter);
			}

			if(softhdr) {
			  memcpy(prevImage, currRealImage, imageWidth*imageHeight*2);
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

				printf("Stage 15: Gain Adjustments\n");
				if(softhdr && !offline) {
					int currGain = 670; //camera->getGain();
					if(counter % 2 == 0)
						currGain += 150;
					else
						currGain -= 150;
					cam->setGain(currGain);
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
