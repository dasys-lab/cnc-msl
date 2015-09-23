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
#include "filters/FilterLinePointsCalib.h"
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
#include "camControl/Whitepoint.h"
#include "camControl/Brightness.h"

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
		bool lines = false;
		bool log = false;
		bool roi = false;
		bool drawBall = false;
		bool directions = false;
		bool drawROI = false;
		bool drawRoland=false;
		bool softhdr=false;
		bool input=false;
		bool sendROIImage=false, sendGrayImage=false;
		bool localizeDebugFlag=false;

		int fixed_number = 1;

		XVDisplay * xvDisplay = NULL;
		XVDisplay * xvDisplay2 = NULL;

		if(argc > 1){

			for(int i = 1; i < argc; i++){
				if(std::string(argv[i]) == "--false")
					display_frames = false;
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
				if(std::string(argv[i]) == "--lines")
					lines = true;
				if(std::string(argv[i]) == "--log")
					log = true;
				if(std::string(argv[i]) == "--sshImage")
					sshImage = true;
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
			}


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
		ImageMaskHelper imageMaskHelper(area, area);
		BallHelper ballHelper(area);

		FilterYUVExtractSubImages filterYUVExtractSubImages(imagesize_.width, imagesize_.height, area);
		FilterLinePointsCalib filterLinePoints(area);
		FilterLinePointsROI filterLinePointsROI(area);
		FilterDrawScanLines filterDrawScanLines(area, area);
		FilterSoftHDR filterSoftHDR(imagesize_.width, imagesize_.height);

		FilterYUVToRGB filterYUVToRGB(imagesize_.width, imagesize_.height);
		FilterYUVToGray filterYUVToGray(imagesize_.width, imagesize_.height);
		FilterHoughCalib filterHoughCalib(imagesize_.width, imagesize_.height);

		FilterSobelDir filterSobelDir(area, area);
		FilterTemplateMatching filterTMatch(area, area);

		FilterAdjustGain filterAdjustGain(imagesize_.width, imagesize_.height);
		LocalizeDebug localizeDebug;


		//Auto gain
		Brightness *br;
		if(enableGainFilter && !offline) br=Brightness::getInstance(640, 480, cam);
		Whitepoint *wp;
		if(enableGainFilter && !offline) wp=Whitepoint::getInstance(640, 480, cam);

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

		camera::Frame frame;
		if(!offline){
			printf("Stop ISO Transmission\n");

			cam->stopCapture();

			printf("Before Start ISO Transmission\n");
			cam->startCapture();
		}
		else
			saved_image = (unsigned char *) malloc(imagesize_.width*imagesize_.height*3);


		while(ros::ok()){

			std::cout << "DEBUG after main while loop" << std::endl;

			unsigned long long visionTimeOmniCamLong;

			if(!offline){


				std::cout << "DEBUG start Capture" << std::endl;

				if (!cam->getFrame(frame))
				{
					std::cerr << "Error while capturing image, aborting!" << std::endl;
					std::cout << "Error while capturing image, aborting!" << std::endl;
					exit(1);
				}

				cam->setWhiteBalance(wb);



				if(drawRGB)
				{
					std::cout << "DEBUG filterYUVtoRGB" << std::endl;
					imageRGB = filterYUVToRGB.process((unsigned char *) frame.getImagePtr(), imagesize_.width * imagesize_.height * 2);
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
					fwrite(currImage, sizeof(char), imagesize_.width*imagesize_.height*3, logfile);
					fclose(logfile);
					writeCounter++;
				}


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
				if(drawRGB) {
					imageRGB = filterYUVToRGB.process(currImage, imagesize_.width* imagesize_.height*2);
				}
				visionTimeOmniCamLong = supplementary::DateTime::getUtcNowC();
			}

			struct timeval tv_before;
			gettimeofday(&tv_before, NULL);

			std::vector<ROI> rois;
			rois.clear();

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


			printf("Stage 3: Detect ROIs\n");
			roiData = filterLinePointsROI.process((unsigned char *) image_roi, area, area, linePointsROI, distanceHelper, scanHelperBall);

			//New Ball
			//does what?
			printf("Stage 4: Compute Sobel on ROIs\n");
			imageSDir = filterSobelDir.process(image_uv, image_uv, roiData, area, area, edgethresh, edgemaskthresh);

			printf("Stage 5: Apply Templatematching\n");
			imBall = filterTMatch.process(imageSDir, balls, ballCount, image_uv, roiData, maskThresh, area, area, 7, 14, 5, image_roi);
			clusterCount = ballClusterHelp.clusterBalls(balls, ballCount, cluster, 200);

			cout << "Directions:" << endl;
			double angle=100000;
			for(int i=0; i<clusterCount; i++) {
				double bX = cluster[i].x-area/2.0;
				double bY = cluster[i].y-area/2.0;
				double d = bX*bX + bY*bY;
				double irad = cluster[i].sizeSum / cluster[i].balls;

                		bool valid=true;

		                //Ignore Balls which are in other balls
			        for(int n=0; n<clusterCount; n++) {
                		        if(n==i) continue;
		                        int nrad = cluster[n].sizeSum / cluster[n].balls;
                		        int xdist = cluster[i].x - cluster[n].x;
		                        int ydist = cluster[i].y - cluster[n].y;

                		        if(xdist < 0) xdist = -xdist;
		                        if(ydist < 0) ydist = -ydist;

	                	        if(nrad>irad && (xdist*ydist < nrad*nrad)) {
        	                	        valid=false;
                		        }
		                }
		                if(!valid)
                		        continue;

				if(irad>=9 && irad<=13 && d < 150*150 && d > 120*120) {
					angle = atan2(-bY, bX)*180/3.141592;
					if(angle < 0) angle += 360;
					cout << "Direction: "<< angle << " "  << bX << " " << bY << " d " << sqrt(d) << " " << irad << endl;
				}
			}

			printf("Stage 6: Detect Linepoints\n");
			//Get LinePoints for Self-Localization
			image_gray = filterLinePoints.process((unsigned char *) image_gray, area, area, linePoints, distanceHelper, scanHelper, angle);


			// Filters for holder calibration
			if(drawScanLines)
				image_gray = filterDrawScanLines.process((unsigned char *) image_gray, area, area, scanHelper, true);


			struct timeval tv_after;
			gettimeofday(&tv_after, NULL);

			long timediff = tv_after.tv_usec - tv_before.tv_usec;
			if(timediff < 0)
				timediff += 1000000;

			printf("\n\nTime for FilterChain: %ld\n\n", timediff);

			if(localizeDebugFlag && counter >= 0){
				SpicaHelper::vdd->list.clear();
				for(unsigned int i = 0; i < linePoints.size(); i++){
					msl_msgs::Point2dInfo tmp;
					tmp.x = linePoints[i].x;
					tmp.y = linePoints[i].y;
					SpicaHelper::vdd->list.push_back(tmp);
					msl_msgs::PositionInfo pi;
					pi.x = 0;
					pi.y = 0;
					pi.angle = 0;
					pi.certainty = 1.0;
					SpicaHelper::vdd->position = pi;
					SpicaHelper::vdd->distanceScan = SpicaHelper::wm->distanceScan;
					SpicaHelper::vdd->ball = SpicaHelper::wm->ball;
					SpicaHelper::vdd->locType.type = SpicaHelper::wm->odometry.locType.type;
					SpicaHelper::vdd->obstacles = SpicaHelper::wm->obstacles;
					SpicaHelper::vdd->senderID = supplementary::SystemConfig::getOwnRobotID();
				}
				SpicaHelper::sendDebugMsg();
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
					//ballHelper.visualizeBall(image_gray, area, p, 10);
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
				//ballHelper.visualizeBall(image_gray, area, p, 10);
				filterLinePointsROI.visualizeROIs(image_gray, roiData, area, area);
				SpicaHelper::streamGreyMJPEG(image_gray, area, area);
			} else if (sendROIImage) {
				SpicaHelper::streamGreyMJPEG(image_roi, area, area);
			}

			SpicaHelper::send();
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
				if(enableGainFilter) br->process(frame.getImagePtr(), counter);
				//wp->process(frame.getImagePtr(), counter);

			}
			else {
				// pause offline player
				usleep(100);
			}
		} //end while
	} // end big try block
	catch (camera::CameraException e) { //(Camera1394Exception e) {
		std::cout << "Exception: " << e.what() << std::endl;
		exit(1);
	}
	exit(0);
}
