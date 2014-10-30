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
#include <signal.h>

#include <sys/time.h>
#include "CameraQuickCam.h"
#include "XVDisplay.h"

#include <byteswap.h>

//#include <ipp.h>

#include "filters/FilterYUVToRGB.h"
#include "filters/FilterYUVToGray.h"
#include "filters/FilterYUVQuickCamToYUV.h"
#include "filters/FilterLinePoints.h"
#include "filters/FilterYUVExtractImages.h"
#include "filters/FilterBox.h"
#include "filters/FilterExtractBlobsDirected.h"
#include "filters/FilterDrawScanLines.h"
#include "filters/FilterCalibrationDirected.h"
#include "filters/FilterAdaptiveROIDirected.h"

#include "filters/FilterSobelDir.h"
#include "filters/FilterTemplateMatchingGoalie.h"
#include "filters/FilterHistoLin.h"

#include "filters/FilterLinePointsROIDirected.h"

#include "filters/FilterAdjustGainDirected.h"
#include "filters/FilterYUVCountColoredDots.h"
#include "helpers/ExtensionDirectedHelper.h"
//#include "filters/FilterWriteBallColors.h"
 
#include <iostream>
#include "helpers/DistanceLookupHelper.h"
//#include "helpers/BallHelperDirected.h"
#include "helpers/SpicaDirectedHelper.h"
#include "helpers/SharedBallDirectedHelper.h"
//#include "helpers/CorrectedOdometryDummy.h"
#include "helpers/BallClusterHelp.h"
#include "helpers/BallHelperDirectedGoalie.h"

#include "helpers/ScanLineHelperDirected.h"

#include "helpers/TimeHelper.h"

#include <SystemConfig.h>
#include <DateTime.h>

using namespace std;

struct IppiSize {
	int height;
	int width;
};


CameraQuickCam * camera = NULL;


void int_handler(int sig)
{
	int mypid = getpid();
	printf("[%i] Ouch - shot in the ...\n", mypid);
	printf("[%i] exit\n", mypid);
	if(camera == NULL)
		printf("Camera is already NULL\n");
	else
		delete camera;
	exit(2);
}




int main(int argc,char *argv[]){
	signal(SIGINT, int_handler);

	IppiSize imagesize_;
	imagesize_.width = 640; // camera.getDeviceWidth();
	imagesize_.height = 480; //camera.getDeviceHeight();

	int refx = 0;
	int refy = 0;

	int areaWidth = 640;
	int areaHeight = 480;

        bool display_frames = true;
        bool display_gray = false;
        bool offline = false;
        bool fixed_image = false;
        bool log = false;
	bool segmented = false;
	bool directions = false;
	bool rgb = false;
	bool roi = false;
	bool drawScanLines = false;
	bool drawBall = false;
	bool writeBallColor = false;
	bool calibrate = false;
	bool drawROI = false;
        int fixed_number = 1;

	try {

		SystemConfigPtr sc = SystemConfig::getInstance();

	
		if(argc > 1){

                        for(int i = 1; i < argc; i++){
                                if(std::string(argv[i]) == "--false")
                                        display_frames = false;
                                if(std::string(argv[i]) == "--gray") {
                                        display_gray = true;
					display_frames = true;
				}
                                if(std::string(argv[i]) == "--offline")
                                        offline = true;
				if(std::string(argv[i]) == "--rgb") {
					display_frames = true;
					rgb = true;
				}
                                if(std::string(argv[i]) == "--log")
                                        log = true;
				if(std::string(argv[i]) == "--segmented") {
					display_frames = true;
					segmented = true;
				}
				if(std::string(argv[i]) == "--dir") {
					display_frames = true;
					directions = true;
				}
				if(std::string(argv[i]) == "--scanlines") {
					display_frames = true;
					drawScanLines = true;
				}
				if(std::string(argv[i]) == "--roi") {
					display_frames = true;
					roi = true;
				}
				if(std::string(argv[i]) == "--drawROI")
					drawROI = true;
				if(std::string(argv[i]) == "--ball")
					drawBall = true;
				if(std::string(argv[i]) == "--writeBallColors")
					writeBallColor = true;
				if(std::string(argv[i]) == "--calibrate")
					calibrate = true;
                                if(std::string(argv[i]) == "--fixed"){
                                        fixed_image = true;
                                        fixed_number = atoi(argv[i+1]);
                                }	
				if(std::string(argv[i]) == "--begin"){
                                        fixed_number = atoi(argv[i+1]);
                                }

                        }


                }

		XVDisplay * xvDisplay = NULL;
		XVDisplay * xvDisplay2 = NULL;
		XVDisplay * xvDisplay3 = NULL;
		if(display_frames) {
			// capture
			xvDisplay = new XVDisplay(imagesize_.width, imagesize_.height, XV_UYVY);
			xvDisplay->setTitle("capture");

			// white-balanced
			xvDisplay2 = new XVDisplay(imagesize_.width, imagesize_.height, XV_UYVY);
			xvDisplay2->setTitle("whitebalanced");

			// gray
			xvDisplay3 = new XVDisplay(imagesize_.width, imagesize_.height, XV_UYVY);
			xvDisplay3->setTitle("linepoints");
		}

		if(!offline)
			camera = new CameraQuickCam();

		Configuration *vision = (*sc)["Vision"];

		bool autoGain = vision->get<bool>("Vision", "CameraQuickSettings", "AutoGain", NULL);
		int gamma = vision->get<int>("Vision", "CameraQuickSettings", "Gamma", NULL);
		int gain = vision->get<int>("Vision", "CameraQuickSettings", "Gain", NULL);
		int hue = vision->get<int>("Vision", "CameraQuickSettings", "Hue", NULL);
		int exposure = vision->get<int>("Vision", "CameraQuickSettings", "Exposure", NULL);
		int saturation = vision->get<int>("Vision", "CameraQuickSettings", "Saturation", NULL);
		int edgethresh = vision->get<int>("Vision", "CameraQuickSettings", "edgethres", NULL);
		int edgemaskthresh = vision->get<int>("Vision", "CameraQuickSettings", "Edgemaskthres", NULL);
		int maskThresh = vision->get<int>("Vision", "CameraQuickSettings", "TemplMaskThres", NULL);
		int sharpness = vision->get<int>("Vision", "CameraQuickSettings", "Sharpness", NULL);
		int brightness = vision->get<int>("Vision", "CameraQuickSettings", "Brightness", NULL);
		int contrast = vision->get<int>("Vision", "CameraQuickSettings", "Contrast", NULL);
		int wbal = vision->get<int>("Vision", "CameraQuickSettings", "WB1", NULL);

		printf("AutoGain: %d\n", autoGain);
		printf("Gamma %d\n", gamma);
		printf("Gain %d\n", gain);
		printf("Hue %d\n",hue);
		printf("Exposure %d\n", exposure);
		printf("Saturation %d\n", saturation);
		printf("Sharpness %d\n", sharpness);
		printf("Brightness %d\n", brightness);
		printf("Contrast %d\n", contrast);

		if(!offline) {
//			camera->set_auto_white_balance_off();
			camera->set_auto_exposure_off();
			camera->set_sharpness(sharpness);
			camera->set_brightness(brightness); 
			camera->set_contrast(contrast); 			
			camera->set_gamma(gamma);
			camera->set_saturation(saturation);
			camera->set_hue(hue);
			camera->set_exposure(exposure);


			camera->set_gain(gain);
			//camera->set_white_balance(wbal);
		}



		unsigned char * currImage = NULL;
		unsigned char * imageRGB = NULL;
		unsigned char * imageYUV = NULL;
		unsigned char * image_gray = NULL;
		unsigned char * image_uv = NULL;
		unsigned char * imageHSV = NULL;
		unsigned char * saved_image = NULL;
		unsigned char * imBall = NULL;
		unsigned char * imageSDir = NULL;
		unsigned char * image_roi = NULL;
		unsigned char * image_roi_saved = NULL;
		unsigned char * imagelin = NULL;

		if(offline)
			saved_image = (unsigned char *) malloc(imagesize_.width*imagesize_.height*3);

		image_roi_saved = (unsigned char *) malloc(imagesize_.width*imagesize_.height);

		SpicaDirectedHelper::initialize();
		SharedBallDirectedHelper * sharedBallHelper = SharedBallDirectedHelper::getInstance();

		DistanceLookupHelper distanceHelper("DistanceLookupQuickCam.dat", areaWidth, areaHeight);
		BallHelperDirectedGoalie ballHelper;
		//CorrectedOdometryDummy correctedOdometryDummy;

		ScanLineHelperDirected scanLineHelp(imagesize_.width, imagesize_.height);
		FilterDrawScanLines filterDrawScanLines(imagesize_.width, imagesize_.height);

		FilterAdjustGainDirected filterAdjustGain(imagesize_.width, imagesize_.height, gain, exposure);

		FilterYUVToRGB filterYUVToRGB(imagesize_.width, imagesize_.height);
		FilterYUVQuickCamToYUV filterYUVQuickCamToYUV(imagesize_.width, imagesize_.height);

		FilterYUVExtractImages filterYUVExtractImages(imagesize_.width, imagesize_.height, refx, refy, areaWidth, areaHeight);
		FilterBox filterBox(imagesize_.width, imagesize_.height);
		FilterAdaptiveROIDirected filterAdaptiveROIDirected(imagesize_.width, imagesize_.height, areaWidth, areaHeight);
		FilterExtractBlobsDirected filterExtractBlobs(imagesize_.width, imagesize_.height);

		FilterYUVToGray filterYUVToGray(imagesize_.width, imagesize_.height);

		FilterLinePointsROIDirected linePointROI(imagesize_.width, imagesize_.height);

		FilterHistoLin filterHistLin(imagesize_.width, imagesize_.height);
		FilterSobelDir filterSobelDir(imagesize_.width, imagesize_.height);
		FilterTemplateMatchingGoalie filterTMatch(imagesize_.width, imagesize_.height);
		FilterCalibrationDirected filterCalibrationDirected(imagesize_.width,  imagesize_.height);
		
		//FilterWriteBallColors writeBallColors(imagesize_.width, imagesize_.height);
		FilterYUVCountColoredDots dotCounter(imagesize_.width, imagesize_.height, refx, refy, areaWidth, areaHeight);
		ExtensionDirectedHelper extHelper;
		extHelper.initialize();

		int *balls;
		int ballCount = 0, clusterCount = 0;
		BallClusterHelp ballClusterHelp;
		ballCluster *cluster = new ballCluster[200];

		std::vector<LinePoint> linePoints;
		linePoints.clear();


		int imCounter = fixed_number;
		int counter = 0;
		int writeCounter = 1;

		struct timeval tv_before;
		gettimeofday(&tv_before, NULL);

		std::vector<ROIData> roiData;
		roiData.clear();
		ROIData curBallROI;

		while(1){
			
			unsigned long long visionTimeOmniCamLong;

			gettimeofday(&tv_before, NULL);

			if(!offline) {
				camera->captureBegin();

				currImage = (unsigned char *) camera->getCaptureBuffer();

				counter++;

				if(log && counter >= 200 && counter % 1 == 0){

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

				currImage = saved_image;
			}


			visionTimeOmniCamLong = supplementary::DateTime::getUtcNowC();

			visionTimeOmniCamLong -= 700000;

			std::vector<ROI> rois;
			rois.clear();

			std::vector<BlobBounds> ballBlobs;
			ballBlobs.clear();


			TimeHelper::getInstance()->setVisionTimeOmniCam(visionTimeOmniCamLong);

			imageYUV = filterYUVQuickCamToYUV.process(currImage, imagesize_.width* imagesize_.height*2);


////////////////////////////////////////////////
			long long dc = dotCounter.process(imageYUV);

			std::cout << "Red Pixels:  " << dc << std::endl;
			if(dc > 10000) {
				cout << "To Many colored Pixels ... I try to extend upper Extension " << dc << endl;


				int error = extHelper.extendUpperExtension();
				if(error > 0) {
					cout << "Succeded ------------" << error << endl;
					//usleep(500000);
				}
				else {
					cout << "Failed with Errorcode: " << error << endl;
				}
				continue;
			}
////////////////////////////////////////////////





			if(display_gray)
				image_gray = filterYUVToGray.process((unsigned char *) imageYUV, imagesize_.width* imagesize_.height*2);
			filterYUVExtractImages.process(imageYUV, image_gray, image_uv, image_roi);

			memcpy(image_roi_saved, image_roi, imagesize_.width* imagesize_.height);

			if(calibrate)
				filterCalibrationDirected.process(image_gray, imagesize_.width, imagesize_.height);

			roiData.clear();
			/*
			roiData = linePointROI.process(image_roi, imagesize_.width, imagesize_.height, linePoints, distanceHelper, scanLineHelp);
			roiData.push_back(curBallROI);
			*/
			filterBox.process(image_roi_saved, areaWidth, areaHeight);
			filterAdaptiveROIDirected.process(filterBox.getResult());
			filterExtractBlobs.process(filterAdaptiveROIDirected.getResult(), areaWidth, areaHeight, rois, 255, ballBlobs, 5000);

			//ballHelperDirected.getBallFromBlobs(ballBlobs, NULL);

			//correctedOdometryDummy.iterate();
			if(rgb)
				imageRGB = filterYUVToRGB.process(imageYUV, imagesize_.width* imagesize_.height*2);

			//Gauss (Box) Filtering of UV_Image
			filterBox.process(image_uv, imagesize_.width, imagesize_.height);
			
			//image_uv = filterHistLin.process(image_uv, imagesize_.width, imagesize_.height, 0);
			/*
			imageSDir = filterSobelDir.process(image_uv, image_uv, roiData, imagesize_.width, imagesize_.height, edgethresh, edgemaskthresh);
			imBall = filterTMatch.process(imageSDir, balls, ballCount, image_uv, roiData, maskThresh, imagesize_.width, imagesize_.height, 4, 41, 10);

			clusterCount = ballClusterHelp.clusterBalls(balls, ballCount, cluster, 200);
			*/
			Point p = ballHelper.getBallFromBlobs(cluster, clusterCount, roiData, ballBlobs);
			if(roiData.size() > 0)
				curBallROI = roiData[0];

//tv_before = tv_after;

// 			if(writeBallColor) {
// 				if(clusterCount>0)
// 					writeBallColors.process(imageYUV, cluster[0], "fann/ball.data", imagesize_.width, imagesize_.height);
// 			}

			if(xvDisplay != NULL){
				
				//xvDisplay->displayFrameRGB((char *) imageRGB);
				//xvDisplay->displayFrameGRAY((char *) filterBox.getResultChar());
				if(display_gray) {
					ballHelper.visualizeBall(image_gray, imagesize_.width, p, 10);

					if(drawROI)
                                                linePointROI.visualizeROIs(image_gray, roiData, imagesize_.width, imagesize_.height);
					if(drawBall)
						ballClusterHelp.visualizeCluster(image_gray, imagesize_.width, imagesize_.height, cluster, clusterCount);
					// Filters for holder calibration
					if(drawScanLines)
						image_gray = filterDrawScanLines.process((unsigned char *) image_gray, imagesize_.width, imagesize_.height, scanLineHelp, true);
					xvDisplay->displayFrameGRAY((char *) image_gray);
				}
				else if(segmented) {
					if(drawROI)
                                                linePointROI.visualizeROIs(image_uv, roiData, imagesize_.width, imagesize_.height);

					if(drawBall)
					ballClusterHelp.visualizeCluster(image_uv, imagesize_.width, imagesize_.height, cluster, clusterCount);
					xvDisplay->displayFrameGRAY((char *) image_uv);
				}
				else if(directions) {
					if(drawROI)
                                                linePointROI.visualizeROIs(imageSDir, roiData, imagesize_.width, imagesize_.height);

					if(drawBall)
					ballClusterHelp.visualizeCluster(imageSDir, imagesize_.width, imagesize_.height, cluster, clusterCount);
					xvDisplay->displayFrameGRAY((char *) imageSDir);
				}
				else if(rgb)
					xvDisplay->displayFrameRGB((char *) imageRGB);
				else if(roi) {
					if(drawROI)
                                                linePointROI.visualizeROIs(image_roi, roiData, imagesize_.width, imagesize_.height);

					if(drawBall)
						ballClusterHelp.visualizeCluster(image_roi, imagesize_.width, imagesize_.height, cluster, clusterCount);
					xvDisplay->displayFrameGRAY((char *) image_roi);
				}
				else {
					xvDisplay->displayFrameGRAY((char *) filterAdaptiveROIDirected.getResult());
					printf("Try to display adaptive ROI\n");
				}
			}

			if(!offline) {
				camera->captureEnd();
				//filterAdjustGain.process(camera, image_gray, 100, imagesize_.width, imagesize_.height);
			}
			else
				usleep(300000);

			struct timeval tv_after;
			gettimeofday(&tv_after, NULL);

			long timediff = tv_after.tv_usec - tv_before.tv_usec;
			if(timediff < 0)
				timediff += 1000000;

			printf("\n\nTime for FilterChain: %ld\n\n", timediff);


		}


	} catch (CameraQuickCamException e) {
		std::cout << "Exception: " << e.what() << std::endl;
		exit(1);
	}
	exit(0);
}
