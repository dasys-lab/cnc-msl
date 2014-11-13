/*
 * $Id: VisionImageConverter.cpp 1531 2006-08-01 21:36:57Z phbaer $
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

#include "Camera1394.h"
#include "XVDisplay.h"

#include <byteswap.h>

//#include <ipp.h>


#include "filters/FilterYUVToRGB.h"
#include "filters/FilterYUVToYUVFull.h"
#include "filters/FilterYUVQuickCamToYUV.h"

#include <iostream>


struct IppiSize {
	int height;
	int width;
};


int main(int argc,char *argv[]){


	bool display_frames = true;
	bool directedCam = false;
	XVDisplay * xvDisplay = NULL;

	if(argc > 1){

		for(int i = 1; i < argc; i++){
			if(std::string(argv[i]) == "--false")
				display_frames = false;
			if(std::string(argv[i]) == "--directedCam")
				directedCam = true;

		}

	}



	if(display_frames){
		xvDisplay = new XVDisplay(640, 480, XV_UYVY);
	}



	IppiSize imagesize_;
	imagesize_.width = 640; //camera.getDeviceWidth();
	imagesize_.height = 480; //camera.getDeviceHeight();


	unsigned char * currImage = NULL;

	//ScanLineHelper scanHelper(238, 349, 60, 254, 180);

	FilterYUVToRGB filterYUVToRGB(imagesize_.width, imagesize_.height);
	FilterYUVToYUVFull filterYUVToYUVFull(imagesize_.width, imagesize_.height);
	FilterYUVQuickCamToYUV filterYUVQuickCamToYUV(imagesize_.width, imagesize_.height);

	unsigned char * saved_image = (unsigned char *) malloc(imagesize_.width*imagesize_.height*2);
	unsigned char * yuv_image = NULL;

	unsigned int counter = 0;
	//bool drawOutput = true;

	int imCounter = 1;

	while(1){



		char filename[256];
		char filename2[256];
		char path[256];
		char path2[256];
		strcpy(path, getenv("VISION_LOG"));
		strcpy(path2, getenv("VISION_LOG"));

		sprintf(filename, "/log-image-%04d.raw", imCounter);
		char * path_filename = strcat(path, filename);
		sprintf(filename2, "/log-image-%04d.raw", imCounter);
		char * path_filename2 = strcat(path2, filename2);

		imCounter++;

		//camera.captureBegin();
		printf("LogFileName: %s\n", path_filename);

		FILE * logfile = fopen(path_filename, "r");

		if(logfile != NULL){

			fread(saved_image, sizeof(char), imagesize_.width*imagesize_.height*2, logfile);
			fclose(logfile);

			currImage = filterYUVToYUVFull.process(saved_image, imagesize_.width*imagesize_.height*2);

			FILE * logfile2 = fopen(path_filename2, "w");
			fwrite(currImage, sizeof(char), imagesize_.width*imagesize_.height*3, logfile2);
			fclose(logfile2);



		}
		else{
			printf("Log file not found\n");
			break;
		}


		if(directedCam){
			yuv_image = filterYUVQuickCamToYUV.process((unsigned char *) saved_image, imagesize_.width* imagesize_.height*2);
			currImage = filterYUVToRGB.process(yuv_image, imagesize_.width*imagesize_.height*2);
		}
		else {
			currImage = filterYUVToRGB.process((unsigned char *) saved_image, imagesize_.width*imagesize_.height*2);
		}

		if(xvDisplay != NULL){
			xvDisplay->displayFrameRGB((char *) currImage);
		}

		//camera.captureEnd();
		usleep(10000);

		counter++;


	}


	exit(0);
}
