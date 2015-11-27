/*
 * $Id: ROIHelperOmniCam.cpp 1531 2006-08-01 21:36:57Z phbaer $
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
#include "ROIHelperOmniCam.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>

#include "TimeHelper.h"


ROIHelperOmniCam * ROIHelperOmniCam::instance_ = NULL;

ROIHelperOmniCam * ROIHelperOmniCam::getInstance(){

	if(instance_ == NULL)
		instance_ = new ROIHelperOmniCam();
	return instance_;


}


ROIHelperOmniCam::ROIHelperOmniCam(){

	this->sc = SystemConfig::getInstance();

	Configuration *vision = (*this->sc)["Vision"];

	int imWidth = vision->get<int>("Vision", "ImageArea", NULL);
	int imHeight = vision->get<int>("Vision", "ImageArea", NULL);
	CameraZ = vision->get<int>("Vision", "CameraZ", NULL);

	mx = (short)imHeight/2;
	my = (short)imWidth/2;

	scWIDTH = imWidth;
	scHEIGHT = imHeight;


	init();

}

ROIHelperOmniCam::~ROIHelperOmniCam(){


	cleanup();

}


void ROIHelperOmniCam::init(){



}

void ROIHelperOmniCam::cleanup(){


}


ROI ROIHelperOmniCam::getROIForObject(double x, double y, double z, double radius, DistanceLookupHelper & distanceHelper){


	ROI resultROI;
	resultROI.confidence = 0.0;

	double lineAngle = -atan2(y,x);
	short ax = (short) lrint((cos(lineAngle)*10.0) + mx);
	short ay = (short) lrint((sin(lineAngle)*10.0) + my);
	short ex = (short) lrint(cos(lineAngle)*(mx - 5.0) + mx);
	short ey = (short) lrint(sin(lineAngle)*(my - 5.0) + my);

	double dist = sqrt(x*x + y*y + (z - CameraZ)*(z - CameraZ));
	double distXY = sqrt(x*x + y*y);

	double angleDiff = atan2(radius, dist);

	double angle = atan2((z - CameraZ), distXY);
	double angleMin = angle - angleDiff;
	double angleMax = angle + angleDiff;

	int xcoordMin = 0;
	int ycoordMin = 0;
	int xcoordMax = 0;
	int ycoordMax = 0;
	int xcoordMean = 0;
	int ycoordMean = 0;

	goForLine(ax, ay, ex, ey, angleMin, distanceHelper, &xcoordMin, &ycoordMin);
	goForLine(ax, ay, ex, ey, angleMax, distanceHelper, &xcoordMax, &ycoordMax);
	goForLine(ax, ay, ex, ey, angle, distanceHelper, &xcoordMean, &ycoordMean);

	double ROIlength = sqrt((xcoordMin - xcoordMax)*(xcoordMin - xcoordMax) + (ycoordMin - ycoordMax)*(ycoordMin - ycoordMax))/2.0;

	resultROI.bottom = (int) lrint(xcoordMean + ROIlength*1.0);
	resultROI.top = (int) lrint(xcoordMean - ROIlength*1.0);
	resultROI.left = (int) lrint(ycoordMean - ROIlength*1.0);
	resultROI.right = (int) lrint(ycoordMean + ROIlength*1.0);

	return resultROI;

}




void ROIHelperOmniCam::goForLine(short ax, short ay, short ex, short ey, double angle, DistanceLookupHelper & distanceHelper, int * xcoord, int * ycoord){


	short x = ax;
	short y = ay;
	short D = 0;
	short HX = ex - ax;
	short HY = ey - ay;
	short xInc = 1;
	short yInc = 1;

	double minAngleDiff = 2.0*M_PI;
	int minIndexX = 0;
	int minIndexY = 0;

	double * LookupTable = distanceHelper.getLookupTable();

//	printf("ax: %d ay: %d ex: %d ey: %d\n", ax, ay, ex, ey);

	//printf("linePointer: %d\n", line);
	//printf("nPointer: %d\n", nPoints);
		
	if(HX < 0) {
		xInc = -1; 
		HX = -HX;
	}

	if(HY < 0) {
		yInc = -1; 
		HY = -HY;
	}

	if(HY <= HX) {
		short c = 2 * HX; 
		short M = 2 * HY;
		short counter = 0;
		while(1) { 

			if(x >= 0 && x < scHEIGHT && y >= 0 && y < scWIDTH){

				double angleXY = atan2(-CameraZ, LookupTable[x*scWIDTH + y]);
				if(angleXY > M_PI/2.0)
					angleXY = angleXY - M_PI/2.0;

				double angleDiff = fabs(angleXY - angle);
				if(angleDiff < minAngleDiff){
					minAngleDiff = angleDiff;
					minIndexX = x;
					minIndexY = y;

				}
			}
			else{
				//printf("out of image!!!!!!!!!!!!!!!!!!!!!!!!!!!!1\n");
				break;
			}


			if(x == ex) 
				break;

			x = x + xInc;
			D = D + M;

			if(D > HX) {
				y = y + yInc; 
				D = D - c;
			}
		}
		
		*xcoord = minIndexX;
		*ycoord = minIndexY;
	}	
	else {

		short c = 2 * HY; 
		short M = 2 * HX;
		short counter = 0;
		while(1) {
			
			if(x >= 0 && x < scHEIGHT && y >= 0 && y < scWIDTH){

				double angleXY = atan2(-CameraZ, LookupTable[x*scWIDTH + y]);
				if(angleXY > M_PI/2.0)
					angleXY = angleXY - M_PI/2.0;

				double angleDiff = fabs(angleXY - angle);
				if(angleDiff < minAngleDiff){
					minAngleDiff = angleDiff;
					minIndexX = x;
					minIndexY = y;

				}

			}
			else{
				//printf("out of image!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
				break;
			}

			if(y == ey) 
				break;

			y = y + yInc;
			D = D + M;

			if(D > HY) {
				x = x + xInc; 
				D = D - c;
			}
		}
		*xcoord = minIndexX;
		*ycoord = minIndexY;
	}





}



