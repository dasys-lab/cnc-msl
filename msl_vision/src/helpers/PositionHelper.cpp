/*
 * $Id: PositionHelperDirected.cpp 1531 2009-12-27 flseute $
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
#include "PositionHelper.h"
#include "DistanceLookupHelper.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <sys/time.h>

PositionHelper *  PositionHelper::instance = NULL;


PositionHelper::PositionHelper() {

// 	this->sc = SystemConfig::getInstance();

// 	FieldLength = (*this->sc)["Globals"]->get<double>("Globals", "FootballField", "FieldLength", NULL);
// 	FieldWidth = (*this->sc)["Globals"]->get<double>("Globals", "FootballField", "FieldWidth", NULL);
/*	this->MX=640/2; //images size
	this->MY=480/2;

	this->fieldLength = 8900.0; //mm
	this->fieldWidth = 5580.0; //mm

	this->camProjDistX = 480.0; //px
	this->camProjDistY = 480.0; //px
	this->camZ = 790.0; //mm
	this->camX = 0.0; //mm
*/
	//this->dist2center = 421.0; //px

	this->ball_r=127.3; //mm

	//this->camAngle = atan((fieldLength/2) / camZ) + (atan((MX-dist2center) / camProjDistX));


	this->sc = SystemConfig::getInstance();

	Configuration *vision = (*this->sc)["Vision"];

	int imWidth = vision->get<int>("Vision", "ImageArea", NULL);
	int imHeight = vision->get<int>("Vision", "ImageArea", NULL);
	CameraZ = vision->get<int>("Vision", "CameraZ", NULL);

	mx = (short)imHeight/2;
	my = (short)imWidth/2;

	scWIDTH = imWidth;
	scHEIGHT = imHeight;

	
}

PositionHelper * PositionHelper::getInstance(){
	
	if(instance == NULL){
		instance = new PositionHelper();
	}
	
	return instance;

}

Point3D PositionHelper::getBallPositionFromBallMid(double x, double y) {

	Point3D ret;
	double * LookupTable = DistanceLookupHelper::getCreatedInstance()->getLookupTable();

	short x_ = (short) lrint(x);
	short y_ = (short) lrint(y);

	double distXY = LookupTable[x_*scWIDTH + y_];

	if(distXY < 0.0){
		ret.x = 0.0;
		ret.y = 0.0;
		ret.z = 0.0;
		
		return ret;

	}

	double projDist = distXY - (120.0/CameraZ)*distXY;


	double lineAngle = atan2(y-my,x-mx);



	//double projDist = 
	
	ret.x = cos(-lineAngle)*projDist;
	ret.y = sin(-lineAngle)*projDist;
	ret.z = 0.0;

	return ret;

}

Point3D PositionHelper::getPointCam2Point3D(double x, double y, double r, double ball_r) {
	//sollte klar sein, was das macht ;)

	//printf("XYR Ball! %f %f %f\n", x, y, r);

	r += 1.0;
	double * LookupTable = DistanceLookupHelper::getCreatedInstance()->getLookupTable();


	double lineAngle = atan2(y-my,x-mx);

	short ax = (short) lrint((cos(lineAngle)*(-r)) + x);
	short ay = (short) lrint((sin(lineAngle)*(-r)) + y);
	short ex = (short) lrint((cos(lineAngle)* r) + x);
	short ey = (short) lrint((sin(lineAngle)* r) + y);

	short x_ = (short) lrint(x);
	short y_ = (short) lrint(y);


//Strahl wird durch Bild(x,y) auf den Boden geschickt, distXY ist der Abstand vom Robotermittelpunkt zum Treffpunkt des Strahls auf den Boden in mm
	int pos = x_*scWIDTH + y_;
	if(pos<0 || pos>DistanceLookupHelper::getCreatedInstance()->imHeight*DistanceLookupHelper::getCreatedInstance()->imWidth) pos=0;
	
	//double distXY = LookupTable[x_*scWIDTH + y_];
	double distXY = LookupTable[pos];

	pos=ax*scWIDTH + ay;
	if(pos<0 || pos>DistanceLookupHelper::getCreatedInstance()->imHeight*DistanceLookupHelper::getCreatedInstance()->imWidth) pos=0;
	double distA = LookupTable[pos];
	//double distA = LookupTable[ax*scWIDTH + ay];

        pos=ex*scWIDTH + ey;
        if(pos<0 || pos>DistanceLookupHelper::getCreatedInstance()->imHeight*DistanceLookupHelper::getCreatedInstance()->imWidth) pos=0;
	double distE = LookupTable[pos];


	double angleXY = atan2(-CameraZ,distXY);

	if(distXY < 0.0){
		angleXY += M_PI;
	}

	double angleA = atan2(-CameraZ,distA);

	if(distA < 0.0){
		angleA += M_PI;
	}

	double angleE = atan2(-CameraZ,distE);

	if(distE < 0.0){
		angleE += M_PI;
	}

	double angleDiff = fabs(angleA - angleE);
	if(angleDiff > M_PI)
		angleDiff = fabs(angleDiff - 2.0*M_PI);

	angleDiff /= 2.0;


	double realDist = ball_r/tan(angleDiff);


	double projDist = cos(angleXY)*realDist;
	double z = CameraZ + sin(angleXY)*realDist;
	
	Point3D ret;
	//printf("LineAngle %f\n", lineAngle);

	ret.x = cos(-lineAngle)*projDist;
	ret.y = sin(-lineAngle)*projDist;
	ret.z = z;


	return ret;
}

Point3D PositionHelper::getPointCam2Point3D(double x, double y, double r) {
	return getPointCam2Point3D(x, y, r, this->ball_r);
}

Point PositionHelper::getPointCam2Field(double x, double y) {
	//Brauch ich für ne Lookup table (minimaler ballradius)

	double * LookupTable = DistanceLookupHelper::getCreatedInstance()->getLookupTable();

	short x_ = (short) lrint(x);
	short y_ = (short) lrint(y);

	double dist = LookupTable[x_*scWIDTH + y_];
	double angle = -atan2(y-my,x-mx);


	Point ret;
	ret.x = cos(angle)*dist;
	ret.y = cos(angle)*dist;

	return ret;
}

Point PositionHelper::getPointCam2Angle(double x, double y) {
	//Brauch ich z.z. nidd, könnte aber nützlich sein
	Point p1;

	//p1.x = atan((x - this->MX) / this->camProjDistX);
	//p1.y = atan((y - this->MY) / this->camProjDistY);

	return p1;
}

Point PositionHelper::getPointField2Cam(double x, double y) {
	//brauch ich z.z. nidd
	Point p1;

//	p1.x = atan(x / this->camZ) - this->camAngle;
//	p1.y = atan(y / sqrt(x*x + this->camZ*this->camZ));

//	p1 = getAngle2Cam(p1.x, p1.y);
// 	p1.x = tan(p1.x) * this->camProjDistX + this->MX;
// 	p1.y = -tan(p1.y) * this->camProjDistY + this->MY;

	return p1;
}

Point PositionHelper::getAngle2Cam(double x, double y) {
	//Brauch ich nidd sdelbst
	Point p1;

	/*p1.x = tan(x) * this->camProjDistX + this->MX;
	p1.y = tan(y) * this->camProjDistY + this->MY;*/

	return p1;
}

Point PositionHelper::getPoint3D2Cam(double x, double y, double z) {
	//Berechnet den Camerapunkt aus den 3d-Koordinaten

	Point ret;

	double lineAngle = -atan2(y,x);
	short ax = (short) lrint((cos(lineAngle)*10.0) + mx);
	short ay = (short) lrint((sin(lineAngle)*10.0) + my);
	short ex = (short) lrint(cos(lineAngle)*(mx - 5.0) + mx);
	short ey = (short) lrint(sin(lineAngle)*(my - 5.0) + my);

	//double dist = sqrt(x*x + y*y + (z - CameraZ)*(z - CameraZ));
	double distXY = sqrt(x*x + y*y);

	//double angleDiff = atan2(radius, dist);

	double angle = atan2((z - CameraZ), distXY);

	int xcoordMean = 0;
	int ycoordMean = 0;

	goForLine(ax, ay, ex, ey, angle, &xcoordMean, &ycoordMean);

	ret.x = xcoordMean;
	ret.y = ycoordMean;

	return ret;
}

double PositionHelper::getPoint3D2Radius(double x, double y, double z) {
	//Berechnet den Ballradius bei gegebener XYZ Koordinate: Brauch ich für Lookuptable!

	double lineAngle = -atan2(y,x);
	short ax = (short) lrint((cos(lineAngle)*10.0) + mx);
	short ay = (short) lrint((sin(lineAngle)*10.0) + my);
	short ex = (short) lrint(cos(lineAngle)*(mx - 5.0) + mx);
	short ey = (short) lrint(sin(lineAngle)*(my - 5.0) + my);

	double dist = sqrt(x*x + y*y + (z - CameraZ)*(z - CameraZ));
	double distXY = sqrt(x*x + y*y);

	double angleDiff = atan2(this->ball_r, dist);

	double angle = atan2((z - CameraZ), distXY);
	double angleMin = angle - angleDiff;
	double angleMax = angle + angleDiff;

	int xcoordMin = 0;
	int ycoordMin = 0;
	int xcoordMax = 0;
	int ycoordMax = 0;
	int xcoordMean = 0;
	int ycoordMean = 0;

	goForLine(ax, ay, ex, ey, angleMin, &xcoordMin, &ycoordMin);
	goForLine(ax, ay, ex, ey, angleMax, &xcoordMax, &ycoordMax);
	goForLine(ax, ay, ex, ey, angle, &xcoordMean, &ycoordMean);

	double ROIlength = sqrt((xcoordMin - xcoordMax)*(xcoordMin - xcoordMax) + (ycoordMin - ycoordMax)*(ycoordMin - ycoordMax))/2.0;

	return ROIlength; 
}

double PositionHelper::abs_double(double d) {

	if (d > 0)
		return d;
	else
		return -d;

}


void PositionHelper::goForLine(short ax, short ay, short ex, short ey, double angle, int * xcoord, int * ycoord){


	double * LookupTable = DistanceLookupHelper::getCreatedInstance()->getLookupTable();


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

	//double * LookupTable = distanceHelper.getLookupTable();

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

