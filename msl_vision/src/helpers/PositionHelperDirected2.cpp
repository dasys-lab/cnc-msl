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
#include "PositionHelperDirected.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <sys/time.h>

PositionHelperDirected *  PositionHelperDirected::instance = NULL;


PositionHelperDirected::PositionHelperDirected() {

	this->MX=640/2; //images size
	this->MY=480/2;

	this->fieldLength = 8900.0; //mm
	this->fieldWidth = 5580.0; //mm

	this->camProjDistX = 480.0; //px
	this->camProjDistY = 480.0; //px
	this->camZ = 790.0; //mm
	this->camX = 0.0; //mm

	this->dist2center = 320.0; //px setting

	this->ball_r=110.0; //mm

	this->camAngle = atan((fieldLength/2) / camZ) + (atan((MX-dist2center) / camProjDistX));
	
}

PositionHelperDirected * PositionHelperDirected::getInstance(){
	
	if(instance == NULL){
		instance = new PositionHelperDirected();
	}
	
	return instance;

}

Point3D PositionHelperDirected::getPointCam2Point3D(double x, double y, double r, double ball_r) {
	x = 640-x;
	Point3D p;
	double ball_dist;
	double dx, dy, dz;

	Point p1, p2, p3;

	p1 = getPointCam2Angle(x+r, y);
	p2 = getPointCam2Angle(x-r, y);
	p3 = getPointCam2Angle(x, y);

	p.angleVerti = p1.x-p2.x;
	p.angleHori = p1.y;

	ball_dist = fabs(ball_r / tan(abs_double(p.angleVerti)/2));

	//p.angleHori = p.angleHori + this->camAngle;

	p.angleVerti = p3.x + this->camAngle;

	dz = cos(p.angleVerti) * ball_dist;
//	if(p.angleVerti > M_PI/2.0)
//		dz = -1.0*dz;

	//p.z = this->camZ - dz;

	double projDist = sqrt(ball_dist*ball_dist - dz*dz); 

	dx = cos(p.angleHori) * projDist;
	dy = sin(p.angleHori) * projDist;

	p.z = this->camZ - dz;
	p.x = this->camX + dx;
	p.y = dy;
	return p;
}

Point3D PositionHelperDirected::getPointCam2Point3D(double x, double y, double r) {
	return getPointCam2Point3D(x, y, r, this->ball_r);
}

Point PositionHelperDirected::getPointCam2Field(double x, double y) {
	x = 640-x;
	Point p1;

	p1 = getPointCam2Angle(x, y);
 

	double horizAngle = this->camAngle + p1.x;
	if(horizAngle >= M_PI/2.0){
		horizAngle = M_PI/2.0;
	}

	p1.x = tan(horizAngle) * this->camZ;
	p1.y = -tan(p1.y) * p1.x;

	return p1;
}

Point PositionHelperDirected::getPointCam2Angle(double x, double y) {
	Point p1;

	p1.x = atan((x - this->MX) / this->camProjDistX);
	p1.y = atan((y - this->MY) / this->camProjDistY);

	return p1;
}

Point PositionHelperDirected::getPointField2Cam(double x, double y) {
	Point p1;

	p1.x = atan(x / this->camZ) - this->camAngle;
	p1.y = atan(y / sqrt(x*x + this->camZ*this->camZ));

	p1 = getAngle2Cam(p1.x, p1.y);
// 	p1.x = tan(p1.x) * this->camProjDistX + this->MX;
// 	p1.y = -tan(p1.y) * this->camProjDistY + this->MY;

	return p1;
}

Point PositionHelperDirected::getAngle2Cam(double x, double y) {
	Point p1;

	p1.x = tan(x) * this->camProjDistX + this->MX;
	p1.y = -tan(y) * this->camProjDistY + this->MY;
	//p1.y *= -1.0;
	p1.x = 640-p1.x;
	return p1;
}

//stimmt das? ich geh davon aus, das y auf einer gerade liegt und nicht auf einer kreisbahn
Point PositionHelperDirected::getPoint3D2Cam(double x, double y, double z) {
	Point p1;

	double dz = this->camZ - z;
//Eigentlich muesste man hier noch das koordinatensystem um die y-Acshe um den Winkel M_PI/2 - camAngle drehen
	p1.x = atan2(sqrt(x*x + y*y),dz) - this->camAngle;
	p1.y = atan2(y, x);

	p1 = getAngle2Cam(p1.x, p1.y);

	return p1;
}

double PositionHelperDirected::getPoint3D2Radius(double x, double y, double z) {
	Point p1, p2, p3;

	double dz = this->camZ - z;

	p1.x = atan2(sqrt(x*x + y*y), dz) - this->camAngle;
	p1.y = atan2(y,x);


	double angleBallrDiff = atan(ball_r / sqrt(x*x + y*y + dz*dz));

	p2 = getAngle2Cam(p1.x, p1.y);
	p3 = getAngle2Cam(p1.x, p1.y+angleBallrDiff);

	return fabs(p2.y - p3.y);
}

double PositionHelperDirected::abs_double(double d) {

	if (d > 0)
		return d;
	else
		return -d;

}

double PositionHelperDirected::getPointCam2Dist(double x, double y){

	Point p = getPointCam2Field(x, y);
	return sqrt(p.x*p.x + p.y*p.y);


}
