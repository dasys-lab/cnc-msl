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

	//Sony Cam

	SystemConfig* sc = SystemConfig::getInstance();

	Configuration *vision = (*sc)["Vision"];

	LensBigAngle = vision->get<double>("Vision", "Camera1394Sony", "LensBigAngle", NULL);
	LensSmallAngle = vision->get<double>("Vision", "Camera1394Sony", "LensSmallAngle", NULL);
	LensK1 = vision->get<double>("Vision", "Camera1394Sony", "LensK1", NULL);

	CenterDistance = vision->get<double>("Vision", "Camera1394Sony", "CenterDistance", NULL);
	CenterHeight = vision->get<double>("Vision", "Camera1394Sony", "CenterHeight", NULL);
	CameraHeight = vision->get<double>("Vision", "Camera1394Sony", "CameraHeight", NULL);
	CameraMountedRotation = vision->get<double>("Vision", "Camera1394Sony", "CameraMountedRotation", NULL);

	MaxPixelXFromCenter = 240.0 + 240.0*(LensK1*(240.0*240.0));
	MaxPixelYFromCenter = 320.0 + 320.0*(LensK1*(320.0*320.0));

	double maxYAngle = LensBigAngle/2.0;
	maxYAngle = maxYAngle*M_PI/180.0;
	DistanceToPixelPlane = MaxPixelYFromCenter/tan(maxYAngle);

	printf("MaxPixelYFromCenter: %f\n", MaxPixelYFromCenter);
	printf("maxYAngle: %f\n", maxYAngle);
	printf("Tan: %f\n", tan(maxYAngle));
	printf("DistanceToPixelPlane: %f\n", DistanceToPixelPlane);




}

PositionHelperDirected * PositionHelperDirected::getInstance(){

	if(instance == NULL){
		instance = new PositionHelperDirected();
	}

	return instance;

}

Point3D PositionHelperDirected::getPointCam2Point3D(double x, double y, double r, double ball_r) {
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

Point3D PositionHelperDirected::getPointCam2Point3DSony(double x, double y, double r, double ball_r){

	double midX_centered = (x - 240);
	double midY_centered = (y - 320);

	//printf("midX_centered = %f midY_centered = %f\n", midX_centered, midY_centered);

	double midX_centered_new = LensCorrection(midX_centered, midX_centered, midY_centered);
	double midY_centered_new = LensCorrection(midY_centered, midX_centered, midY_centered);

	//printf("midX_centered_new = %f midY_centered_new = %f\n", midX_centered_new, midY_centered_new);

	double ZCoord_raw = -midY_centered_new;
	double YCoord_raw = midX_centered_new;


	double borderX = fabs(midX_centered) + cos(atan2(fabs(midY_centered), fabs(midX_centered)))*r;
	double borderY = fabs(midY_centered) + sin(atan2(fabs(midY_centered), fabs(midX_centered)))*r;

	//printf("BorderX = %f BorderY = %f\n", borderX, borderY);

	double borderX_new = LensCorrection(borderX, borderX, borderY);
	double borderY_new = LensCorrection(borderY, borderX, borderY);

	//printf("BorderX_new = %f BorderY_new = %f\n", borderX_new, borderY_new);

	double radius_new = borderX_new - fabs(midX_centered_new);
	if(radius_new < borderY_new - fabs(midY_centered_new))
		radius_new = borderY_new - fabs(midY_centered_new);

	//printf("Radius_new = %f\n", radius_new);

	double radius_new_mm = radius_new/DistanceToPixelPlane;

	double dist = ball_r/radius_new_mm;

	Point3D p;
	p.x = DistanceToPixelPlane;
	p.y = YCoord_raw;
	p.z = ZCoord_raw;

	Point3D ret = NormalizePoint3D(p);
	ret.x *= dist;
	ret.y *= dist;
	ret.z *= dist;

	ret = CorrectAngle(ret);

	ret.x = -ret.x;
	ret.y = -ret.y;
	ret.z += CameraHeight;

	return ret;

}



Point3D PositionHelperDirected::getPointCam2Point3DSony(double x, double y, double r){

	return getPointCam2Point3DSony(x, y, r, ball_r);


}

Point PositionHelperDirected::getPoint3D2CamSony(double x, double y, double z){

	Point3D p;
	p.z = z - CameraHeight;
	p.x = -x;
	p.y = -y;

	p = CorrectAngleInv(p);
	p = NormalizePoint3D(p);

	double factor = DistanceToPixelPlane/p.x;
	p.x *= factor;
	p.y *= factor;
	p.z *= factor;

	double midY_centered_new = -p.z;
	double midX_centered_new = p.y;


	Point ret;
	ret.x = midX_centered_new - midX_centered_new*(0.7*LensK1*(midX_centered_new*midX_centered_new + midY_centered_new*midY_centered_new)) + 240.0;
	ret.y = midY_centered_new - midY_centered_new*(0.7*LensK1*(midX_centered_new*midX_centered_new + midY_centered_new*midY_centered_new)) + 320.0;


	return ret;


}


Point PositionHelperDirected::getPointCam2FieldSony(double x, double y){

	double midX_centered = (x - 240);
	double midY_centered = (y - 320);

	double midX_centered_new = LensCorrection(midX_centered, midX_centered, midY_centered);
	double midY_centered_new = LensCorrection(midY_centered, midX_centered, midY_centered);

	double ZCoord_raw = -midY_centered_new;
	double YCoord_raw = midX_centered_new;

	Point3D p;
	p.x = DistanceToPixelPlane;
	p.y = YCoord_raw;
	p.z = ZCoord_raw;

	p = NormalizePoint3D(p);

	Point3D p_corrected = CorrectAngle(p);

	p_corrected.x = -p_corrected.x;
	p_corrected.y = -p_corrected.y;

	double dist_in_dir = sqrt(p_corrected.x*p_corrected.x + p_corrected.y*p_corrected.y)*(-CameraHeight/p_corrected.z);

	double dir = atan2(p_corrected.y, p_corrected.x);

	Point ret;
	ret.x = cos(dir)*dist_in_dir;
	ret.y = sin(dir)*dist_in_dir;


	return ret;


}


double PositionHelperDirected::getPoint3D2RadiusSony(double x, double y, double z){


	Point pMid = getPoint3D2CamSony(fabs(x), y, fabs(z));

	Point pMidX = getPoint3D2CamSony(fabs(x) + ball_r, y, fabs(z));
	Point pMidZ = getPoint3D2CamSony(fabs(x), y, fabs(z) + ball_r);

	double dist1 = sqrt( (pMid.x -pMidX.x)*(pMid.x -pMidX.x) + (pMid.y - pMidX.y)*(pMid.y -pMidX.y) );
	double dist2 = sqrt( (pMid.x -pMidZ.x)*(pMid.x -pMidZ.x) + (pMid.y - pMidZ.y)*(pMid.y -pMidZ.y) );

	double ret = dist1;
	if(ret < dist2)
		ret = dist2;

	return dist2;


}

double PositionHelperDirected::LensCorrection(double v, double x, double y){

	double ret = v + v*(LensK1*(x*x + y*y));
	return ret;


}


Point3D PositionHelperDirected::NormalizePoint3D(Point3D p){

	double dist = sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
	Point3D ret;
	ret.x = p.x /dist;
	ret.y = p.y /dist;
	ret.z = p.z /dist;

	return ret;

}

Point3D PositionHelperDirected::CorrectAngle(Point3D p){

	double alpha = atan2(-CameraHeight, CenterDistance);

	Point3D ret;
	ret.x = p.x * cos(alpha) - sin(alpha)*p.z;
	ret.y = p.y;
	ret.z = p.x * sin(alpha) + cos(alpha)*p.z;

	return ret;

}


Point3D PositionHelperDirected::CorrectAngleInv(Point3D p){

	double alpha = - atan2(-CameraHeight, CenterDistance);

	Point3D ret;
	ret.x = p.x * cos(alpha) - sin(alpha)*p.z;
	ret.y = p.y;
	ret.z = p.x * sin(alpha) + cos(alpha)*p.z;

	return ret;

}







