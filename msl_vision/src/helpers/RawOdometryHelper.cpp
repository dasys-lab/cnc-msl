/*
 * $Id: RawOdometryHelper.cpp 2028 2007-04-11 19:28:28Z cn $
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
#include "RawOdometryHelper.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>

#include <sys/time.h>
#include "SpicaDirectedHelper.h"
#include "TimeHelper.h"
#include "Logger.h"

using namespace msl_actuator_msgs;

RawOdometryHelper * RawOdometryHelper::instance_ = NULL;

RawOdometryHelper * RawOdometryHelper::getInstance(){
	if(instance_ == NULL)
		instance_ = new RawOdometryHelper();
	return instance_;
}


void RawOdometryHelper::handleRawOdometryInfo(const msl_actuator_msgs::RawOdometryInfo::ConstPtr& message) {

printf("GOT MSG RawOdometry\n");
	printf("1234567891 x: %f y: %f heading: %f %lld\n", message->position.x, message->position.y, message->position.angle, message->timestamp);

	Position pos;
	pos.x = message->position.x;
	pos.y = message->position.x;
	pos.heading = message->position.angle;

	integrateData(pos, message->timestamp);


	Logger::getInstance()->logRawOdometry(pos, message->timestamp);



//	return 0;

}





RawOdometryHelper::RawOdometryHelper() : mutex() {

	initialized = false;

	newPosition.x = 0.0;
	newPosition.y = 0.0;
	newPosition.heading = 0.0;

	for(int i = 0; i < RAWODOBUFSIZE; i++){
		positionBuffer[i] = newPosition;
		timestampBuffer[i] = 0;
	}

	//visionIndex = 0;
	odoIndex = 0;
	ros::NodeHandle nh;
	sub = nh.subscribe<RawOdometryInfo, RawOdometryHelper>("RawOdometry", 1, &RawOdometryHelper::handleRawOdometryInfo, (this), ros::TransportHints().udp());
	init();
}


RawOdometryHelper::~RawOdometryHelper(){

	cleanup();

}



void RawOdometryHelper::init(){


}


void RawOdometryHelper::cleanup(){


}


void RawOdometryHelper::integrateData(Position pos, unsigned long long timestamp) {

	boost::mutex::scoped_lock lock(this->mutex);

	int logTime = (int) ((timestamp/1000) % 1000000);
	if(logTime > 500000)
		logTime -= 1000000;

	printf("1234567895 %f %f %f %d\n", pos.x, pos.y, pos.heading, logTime);

	printf("Integrate RawOdometry\n");

	if(initialized){
		odoIndex++;
		if(odoIndex >= RAWODOBUFSIZE)
			odoIndex -= RAWODOBUFSIZE;

	}

	if (!initialized){
		oldPosition.x = pos.x;
		oldPosition.y = pos.y;
		oldPosition.heading = pos.heading;

		oldPosition2.x = pos.x;
		oldPosition2.y = pos.y;
		oldPosition2.heading = pos.heading;

		initialized = true;
	}

	newPosition.x = pos.x;
	newPosition.y = pos.y;
	newPosition.heading = pos.heading;

	positionBuffer[odoIndex] = newPosition;
	timestampBuffer[odoIndex] = timestamp;


}


Position RawOdometryHelper::getUpdateVectorAndReset() {

	Position pos;

	boost::mutex::scoped_lock lock(this->mutex);

	if (!initialized){
		pos.x = 0.0;
		pos.y = 0.0;
		pos.heading = 0.0;

	} else {

		int visionIndex = getVisionIndex();

		int previousIndex = visionIndex - 1;
		if(previousIndex < 0)
			previousIndex += RAWODOBUFSIZE;

		pos.x = positionBuffer[visionIndex].x - positionBuffer[previousIndex].x;
		pos.y = positionBuffer[visionIndex].y - positionBuffer[previousIndex].y;
		pos.heading = positionBuffer[visionIndex]. heading - positionBuffer[previousIndex].heading;

		if (pos.heading > M_PI) {
			pos.heading -= 2.0*M_PI;
		}

		if (pos.heading < -M_PI) {
			pos.heading += 2.0*M_PI;
		}

		//positionBuffer[previousIndex] = positionBuffer[visionIndex];

	}

	return pos;
}

Position RawOdometryHelper::getUpdateVectorAndReset2() {
	Position pos;

	boost::mutex::scoped_lock lock(this->mutex);

	if (!initialized){
		pos.x = 0.0;
		pos.y = 0.0;
		pos.heading = 0.0;

	} else {

		pos.x = newPosition.x - oldPosition2.x;
		pos.y = newPosition.y - oldPosition2.y;
		pos.heading = newPosition.heading - oldPosition2.heading;

		if (pos.heading > M_PI) {
			pos.heading -= 2.0*M_PI;
		}

		if (pos.heading < -M_PI) {
			pos.heading += 2.0*M_PI;
		}

		oldPosition2.x = newPosition.x;
		oldPosition2.y = newPosition.y;
		oldPosition2.heading = newPosition.heading;
	}

	return pos;
}


Position RawOdometryHelper::getPositionData(){

	return positionBuffer[getVisionIndex()];

}

Position RawOdometryHelper::getPositionData(unsigned long long time){

	int ret;
	int minIndex = 0;
	unsigned long long minDiff = 10000000;
	for(int i = 0; i < RAWODOBUFSIZE; i++){

		unsigned long long timeDiff = TimeHelper::getTimeDiff(time,timestampBuffer[i]);
		if(timeDiff < minDiff){
			minDiff = timeDiff;
			minIndex = i;
		}
	}

	printf("1234567891 posdata: %lld\n", time);

	if(minDiff >= 10000000){
		printf("1234567891 RawOdometryHelper Posdata: Something going wrong with Odometry - using odoIndex\n");
		return positionBuffer[odoIndex];
	}
	else
		return positionBuffer[minIndex];



}


Position RawOdometryHelper::updatePositionWithOdoData(Position pos){

	Position retPos = pos;

	int currIndex = getVisionIndex();

	while(currIndex != odoIndex){

		int previousIndex = currIndex - 1;
		if(previousIndex < 0)
			previousIndex += RAWODOBUFSIZE;

		Position update = getPosDiffVector(positionBuffer[currIndex], positionBuffer[previousIndex]);

		retPos = updatePositionWithVector(retPos, update.x, update.y, update.heading, positionBuffer[previousIndex]);

		currIndex++;
		if(currIndex >= RAWODOBUFSIZE)
			currIndex -= RAWODOBUFSIZE;


	}

	for(int i = 0; i < 1; i++){

		int previousIndex = odoIndex - 1;
		if(previousIndex < 0)
			previousIndex += RAWODOBUFSIZE;

		Position update = getPosDiffVector(positionBuffer[odoIndex], positionBuffer[previousIndex]);

		retPos = updatePositionWithVector(retPos, update.x, update.y, update.heading, positionBuffer[previousIndex]);

	}

	return retPos;

}


Position RawOdometryHelper::getPosDiffVector(Position posNew, Position posOld){

	Position ret;
	ret.x = posNew.x - posOld.x;
	ret.y = posNew.y - posOld.y;
	ret.heading = posNew.heading - posOld.heading;
	if(ret.heading > M_PI)
		ret.heading -= 2.0*M_PI;
	if(ret.heading < -M_PI)
		ret.heading += 2.0*M_PI;

	return ret;

}


Position RawOdometryHelper::updatePositionWithVector(Position pos, double deltaX, double deltaY, double deltaH, Position relPos){

	Position retPos = pos;

	double rotAngle = pos.heading - relPos.heading;
	if(rotAngle > M_PI){
		rotAngle -= 2.0*M_PI;
	}

	if(rotAngle < -M_PI){
		rotAngle += 2.0*M_PI;

	}

	double newDeltaX = cos(rotAngle)*deltaX - sin(rotAngle)*deltaY;
	double newDeltaY = sin(rotAngle)*deltaX + cos(rotAngle)*deltaY;

	//printf("UP: newV deltaX: %f deltaY: %f\n", newDeltaX, newDeltaY);

	retPos.x += newDeltaX;
	retPos.y += newDeltaY;
	retPos.heading += deltaH;
	if(retPos.heading > M_PI)
		retPos.heading -= 2.0*M_PI;
	if(retPos.heading < -M_PI)
		retPos.heading += 2.0*M_PI;

	return retPos;

}


Point RawOdometryHelper::ego2AlloOnVision(Point p){

	return ego2Allo(p, positionBuffer[getVisionIndex()]);

}


Point RawOdometryHelper::allo2EgoOnVision(Point p){

	return allo2Ego(p, positionBuffer[getVisionIndex()]);


}


Point RawOdometryHelper::ego2AlloOnOdo(Point p){

	return ego2Allo(p, positionBuffer[odoIndex]);


}


Point RawOdometryHelper::allo2EgoOnOdo(Point p){


	return allo2Ego(p, positionBuffer[odoIndex]);

}


Point RawOdometryHelper::ego2Allo(Point p, Position pos){


	Point allo;
	allo.x = pos.x;
	allo.y = pos.y;

	allo.x += cos(pos.heading)*p.x - sin(pos.heading)*p.y;
	allo.y += sin(pos.heading)*p.x + cos(pos.heading)*p.y;

	return allo;

}


Point RawOdometryHelper::allo2Ego(Point p, Position pos){


	Point ego;

	double x = p.x - pos.x;
	double y = p.y - pos.y;

	double angle = atan2(y, x) - pos.heading;
	double dist = sqrt(x*x + y*y);

	ego.x = cos(angle)*dist;
	ego.y = sin(angle)*dist;

	return ego;


}


Velocity RawOdometryHelper::ego2Allo(Velocity vel, Position pos){

	Velocity allo;

	allo.vx = cos(pos.heading)*vel.vx - sin(pos.heading)*vel.vy;
	allo.vy = sin(pos.heading)*vel.vx + cos(pos.heading)*vel.vy;

	return allo;


}


Velocity RawOdometryHelper::allo2Ego(Velocity vel, Position pos){

	Velocity ego;

	double angle = atan2(vel.vy,vel.vx) - pos.heading;
	double length = sqrt(vel.vx*vel.vx + vel.vy*vel.vy);

	ego.vx = cos(angle)*length;
	ego.vy = sin(angle)*length;

	return ego;


}


Position RawOdometryHelper::getVisionPos(){

	return positionBuffer[getVisionIndex()];

}

Position RawOdometryHelper::getOdoPos(){

	return positionBuffer[odoIndex];

}

int RawOdometryHelper::getVisionIndex(){

	int ret;
	int minIndex = 0;
	unsigned long long minDiff = 10000000;
	for(int i = 0; i < RAWODOBUFSIZE; i++){
		unsigned long long timeDiff = TimeHelper::getInstance()->getTimeDiffToOmniCam(timestampBuffer[i]);
		if(timeDiff < minDiff){
			minDiff = timeDiff;
			minIndex = i;
		}
	}

	printf("1234567891 %lld\n", TimeHelper::getInstance()->getVisionTimeOmniCam());

	if(minDiff >= 10000000){

		printf("1234567891 RawOdometryHelper: Something going wrong with Odometry - using odoIndex\n");
		return odoIndex;
	}
	else
		return minIndex;

}

int RawOdometryHelper::getOdoIndex(){

	return odoIndex;

}

Position * RawOdometryHelper::getPositionBuffer(){

	return positionBuffer;

}

unsigned long long * RawOdometryHelper::getTimestampBuffer(){

	return timestampBuffer;

}

