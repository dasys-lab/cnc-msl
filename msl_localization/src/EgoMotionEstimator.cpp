/*
 * $Id: EgoMotionEstimator.cpp 1531 2006-08-01 21:36:57Z phbaer $
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
#include "EgoMotionEstimator.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>

EgoMotionEstimator * EgoMotionEstimator::instance_ = NULL;

EgoMotionEstimator * EgoMotionEstimator::getInstance(){
	if(instance_ == NULL)
		instance_ = new EgoMotionEstimator();
	return instance_;
}


EgoMotionEstimator::EgoMotionEstimator(){
	//rawHelper = RawOdometryHelper::getInstance();
	init();
}

EgoMotionEstimator::~EgoMotionEstimator(){
	cleanup();
}


void EgoMotionEstimator::init(){
}

void EgoMotionEstimator::cleanup(){
}


MovingRobot EgoMotionEstimator::trackObject(Position * posBuffer, unsigned long long * timestampBuf, int length, int lastIndex, double timeBack){
	int validCounter = 0;

	unsigned long long timediff = 1000000; //TimedifftoOmniCam
//	if(timediff > 1.0E07){
//		MovingRobot mr;
//		mr.position.x = 0.0;
//		mr.position.y = 0.0;
//		mr.position.heading = 0.0;
//		mr.velocity.vx = 0.0;
//		mr.velocity.vy = 0.0;
//		mr.velocity.w = 0.0;
//		return mr;
//	}


	validCounter = 1;
	int startIndex = lastIndex;
	int currIndex = lastIndex - 1;


	while(1){
		timediff = timestampBuf[lastIndex] - timestampBuf[currIndex];
		if(timediff > timeBack || currIndex == lastIndex)
			break;
		
		validCounter++;
		startIndex = currIndex;

		currIndex--;
		if(currIndex < 0)
			currIndex += length;
	}

	if(validCounter == 1){
		MovingRobot mr;
		mr.position.x = posBuffer[lastIndex].x;
		mr.position.y = posBuffer[lastIndex].y;
		mr.position.heading = posBuffer[lastIndex].heading;
		mr.velocity.vx = 0.0;
		mr.velocity.vy = 0.0;
		mr.velocity.w = 0.0;
		return mr;
	}

	MovingRobot mr;
	double lambda = 0.0;

	if(validCounter <= 3)
		lambda = 0.005;
	else
		lambda = 0.005*pow(0.5, validCounter - 3);

//	Estimate the robot rotational velocity
	double sumTimes = 0.0;
	double sumTimesSquare = 0.0;
	double sumTimeHeadings = 0.0;
	double sumHeadings = 0.0;

	sumHeadings += 0.0;
	currIndex = startIndex;
	
	while(currIndex != lastIndex){
		unsigned long long timediff = timestampBuf[lastIndex] - timestampBuf[currIndex];

		sumTimesSquare += (timediff/-1.0E07)*(timediff/-1.0E07);
		sumTimes += (timediff/-1.0E07);

		double headingAdd = posBuffer[currIndex].heading - posBuffer[lastIndex].heading;
		if(headingAdd > M_PI)
			headingAdd -= 2.0*M_PI;
		if(headingAdd < -M_PI)
			headingAdd += 2.0*M_PI;

		sumHeadings += headingAdd;
		sumTimeHeadings += headingAdd*(timediff/-1.0E07);

		currIndex++;
		if(currIndex >= length)
			currIndex -= length;
	}

	double w = (validCounter*sumTimeHeadings - sumTimes*sumHeadings)/(validCounter*sumTimesSquare - sumTimes*sumTimes);
	double heading = (sumTimesSquare*sumHeadings - sumTimes*sumTimeHeadings)/(validCounter*sumTimesSquare - sumTimes*sumTimes);

	heading += posBuffer[lastIndex].heading;
	if(heading > M_PI)
		heading -= 2.0*M_PI;
	if(heading < -M_PI)
		heading += 2.0*M_PI;

	double vx = 0.0;
	double vy = 0.0;
	double x = 0.0;
	double y = 0.0;

	if(fabs(w) < 1.0E-03){
		double sumPointsX = posBuffer[lastIndex].x;
		double sumPointsY = posBuffer[lastIndex].y;
		double sumTimePointsX = 0.0;
		double sumTimePointsY = 0.0;

		currIndex = startIndex;
		
		while(currIndex != lastIndex){
			unsigned long long timediff = timestampBuf[lastIndex] - timestampBuf[currIndex];

			sumPointsX += posBuffer[currIndex].x;
			sumPointsY += posBuffer[currIndex].y;	
			sumTimePointsX += posBuffer[currIndex].x*(timediff/-1.0E07);
			sumTimePointsY += posBuffer[currIndex].y*(timediff/-1.0E07);
	
			currIndex++;
			if(currIndex >= length)
				currIndex -= length;
		}

		vx = (validCounter*sumTimePointsX - sumTimes*sumPointsX)/(validCounter*sumTimesSquare - sumTimes*sumTimes);
		vy = (validCounter*sumTimePointsY - sumTimes*sumPointsY)/(validCounter*sumTimesSquare - sumTimes*sumTimes);

		x = (sumTimesSquare*sumPointsX - sumTimes*sumTimePointsX)/(validCounter*sumTimesSquare - sumTimes*sumTimes);
		y = (sumTimesSquare*sumPointsY - sumTimes*sumTimePointsY)/(validCounter*sumTimesSquare - sumTimes*sumTimes);
	}
	else {
		double sumSi = 0.0;
		double sumCi = 0.0;
		double sumComplexX = 0.0;
		double sumComplexY = 0.0;
		double sumSiCiSquare = 0.0;
		double sumPointsX = posBuffer[lastIndex].x;
		double sumPointsY = posBuffer[lastIndex].y;

		currIndex = startIndex;
		
		while(currIndex != lastIndex){
			unsigned long long timediff = timestampBuf[lastIndex] - timestampBuf[currIndex];

			double si = sin(w*timediff/-1.0E07)/w;
			double ci = (cos(w*timediff/-1.0E07) - 1.0)/w;

			sumSi += si;
			sumCi += ci;

			//ToDo if problems then look here
			sumComplexX += si*posBuffer[currIndex].x - ci*posBuffer[currIndex].y;
			sumComplexY += ci*posBuffer[currIndex].x + si*posBuffer[currIndex].y;

			sumSiCiSquare += si*si + ci*ci;

			sumPointsX += posBuffer[currIndex].x;
			sumPointsY += posBuffer[currIndex].y;

	
			currIndex++;
			if(currIndex >= length)
				currIndex -= length;
		}

		double d = sumSi*sumSi + sumCi*sumCi - validCounter*sumSiCiSquare;
		d = 1.0/d;

		vx = d*(sumSi*sumPointsX - sumCi*sumPointsY - validCounter*sumComplexX);
		vy = d*(sumSi*sumPointsY + sumCi*sumPointsX - validCounter*sumComplexY);

		x = (sumPointsX - sumSi*vx - sumCi*vy)/validCounter;
		y = (sumPointsY + sumCi*vx - sumSi*vy)/validCounter;
	}

	mr.velocity.vx = vx;
	mr.velocity.vy = vy;
	mr.velocity.w = w;

	mr.position.x = x;
	mr.position.y = y;
	mr.position.heading = heading;

	return mr;
}

