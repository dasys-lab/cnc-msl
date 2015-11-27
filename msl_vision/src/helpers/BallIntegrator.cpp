/*
 * $Id: BallIntegrator.cpp 1531 2006-08-01 21:36:57Z phbaer $
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
#include "BallIntegrator.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>

#include "RawOdometryHelper.h"

BallIntegrator * BallIntegrator::instance_ = NULL;

BallIntegrator * BallIntegrator::getInstance(){

	if(instance_ == NULL)
		instance_ = new BallIntegrator();
	return instance_;


}




BallIntegrator::BallIntegrator(){


	points.clear();
	containers.clear();

	currContainer = NULL;

	init();

}


BallIntegrator::~BallIntegrator(){

	cleanup();

}



void BallIntegrator::init(){


}

void BallIntegrator::cleanup(){


}

void BallIntegrator::decreaseDirtyPointCertainty() {
	int listLength = (int) points.size();
	for (int i = listLength - 1; i >= 0; i--) {
		if(points[i].dirty) points[i].certainty = decreaseCertainty(points[i].certainty);
	}

	for (int i = listLength - 1; i >= 0; i--) {
		points[i].dirty = true;
	}

	double sumOfCertainties = 0.0;
	double maxCertainty = -1.0;
	unsigned int maxIndex = 0;	
	for(unsigned int i = 0; i < points.size(); i++){
		sumOfCertainties += points[i].certainty;
		if(points[i].certainty > maxCertainty){
			maxCertainty = points[i].certainty;
			maxIndex = i;
		}
	}

	if(maxCertainty >= 0.35 && sumOfCertainties >= 0.35){

		printf("Point calculated\n");
		currPoint = points[maxIndex].point;
//		currPoint.confidence = points[maxIndex].point.confidence*(maxCertainty*maxCertainty/sumOfCertainties);
		currPoint.confidence = points[maxIndex].point.confidence*sqrt(1.0/listLength);
		currPoint.valid = true;

		currContainer = containers[maxIndex];
	}
	else {
		currPoint.valid = false;
		currPoint.x = 0.0;
		currPoint.y = 0.0;
		currPoint.z = 0.0;
		currPoint.confidence = 0.0;

		currContainer = NULL;		
	}

	printf("Stefans BallPos BallTracker - BallIntegrator - currPoint %f %f %f\n", currPoint.x, currPoint.y, sqrt(currPoint.x*currPoint.x + currPoint.y*currPoint.y));
	printf("Stefans BallPos BallTracker - BallIntegrator - certainty = %f nHypothesis = %d\n", maxCertainty, points.size());



}

void BallIntegrator::integratePoint(ObservedPoint p_, double threshold){

	//printf("Stefans integratePoint\n");

	bool found = false;

	ObservedPoint p = p_;

	Point egoPoint;
	egoPoint.x = p_.x;
	egoPoint.y = p_.y;

	Position integrationPos = RawOdometryHelper::getInstance()->getPositionData(p.timestamp);

	printf("BallIntegrator TimeStamp %lld\n", p.timestamp);

//	Point alloPoint = RawOdometryHelper::getInstance()->ego2Allo(egoPoint, integrationPos);
	Point alloPoint = RawOdometryHelper::getInstance()->ego2Allo(egoPoint, refPos);

	printf("MovingObject after ego2Allo: %f %f\n", alloPoint.x, alloPoint.y);


	p.x = alloPoint.x;
	p.y = alloPoint.y;

//ToDo
	
	if (p.valid) {
		printf("Point valid + size = %d\n", points.size());
		
		for (unsigned int i = 0; i < points.size(); i++) {

			double dist = sqrt((points[i].point.x - p.x)*(points[i].point.x - p.x) + (points[i].point.y - p.y)*(points[i].point.y - p.y));

			if (dist < threshold) {
				found = true;
				points[i].certainty = increaseCertainty(points[i].certainty);
				containers[i]->integratePoint(p);
				points[i].point = p;
				containers[i]->integratePoint(p);
				points[i].dirty = false;

			} else {
				//points[i].certainty = decreaseCertainty(points[i].certainty);
			}
		}
	
		if (!found) {
		printf("Point not found\n");
			PointHypothesis newPoint;
			newPoint.certainty = 0.5;
			newPoint.point = p;
			newPoint.dirty = false;
			ObjectContainer * newContainer = new ObjectContainer(1200);
			newContainer->integratePoint(p);
			containers.push_back(newContainer);
			points.push_back(newPoint);
		}
				
		double maxCertainty = -1.0;
		unsigned int maxIndex = 0;
	
		for (unsigned int i = 0; i < points.size(); i++) {
			if (points[i].certainty > maxCertainty) {
				maxCertainty = points[i].certainty;
				maxIndex = i;
			}
		}
		
		int listLength = (int) points.size();
	
		for (int i = listLength - 1; i >= 0; i--) {
			//printf("i = %d\n", i);
			double dist = sqrt((points[i].point.x - p.x)*(points[i].point.x - p.x) + (points[i].point.y - p.y)*(points[i].point.y - p.y));


			if ((points[i].certainty < 0.35) ||
				((maxIndex != i) && (dist < threshold)))
			{

				printf("PointRemoval!\n");

				std::vector<PointHypothesis>::iterator itRemove = points.begin() + i;
				points.erase(itRemove);

				ObjectContainer * removeContainer = containers[i];
				delete removeContainer;

				std::vector<ObjectContainer * >::iterator itRemoveCont = containers.begin() + i;
				containers.erase(itRemoveCont);

			}
		}

	} else {
		int listLength = (int) points.size();
		for (int i = listLength - 1; i >= 0; i--) {
			//points[i].certainty = decreaseCertainty(points[i].certainty);
			points[i].dirty = true;
			if (points[i].certainty < 0.35) {
				std::vector<PointHypothesis>::iterator itRemove = points.begin() + i;
				points.erase(itRemove);

				ObjectContainer * removeContainer = containers[i];
				delete removeContainer;

				std::vector<ObjectContainer * >::iterator itRemoveCont = containers.begin() + i;
				containers.erase(itRemoveCont);

			}
		}
	}


}


ObservedPoint BallIntegrator::getPoint(){

	return currPoint;

}

ObjectContainer * BallIntegrator::getContainer(){

	return currContainer;

}


double BallIntegrator::increaseCertainty(double certainty){

	double retCertainty = certainty + 0.05;

	if(retCertainty > 1.0)
		retCertainty = 1.0;

	return retCertainty;

}


double BallIntegrator::decreaseCertainty(double certainty){

	double retCertainty = certainty - 0.05;

	if(retCertainty < 0.0)
		retCertainty = 0.0;

	return retCertainty;

}

void BallIntegrator::setRefPosition(Position pos){

	refPos = pos;
}


Position BallIntegrator::getRefPosition(){

	return refPos;

}

