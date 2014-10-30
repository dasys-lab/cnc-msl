/*
 * $Id: ObjectContainer.cpp 1531 2006-08-01 21:36:57Z phbaer $
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
#include "ObjectContainer.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include "TimeHelper.h"
#include <string.h>

ObjectContainer::ObjectContainer(int size_){

	size = size_;

	points = (ObservedPoint *) malloc(size*sizeof(ObservedPoint));
	memset(points, 0, size*sizeof(ObservedPoint));

	startIndex = 0;
	lastIndex = 0;
	lastValidIndex = 0;
	points[0].valid = false;

	//printf("+++++++++++++++++++++lastIndex %d %d\n", lastIndex, size*sizeof(ObservedPoint));

	validCounter = 0;

	init();

}


ObjectContainer::~ObjectContainer(){

	printf("Destructor of ObjectContainer\n");

	cleanup();

}



void ObjectContainer::init(){


}

void ObjectContainer::cleanup(){

	free(points);


}


void ObjectContainer::integratePoint(ObservedPoint p){

	//printf("ObjectContainer integratePoint %d %d %d\n", lastIndex, startIndex, validCounter);



	if(lastIndex != startIndex || (lastIndex == startIndex && validCounter == 1)){
		lastIndex++;
		if(lastIndex >= size)
			lastIndex -= size;
	}

	//printf("ObjectContainer integratePoint %d\n", lastIndex);
	points[lastIndex] = p;

	//printf("p.valid %d - points.valid %d\n", p.valid, points[lastIndex].valid);

	lastValidIndex = -1;
	validCounter = 0;
	if(points[lastIndex].valid){
		validCounter++;
		lastValidIndex = lastIndex;
	}

	int currIndex = startIndex;

	while(currIndex != lastIndex){

		if(points[currIndex].valid){
			validCounter++;
			if(lastValidIndex != lastIndex)
				lastValidIndex = currIndex;
		}

		currIndex++;
		if(currIndex >= size)
			currIndex -= size;

	}

	//printf("ObjectContainer integratePoint %d %d\n", lastValidIndex, validCounter);


}


void ObjectContainer::invalidate(int ms){

	printf("BallPos ObjectContainer invalidate: %d %d\n", lastValidIndex, validCounter);

	int currIndex = startIndex;

	unsigned long long timeDiff = 0;

	while(currIndex != lastIndex){
		timeDiff = TimeHelper::getInstance()->getTimeDiffToOmniCam(points[currIndex].timestamp);
		//printf("BallPos Invalidate %lld\n", timeDiff);
		if(timeDiff > ms*10000){
			points[currIndex].valid = false;
			startIndex = currIndex + 1;
			if(startIndex >= size)
				startIndex -= size;
		}

		currIndex++;
		if(currIndex >= size)
			currIndex -= size;
	}

	timeDiff = TimeHelper::getInstance()->getTimeDiffToOmniCam(points[lastIndex].timestamp);

	if(timeDiff > ms*10000){
		points[lastIndex].valid = false;
		startIndex = lastIndex;
	}


	validCounter = 0;
	lastValidIndex = -1;
	if(points[lastIndex].valid){
		validCounter++;
		lastValidIndex = lastIndex;
	}

	currIndex = startIndex;

	while(currIndex != lastIndex){

		if(points[currIndex].valid){
			validCounter++;
			if(lastValidIndex != lastIndex)
				lastValidIndex = currIndex;
		}

		currIndex++;
		if(currIndex >= size)
			currIndex -= size;
	}

	//printf("ObjectContainer invalidate: %d %d\n", lastValidIndex, validCounter);


}


void ObjectContainer::reset(){

	startIndex = 0;
	lastIndex = 0;
	lastValidIndex = 0;
	validCounter = 0;
	points[0].valid = false;

}



int ObjectContainer::getSize(){

	return size;

}

int ObjectContainer::getStartIndex(){

	return startIndex;

}

int ObjectContainer::getLastIndex(){

	return lastIndex;

}

int ObjectContainer::getNumberValidPoints(){

	return validCounter;

}


ObservedPoint * ObjectContainer::getPoints(){

	return points;

}



