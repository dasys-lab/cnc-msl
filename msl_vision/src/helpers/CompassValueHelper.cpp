/*
 * $Id: CompassValueHelper.cpp 2028 2007-04-11 19:28:28Z cn $
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
#include "CompassValueHelper.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <DateTime.h>

#include <sys/time.h>
#include "SpicaHelper.h"
#include "Logger.h"


using namespace msl_sensor_msgs;

CompassValueHelper * CompassValueHelper::instance_ = NULL;

CompassValueHelper * CompassValueHelper::getInstance(){

	if(instance_ == NULL)
		instance_ = new CompassValueHelper();
	return instance_;

}


void CompassValueHelper::handleCompassInfo(const CompassInfo::ConstPtr& message) {


	printf("GOT COMPASS VALUE %ld\n", message->value);

	integrateData(message->value);

	Logger::getInstance()->logCompassValue(message->value, supplementary::DateTime::getUtcNowC());

}


CompassValueHelper::CompassValueHelper() : mutex() {

	compassValue = 0;
	updateCycles = 30;

	sub = SpicaHelper::visionNode->subscribe<CompassInfo, CompassValueHelper>("CompassInfo", 1, &CompassValueHelper::handleCompassInfo, (this), ros::TransportHints().udp());

	workWithoutCompass = false;
	char * envVariable = getenv("VISION_FORCE");
	if(envVariable != NULL)
		workWithoutCompass = true;

	spinner = new ros::AsyncSpinner(1);
	spinner->start();



	init();


}


CompassValueHelper::~CompassValueHelper(){

	cleanup();

}



void CompassValueHelper::init(){


}


void CompassValueHelper::cleanup(){


}


void CompassValueHelper::integrateData(int value) {

	boost::mutex::scoped_lock(this->mutex);

	printf("CompassValueCAN: %d\n", value);
	updateCycles = 0;
	compassValue = value;

}



int CompassValueHelper::getCompassData(){

	boost::mutex::scoped_lock(this->mutex);

	updateCycles++;

	if(updateCycles >= 150){
		printf("Without Compass Data I cannot work!\n");
		//if(!workWithoutCompass)
		//	exit(1);
		return -1;
	}

	if(updateCycles >= 30){
		printf("Tried to get Compass Data, but without success!\n");
		return -1;
	}

	return compassValue;

}

int CompassValueHelper::getCompassData2(){

	boost::mutex::scoped_lock(this->mutex);

	if(updateCycles >= 150){
		printf("Without Compass Data I cannot work!\n");
		//if(!workWithoutCompass)
		//	exit(1);
		return -1;
	}

	if(updateCycles >= 30){
		printf("Tried to get Compass Data, but without success!\n");
		return -1;
	}

	return compassValue;

}

