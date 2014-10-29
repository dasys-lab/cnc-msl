/*
 * $Id: SharedBallDirectedHelper.cpp 2028 2007-04-11 19:28:28Z cn $
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
#include "SharedBallDirectedHelper.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <DateTime.h>
#include "SpicaHelper.h"

#include <sys/time.h>
#include "SpicaDirectedHelper.h"
#include "TimeHelper.h"
#include "Logger.h"

using namespace CNSensorMsgs;

SharedBallDirectedHelper *SharedBallDirectedHelper::instance = NULL;

void SharedBallDirectedHelper::handleSharedBallInfo(const SharedBallInfo::ConstPtr& message) {

	// TODO: Handle SharedBallInfo

	boost::mutex::scoped_lock(this->mutex);
	
	printf("Received SharedBall %f %f %f %d\n", message->point.x, message->point.y, message->confidence, message->evidence);

	ball.x = message->point.x;
	ball.y = message->point.y;
	ball.confidence = message->confidence;
	ball.evidence = message->evidence;
	ball.timestamp = castor::DateTime::getUtcNowC();

}

SharedBallDirectedHelper::SharedBallDirectedHelper() : mutex() {

	initialized = false;

	sub = SpicaHelper::visionNode->subscribe<SharedBallInfo, SharedBallDirectedHelper>("SharedBallInfo,", 1, &SharedBallDirectedHelper::handleSharedBallInfo, (this), ros::TransportHints().udp());
	init();
}


SharedBallDirectedHelper::~SharedBallDirectedHelper(){

	cleanup();

}


SharedBallDirectedHelper *SharedBallDirectedHelper::getInstance() {
	if (instance == NULL) {
		instance = new SharedBallDirectedHelper();
	}
	return instance;
}


void SharedBallDirectedHelper::init(){


}


void SharedBallDirectedHelper::cleanup(){


}

SharedBall SharedBallDirectedHelper::getBall(){

	boost::mutex::scoped_lock(this->mutex);
	return ball;	

}
