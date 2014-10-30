/*
 * $Id: SharedBallHelper.cpp 2028 2007-04-11 19:28:28Z cn $
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
#include "SharedBallHelper.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>

#include <sys/time.h>
#include "SpicaHelper.h"
#include "TimeHelper.h"
#include "Logger.h"

SharedBallHelper *SharedBallHelper::instance = NULL;

void SharedBallHelper::handleSharedBallInfo(const SharedBallInfo::ConstPtr& message) {

	// TODO: Handle SharedBallInfo
}

SharedBallHelper::SharedBallHelper() {

	initialized = false;

	/*if (SpicaHelper::visionCEP != NULL) {
		SpicaHelper::visionCEP->getSharedBall().added = boost::bind(
				&SharedBallHelper::handleSharedBallInfo, this, _1, _2);
	}*/

	init();
}


SharedBallHelper::~SharedBallHelper(){

	cleanup();

}

SharedBallHelper *SharedBallHelper::getInstance() {
	if (instance == NULL) {
		instance = new SharedBallHelper();
	}
	return instance;
}


void SharedBallHelper::init(){


}


void SharedBallHelper::cleanup(){


}


