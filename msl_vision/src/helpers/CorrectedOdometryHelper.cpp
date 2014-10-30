/*
 * $Id: CorrectedOdometryHelper.cpp 2028 2007-04-11 19:28:28Z cn $
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
#include "CorrectedOdometryHelper.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>

#include <sys/time.h>
#include "SpicaDirectedHelper.h"
#include "TimeHelper.h"
#include "Logger.h"

CorrectedOdometryHelper *CorrectedOdometryHelper::instance = NULL;

void CorrectedOdometryHelper::handleCorrectedOdometryInfo() {

	// TODO: Handle CorrectedOdometryInfo
}

CorrectedOdometryHelper::CorrectedOdometryHelper() {

	initialized = false;

	/*if (SpicaDirectedHelper::visionDirectedCEP != NULL) {
		SpicaDirectedHelper::visionDirectedCEP->getCoi().added = boost::bind(
				&CorrectedOdometryHelper::handleCorrectedOdometryInfo, this, _1, _2);
	}*/

	init();
}


CorrectedOdometryHelper::~CorrectedOdometryHelper(){

	cleanup();

}


CorrectedOdometryHelper *CorrectedOdometryHelper::getInstance() {
	if (instance == NULL) {
		instance = new CorrectedOdometryHelper();
	}
	return instance;
}


void CorrectedOdometryHelper::init(){


}


void CorrectedOdometryHelper::cleanup(){


}


