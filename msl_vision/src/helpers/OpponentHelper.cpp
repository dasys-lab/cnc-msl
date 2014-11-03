/*
 * $Id: OpponentHelper.cpp 1935 2007-03-19 19:50:12Z phbaer $
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
#include "OpponentHelper.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#include <sys/time.h>
//#include <BallMessage.h>

//#include "../global/Packets.h"
//#include "PacketHelper.h"
#include "FootballField.h"
#include <msl_sensor_msgs/BallInfo.h>
#include <msl_sensor_msgs/ObstacleInfo.h>
#include "SpicaHelper.h"


OpponentHelper::OpponentHelper() : sc() {

	this->sc = SystemConfig::getInstance();

	FootballField::getInstance();

	MX = (*this->sc)["Vision.h"]->get<int>("Vision", "CameraMX", NULL);
	MY = (*this->sc)["Vision.h"]->get<int>("Vision", "CameraMY", NULL);

	LocalizationSuccess = (*this->sc)["Localization"]->get<double>("Localization", "LocalizationSuccess", NULL);

	init();


}


OpponentHelper::~OpponentHelper(){

	cleanup();

}



void OpponentHelper::init(){


}


void OpponentHelper::cleanup(){


}


void OpponentHelper::processOpponentBlobs(std::vector<BlobBounds> & potOpponentBlobs, Particle * maxParticle){

/*
	//Point2dInfoPtrListPtr positions = Point2dInfoPtrListPtr(new Point2dInfoPtrList);
	OpponentInfo oi;

	oi->setPositionInfos(positions);

	std::vector<ObservedPoint> observedPoints;
	observedPoints.clear();

	for(unsigned int i = 0; i < potOpponentBlobs.size(); i++){

		int midX = (potOpponentBlobs[i].top + potOpponentBlobs[i].bottom)/2;
		int midY = (potOpponentBlobs[i].left + potOpponentBlobs[i].right)/2;

		double angle = -atan2((midY - MY), (midX - MX));

		Point2dInfo opponentPoint;// = Point2dInfo::create();

		double xcoord = cos(angle)*potOpponentBlobs[i].minDistance;
		double ycoord = sin(angle)*potOpponentBlobs[i].minDistance;

		opponentPoint.x = (xcoord);
		opponentPoint.y = (ycoord);

		ObservedPoint op;
		op.x = xcoord;
		op.y = ycoord;
		op.valid = true;

		positions->push_back(opponentPoint);
		observedPoints.push_back(op);


	}


	unsigned char * validity = (unsigned char *) malloc(observedPoints.size()*sizeof(unsigned char));
	unsigned char * assignedPoints = (unsigned char *) malloc(observedPoints.size()*sizeof(unsigned char));

	free(validity);
	free(assignedPoints);
*/
}





