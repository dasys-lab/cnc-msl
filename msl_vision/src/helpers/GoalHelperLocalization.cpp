/*
 * $Id: GoalHelperLocalization.cpp 2032 2007-04-11 20:19:07Z phbaer $
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
#include "GoalHelperLocalization.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#include <sys/time.h>
//#include <GoalMessage.h>
//#include <FreeAreaMessage.h>
//#include <YellowGoalInfo.h>
//#include <BlueGoalInfo.h>
//#include <YellowFreeAreaInfo.h>
//#include <BlueFreeAreaInfo.h>
#include "SpicaHelper.h"
#include "FootballField.h"

//#include "../global/Packets.h"
//#include "PacketHelper.h"

//using namespace carpenoctem::messages;

GoalHelperLocalization::GoalHelperLocalization(){

	FootballField::getInstance();

	msgid = 0;	
	init();


}


GoalHelperLocalization::~GoalHelperLocalization(){

	cleanup();

}



void GoalHelperLocalization::init(){


}


void GoalHelperLocalization::cleanup(){


}


void GoalHelperLocalization::getGoalsFromPosition(Position & position){


	//YellowGoal
/*
	Goal yellowGoal;

	Point yLeftPostAllo;
	yLeftPostAllo.x = FootballField::FieldLength/2.0;
	yLeftPostAllo.y = FootballField::GoalWidth/2.0;

	Point yRightPostAllo;
	yRightPostAllo.x = FootballField::FieldLength/2.0;
	yRightPostAllo.y = -FootballField::GoalWidth/2.0;
		
	yellowGoal.leftPost = getAlloPoint(yLeftPostAllo, position);
	yellowGoal.rightPost = getAlloPoint(yRightPostAllo, position);


	if (SpicaHelper::visionCEP) {
		Point2dInfoPtr leftPost = Point2dInfo::create();
		Point2dInfoPtr rightPost = Point2dInfo::create();
	
		leftPost->setX(yellowGoal.leftPost.x);
		leftPost->setY(yellowGoal.leftPost.y);
	
		rightPost->setX(yellowGoal.rightPost.x);
		rightPost->setY(yellowGoal.rightPost.y);

		YellowGoalInfoPtr gi = YellowGoalInfo::create();

		gi->setIsYellow(true);
		gi->setLeftPost(leftPost);
		gi->setRightPost(rightPost);

		SpicaHelper::wm->getData()->push_back(gi);

		YellowFreeAreaInfoPtr fai = YellowFreeAreaInfo::create();
		
		//fai = new CarpeNoctem::Messages::Information::YFreeAreaInfo;
		fai->setIsYellow(true);
		fai->setAngle1(atan2(yellowGoal.leftPost.y, yellowGoal.leftPost.x));
		fai->setAngle2(atan2(yellowGoal.rightPost.y, yellowGoal.rightPost.x));

		SpicaHelper::wm->getData()->push_back(fai);

	}


	//BlueGoal

	Goal blueGoal;
7
	Point bLeftPostAllo;
	bLeftPostAllo.x = -FootballField::FieldLength/2.0;
	bLeftPostAllo.y = -FootballField::GoalWidth/2.0;

	Point bRightPostAllo;
	bRightPostAllo.x = -FootballField::FieldLength/2.0;
	bRightPostAllo.y = FootballField::GoalWidth/2.0;
		
	blueGoal.leftPost = getAlloPoint(bLeftPostAllo, position);
	blueGoal.rightPost = getAlloPoint(bRightPostAllo, position);


	if (SpicaHelper::visionCEP) {
		Point2dInfoPtr leftPost = Point2dInfo::create();
		Point2dInfoPtr rightPost = Point2dInfo::create();
	
		leftPost->setX(blueGoal.leftPost.x);
		leftPost->setY(blueGoal.leftPost.y);
	
		rightPost->setX(blueGoal.rightPost.x);
		rightPost->setY(blueGoal.rightPost.y);

		BlueGoalInfoPtr gi = BlueGoalInfo::create();

		gi->setIsYellow(false);
		gi->setLeftPost(leftPost);
		gi->setRightPost(rightPost);

		SpicaHelper::wm->getData()->push_back(gi);

		BlueFreeAreaInfoPtr fai = BlueFreeAreaInfo::create();

		
		//fai = new CarpeNoctem::Messages::Information::YFreeAreaInfo;
		fai->setIsYellow(false);
		fai->setAngle1(atan2(blueGoal.leftPost.y, blueGoal.leftPost.x));
		fai->setAngle2(atan2(blueGoal.rightPost.y, blueGoal.rightPost.x));

		SpicaHelper::wm->getData()->push_back(fai);

	}

*/
}


Point GoalHelperLocalization::getAlloPoint(Point point, Position position){


	Point egoPoint;

	double x = point.x - position.x;
	double y = point.y - position.y;

	double angle = atan2(y,x) - position.heading;
	double dist = sqrt(x*x + y*y); 

	egoPoint.x = cos(angle)*dist;
	egoPoint.y = sin(angle)*dist;

	return egoPoint;


}

