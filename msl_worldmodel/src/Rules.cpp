/*
 * Rules.cpp
 *
 *  Created on: Aug 27, 2015
 *      Author: labpc1
 */

#include "Rules.h"

namespace msl
{
	Rules* Rules::getInstance()
	{
		static Rules instance;
		return &instance;
	}

	Rules::Rules()
	{
		sc = SystemConfig::getInstance();
		ballRadius = (*this->sc)["Rules"]->get<double>("Rules.BallRadius", NULL);
		robotRadius = (*this->sc)["Rules"]->get<double>("Rules", "MaxRobotRadius", NULL);
		standbyTime = (*this->sc)["Rules"]->get<double>("Rules.Standards", "StandbyTime", NULL);
		stayAwayRadius = (*this->sc)["Rules"]->get<double>("Rules.Standards", "StayAwayRadius", NULL);
		stayAwayRadiusOpp = (*this->sc)["Rules"]->get<double>("Rules.Standards", "StayAwayRadiusOpp", NULL);
		stayAwayRadiusDropBall = (*this->sc)["Rules"]->get<double>("Rules.Standards", "StayAwayRadiusDropBall", NULL);
		pushDistance = (*this->sc)["Rules"]->get<double>("Rules.Standards", "PushDistance", NULL);
		kickDistance = (*this->sc)["Rules"]->get<double>("Rules.Standards", "KickDistance", NULL);
		penaltyTimeForShot = (*this->sc)["Rules"]->get<double>("Rules.Standards", "PenaltyTimeForShot", NULL);
	}

	Rules::~Rules()
	{
		// TODO Auto-generated destructor stub
	}

	double Rules::getBallRadius()
	{
		return ballRadius;
	}

	double Rules::getRobotRadius()
	{
		return robotRadius;
	}

	double Rules::getStandbyTime()
	{
		return standbyTime;
	}

	double Rules::getStayAwayRadius()
	{
		return stayAwayRadius;
	}

	double Rules::getStayAwayRadiusOpp()
	{
		return stayAwayRadiusOpp;
	}

	double Rules::getStayAwayRadiusDropBall()
	{
		return stayAwayRadiusDropBall;
	}

	double Rules::getPushDistance()
	{
		return pushDistance;
	}

	double Rules::getKickDistance()
	{
		return kickDistance;
	}

	double Rules::getPenaltyTimeForShot()
	{
		return penaltyTimeForShot;
	}


} /* namespace msl */
