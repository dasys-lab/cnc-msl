/*
 * Rules.cpp
 *
 *  Created on: Aug 27, 2015
 *      Author: labpc1
 */

#include "Rules.h"

namespace msl
{

	Rules::Rules()
	{
		sc = SystemConfig::getInstance();
		double ballRadius = (*this->sc)["Rules"]->get<double>("Rules", "BallRadius", NULL);
		double robotRadius = (*this->sc)["Rules"]->get<double>("Rules", "RobotRadius", NULL);;
		double standbyTime = (*this->sc)["Rules"]->get<double>("Rules.Standards", "StandbyTime", NULL);;
		double stayAwayRadius = (*this->sc)["Rules"]->get<double>("Rules.Standards", "StayAwayRadius", NULL);;
		double stayAwayRadiusOpp = (*this->sc)["Rules"]->get<double>("Rules.Standards", "StayAwayRadiusOpp", NULL);;
		double stayAwayRadiusDropBall = (*this->sc)["Rules"]->get<double>("Rules.Standards", "StayAwayRadiusDropBall", NULL);;
		double pushDistance = (*this->sc)["Rules"]->get<double>("Rules.Standards", "PushDistance", NULL);;
		double kickDistance = (*this->sc)["Rules"]->get<double>("Rules.Standards", "KickDistance", NULL);;
		double penaltyTimeForShot = (*this->sc)["Rules"]->get<double>("Rules.Standards", "PenaltyTimeForShot", NULL);;


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
