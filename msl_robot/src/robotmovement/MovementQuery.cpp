/*
 * MovementQuery.cpp
 *
 *  Created on: Apr 27, 2016
 *      Author: Carpe Noctem
 */

#include "msl_robot/robotmovement/MovementQuery.h"
#include "msl_robot/MSLRobot.h"
#include "msl_robot/kicker/Kicker.h"
#include <MSLWorldModel.h>

namespace msl
{
	MovementQuery::MovementQuery()
	{
		this->egoAlignPoint = nullptr;
		this->egoDestinationPoint = nullptr;
		this->additionalPoints = nullptr;
		this->fast = false;
		this->dribble = false;
		this->blockOppPenaltyArea = false;
		this->blockOppGoalArea = false;
		this->blockOwnPenaltyArea = false;
		this->blockOwnGoalArea = false;
		this->block3MetersAroundBall = false;
		this->snapDistance = 0;
		this->angleTolerance = 0;
		this->alloTeamMatePosition = nullptr;
		this->wm = MSLWorldModel::get();

		resetAllPDParameters();
		readConfigParameters();
	}

	MovementQuery::~MovementQuery()
	{

	}

	double MovementQuery::rotationPDForDribble(shared_ptr<geometry::CNPoint2D> egoTarget)
	{
		double angleErr = egoTarget->rotate(this->robot->kicker->kickerAngle)->angleTo();
		double rot = this->pRot * angleErr + this->dRot * geometry::normalizeAngle(angleErr - this->lastRotDribbleErr); //Rotation PD

		// limit rotation acceleration
		if (rot > this->curRotDribble)
		{
			rot = min(rot, this->curRotDribble + this->rotAccStep);
		}
		else
		{
			rot = max(rot, this->curRotDribble - this->rotAccStep);
		}

		// clamp rotation
		rot = min(abs(rot), this->maxRot) * (rot > 0 ? 1 : -1);

		this->curRotDribble = rot;

		this->lastRotDribbleErr = angleErr;
		return rot;
	}

	double MovementQuery::translationPDForDribble(double transOrt)
	{
		double maxCurTrans = this->maxVel;
		double transErr = abs(this->lastRotDribbleErr);
		if (transErr > this->angleDeadBand)
		{
			this->transControlIntegralDribble += this->iTrans * transErr;
			this->transControlIntegralDribble = min(this->transControlIntegralMax, this->transControlIntegralDribble);
		}
		else
		{
			this->transControlIntegralDribble = 0; //Math.Max(0,transControlIntegral-Math.PI*5);
			transErr = 0;
		}
		maxCurTrans -= this->pTrans * transErr + this->transControlIntegralDribble;
		maxCurTrans = max(0.0, maxCurTrans);

		double transTowards = sqrt(maxCurTrans * maxCurTrans - transOrt * transOrt);
		if (std::isnan(transTowards) || transTowards < 50)
			transTowards = 50;

		if (transTowards > this->curTransDribble)
		{
			transTowards = min(transTowards, this->curTransDribble + this->transAccStep);
		}
		else
		{
			transTowards = max(transTowards, this->curTransDribble - this->transDecStep);
		}

		this->curTransDribble = transTowards;

		return sqrt(transTowards * transTowards + transOrt * transOrt);
	}

	double MovementQuery::angleCalcForDribble(double transOrt)
	{
		auto ballPos = this->wm->ball->getEgoBallPosition();
		auto dir = ballPos->normalize();
		auto ort = make_shared<geometry::CNPoint2D>(dir->y, -dir->x);
		dir = dir * this->curTransDribble + ort * transOrt;
		return dir->angleTo();
	}

	void MovementQuery::reset()
	{
		this->egoAlignPoint = nullptr;
		this->egoDestinationPoint = nullptr;
		this->additionalPoints = nullptr;
		this->fast = false;
		this->dribble = false;
		this->blockOppPenaltyArea = false;
		this->blockOppGoalArea = false;
		this->blockOwnPenaltyArea = false;
		this->blockOwnGoalArea = false;
		this->block3MetersAroundBall = false;
		this->snapDistance = 0;
		this->angleTolerance = 0;
		this->alloTeamMatePosition = nullptr;
		this->wm = MSLWorldModel::get();

		resetAllPDParameters();
		readConfigParameters();
	}

	void MovementQuery::resetAllPDParameters()
	{
		resetRotationPDParameters();
		resetTransaltionPDParameters();
	}

	void MovementQuery::resetRotationPDParameters()
	{
		this->curRotDribble = 0;
		this->lastRotDribbleErr = 0;
		readConfigParameters();
	}

	void MovementQuery::resetTransaltionPDParameters()
	{
		this->curTransDribble = 0;
		this->transControlIntegralDribble = 0;
		readConfigParameters();
	}

	void MovementQuery::readConfigParameters()
	{
		supplementary::SystemConfig* supplementary = supplementary::SystemConfig::getInstance();
		// load rotation config parameters
		this->pRot = (*supplementary::SystemConfig::getInstance())["Dribble"]->get<double>("DribbleWater", "pRot",
																							NULL);
		this->dRot = (*supplementary::SystemConfig::getInstance())["Dribble"]->get<double>("DribbleWater", "dRot",
																							NULL);
		this->rotAccStep = (*supplementary::SystemConfig::getInstance())["Dribble"]->get<double>(
				"DribbleWater", "MaxRotationAcceleration", NULL);
		this->maxRot = (*supplementary::SystemConfig::getInstance())["Dribble"]->get<double>("DribbleWater",
																								"MaxRotation", NULL);

		// load translation config patamerters
		this->transAccStep = (*supplementary::SystemConfig::getInstance())["Dribble"]->get<double>("DribbleWater",
																									"MaxAcceleration",
																									NULL);
		this->transDecStep = (*supplementary::SystemConfig::getInstance())["Dribble"]->get<double>("DribbleWater",
																									"MaxDecceleration",
																									NULL);
		this->iTrans = (*supplementary::SystemConfig::getInstance())["Dribble"]->get<double>("DribbleWater", "iTrans",
																								NULL) / M_PI;
		this->pTrans = (*supplementary::SystemConfig::getInstance())["Dribble"]->get<double>("DribbleWater", "pTrans",
																								NULL) / M_PI;
		this->transControlIntegralMax = (*supplementary::SystemConfig::getInstance())["Dribble"]->get<double>(
				"DribbleWater", "maxTransIntegral", NULL);
		this->angleDeadBand = (*supplementary::SystemConfig::getInstance())["Dribble"]->get<double>("DribbleWater",
																									"angleDeadBand",
																									NULL) / 180 * M_PI;
		this->maxVel = (*supplementary::SystemConfig::getInstance())["Dribble"]->get<double>("DribbleWater",
																								"MaxVelocity", NULL);
	}
}
