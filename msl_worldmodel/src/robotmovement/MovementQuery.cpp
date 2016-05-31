/*
 * MovementQuery.cpp
 *
 *  Created on: Apr 27, 2016
 *      Author: Carpe Noctem
 */

#include "robotmovement/MovementQuery.h"
namespace msl
{
	MovementQuery::MovementQuery()
	{
		egoAlignPoint = nullptr;
		egoDestinationPoint = nullptr;
		additionalPoints = nullptr;
		fast = false;
		dribble = false;
		curRotDribble = 0;
		lastRotDribbleErr = 0;
		snapDistance = 0;
		angleTolerance = 0;
		teamMatePosition = nullptr;
		curTransDribble = 0;
		transControlIntegralDribble = 0;
		wm = MSLWorldModel::get();

		resetAllPDParameters();
		readConfigParameters();
	}

	MovementQuery::~MovementQuery()
	{

	}

	double MovementQuery::rotationPDForDribble(shared_ptr<geometry::CNPoint2D> egoTarget)
	{
		double angleErr = egoTarget->rotate(wm->kicker->kickerAngle)->angleTo();
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

		// TODO: should be set by the behaviour, not here, because we don't know whether the behaviour accepts our rot value
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
			this->transControlIntegralDribble = min(transControlIntegralMax, this->transControlIntegralDribble);
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

		// TODO: should be set by the behaviour, not here, because we don't know whether the behaviour accepts our trans value
		this->curTransDribble = transTowards;

		return sqrt(transTowards * transTowards + transOrt * transOrt);
	}

	double MovementQuery::anglePDForDribble(double transOrt)
	{
		auto ballPos = wm->ball->getEgoBallPosition();
		auto dir = ballPos->normalize();
		auto ort = make_shared<geometry::CNPoint2D>(dir->y, -dir->x);
		dir = dir * this->curTransDribble + ort * transOrt;
		// TODO: This is not a PD controller, currently it only calculates the direction to drive during dribble

		return dir->angleTo();
	}

	void MovementQuery::resetAllPDParameters()
	{
		resetRotationPDParameters();
		resetTransaltionPDParameters();
//		resetAnglePDParameters();
	}

	void MovementQuery::resetRotationPDParameters()
	{
		curRotDribble = 0;
		lastRotDribbleErr = 0;
		readConfigParameters();
	}

	void MovementQuery::resetTransaltionPDParameters()
	{
		curTransDribble = 0;
		transControlIntegralDribble = 0;
		readConfigParameters();
	}

//	void MovementQuery::resetAnglePDParameters()
//	{
//	}

	void MovementQuery::readConfigParameters()
	{
		supplementary::SystemConfig* supplementary = supplementary::SystemConfig::getInstance();
		// load rotation config parameters
		pRot = (*supplementary::SystemConfig::getInstance())["Dribble"]->get<double>("DribbleWater", "pRot", NULL);
		dRot = (*supplementary::SystemConfig::getInstance())["Dribble"]->get<double>("DribbleWater", "dRot", NULL);
		rotAccStep = (*supplementary::SystemConfig::getInstance())["Dribble"]->get<double>("DribbleWater", "MaxRotationAcceleration", NULL);
		maxRot = (*supplementary::SystemConfig::getInstance())["Dribble"]->get<double>("DribbleWater", "MaxRotation", NULL);

		// load translation config patamerters
		transAccStep = (*supplementary::SystemConfig::getInstance())["Dribble"]->get<double>("DribbleWater", "MaxAcceleration", NULL);
		transDecStep = (*supplementary::SystemConfig::getInstance())["Dribble"]->get<double>("DribbleWater", "MaxDecceleration", NULL);
		iTrans = (*supplementary::SystemConfig::getInstance())["Dribble"]->get<double>("DribbleWater", "iTrans", NULL) / M_PI;
		pTrans = (*supplementary::SystemConfig::getInstance())["Dribble"]->get<double>("DribbleWater", "pTrans", NULL) / M_PI;
		transControlIntegralMax = (*supplementary::SystemConfig::getInstance())["Dribble"]->get<double>("DribbleWater", "maxTransIntegral", NULL);
		angleDeadBand = (*supplementary::SystemConfig::getInstance())["Dribble"]->get<double>("DribbleWater", "angleDeadBand", NULL) / 180 * M_PI;
		maxVel = (*supplementary::SystemConfig::getInstance())["Dribble"]->get<double>("DribbleWater", "MaxVelocity", NULL);
	}
}
