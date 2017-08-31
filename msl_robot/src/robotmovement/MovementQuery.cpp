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
#include <pathplanner/PathPlannerQuery.h>
#include <pathplanner/evaluator/IPathEvaluator.h>
#include "RawSensorData.h"

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

		this->rotateAroundTheBall = false;
		this->circleRadius = -1;
		this->circleCenterPoint = nullptr;
		this->rectangleUpperLeftCorner = nullptr;
		this->rectangleLowerRightCorner = nullptr;
		this->pathEval = nullptr;

		resetAllPIDParameters();
		readConfigParameters();
	}

	MovementQuery::~MovementQuery()
	{
	}

	shared_ptr<PathPlannerQuery> MovementQuery::getPathPlannerQuery()
	{
		shared_ptr<PathPlannerQuery> ret = make_shared<PathPlannerQuery>();
		ret->additionalPoints = this->additionalPoints;
		ret->block3MetersAroundBall = this->block3MetersAroundBall;
		ret->blockOppGoalArea = this->blockOppGoalArea;
		ret->blockOppPenaltyArea = this->blockOppPenaltyArea;
		ret->blockOwnGoalArea = this->blockOwnGoalArea;
		ret->blockOwnPenaltyArea = this->blockOwnPenaltyArea;
		ret->circleCenterPoint = this->circleCenterPoint;
		ret->circleRadius = this->circleRadius;
		ret->rectangleLowerRightCorner = this->rectangleLowerRightCorner;
		ret->rectangleUpperLeftCorner = this->rectangleUpperLeftCorner;
		return ret;
	}

	/**
	 * PT-Controller for smooth translation acceleration
	 */
	std::valarray<double> MovementQuery::ptController(double angle, double rotation, double translation)
	{
		double input[] = {cos(angle) * translation, sin(angle) * translation, rotation};

		int newJump = max(abs(pastControlInput.back()[0] - pastControlInput.front()[0]),
							abs(pastControlInput.back()[1] - pastControlInput.front()[1]));

		pastControlInput.push(std::valarray<double>(input, 3));

		if (newJump != 0)
		{
			lastJump = newJump;
		}

		// slope variable
		double a = 6.33333 - 4.0 / 3000.0 * lastJump;
		// changing point for slope
		double b = pow(a, 2.0);
		// sending frequency
		double TA = 1.0 / 30.0;

		double n1 = 1.0 - exp(-a * TA) - exp(-a * TA) * a * TA;
		double n2 = exp(-2 * a * TA) - exp(-a * TA) + exp(-a * TA) * TA * a;

		double d1 = -2 * exp(-a * TA);
		double d2 = exp(-2 * a * TA);

				cout << "n1 = " << n1 << endl;
				cout << "n2 = " << n2 << endl;

				cout << "d1 = " << d1 << endl;
				cout << "d2 = " << d2 << endl;

		pastTranslations.push(std::valarray<double>(init, 3));
		pastTranslations.back() += n2 * pastControlInput.front() - d2 * pastTranslations.front();
		pastControlInput.pop();
		pastTranslations.pop();
		pastTranslations.back() += n1 * pastControlInput.front() - d1 * pastTranslations.front();

		return pastTranslations.back();
	}

	double MovementQuery::rotationPDForDribble(shared_ptr<geometry::CNPoint2D> egoTarget)
	{
		cout << "MovementQuery::rotationPDForDribble: egoTarget = " << egoTarget->toString();

		double angleErr = egoTarget->rotate(this->robot->kicker->kickerAngle)->angleTo();
		double rot = this->pRot * angleErr + this->dRot * geometry::normalizeAngle(angleErr - this->lastRotDribbleErr); // Rotation PD

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

	double MovementQuery::translationPIForDribble(double transOrt)
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
			this->transControlIntegralDribble = 0; // Math.Max(0,transControlIntegral-Math.PI*5);
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

		resetAllPIDParameters();
		readConfigParameters();
	}

	void MovementQuery::blockCircle(shared_ptr<geometry::CNPoint2D> centerPoint, double radius)
	{
		this->circleCenterPoint = centerPoint;
		this->circleRadius = radius;
	}

	void MovementQuery::blockRectangle(shared_ptr<geometry::CNPoint2D> upLeftCorner,
										shared_ptr<geometry::CNPoint2D> lowRightCorner)
	{
		this->rectangleUpperLeftCorner = upLeftCorner;
		this->rectangleLowerRightCorner = lowRightCorner;
	}

	/**
	 * Reset all Parameters for the methods rotationPDForDribble() and  translationPIForDribble()
	 */
	void MovementQuery::resetAllPIDParameters()
	{
		resetRotationPDParameters();
		resetTransaltionPIParameters();
	}

	void MovementQuery::resetRotationPDParameters()
	{
		this->curRotDribble = 0;
		this->lastRotDribbleErr = 0;
		readConfigParameters();
	}

	void MovementQuery::resetTransaltionPIParameters()
	{
		this->curTransDribble = 0;
		this->transControlIntegralDribble = 0;
		readConfigParameters();
	}

	/**
	 * Sets P and D parameters for rotationPDForDribble()
	 * @pParam
	 * @dParam
	 */
	void MovementQuery::setRotationPDParameters(double pParam, double dParam)
	{
		this->pRot = pParam;
		this->dRot = dParam;
	}

	/**
	 * Sets P and I parameters for translationPIForDribble()
	 * @pParam
	 * @iParam
	 */
	void MovementQuery::setTranslationPIParameters(double pParam, double iParam)
	{
		this->pTrans = pParam;
		this->iTrans = iParam;
	}

	/**
	 * Initialize all needed parameters and queues for the PT-Controller
	 */
	void MovementQuery::initializePTControllerParameters()
	{
		// initial pt-controller stuff
		std::queue<std::valarray<double>> controlInput;
		auto odom = wm->rawSensorData->getOwnVelocityMotion();

		if (odom == nullptr)
		{
			cerr << "MovementQuery: no odometry!" << endl;
			return;
		}

		auto translation = odom->translation;
		auto angle = odom->angle;
		auto rotation = (double)odom->rotation;

//		double translation = 0;
//		double angle = 0;
//		double rotation = 0;

		double input[] = {cos(angle) * translation, sin(angle) * translation, rotation};

		if (pastTranslations.empty())
		{
			pastTranslations.push(std::valarray<double>(input, 3));
			pastTranslations.push(std::valarray<double>(input, 3));
		}
		if (pastControlInput.empty())
		{
			pastControlInput.push(std::valarray<double>(input, 3));
			pastControlInput.push(std::valarray<double>(input, 3));
		}

		lastJump = 1000;
	}

	void MovementQuery::clearPTControllerQueues()
	{
		pastControlInput.push(std::valarray<double>(init, 3));
		pastControlInput.push(std::valarray<double>(init, 3));
		pastControlInput.push(std::valarray<double>(init, 3));
		pastTranslations.push(std::valarray<double>(init, 3));
		pastTranslations.push(std::valarray<double>(init, 3));
		pastTranslations.push(std::valarray<double>(init, 3));
		pastControlInput.pop();
		pastControlInput.pop();
		pastControlInput.pop();
		pastTranslations.pop();
		pastTranslations.pop();
		pastTranslations.pop();
	}

	/**
	 * Reads all necessary parameters from Dribble.conf
	 */
	void MovementQuery::readConfigParameters()
	{
		supplementary::SystemConfig *supplementary = supplementary::SystemConfig::getInstance();
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
