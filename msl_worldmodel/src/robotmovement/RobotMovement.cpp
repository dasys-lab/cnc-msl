/*
 * RobotMovement.cpp *
 *
 *  Created on: 17.12.2014
 *      Author: tobi
 */

#include "robotmovement/RobotMovement.h"
#include "container/CNPoint2D.h"
#include "GeometryCalculator.h"
#include "MSLWorldModel.h"
#include "pathplanner/PathProxy.h"
#include "pathplanner/evaluator/PathEvaluator.h"

#include <SystemConfig.h>

namespace msl
{
	double RobotMovement::defaultTranslation;
	double RobotMovement::defaultRotateP;
	double RobotMovement::fastTranslation;
	double RobotMovement::fastRotation;
	double RobotMovement::interceptCarfullyRotateP;
	double RobotMovement::lastRotError = 0;
	double RobotMovement::alignToPointMaxRotation;
	double RobotMovement::alignToPointMinRotation;
	double RobotMovement::alignToPointpRot;
	double RobotMovement::lastRotErrorWithBall = 0;
	double RobotMovement::alignMaxVel;
	double RobotMovement::alignToPointRapidMaxRotation = 2 * M_PI;
	double RobotMovement::lastRotErrorWithBallRapid = 0;

	RobotMovement::~RobotMovement()
	{
		// TODO Auto-generated destructor stub
	}

	MotionControl RobotMovement::moveToPointFast(shared_ptr<geometry::CNPoint2D> egoTarget, shared_ptr<geometry::CNPoint2D> egoAlignPoint, double snapDistance,
													shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> additionalPoints)
	{
		MotionControl mc;
		MSLWorldModel* wm = MSLWorldModel::get();
		shared_ptr<PathEvaluator> eval = make_shared<PathEvaluator>(&wm->pathPlanner);
		shared_ptr<geometry::CNPoint2D> temp = PathProxy::getInstance()->getEgoDirection(egoTarget, eval,
		additionalPoints);
		if(temp != nullptr)
		{
			egoTarget = temp;
		}

		mc.motion.angle = egoTarget->angleTo();
		mc.motion.rotation = egoAlignPoint->rotate(M_PI)->angleTo() * fastRotation;
		if (egoTarget->length() > snapDistance)
		{
			mc.motion.translation = std::min(egoTarget->length(), fastTranslation);
		}
		else
		{
			mc.motion.translation = 0;
		}
		return mc;
	}

	MotionControl RobotMovement::moveToPointCarefully(shared_ptr<geometry::CNPoint2D> egoTarget, shared_ptr<geometry::CNPoint2D> egoAlignPoint, double snapDistance,
														shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> additionalPoints )
	{
		MotionControl mc;
		if (egoTarget->length() > 400)
		{
			MSLWorldModel* wm = MSLWorldModel::get();
			shared_ptr<PathEvaluator> eval = make_shared<PathEvaluator>(&wm->pathPlanner);
			shared_ptr<geometry::CNPoint2D> temp = PathProxy::getInstance()->getEgoDirection(egoTarget, eval,
			additionalPoints);
			if(temp == nullptr)
			{
				cout << "alloTarget = nullptr" << endl;
				temp = egoTarget;
			}
			mc.motion.angle = temp->angleTo();
			mc.motion.rotation = egoAlignPoint->rotate(M_PI)->angleTo() * defaultRotateP;
			if (temp->length() > snapDistance)
			{
				mc.motion.translation = std::min(temp->length(), defaultTranslation);
			}
			else
			{
				mc.motion.translation = 0;
			}
		}
		else
		{
			mc.motion.angle = egoTarget->angleTo();
			mc.motion.rotation = egoAlignPoint->rotate(M_PI)->angleTo() * defaultRotateP;
			if (egoTarget->length() > snapDistance)
			{
				mc.motion.translation = std::min(egoTarget->length(), defaultTranslation);
			}
			else
			{
				mc.motion.translation = 0;
			}
		}
		return mc;
	}

	MotionControl RobotMovement::interceptCarefully(shared_ptr<geometry::CNPoint2D> egoTarget, shared_ptr<geometry::CNPoint2D> egoAlignPoint, double snapDistance,
													shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> additionalPoints)
	{
		MotionControl mc;
		if (egoTarget->length() > 400)
		{
			MSLWorldModel* wm = MSLWorldModel::get();
			shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> additionalPoints = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();
			shared_ptr<PathEvaluator> eval = make_shared<PathEvaluator>(&wm->pathPlanner);
			shared_ptr<geometry::CNPoint2D> temp = PathProxy::getInstance()->getEgoDirection(egoTarget, eval,
			additionalPoints);
			if(temp == nullptr)
			{
				cout << "alloTarget = nullptr" << endl;
				temp = egoTarget;
			}
			mc.motion.angle = temp->angleTo();
			mc.motion.rotation = egoAlignPoint->rotate(M_PI)->angleTo() * interceptCarfullyRotateP;
			if (egoTarget->length() > snapDistance)
			{
				mc.motion.translation = min(defaultTranslation, temp->length());
			}
			else
			{
				mc.motion.translation = 0;
			}
			return mc;
		}
		else
		{
			mc.motion.angle = egoTarget->angleTo();
			mc.motion.rotation = egoAlignPoint->rotate(M_PI)->angleTo() * interceptCarfullyRotateP;
			if (egoTarget->length() > snapDistance)
			{
				mc.motion.translation = min(defaultTranslation, egoTarget->length());
			}
			else
			{
				mc.motion.translation = 0;
			}
			return mc;
		}
	}

	MotionControl RobotMovement::alignToPointNoBall(shared_ptr<geometry::CNPoint2D> egoTarget, shared_ptr<geometry::CNPoint2D> egoAlignPoint, double angleTolerance)
	{
		MotionControl mc;
		double egoTargetAngle = egoTarget->angleTo();
		double deltaTargetAngle = geometry::GeometryCalculator::deltaAngle(egoTargetAngle, M_PI);
		if (fabs(egoTargetAngle) < angleTolerance)
		{
			mc.motion.angle = egoTargetAngle;
			mc.motion.rotation = 0;
			mc.motion.translation = 0;

		}
		else
		{
			mc.motion.angle = egoTargetAngle;
			mc.motion.rotation = -(deltaTargetAngle * defaultRotateP + (deltaTargetAngle - lastRotError) * alignToPointpRot);
			mc.motion.rotation = (mc.motion.rotation < 0 ? -1 : 1) * min(alignToPointMaxRotation, max(fabs(mc.motion.rotation), alignToPointMinRotation));
			mc.motion.translation = 0;
			lastRotError = deltaTargetAngle;
		}
		return mc;
	}

	MotionControl RobotMovement::alignToPointWithBall(shared_ptr<geometry::CNPoint2D> egoAlignPoint, shared_ptr<geometry::CNPoint2D> egoBallPos, double angleTolerance, double ballAngleTolerance)
	{
		MotionControl mc;
		MSLWorldModel* wm = MSLWorldModel::get();
		double egoTargetAngle = egoAlignPoint->angleTo();
		double egoBallAngle = egoBallPos->angleTo();
		double deltaTargetAngle = geometry::GeometryCalculator::deltaAngle(egoTargetAngle, M_PI);
		double deltaBallAngle = geometry::GeometryCalculator::deltaAngle(egoBallAngle, M_PI);

		if (fabs(deltaBallAngle) < ballAngleTolerance && fabs(deltaTargetAngle) < angleTolerance)
		{
			mc.motion.angle = 0;
			mc.motion.rotation = 0;
			mc.motion.translation = 0;
		}
		else
		{
			mc.motion.rotation = -(deltaTargetAngle * defaultRotateP + (deltaTargetAngle - lastRotError) * alignToPointpRot);
			mc.motion.rotation = (mc.motion.rotation < 0 ? -1 : 1) * min(alignToPointMaxRotation, max(fabs(mc.motion.rotation), alignToPointMinRotation));

			lastRotErrorWithBall = deltaTargetAngle;

			// crate the motion orthogonal to the ball
			shared_ptr<geometry::CNPoint2D> driveTo = egoBallPos->rotate(-M_PI / 2.0);
			driveTo = driveTo * mc.motion.rotation;

			// add the motion towards the ball
			driveTo = driveTo + egoBallPos->normalize() * 10;

			mc.motion.angle = driveTo->angleTo();
			mc.motion.translation = min(alignMaxVel, driveTo->length());
		}
		return mc;
	}

	MotionControl RobotMovement::rapidAlignToPointWithBall(shared_ptr<geometry::CNPoint2D> egoAlignPoint, shared_ptr<geometry::CNPoint2D> egoBallPos, double angleTolerance, double ballAngleTolerance)
	{
		MotionControl mc;
		MSLWorldModel* wm = MSLWorldModel::get();
		double egoTargetAngle = egoAlignPoint->angleTo();
		double egoBallAngle = egoBallPos->angleTo();
		double deltaTargetAngle = geometry::GeometryCalculator::deltaAngle(egoTargetAngle, M_PI);
		double deltaBallAngle = geometry::GeometryCalculator::deltaAngle(egoBallAngle, M_PI);

		if (fabs(deltaBallAngle) < ballAngleTolerance && fabs(deltaTargetAngle) < angleTolerance)
		{
			mc.motion.angle = 0;
			mc.motion.rotation = 0;
			mc.motion.translation = 0;
		}
		else
		{
			if (egoAlignPoint->angleTo() > M_PI / 2)
			{
				mc.motion.rotation = alignToPointRapidMaxRotation;
			}
			else if (egoAlignPoint->angleTo() < -M_PI / 2)
			{
				mc.motion.rotation = -alignToPointRapidMaxRotation;
			}
			else
			{
				double clausenValue = 0.0;
				for (int i = 1; i < 10; i++)
				{
					clausenValue += sin(i * egoAlignPoint->angleTo()) / pow(i, 2);
				}
				mc.motion.rotation = egoAlignPoint->angleTo() * abs(clausenValue) * 8;

			}
//			mc.motion.rotation = -(deltaTargetAngle * defaultRotateP
//					+ (deltaTargetAngle - lastRotError) * alignToPointpRot);
//			mc.motion.rotation = (mc.motion.rotation < 0 ? -1 : 1)
//					* min(alignToPointMaxRotation, max(fabs(mc.motion.rotation), alignToPointMinRotation));

			lastRotErrorWithBallRapid = deltaTargetAngle;

			// crate the motion orthogonal to the ball
			shared_ptr<geometry::CNPoint2D> driveTo = egoBallPos->rotate(-M_PI / 2.0);
			driveTo = driveTo * mc.motion.rotation;

			// add the motion towards the ball
			driveTo = driveTo + egoBallPos->normalize() * 10;

			mc.motion.angle = driveTo->angleTo();
			mc.motion.translation = min(alignMaxVel, driveTo->length());
		}
		return mc;
	}
	void RobotMovement::readConfigParameters()
	{
		supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();
		defaultTranslation = (*sc)["Drive"]->get<double>("Drive", "DefaultVelocity", NULL);
		defaultRotateP = (*sc)["Drive"]->get<double>("Drive", "DefaultRotateP", NULL);
		fastTranslation = (*sc)["Drive"]->get<double>("Drive.Fast.Velocity", NULL);
		fastRotation = (*sc)["Drive"]->get<double>("Drive.Fast.RotateP", NULL);
		interceptCarfullyRotateP = (*sc)["Drive"]->get<double>("Drive", "InterceptCarefullyRotateP", NULL);
		alignToPointMaxRotation = (*sc)["Drive"]->get<double>("Drive", "AlignToPointMaxRotation", NULL);
		alignToPointMinRotation = (*sc)["Drive"]->get<double>("Drive", "AlignToPointMinRotation", NULL);
		alignToPointpRot = (*sc)["Drive"]->get<double>("Drive", "AlignToPointpRot",	NULL);
		alignMaxVel = (*sc)["Drive"]->get<double>("Drive", "MaxSpeed", NULL);
	}
}

