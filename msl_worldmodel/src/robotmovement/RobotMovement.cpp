/*
 * RobotMovement.cpp *
 *
 *  Created on: 17.12.2014
 *      Author: tobi
 */

#include "robotmovement/RobotMovement.h"
#include "MSLFootballField.h"
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

	MotionControl RobotMovement::moveToPointFast(shared_ptr<geometry::CNPoint2D> egoTarget,
													shared_ptr<geometry::CNPoint2D> egoAlignPoint, double snapDistance,
													shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> additionalPoints)
	{
		MotionControl mc;
		MSLWorldModel* wm = MSLWorldModel::get();
		shared_ptr<PathEvaluator> eval = make_shared<PathEvaluator>();
		shared_ptr<geometry::CNPoint2D> temp = PathProxy::getInstance()->getEgoDirection(egoTarget, eval,
		additionalPoints);
		if(temp != nullptr)
		{
			cout << "RobotMovement::moveToPointFast::getEgoDirection == nullptr => ownPos not available" << endl;
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

	MotionControl RobotMovement::moveToPointCarefully(shared_ptr<geometry::CNPoint2D> egoTarget,
														shared_ptr<geometry::CNPoint2D> egoAlignPoint,
														double snapDistance,
														shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> additionalPoints )
	{

		MSLWorldModel* wm = MSLWorldModel::get();
		shared_ptr<PathEvaluator> eval = make_shared<PathEvaluator>();
		shared_ptr<geometry::CNPoint2D> temp = PathProxy::getInstance()->getEgoDirection(egoTarget, eval,
		additionalPoints);
		if(temp == nullptr)
		{
			cout << "RobotMovement::moveToPointCarefully::getEgoDirection == nullptr => ownPos not available" << endl;
			temp = egoTarget;
		}
		MotionControl mc;
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
		return mc;
	}

	MotionControl RobotMovement::interceptCarefully(shared_ptr<geometry::CNPoint2D> egoTarget,
													shared_ptr<geometry::CNPoint2D> egoAlignPoint, double snapDistance,
													shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> additionalPoints)
	{
		MotionControl mc;
		if (egoTarget->length() > 400)
		{
			MSLWorldModel* wm = MSLWorldModel::get();
			shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> additionalPoints = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();
			shared_ptr<PathEvaluator> eval = make_shared<PathEvaluator>();
			shared_ptr<geometry::CNPoint2D> temp = PathProxy::getInstance()->getEgoDirection(egoTarget, eval,
			additionalPoints);
			if(temp == nullptr)
			{
				cout << "RobotMovement::interceptCarefully::getEgoDirection == nullptr => ownPos not available" << endl;
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

	MotionControl RobotMovement::alignToPointNoBall(shared_ptr<geometry::CNPoint2D> egoTarget,
													shared_ptr<geometry::CNPoint2D> egoAlignPoint,
													double angleTolerance)
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
			mc.motion.rotation = -(deltaTargetAngle * defaultRotateP
					+ (deltaTargetAngle - lastRotError) * alignToPointpRot);
			mc.motion.rotation = (mc.motion.rotation < 0 ? -1 : 1)
					* min(alignToPointMaxRotation, max(fabs(mc.motion.rotation), alignToPointMinRotation));
			mc.motion.translation = 0;
			lastRotError = deltaTargetAngle;
		}
		return mc;
	}

	MotionControl RobotMovement::alignToPointWithBall(shared_ptr<geometry::CNPoint2D> egoAlignPoint,
														shared_ptr<geometry::CNPoint2D> egoBallPos,
														double angleTolerance, double ballAngleTolerance)
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
			mc.motion.rotation = -(deltaTargetAngle * defaultRotateP
					+ (deltaTargetAngle - lastRotError) * alignToPointpRot);
			mc.motion.rotation = (mc.motion.rotation < 0 ? -1 : 1)
					* min(alignToPointMaxRotation, max(fabs(mc.motion.rotation), alignToPointMinRotation));

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

	MotionControl RobotMovement::rapidAlignToPointWithBall(shared_ptr<geometry::CNPoint2D> egoAlignPoint,
															shared_ptr<geometry::CNPoint2D> egoBallPos,
															double angleTolerance, double ballAngleTolerance)
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

	shared_ptr<msl_actuator_msgs::MotionControl> RobotMovement::ruleActionForBallGetter()
	{
//		MSLWorldModel* wm = MSLWorldModel::get();
//		MSLFootballField* field = MSLFootballField::getInstance();
//		shared_ptr<geometry::CNPoint2D> ballPos = wm->ball.getEgoBallPosition();
//		if (ballPos == nullptr)
//		{
//			return nullptr;
//		}
//		shared_ptr<geometry::CNPosition> ownPos = wm->rawSensorData.getOwnPositionVision(); //OwnPositionCorrected;
//		if (ownPos == nullptr)
//		{
//			return nullptr;
//		}
//		shared_ptr<geometry::CNPoint2D> alloBall = ballPos->egoToAllo(*ownPos);
//		shared_ptr<geometry::CNPoint2D> dest = make_shared<geometry::CNPoint2D>();
//
//		if (!field->isInsideField(alloBall, 500))
//		{ //ball is out, approach it carefully
//			//Console.WriteLine("CASE B");
//			dest.X = ownPos.X - alloBall.X;
//			dest.Y = ownPos.Y - alloBall.Y;
//			dest = field.MapInsideField(alloBall);
//			dest = WorldHelper.Allo2Ego(dest, ownPos);
//			return PlaceRobotCareBall(dest, ballPos, MAX_VELO, WM);
//		}
//		if (field.InsideOwnPenalty(alloBall, 0))
//		{ //handle ball in own penalty
//			if (!field.InsideOwnKeeperArea(alloBall, 200) && field.InsideOwnPenalty(ownPos.Point, 0))
//			{ //if we are already in, and ball is in safe distance of keeper area, get it
//				return null;
//			}
//			if (WorldHelper.TeamMatesInOwnPenalty(WM) > 1)
//			{ //do not enter penalty if someone besides keeper is already in there
//				//dest.X = ownPos.X - alloBall.X;
//				//dest.Y = ownPos.Y - alloBall.Y;
//				dest = field.MapOutOfOwnPenalty(alloBall);
//				dest = WorldHelper.Allo2Ego(dest, ownPos);
//				return PlaceRobotCareBall(dest, ballPos, MAX_VELO, WM);
//			}
//			if (field.InsideOwnKeeperArea(alloBall, 200))
//			{ //ball is dangerously close to keeper area, or even within
//				if (!field.InsideOwnKeeperArea(alloBall, 50))
//				{
//					if ((ownPos.X - alloBall.X) < 150)
//					{
//						return nullptr;
//					}
//				}
//				dest->x = alloBall->x - 200;
//				if (ownPos.Y < alloBall.Y)
//				{
//					dest.Y = alloBall.Y - 500;
//				}
//				else
//				{
//					dest.Y = alloBall.Y + 500;
//				}
//				dest = field.MapOutOfOwnKeeperArea(dest); //drive to the closest side of the ball and hope to get it somehow
//				dest = WorldHelper.Allo2Ego(dest, ownPos);
//				return PlaceRobotCareBall(dest, ballPos, MAX_VELO, WM);
//			}
//
//		}
//		if (field.InsideEnemyPenalty(alloBall, 0))
//		{ //ball is inside enemy penalty area
//			if (WorldHelper.TeamMatesInEnemyPenalty(WM) > 0)
//			{ //if there is someone else, do not enter
//				//dest.X = ownPos.X - alloBall.X;
//				//dest.Y = ownPos.Y - alloBall.Y;
//				dest = field.MapOutOfEnemyPenalty(alloBall);
//				dest = WorldHelper.Allo2Ego(dest, ownPos);
//				return DriveHelper.PlaceRobot(dest, ballPos, MAX_VELO, WM);
//			}
//			if (field.InsideEnemyKeeperArea(alloBall, 50))
//			{ //ball is inside keeper area
//				dest = field.MapOutOfEnemyKeeperArea(alloBall); //just drive as close to the ball as you can
//				dest = WorldHelper.Allo2Ego(dest, ownPos);
//				return DriveHelper.PlaceRobot(dest, ballPos, MAX_VELO, WM);
//			}
//
//		}

		return nullptr;
	}

	void RobotMovement::readConfigParameters()
	{
		supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();
		defaultTranslation = (*sc)["Drive"]->get<double>("Drive.Default.Velocity", NULL);
		defaultRotateP = (*sc)["Drive"]->get<double>("Drive.Default.RotateP", NULL);
		fastTranslation = (*sc)["Drive"]->get<double>("Drive.Fast.Velocity", NULL);
		fastRotation = (*sc)["Drive"]->get<double>("Drive.Fast.RotateP", NULL);
		interceptCarfullyRotateP = (*sc)["Drive"]->get<double>("Drive.Carefully.RotateP", NULL);
		alignToPointMaxRotation = (*sc)["Drive"]->get<double>("Drive", "AlignToPointMaxRotation", NULL);
		alignToPointMinRotation = (*sc)["Drive"]->get<double>("Drive", "AlignToPointMinRotation", NULL);
		alignToPointpRot = (*sc)["Drive"]->get<double>("Drive", "AlignToPointpRot", NULL);
		alignMaxVel = (*sc)["Drive"]->get<double>("Drive", "MaxSpeed", NULL);
	}
}

