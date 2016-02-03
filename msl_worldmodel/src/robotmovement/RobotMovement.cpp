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
	double RobotMovement::maxVelo;

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
		if(egoAlignPoint == nullptr)
		{
			egoAlignPoint = egoTarget;
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

	msl_actuator_msgs::MotionControl RobotMovement::ruleActionForBallGetter()
	{
		MSLWorldModel* wm = MSLWorldModel::get();
		MSLFootballField* field = MSLFootballField::getInstance();
		shared_ptr<geometry::CNPoint2D> ballPos = wm->ball.getEgoBallPosition();
		if (ballPos == nullptr)
		{
			MotionControl mc;
			mc.senderID = -1;
			return mc;
		}
		shared_ptr<geometry::CNPosition> ownPos = wm->rawSensorData.getOwnPositionVision(); //OwnPositionCorrected;
		if (ownPos == nullptr)
		{
			MotionControl mc;
			mc.senderID = -1;
			return mc;
		}
		shared_ptr<geometry::CNPoint2D> alloBall = ballPos->egoToAllo(*ownPos);
		shared_ptr<geometry::CNPoint2D> dest = make_shared<geometry::CNPoint2D>();

		if (!field->isInsideField(alloBall, 500))
		{ //ball is out, approach it carefully
		  //Console.WriteLine("CASE B");
			dest->x = ownPos->x - alloBall->x;
			dest->y = ownPos->y - alloBall->y;
			dest = field->mapInsideField(alloBall);
			dest = dest->alloToEgo(*ownPos);
			return placeRobotCareBall(dest, ballPos, maxVelo);
		}
		if (field->isInsideOwnPenalty(alloBall, 0))
		{ //handle ball in own penalty
			if (!field->isInsideOwnKeeperArea(alloBall, 200) && field->isInsideOwnPenalty(ownPos->getPoint(), 0))
			{ //if we are already in, and ball is in safe distance of keeper area, get it
				MotionControl mc;
				mc.senderID = -1;
				return mc;
			}
			if (wm->robots.teamMatesInOwnPenalty() > 1)
			{ //do not enter penalty if someone besides keeper is already in there
			  //dest.X = ownPos.X - alloBall.X;
			  //dest.Y = ownPos.Y - alloBall.Y;
				dest = field->mapOutOfOwnPenalty(alloBall);
				dest = dest->alloToEgo(*ownPos);
				return placeRobotCareBall(dest, ballPos, maxVelo);
			}
			if (field->isInsideOwnKeeperArea(alloBall, 200))
			{ //ball is dangerously close to keeper area, or even within
				if (!field->isInsideOwnKeeperArea(alloBall, 50))
				{
					if ((ownPos->x - alloBall->x) < 150)
					{
						MotionControl mc;
						mc.senderID = -1;
						return mc;
					}
				}
				dest->x = alloBall->x - 200;
				if (ownPos->y < alloBall->y)
				{
					dest->y = alloBall->y - 500;
				}
				else
				{
					dest->y = alloBall->y + 500;
				}
				dest = field->mapOutOfOwnKeeperArea(dest); //drive to the closest side of the ball and hope to get it somehow
				dest = dest->alloToEgo(*ownPos);
				return placeRobotCareBall(dest, ballPos, maxVelo);
			}

		}
		if (field->isInsideEnemyPenalty(alloBall, 0))
		{ //ball is inside enemy penalty area
			if (wm->robots.teamMatesInOppPenalty() > 0)
			{ //if there is someone else, do not enter
			  //dest.X = ownPos.X - alloBall.X;
			  //dest.Y = ownPos.Y - alloBall.Y;
				dest = field->mapOutOfEnemyPenalty(alloBall);
				dest = dest->alloToEgo(*ownPos);
				return placeRobot(dest, ballPos, maxVelo);
			}
			if (field->isInsideEnemyKeeperArea(alloBall, 50))
			{ //ball is inside keeper area
				dest = field->mapOutOfEnemyKeeperArea(alloBall); //just drive as close to the ball as you can
				dest = dest->alloToEgo(*ownPos);
				return placeRobot(dest, ballPos, maxVelo);
			}

		}
		MotionControl mc;
		mc.senderID = -1;
		return mc;
	}

	MotionControl RobotMovement::placeRobotCareBall(shared_ptr<geometry::CNPoint2D> destinationPoint,
													shared_ptr<geometry::CNPoint2D> headingPoint, double translation)
	{
		double rotTol = M_PI / 30.0;
		double destTol = 100.0;
		MSLWorldModel* wm = MSLWorldModel::get();
		if (destinationPoint->length() < destTol)
		{
			MotionControl rot;
			if (wm->ball.getAlloBallPosition() != nullptr)
			{
				shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> additionalPoints = make_shared<
				vector<shared_ptr<geometry::CNPoint2D>>>();
				additionalPoints->push_back(wm->ball.getAlloBallPosition());
				//DriveToPointAndAlignCareObstacles
				rot = moveToPointCarefully(destinationPoint, headingPoint, 0 , additionalPoints);
			}
			else
			{
				rot = moveToPointCarefully(destinationPoint, headingPoint, 0 , nullptr);
			}
			rot.motion.translation = 0.0;
			if (headingPoint == nullptr)
			{
				return rot;
			}
			double angle = headingPoint->angleTo();
			if (abs(angle - wm->kicker.kickerAngle) > rotTol)
			{
				return rot;
			}
			else
			{
				MotionControl bm;

				bm.motion.rotation = 0.0;
				bm.motion.translation = 0.0;
				bm.motion.angle = 0.0;
				return bm;
			}
		}
		else
		{
			//linear
			double trans = min(translation, 1.2 * destinationPoint->length());

			MotionControl bm;
			//DriveHelper.DriveToPointAndAlignCareBall(destinationPoint, headingPoint, trans, wm);
			if (wm->ball.getAlloBallPosition() != nullptr)
			{
				shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> additionalPoints = make_shared<
				vector<shared_ptr<geometry::CNPoint2D>>>();
				additionalPoints->push_back(wm->ball.getAlloBallPosition());
				//DriveToPointAndAlignCareObstacles
				bm = moveToPointCarefully(destinationPoint, headingPoint, 0 , additionalPoints);
			}
			else
			{
				bm = moveToPointCarefully(destinationPoint, headingPoint, 0 , nullptr);
			}

			return bm;
		}
	}

	MotionControl RobotMovement::placeRobot(shared_ptr<geometry::CNPoint2D> destinationPoint, shared_ptr<geometry::CNPoint2D> headingPoint, double translation)
	{
		double rotTol = M_PI / 30.0;
		double destTol = 100.0;

		if (destinationPoint->length() < destTol)
		{
			// DriveToPointAndAlignCareObstacles(destinationPoint, headingPoint, translation, wm);
			MotionControl rot = moveToPointCarefully(destinationPoint, headingPoint, 0 , nullptr);
			rot.motion.translation = 0.0;
			if (headingPoint == nullptr)
			{
				return rot;
			}
			double angle = headingPoint->angleTo();
			MSLWorldModel* wm = MSLWorldModel::get();
			if (abs(angle - wm->kicker.kickerAngle) > rotTol)
			{
				return rot;
			}
			else
			{
				MotionControl bm;

				bm.motion.rotation = 0.0;
				bm.motion.translation = 0.0;
				bm.motion.angle = 0.0;
				return bm;
			}
		}
		else
		{

			//linear
			double trans = min(translation, 1.2 * destinationPoint->length());

			MotionControl bm = moveToPointCarefully(destinationPoint, headingPoint, 0 , nullptr);

			return bm;
		}
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
		maxVelo = (*sc)["Behaviour"]->get<double>("Behaviour", "MaxSpeed", NULL);
	}
}

