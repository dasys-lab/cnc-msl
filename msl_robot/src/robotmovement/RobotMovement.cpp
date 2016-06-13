/*
 * RobotMovement.cpp *
 *
 *  Created on: 17.12.2014
 *      Author: tobi
 */

#include "msl_robot/robotmovement/RobotMovement.h"
#include "MSLFootballField.h"
#include "container/CNPoint2D.h"
#include "GeometryCalculator.h"
#include "MSLWorldModel.h"
#include "pathplanner/PathProxy.h"
#include "pathplanner/evaluator/PathEvaluator.h"
#include "msl_robot/robotmovement/SearchArea.h"
#include "msl_robot/robotmovement/AlloSearchArea.h"
#include "msl_robot/robotmovement/MovementQuery.h"
#include "Ball.h"
#include "RawSensorData.h"
#include "Robots.h"
#include "obstaclehandler/Obstacles.h"
#include "Kicker.h"

#include <SystemConfig.h>

// remove commentary for debug output
//#define RM_DEBUG

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
	int RobotMovement::randomCounter = 0;
	int RobotMovement::beamSize = 3;
	shared_ptr<geometry::CNPoint2D> RobotMovement::randomTarget = nullptr;
	shared_ptr<vector<shared_ptr<SearchArea>>> RobotMovement::fringe = make_shared<vector<shared_ptr<SearchArea>>>();
	shared_ptr<vector<shared_ptr<SearchArea>>> RobotMovement::next = make_shared<vector<shared_ptr<SearchArea>>>();
	double RobotMovement::assume_enemy_velo = 4500;
	double RobotMovement::assume_ball_velo = 5000;
	double RobotMovement::interceptQuotient = RobotMovement::assume_ball_velo / RobotMovement::assume_enemy_velo;
	double RobotMovement::robotRadius = 300;

	double RobotMovement::lastRotErr = 0;
	double RobotMovement::curRot = 0;
	double RobotMovement::curTrans = 0;
	double RobotMovement::transControlIntegral = 0;

	double RobotMovement::transAccStep = (*supplementary::SystemConfig::getInstance())["Dribble"]->get<double>(
			"DribbleWater", "MaxAcceleration", NULL);
	double RobotMovement::transDecStep = (*supplementary::SystemConfig::getInstance())["Dribble"]->get<double>(
			"DribbleWater", "MaxDecceleration", NULL);
	double RobotMovement::iTrans = (*supplementary::SystemConfig::getInstance())["Dribble"]->get<double>("DribbleWater",
																											"iTrans",
																											NULL) / M_PI;
	double RobotMovement::pTrans = (*supplementary::SystemConfig::getInstance())["Dribble"]->get<double>("DribbleWater",
																											"pTrans",
																											NULL) / M_PI;
	double RobotMovement::transControlIntegralMax =
			(*supplementary::SystemConfig::getInstance())["Dribble"]->get<double>("DribbleWater", "maxTransIntegral",
			NULL);
	double RobotMovement::pRot = (*supplementary::SystemConfig::getInstance())["Dribble"]->get<double>("DribbleWater",
																										"pRot", NULL);
	double RobotMovement::dRot = (*supplementary::SystemConfig::getInstance())["Dribble"]->get<double>("DribbleWater",
																										"dRot", NULL);
	double RobotMovement::rotAccStep = (*supplementary::SystemConfig::getInstance())["Dribble"]->get<double>(
			"DribbleWater", "MaxRotationAcceleration", NULL);
	double RobotMovement::maxRot = (*supplementary::SystemConfig::getInstance())["Dribble"]->get<double>(
			"DribbleWater", "MaxRotation", NULL);
	double RobotMovement::angleDeadBand = (*supplementary::SystemConfig::getInstance())["Dribble"]->get<double>(
			"DribbleWater", "angleDeadBand", NULL) / 180 * M_PI;
	double RobotMovement::maxVel = (*supplementary::SystemConfig::getInstance())["Dribble"]->get<double>(
			"DribbleWater", "MaxVelocity", NULL);

	RobotMovement::RobotMovement()
	{
		this->wm = MSLWorldModel::get();
		this->pp = PathProxy::getInstance();
	}

	RobotMovement::~RobotMovement()
	{
	}

	// new RobotMovement ===========================================================================================

	/**
	 * Uses MovementQuery with Parameters
	 *
	 * necessary Parameters:
	 * @param egoDestinationPoint
	 *
	 * additional Parameters for adaption:
	 * @param egoAlignPoint
	 * @param snapDistance
	 * @param additionalPoints
	 * @param fast
	 *
	 * @param dribble
	 * if dribble == true you can adapt the rotation and translation PD parameters for
	 * query->curTransDribble
	 * query->curRotDribble
	 *
	 */
	msl_actuator_msgs::MotionControl RobotMovement::moveToPoint(shared_ptr<MovementQuery> const query)
	{
		msl_actuator_msgs::MotionControl mc;

		if (query == nullptr || query->egoDestinationPoint == nullptr)
		{
			return setNAN();
		}
		shared_ptr<PathEvaluator> eval = make_shared<PathEvaluator>();
		shared_ptr<geometry::CNPoint2D> egoTarget = this->pp->getEgoDirection(query->egoDestinationPoint, eval,
																				query->additionalPoints);

		// ANGLE
		mc.motion.angle = egoTarget->angleTo();

		// TRANSLATION
		if (egoTarget->length() > query->snapDistance)
		{
			mc.motion.translation = min(egoTarget->length(), (query->fast ? this->fastTranslation : this->defaultTranslation));
		}
		else
		{
			mc.motion.translation = 0;
		}

		// ROTATION
		if (query->egoAlignPoint != nullptr)
		{
			mc.motion.rotation = query->egoAlignPoint->rotate(M_PI)->angleTo() * (query->fast ? this->fastRotation : this->defaultRotateP);
		}

		// dribble behavior -> used from dribbleToPointConservative ==============================================
		if (query->dribble)
		{ //todo
			mc.motion.rotation = query->rotationPDForDribble(egoTarget);
			double rotPointDist = 350.0;
			if (auto ballPos = wm->ball->getEgoBallPosition())
			{
				rotPointDist = min(350.0, ballPos->length()); //the point around which we rotate
			}

			double transOrt = mc.motion.rotation * rotPointDist; //the translation corresponding to the curve we drive

			mc.motion.translation = query->translationPDForDribble(transOrt);
			mc.motion.angle = query->angleCalcForDribble(transOrt);
		}

#ifdef RM_DEBUG
		cout << "RobotMovement::moveToPoint: Angle = " << mc.motion.angle << " Trans = " << mc.motion.translation << " Rot = " << mc.motion.rotation << endl;
#endif
		return mc;
	}

	/*
	 * Checks if Robot respects the rules.
	 * If everything is fine, translation, rotation and angle are set to NaN.
	 * Don't use the return value without checking.
	 *
	 * @return msl_actuator_msgs::MotionControl
	 */
	msl_actuator_msgs::MotionControl RobotMovement::ruleActionForBallGetter()
	{
		// TODO introduce destination method-parameter for improving this method...
		// TODO add config parameters for all static numbers in here!
		shared_ptr<geometry::CNPoint2D> egoBallPos = wm->ball->getEgoBallPosition();
		shared_ptr<geometry::CNPosition> ownPos = wm->rawSensorData->getOwnPositionVision(); //OwnPositionCorrected;
		if (egoBallPos == nullptr || ownPos == nullptr)
		{
			return setNAN();
		}
		shared_ptr<geometry::CNPoint2D> alloBall = egoBallPos->egoToAllo(*ownPos);
		shared_ptr<geometry::CNPoint2D> dest = make_shared<geometry::CNPoint2D>();

		//ball is out, approach it carefully ================================================================
		if (!wm->field->isInsideField(alloBall, 500))
		{
			dest = ownPos - alloBall;
			dest = wm->field->mapInsideField(alloBall);
			dest = dest->alloToEgo(*ownPos);
			return placeRobot(dest, egoBallPos);
		}
		//handle ball in own penalty ========================================================================
		if (wm->field->isInsideOwnPenalty(alloBall, 0))
		{
			if (!wm->field->isInsideOwnGoalArea(alloBall, 200)
					&& wm->field->isInsideOwnPenalty(ownPos->getPoint(), 0))
			{
				//if we are already in, and ball is in safe distance of keeper area, get it
				return setNAN();
			}
			if (wm->robots->teammates.teamMatesInOwnPenalty() > 1)
			{
				//do not enter penalty if someone besides keeper is already in there
				dest = wm->field->mapOutOfOwnPenalty(alloBall);
				dest = dest->alloToEgo(*ownPos);
				return placeRobot(dest, egoBallPos);
			}
			if (wm->field->isInsideOwnGoalArea(alloBall, 200))
			{
				//ball is dangerously close to keeper area, or even within
				if (!wm->field->isInsideOwnGoalArea(alloBall, 50))
				{
					if ((ownPos->x - alloBall->x) < 150)
					{
						return setNAN();
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
				dest = wm->field->mapOutOfOwnGoalArea(dest); //drive to the closest side of the ball and hope to get it somehow
				dest = dest->alloToEgo(*ownPos);
				return placeRobot(dest, egoBallPos);
			}

		}
		//ball is inside enemy penalty area ===============================================================
		if (wm->field->isInsideOppPenalty(alloBall, 0))
		{
			if (wm->robots->teammates.teamMatesInOppPenalty() > 0)
			{
				//if there is someone else, do not enter
				dest = wm->field->mapOutOfOppPenalty(alloBall);
				dest = dest->alloToEgo(*ownPos);
				return placeRobot(dest, egoBallPos);
			}
			if (wm->field->isInsideOppGoalArea(alloBall, 50))
			{
				//ball is inside keeper area
				dest = wm->field->mapOutOfOppGoalArea(alloBall); //just drive as close to the ball as you can
				dest = dest->alloToEgo(*ownPos);
				return placeRobot(dest, egoBallPos);
			}

		}
#ifdef RM_DEBUG
		cout << "RobotMovement::ruleActionForBallGetter: Angle = " << mc.motion.angle << " Trans = " << mc.motion.translation << " Rot = " << mc.motion.rotation << endl;
#endif
		return setNAN();
	}

	/*
	 * Used in ruleActionForBallGetter()
	 *
	 * @return motion command by using moveToPoint() depending on
	 * 		the distance to the destination and
	 * 		the ball as additional point
	 */
	msl_actuator_msgs::MotionControl RobotMovement::placeRobot(shared_ptr<geometry::CNPoint2D> dest,
															shared_ptr<geometry::CNPoint2D> headingPoint)
	{
		msl_actuator_msgs::MotionControl mc;
		double destTol = 100.0;
		auto ballPos = wm->ball->getEgoBallPosition();
		if (dest->length() < destTol)
		{
			// only align to point
			std::shared_ptr<MovementQuery> query = make_shared<MovementQuery>();
			query->egoDestinationPoint = dest;
			query->egoAlignPoint = ballPos;
			query->additionalPoints = nullptr;

			mc = moveToPoint(query);
			mc.motion.translation = 0;

			return mc;
		}
		else
		{
			if (wm->ball->getAlloBallPosition() != nullptr)
			{
				shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> additionalPoints = make_shared<
				vector<shared_ptr<geometry::CNPoint2D>>>();
				additionalPoints->push_back(wm->ball->getAlloBallPosition());

				std::shared_ptr<MovementQuery> query = make_shared<MovementQuery>();
				query->egoDestinationPoint = dest;
				query->egoAlignPoint = ballPos;
				query->additionalPoints = additionalPoints;

				mc = moveToPoint(query);
			}
			else
			{
				std::shared_ptr<MovementQuery> query = make_shared<MovementQuery>();
				query->egoDestinationPoint = dest;
				query->additionalPoints = nullptr;
				query->egoAlignPoint = headingPoint;

				if (headingPoint == nullptr)
				{
					query->egoAlignPoint = dest;
				}

				mc = moveToPoint(query);
			}
#ifdef RM_DEBUG
		cout << "RobotMovement::placeRobot: Angle = " << mc.motion.angle << " Trans = " << mc.motion.translation << " Rot = " << mc.motion.rotation << endl;
#endif
			return mc;
		}
	}

	/*
	 * @return motion command to a random destination point
	 */
	msl_actuator_msgs::MotionControl RobotMovement::driveRandomly(double translation)
	{
		msl::PathProxy* pp = msl::PathProxy::getInstance();
		shared_ptr<msl::PathEvaluator> eval = make_shared<msl::PathEvaluator>();
		if (randomCounter == 0)
		{
			std::random_device rd;
			std::mt19937 gen(rd());
			std::uniform_real_distribution<> dis(0, 1);
			double ang = (dis(gen) - 0.5) * 2 * M_PI;
			randomTarget = make_shared<geometry::CNPoint2D>(cos(ang) * 5000, sin(ang) * 5000);
		}

		auto dest = pp->getEgoDirection(randomTarget, eval);

		if (dest == nullptr)
		{
			dest = randomTarget;
			translation = 100;
		}
		msl_actuator_msgs::MotionControl bm;
		bm.motion.rotation = 0;
		bm.motion.translation = translation;
		bm.motion.angle = atan2(dest->y, dest->x);
		randomCounter = (randomCounter + 1) % 28;
#ifdef RM_DEBUG
		cout << "RobotMovement::driveRandomly: Angle = " << mc.motion.angle << " Trans = " << mc.motion.translation << " Rot = " << mc.motion.rotation << endl;
#endif
		return bm;
	}

	/**
	 * Uses MovementQuery with Parameters:
	 *
	 * necessary parameters:
	 * @teamMatePosition
	 *
	 */
	msl_actuator_msgs::MotionControl RobotMovement::moveToFreeSpace(shared_ptr<MovementQuery> query)
	{
		auto teamMatePosition = query->alloTeamMatePosition;
		msl_actuator_msgs::MotionControl mc;

		shared_ptr<geometry::CNPosition> ownPos = wm->rawSensorData->getOwnPositionVision();
		if (ownPos == nullptr)
		{
			return setNAN();
		}

		shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> ops = wm->obstacles->getEgoVisionObstaclePoints(); //WM.GetTrackedOpponents();
		fringe->clear();
		for (int i = 0; i < 16; i++)
		{
			for (int d = 0; d < 8000; d += 2000)
			{
				shared_ptr<AlloSearchArea> s = AlloSearchArea::getAnArea(i * M_PI / 8, (i + 1) * M_PI / 8, d, d + 2000,
																			ownPos->getPoint(), ownPos);
				if (s->isValid())
				{

					s->val = evalPointDynamic(s->midP, teamMatePosition, ownPos,
												wm->obstacles->getEgoVisionObstaclePoints());
					fringe->push_back(s);
				}
			}
		}
		stable_sort(fringe->begin(), fringe->end(), SearchArea::compareTo);
		shared_ptr<SearchArea> best = fringe->at(0);
		shared_ptr<SearchArea> cur;

		for (int i = 0; i < 100 && fringe->size() > 0; i++)
		{

			next->clear();
			for (int j = 0; j < beamSize; j++)
			{
				if (fringe->size() == 0)
				{
					break;
				}
				cur = fringe->at(0);
				fringe->erase(fringe->begin());
				if (j == 0 && cur->val > best->val)
				{
					best = cur;
				}
				shared_ptr<vector<shared_ptr<SearchArea>>> expanded = cur->expand();
				for (int i = 0; expanded->size(); i++)
				{
					next->push_back(expanded->at(i));
				}
			}
			for (int j = 0; j < next->size(); j++)
			{
				next->at(j)->val = evalPointDynamic(next->at(j)->midP, teamMatePosition, ownPos,
													wm->obstacles->getEgoVisionObstaclePoints());
				fringe->push_back(next->at(j));
			}
			stable_sort(fringe->begin(), fringe->end(), SearchArea::compareTo);
		}
		shared_ptr<geometry::CNPoint2D> dest =
				wm->field->mapOutOfOppGoalArea(wm->field->mapInsideField(best->midP))->alloToEgo(*ownPos);
		shared_ptr<geometry::CNPoint2D> align = teamMatePosition->alloToEgo(*ownPos);

//		mc = placeRobotAggressive(dest, align, maxTrans);

		shared_ptr<MovementQuery> q = make_shared<MovementQuery>();
		q->egoDestinationPoint = dest;
		q->egoAlignPoint = align;
		q->fast = true;
		// todo: test if experimatallyMoveToPoint() does nearly the same as placeRobotAggressive()
		mc = moveToPoint(q);
#ifdef RM_DEBUG
		cout << "RobotMovementmoveToFreeSpace: Angle = " << mc.motion.angle << " Trans = " << mc.motion.translation << " Rot = " << mc.motion.rotation << endl;
#endif
		return mc;
	}

	/*
	 * Used in moveToFreeSpace()
	 */
	double RobotMovement::evalPointDynamic(
			shared_ptr<geometry::CNPoint2D> alloP, shared_ptr<geometry::CNPoint2D> alloPassee,
			shared_ptr<geometry::CNPosition> ownPos, shared_ptr<vector<shared_ptr<geometry::CNPoint2D> > > opponents)
	{
		double ret = 0;

		//distance to point:
		ret -= ownPos->distanceTo(alloP) / 10.0;
		MSLWorldModel* wm = MSLWorldModel::get();
		shared_ptr<geometry::CNPoint2D> oppGoalMid = make_shared<geometry::CNPoint2D>(wm->field->getFieldLength() / 2,
																						0);

		shared_ptr<geometry::CNPoint2D> passee2p = alloP - alloPassee;
		//if (passee2p.X < 0 && Math.Abs(alloP.Y) < 1000) return Double.MinValue;

		shared_ptr<geometry::CNPoint2D> passee2LeftOwnGoal = make_shared<geometry::CNPoint2D>(
				-wm->field->getFieldLength() / 2.0 - alloPassee->x,
				wm->field->getGoalWidth() / 2.0 + 1000 - alloPassee->y);
		shared_ptr<geometry::CNPoint2D> passee2RightOwnGoal = make_shared<geometry::CNPoint2D>(
				-wm->field->getFieldLength() / 2.0 - alloPassee->x,
				-wm->field->getGoalWidth() / 2.0 - 1000 - alloPassee->y);

		if (!geometry::leftOf(passee2LeftOwnGoal, passee2p) && geometry::leftOf(passee2RightOwnGoal, passee2p))
		{
			return numeric_limits<double>::min();
		}
		if (wm->field->isInsideOppPenalty(alloPassee, 800) && wm->field->isInsideOppPenalty(alloP, 600))
		{
			return numeric_limits<double>::min();
		}

		ret += passee2p->x / 9.0;
		if (alloP->x < 0)
		{
			ret += alloP->x / 10.0;
		}

		if (alloP->y * alloPassee->y < 0)
		{
			ret += min(0.0, alloP->x);
		}
		if (alloP->angleToPoint(oppGoalMid) > M_PI / 3.0)
		{
			ret -= alloP->angleToPoint(oppGoalMid) * 250.0;
		}
		//distance to passeee:

		//Point2D p2GoalVec = goalPos - p;
		double dist2Passee = passee2p->length();

		//nice passing distances: 4000..9000:
		if (dist2Passee < 2000)
		{
			return numeric_limits<double>::min();
		}
		if (dist2Passee < 4000)
		{
			ret -= 4000 - dist2Passee;
		}
		else if (dist2Passee > 9000)
		{
			ret -= dist2Passee - 9000;
		}

		//else if (dist2Ball > 5000) ret -= (dist2Ball-5000)*(dist2Ball - 5000);

		double t, v;
		double ortX, ortY;
		double catchFactor = 0;

		//double goalFactor = 0;

		for (int i = 0; i < opponents->size(); i++)
		{
			//pass corridor
			t = ((opponents->at(i)->x - alloPassee->x) * passee2p->x
					+ (opponents->at(i)->y - alloPassee->y) * passee2p->y)
					/ (passee2p->x * passee2p->x + passee2p->y * passee2p->y);

			if (t > 0)
			{
				if (t < 1.0)
				{

					ortX = opponents->at(i)->x - (alloPassee->x + t * (passee2p->x));
					ortY = opponents->at(i)->y - (alloPassee->y + t * (passee2p->y));
					v = max(0.0, sqrt(ortX * ortX + ortY * ortY) - robotRadius) / dist2Passee;

					if (v / t * interceptQuotient < 1)
					{

						double cf = 5000 * (1 - ((v / t) * interceptQuotient));
						catchFactor = max(catchFactor, cf);
					}
				}
				ortX = opponents->at(i)->x - alloP->x;
				ortY = opponents->at(i)->y - alloP->y;
				v = sqrt(ortX * ortX + ortY * ortY);
				if (v < 2500)
					ret -= (2500 - v) * 10;
			}
		}
		ret -= catchFactor;

		//ret -= goalFactor;
#ifdef RM_DEBUG
		cout << "RobotMovement::evalPointDynamic: Angle = " << mc.motion.angle << " Trans = " << mc.motion.translation << " Rot = " << mc.motion.rotation << endl;
#endif
		return ret;
	}

	msl_actuator_msgs::MotionControl RobotMovement::setNAN()
	{
		msl_actuator_msgs::MotionControl mc;
		mc.motion.rotation = NAN;
		mc.motion.translation = NAN;
		mc.motion.angle = NAN;
		mc.senderID = -1;
		return mc;
	}

// old RobotMovement <==========================================================================================
/*
	msl_actuator_msgs::MotionControl RobotMovement::nearGoalArea(msl_actuator_msgs::MotionControl bm)
	{
		msl::MSLWorldModel* wm = msl::MSLWorldModel::get();
		auto ownPos = wm->rawSensorData->getOwnPositionVision();
		if (ownPos == nullptr)
		{
			return bm;
		}

		shared_ptr<geometry::CNPoint2D> dir = make_shared<geometry::CNPoint2D>(
				bm.motion.translation * cos(bm.motion.angle), bm.motion.translation * sin(bm.motion.angle));

//		cout << "RobotMovement: ego dir  " << dir->x << " " << dir->y << endl;
		dir = dir->egoToAllo(*ownPos);
		if (wm->field->isInsideOppGoalArea(dir, 150)
				|| wm->field->isInsideOppGoalArea(ownPos->getPoint(), 1200))
		{
			dir = dir - ownPos;
//			cout << "RobotMovement: allo dir  " << dir->x << " " << dir->y << endl;
			dir->x = min(dir->x, 0.0);
//			cout << "RobotMovement: allo cut " << dir->x << " " << dir->y << endl;
			dir = (dir + ownPos)->alloToEgo(*ownPos);

//			cout << "RobotMovement: ego final " << dir->x << " " << dir->y << endl;

			bm.motion.angle = dir->angleTo();
			bm.motion.translation = dir->length();
//			cout << "RobotMovement: insideOppArea \t" << bm.motion.angle << "\t" << bm.motion.translation << "\t" << bm.motion.rotation << endl;
		}
		return bm;
	}
	*/
/*
	msl_actuator_msgs::MotionControl RobotMovement::driveToPointAlignNoAvoidance(
			shared_ptr<geometry::CNPoint2D> destination, shared_ptr<geometry::CNPoint2D> alignPoint, double translation,
			bool alignSlow)
	{
		msl_actuator_msgs::MotionControl bm = driveToPointNoAvoidance(destination, translation);

		//if we dont need to align
		if (alignPoint == nullptr)
		{
//				Console.WriteLine("MC: angle" + bm.Motion.Angle + " trans" + bm.Motion.Translation);
			return bm;
		}
		else
		{
			// Align (with compensation if necessary)
			msl_actuator_msgs::MotionControl tmp = alignToPointNoBall(alignPoint, alignPoint, 0);
			bm.motion.rotation = tmp.motion.rotation;
			return bm;

		}
	}
*/
	/*
	msl_actuator_msgs::MotionControl RobotMovement::driveToPointNoAvoidance(shared_ptr<geometry::CNPoint2D> egoDest,
																			double translation)
	{
		//motion message
		msl_actuator_msgs::MotionControl bm;
		// Angle
		bm.motion.angle = egoDest->angleTo();
		// Translation
		bm.motion.translation = translation;

		return bm;
	}
	*/
/*
	msl_actuator_msgs::MotionControl RobotMovement::driveRandomly(int translation)
	{
		msl::PathProxy* pp = msl::PathProxy::getInstance();
		shared_ptr<msl::PathEvaluator> eval = make_shared<msl::PathEvaluator>();
		if (randomCounter == 0)
		{
			std::random_device rd;
			std::mt19937 gen(rd());
			std::uniform_real_distribution<> dis(0, 1);
			double ang = (dis(gen) - 0.5) * 2 * M_PI;
			randomTarget = make_shared<geometry::CNPoint2D>(cos(ang) * 5000, sin(ang) * 5000);
		}

		auto dest = pp->getEgoDirection(randomTarget, eval);

		if (dest == nullptr)
		{
			dest = randomTarget;
			translation = 100;
		}
		msl_actuator_msgs::MotionControl bm;
		bm.motion.rotation = 0;
		bm.motion.translation = translation;
		bm.motion.angle = atan2(dest->y, dest->x);
		randomCounter = (randomCounter + 1) % 28;
		return bm;
	}
*/
	void RobotMovement::reset()
	{
		iTrans = 0;
		lastRotErr = 0;
		msl::MSLWorldModel* wm = msl::MSLWorldModel::get();
		if (wm->rawSensorData->getLastMotionCommand() != nullptr)
		{
			curRot = wm->rawSensorData->getLastMotionCommand()->motion.rotation;
			curTrans = wm->rawSensorData->getLastMotionCommand()->motion.translation;

		}
		else
		{
			curRot = 0;
			curTrans = 0;
		}
	}
/*// todo:
	shared_ptr<msl_actuator_msgs::MotionControl> RobotMovement::dribbleToPointConservative(
			shared_ptr<geometry::CNPoint2D> egoTarget, shared_ptr<geometry::CNPoint2D>& ppp)
	{
		msl::MSLWorldModel* wm = msl::MSLWorldModel::get();

		auto ballPos = wm->ball->getEgoBallPosition();

		if (ballPos == nullptr)
			return nullptr;
		if (ballPos->length() > 1000)
			return nullptr;
//		cout << "RobotMovement: dribble ego target: " << egoTarget->toString() << endl;
//		cout << "RobotMovement: dribble allo target: "
//				<< egoTarget->egoToAllo(*wm->rawSensorData.getOwnPositionVision())->toString() << endl;
		double pathPlanningMaxTrans = maxVel;
		double frontAngle = wm->kicker->kickerAngle;

//		cout << "RobotMovement: pathPlanningMaxTrans " << pathPlanningMaxTrans << endl;
		shared_ptr<msl::PathEvaluator> eval = make_shared<msl::PathEvaluator>();
		msl::PathProxy* pp = msl::PathProxy::getInstance();

		auto target = pp->getEgoDirection(egoTarget, eval);

		ppp = target;
		if (target == nullptr)
			return nullptr;

		double angleErr = target->rotate(frontAngle)->angleTo();
//		double angleErr = geometry::deltaAngle(frontAngle, target->angleTo()); //the current error

//		cout << "RobotMovement: angleErr " << angleErr << endl;
//		if(Math.Abs(angleErr)>0.8) {
//		 double minAngle=300;
//		 double ang=0;
//		 bool found = false;
//		 Position ownPos = wm.OwnPositionCorrected;
//		 List<TrackedOpponent> topl = wm.GetTrackedOpponents();
//		 if(ownPos != null && topl!=null) {
//		 foreach(TrackedOpponent t in  topl) {
//		 Point2D op = WorldHelper.Allo2Ego(t.Pos, ownPos);
//		 if(op.Distance() < 1500) {
//		 ang = op.Angle()-frontAngle;
//		 if(ang>Math.PI) ang -= 2*Math.PI;
//		 if(ang<-Math.PI) ang += 2*Math.PI;
//		 found = true;
//		 if(Math.Abs(minAngle) > Math.Abs(ang)) minAngle = ang;
//		 }
//		 }
//		 if(found && Math.Abs(minAngle) < Math.Abs(angleErr) && minAngle*angleErr>0) {
//		 angleErr += 2.0*Math.PI*Math.Sign(-angleErr);
//		 }
//		 }
//		 }

		double rotPointDist = max(200.0, min(350.0, ballPos->length())); //the point around which we rotate
		double distToOpp;
		auto opp = wm->robots->opponents.getClosestToBall(distToOpp);
		if (distToOpp < 800)
		{
			rotPointDist = 50;
		}

//		cout << "RobotMovement: rotPointDist " << rotPointDist << endl;

		msl_actuator_msgs::MotionControl bm;

		bm.motion.rotation = pRot * angleErr + dRot * geometry::normalizeAngle(angleErr - lastRotErr); //Rotation PD

//		cout << "RobotMovement: rotation " << bm.motion.rotation << endl;

		if (bm.motion.rotation > curRot)
		{ //limit rotation acceleration
			bm.motion.rotation = min(bm.motion.rotation, curRot + rotAccStep);
		}
		else
		{
			bm.motion.rotation = max(bm.motion.rotation, curRot - rotAccStep);
		}

		bm.motion.rotation = min(abs(bm.motion.rotation), maxRot) * (bm.motion.rotation > 0 ? 1 : -1); //clamp rotation
		curRot = bm.motion.rotation;

		lastRotErr = angleErr;

		double transOrt = bm.motion.rotation * rotPointDist; //the translation corresponding to the curve we drive
		double maxCurTrans = pathPlanningMaxTrans;
		double transErr = abs(angleErr);
		if (transErr > angleDeadBand)
		{
			transControlIntegral += iTrans * transErr;
			transControlIntegral = min(transControlIntegralMax, transControlIntegral);
		}
		else
		{
			transControlIntegral = 0; //Math.Max(0,transControlIntegral-Math.PI*5);
			transErr = 0;
		}
		maxCurTrans -= pTrans * transErr + transControlIntegral;
		maxCurTrans = max(0.0, maxCurTrans);

//		cout << "RobotMovement: pathvel " << pathPlanningMaxTrans << " maxVel " << maxCurTrans << endl;
		//maxCurTrans *= Math.Min(1,(Math.PI/180*30)/Math.Abs(angleErr*angleErr));

		double transTowards = sqrt(maxCurTrans * maxCurTrans - transOrt * transOrt);
		if (std::isnan(transTowards) || transTowards < 50)
			transTowards = 50;

		if (transTowards > curTrans)
		{
			transTowards = min(transTowards, curTrans + transAccStep);
		}
		else
			transTowards = max(transTowards, curTrans - transDecStep);
		curTrans = transTowards;


		auto dir = ballPos->normalize();
		shared_ptr<geometry::CNPoint2D> ort = make_shared<geometry::CNPoint2D>(dir->y, -dir->x);
		dir = dir * transTowards + ort * transOrt;
		bm.motion.angle = dir->angleTo();

		bm.motion.translation = sqrt(transTowards * transTowards + transOrt * transOrt);

//		cout << "RobotMovement: " << bm.motion.translation << " " << bm.motion << endl;

		return make_shared<msl_actuator_msgs::MotionControl>(bm);

	}
*/
	double RobotMovement::lastTurnTime = -1;
	void RobotMovement::updateLastTurnTime()
	{
		lastTurnTime = supplementary::DateTime::getUtcNow().getTicks();
	}
/*
	shared_ptr<geometry::CNPoint2D> RobotMovement::dribbleNeedToTurn(
			shared_ptr<geometry::CNPosition> ownPos, shared_ptr<geometry::CNPoint2D> ballPos,
			shared_ptr<geometry::CNPoint2D> pathPlanningPoint)
	{
		if (lastTurnTime > 0 && (supplementary::DateTime::getUtcNow().getTicks() - lastTurnTime) < 1000 * 10000)
			return nullptr;
		msl::MSLWorldModel* wm = msl::MSLWorldModel::get();
		auto dstscan = wm->rawSensorData->getDistanceScan();
		auto oppInFront = wm->robots->opponents.getInCorridor(ballPos->angleTo(), 150);
		double distInFront = (oppInFront == nullptr ? std::numeric_limits<double>::max() : oppInFront->length() - 300);

		double minInFrontDist = 800;
//		if (od!=null && od.Motion!=null) {
//		 minInFrontDist = Math.Max(minInFrontDist,Math.Min(2000,od.Motion.Translation+800));
//		 }
		if (ballPos != nullptr && pathPlanningPoint != nullptr
				&& abs(geometry::deltaAngle(ballPos->angleTo(), pathPlanningPoint->angleTo())) > M_PI * 4.65 / 6.0)
		{
			lastTurnTime = supplementary::DateTime::getUtcNow().getTicks();

			return pathPlanningPoint->egoToAllo(*ownPos);
		}
		else if (ballPos != nullptr && dstscan != nullptr && distInFront < minInFrontDist && distInFront > 300)
		{

			auto goalMid = make_shared<geometry::CNPoint2D>(wm->field->getFieldLength() / 2.0, 0)->alloToEgo(*ownPos);

			auto opponentGoal = wm->robots->opponents.getInCorridor(goalMid->angleTo(), 300);
			if (opponentGoal != nullptr && opponentGoal->length() > 3000)
			{
				return make_shared<geometry::CNPoint2D>(wm->field->getFieldLength() / 2.0, 0);
			}

			if (oppInFront != nullptr)
			{
				lastTurnTime = supplementary::DateTime::getUtcNow().getTicks();
				return oppInFront->rotate(M_PI)->egoToAllo(*ownPos);
			}
			else if (ballPos != nullptr)
			{
				lastTurnTime = supplementary::DateTime::getUtcNow().getTicks();
				return ballPos->rotate(M_PI)->egoToAllo(*ownPos);
			}
			else
				return nullptr;
		}
		return nullptr;

	}
	*/
/*
	msl_actuator_msgs::MotionControl RobotMovement::moveToPointFast(shared_ptr<geometry::CNPoint2D> egoTarget,
													shared_ptr<geometry::CNPoint2D> egoAlignPoint, double snapDistance,
													shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> additionalPoints)
	{
		msl_actuator_msgs::MotionControl mc;
		MSLWorldModel* wm = MSLWorldModel::get();
		shared_ptr<PathEvaluator> eval = make_shared<PathEvaluator>();
		shared_ptr<geometry::CNPoint2D> temp = PathProxy::getInstance()->getEgoDirection(egoTarget, eval,
		additionalPoints);
		if(temp != nullptr)
		{
			egoTarget = temp;
		}
		else
		{
			cout << "RobotMovement::moveToPointFast::getEgoDirection == nullptr => ownPos not available" << endl;
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
*/
	msl_actuator_msgs::MotionControl RobotMovement::moveToPointCarefully(shared_ptr<geometry::CNPoint2D> egoTarget,
														shared_ptr<geometry::CNPoint2D> egoAlignPoint,
														double snapDistance,
														shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> additionalPoints)
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
	/*
	 MotionControl RobotMovement::interceptCarefully(shared_ptr<geometry::CNPoint2D> egoTarget,
	 shared_ptr<geometry::CNPoint2D> egoAlignPoint, double snapDistance,
	 shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> additionalPoints)
	 {
	 MotionControl mc;
	 if (egoTarget->length() > 400)
	 {
	 MSLWorldModel* wm = MSLWorldModel::get();
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
	 */

	/**
	 * could be replaced with
	 * query->egoDestinationPoint = egoTerget
	 * query->egoAlignPoint = egoAlignPoint
	 * query->angleTolerance = angletolerance
	 * mc = rm.moveToPoint(query);
	 * mc.motion.translation = 0;
	 *
	 * and maybe use PID controller
	 * mc.motion.rotation = PID controller
	 */
	msl_actuator_msgs::MotionControl RobotMovement::alignToPointNoBall(shared_ptr<geometry::CNPoint2D> egoTarget,
													shared_ptr<geometry::CNPoint2D> egoAlignPoint,
													double angleTolerance)
	{
		msl_actuator_msgs::MotionControl mc;

		//What is the sense of an ego target if translation is 0 anyways?
		double egoTargetAngle = egoTarget->angleTo();
		double deltaTargetAngle = geometry::deltaAngle(egoAlignPoint->angleTo(), M_PI);
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

	msl_actuator_msgs::MotionControl RobotMovement::alignToPointWithBall(shared_ptr<geometry::CNPoint2D> egoAlignPoint,
														shared_ptr<geometry::CNPoint2D> egoBallPos,
														double angleTolerance, double ballAngleTolerance)
	{
		msl_actuator_msgs::MotionControl mc;
		MSLWorldModel* wm = MSLWorldModel::get();
		double egoTargetAngle = egoAlignPoint->angleTo();
		double egoBallAngle = egoBallPos->angleTo();
		double deltaTargetAngle = geometry::deltaAngle(egoTargetAngle, M_PI);
		double deltaBallAngle = geometry::deltaAngle(egoBallAngle, M_PI);

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
/*
	msl_actuator_msgs::MotionControl RobotMovement::rapidAlignToPointWithBall(shared_ptr<geometry::CNPoint2D> egoAlignPoint,
															shared_ptr<geometry::CNPoint2D> egoBallPos,
															double angleTolerance, double ballAngleTolerance)
	{
		msl_actuator_msgs::MotionControl mc;
		MSLWorldModel* wm = MSLWorldModel::get();
		double egoTargetAngle = egoAlignPoint->angleTo();
		double egoBallAngle = egoBallPos->angleTo();
		double deltaTargetAngle = geometry::deltaAngle(egoTargetAngle, M_PI);
		double deltaBallAngle = geometry::deltaAngle(egoBallAngle, M_PI);

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
	*/
/*
	msl_actuator_msgs::MotionControl RobotMovement::ruleActionForBallGetter()
	{
		MSLWorldModel* wm = MSLWorldModel::get();
		shared_ptr<geometry::CNPoint2D> ballPos = wm->ball->getEgoBallPosition();
		if (ballPos == nullptr)
		{
			msl_actuator_msgs::MotionControl mc;
			mc.senderID = -1;
			return mc;
		}
		shared_ptr<geometry::CNPosition> ownPos = wm->rawSensorData->getOwnPositionVision(); //OwnPositionCorrected;
		if (ownPos == nullptr)
		{
			msl_actuator_msgs::MotionControl mc;
			mc.senderID = -1;
			return mc;
		}
		shared_ptr<geometry::CNPoint2D> alloBall = ballPos->egoToAllo(*ownPos);
		shared_ptr<geometry::CNPoint2D> dest = make_shared<geometry::CNPoint2D>();

		if (!wm->field->isInsideField(alloBall, 500))
		{ //ball is out, approach it carefully
		  //Console.WriteLine("CASE B");
			dest->x = ownPos->x - alloBall->x;
			dest->y = ownPos->y - alloBall->y;
			dest = wm->field->mapInsideField(alloBall);
			dest = dest->alloToEgo(*ownPos);
			return placeRobotCareBall(dest, ballPos, maxVelo);
		}
		if (wm->field->isInsideOwnPenalty(alloBall, 0))
		{ //handle ball in own penalty
			if (!wm->field->isInsideOwnGoalArea(alloBall, 200)
					&& wm->field->isInsideOwnPenalty(ownPos->getPoint(), 0))
			{ //if we are already in, and ball is in safe distance of keeper area, get it
				msl_actuator_msgs::MotionControl mc;
				mc.senderID = -1;
				return mc;
			}
			if (wm->robots->teammates.teamMatesInOwnPenalty() > 1)
			{ //do not enter penalty if someone besides keeper is already in there
			  //dest.X = ownPos.X - alloBall.X;
			  //dest.Y = ownPos.Y - alloBall.Y;
				dest = wm->field->mapOutOfOwnPenalty(alloBall);
				dest = dest->alloToEgo(*ownPos);
				return placeRobotCareBall(dest, ballPos, maxVelo);
			}
			if (wm->field->isInsideOwnGoalArea(alloBall, 200))
			{ //ball is dangerously close to keeper area, or even within
				if (!wm->field->isInsideOwnGoalArea(alloBall, 50))
				{
					if ((ownPos->x - alloBall->x) < 150)
					{
						msl_actuator_msgs::MotionControl mc;
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
				dest = wm->field->mapOutOfOwnGoalArea(dest); //drive to the closest side of the ball and hope to get it somehow
				dest = dest->alloToEgo(*ownPos);
				return placeRobotCareBall(dest, ballPos, maxVelo);
			}

		}
		if (wm->field->isInsideOppPenalty(alloBall, 0))
		{ //ball is inside Opp penalty area
			if (wm->robots->teammates.teamMatesInOppPenalty() > 0)
			{ //if there is someone else, do not enter
			  //dest.X = ownPos.X - alloBall.X;
			  //dest.Y = ownPos.Y - alloBall.Y;
				dest = wm->field->mapOutOfOppPenalty(alloBall);
				dest = dest->alloToEgo(*ownPos);
				return placeRobot(dest, ballPos, maxVelo);
			}
			if (wm->field->isInsideOppGoalArea(alloBall, 50))
			{ //ball is inside keeper area
				dest = wm->field->mapOutOfOppGoalArea(alloBall); //just drive as close to the ball as you can
				dest = dest->alloToEgo(*ownPos);
				return placeRobot(dest, ballPos, maxVelo);
			}

		}
		msl_actuator_msgs::MotionControl mc;
		mc.senderID = -1;
		return mc;
	}
*/
	/**
	 * could be replaced with:
	 * movQuery->egoDestinationPoint = destinationPoint;
     * movQuery->egoAlignPoint = headingPoint;
     * mc = rm.moveToPoint(movQuery);
     *
     * if (destinationPoint->length() < 100)
     * {
     * 		mc.motion.translation = 0;
     * }
	 */
	/*
	msl_actuator_msgs::MotionControl RobotMovement::placeRobotCareBall(shared_ptr<geometry::CNPoint2D> destinationPoint,
													shared_ptr<geometry::CNPoint2D> headingPoint, double translation)
	{
		double rotTol = M_PI / 30.0;
		double destTol = 100.0;
		MSLWorldModel* wm = MSLWorldModel::get();
		if (destinationPoint->length() < destTol)
		{

			return alignToPointNoBall(destinationPoint, headingPoint, rotTol);
		}
		else
		{
			//linear
			double trans = min(translation, 1.2 * destinationPoint->length());

			msl_actuator_msgs::MotionControl mc;
			//DriveHelper.DriveToPointAndAlignCareBall(destinationPoint, headingPoint, trans, wm);
			if (wm->ball->getAlloBallPosition() != nullptr)
			{
				shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> additionalPoints = make_shared<
				vector<shared_ptr<geometry::CNPoint2D>>>();
				additionalPoints->push_back(wm->ball->getAlloBallPosition());
				//DriveToPointAndAlignCareObstacles
				//cout << "RobotMovement: playeRobotCareBall 1" << endl;
				mc = moveToPointCarefully(destinationPoint, headingPoint, 0 , additionalPoints);
			}
			else
			{
				//cout << "RobotMovement: playeRobotCareBall 2" << endl;
				mc = moveToPointCarefully(destinationPoint, headingPoint, 0 , nullptr);
			}
			//cout << "RobotMovement: placeRobotCareBall return." << endl;
			return mc;
		}
	}
	*/
/*
	msl_actuator_msgs::MotionControl RobotMovement::placeRobot(shared_ptr<geometry::CNPoint2D> destinationPoint,
											shared_ptr<geometry::CNPoint2D> headingPoint, double translation)
	{
		double rotTol = M_PI / 30.0;
		double destTol = 100.0;

		if (destinationPoint->length() < destTol)
		{
			// DriveToPointAndAlignCareObstacles(destinationPoint, headingPoint, translation, wm);
			msl_actuator_msgs::MotionControl rot = moveToPointCarefully(destinationPoint, headingPoint, 0, nullptr);
			rot.motion.translation = 0.0;
			if (headingPoint == nullptr)
			{
				return rot;
			}
			double angle = headingPoint->angleTo();
			MSLWorldModel* wm = MSLWorldModel::get();
			if (abs(angle - wm->kicker->kickerAngle) > rotTol)
			{
				return rot;
			}
			else
			{
				msl_actuator_msgs::MotionControl bm;

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

			msl_actuator_msgs::MotionControl bm = moveToPointCarefully(destinationPoint, headingPoint, 0, nullptr);

			return bm;
		}
	}
	*/
/*
	msl_actuator_msgs::MotionControl RobotMovement::driveRandomly(double translation)
	{
		if (randomCounter == 0)
		{
			randomTarget = getRandomTarget();
		}

		shared_ptr<geometry::CNPoint2D> dest = PathProxy::getInstance()->getEgoDirection(
				randomTarget, make_shared<PathEvaluator>());
//				pp.GetEgoDirection(randomTarget,wm,false,translation, out translation);
		if (dest == nullptr)
		{
			dest = randomTarget;
			translation = 100;
		}
		msl_actuator_msgs::MotionControl bm;
		bm.motion.rotation = 0;
		bm.motion.translation = translation;
		bm.motion.angle = atan2(dest->y, dest->x);
		randomCounter = (randomCounter + 1) % 28;
		return bm;
	}
*/
	/*
	shared_ptr<geometry::CNPoint2D> RobotMovement::getRandomTarget()
	{
		double ang = (rand() - 0.5) * 2 * M_PI;
		shared_ptr<geometry::CNPoint2D> dest = make_shared<geometry::CNPoint2D>(cos(ang) * 5000, sin(ang) * 5000);
		return dest;
	}
	*/
	/**
	 * alloPassee = allo team mate position
	 * maxTrans = max translation
	 */
	/*
	msl_actuator_msgs::MotionControl RobotMovement::moveToFreeSpace(shared_ptr<geometry::CNPoint2D> alloPassee, double maxTrans)
	{
		msl_actuator_msgs::MotionControl mc;
		MSLWorldModel* wm = MSLWorldModel::get();
		shared_ptr<geometry::CNPosition> ownPos = wm->rawSensorData->getOwnPositionVision();
		if (ownPos == nullptr)
		{
			msl_actuator_msgs::MotionControl mc;
			mc.senderID = -1;
			return mc;
		}

		shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> ops = wm->obstacles->getEgoVisionObstaclePoints(); //WM.GetTrackedOpponents();
		fringe->clear();
		for (int i = 0; i < 16; i++)
		{
			for (int d = 0; d < 8000; d += 2000)
			{
				shared_ptr<AlloSearchArea> s = AlloSearchArea::getAnArea(i * M_PI / 8, (i + 1) * M_PI / 8, d, d + 2000,
																			ownPos->getPoint(), ownPos);
				if (s->isValid())
				{

					s->val = evalPointDynamic(s->midP, alloPassee, ownPos, wm->obstacles->getEgoVisionObstaclePoints());
					fringe->push_back(s);
				}
			}
		}
		stable_sort(fringe->begin(), fringe->end(), SearchArea::compareTo);
		shared_ptr<SearchArea> best = fringe->at(0);
		shared_ptr<SearchArea> cur;

		for (int i = 0; i < 100 && fringe->size() > 0; i++)
		{

			next->clear();
			for (int j = 0; j < beamSize; j++)
			{
				if (fringe->size() == 0)
				{
					break;
				}
				cur = fringe->at(0);
				fringe->erase(fringe->begin());
				if (j == 0 && cur->val > best->val)
				{
					best = cur;
				}
				shared_ptr<vector<shared_ptr<SearchArea>>> expanded = cur->expand();
				for (int i = 0; expanded->size(); i++)
				{
					next->push_back(expanded->at(i));
				}
			}
			for (int j = 0; j < next->size(); j++)
			{
				next->at(j)->val = evalPointDynamic(next->at(j)->midP, alloPassee, ownPos,
													wm->obstacles->getEgoVisionObstaclePoints());
				fringe->push_back(next->at(j));
			}
			stable_sort(fringe->begin(), fringe->end(), SearchArea::compareTo);
		}
		shared_ptr<geometry::CNPoint2D> dest =
				wm->field->mapOutOfOppGoalArea(wm->field->mapInsideField(best->midP))->alloToEgo(*ownPos);
		shared_ptr<geometry::CNPoint2D> align = alloPassee->alloToEgo(*ownPos);

		mc = placeRobotAggressive(dest, align, maxTrans);
		return mc;
	}
*/
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
/*
	msl_actuator_msgs::MotionControl RobotMovement::placeRobotAggressive(shared_ptr<geometry::CNPoint2D> destinationPoint,
														shared_ptr<geometry::CNPoint2D> headingPoint,
														double translation)
	{
		double rotTol = M_PI / 30.0;
		double destTol = 100.0;

		if (destinationPoint->length() < destTol)
		{
			return alignToPointNoBall(destinationPoint, headingPoint, rotTol);
		}
		else
		{
			//linear
			double trans = min(translation, 1.6 * destinationPoint->length());
			msl_actuator_msgs::MotionControl bm = moveToPointCarefully(destinationPoint, headingPoint, 0, nullptr); //DriveHelper.DriveToPointAndAlignCareObstacles(destinationPoint, headingPoint, trans, wm);
			return bm;
		}
	}
	*/
/*
	double RobotMovement::evalPointDynamic(shared_ptr<geometry::CNPoint2D> alloP,
											shared_ptr<geometry::CNPoint2D> alloPassee,
											shared_ptr<geometry::CNPosition> ownPos,
											shared_ptr<vector<shared_ptr<geometry::CNPoint2D> > > opponents)
	{
		double ret = 0;

		//distance to point:
		ret -= ownPos->distanceTo(alloP) / 10.0;
		MSLWorldModel* wm = MSLWorldModel::get();
		shared_ptr<geometry::CNPoint2D> oppGoalMid = make_shared<geometry::CNPoint2D>(wm->field->getFieldLength() / 2,
																						0);

		shared_ptr<geometry::CNPoint2D> passee2p = alloP - alloPassee;
		//if (passee2p.X < 0 && Math.Abs(alloP.Y) < 1000) return Double.MinValue;

		shared_ptr<geometry::CNPoint2D> passee2LeftOwnGoal = make_shared<geometry::CNPoint2D>(
				-wm->field->getFieldLength() / 2.0 - alloPassee->x,
				wm->field->getGoalWidth() / 2.0 + 1000 - alloPassee->y);
		shared_ptr<geometry::CNPoint2D> passee2RightOwnGoal = make_shared<geometry::CNPoint2D>(
				-wm->field->getFieldLength() / 2.0 - alloPassee->x,
				-wm->field->getGoalWidth() / 2.0 - 1000 - alloPassee->y);

		if (!geometry::leftOf(passee2LeftOwnGoal, passee2p) && geometry::leftOf(passee2RightOwnGoal, passee2p))
		{
			return numeric_limits<double>::min();
		}
		if (wm->field->isInsideOppPenalty(alloPassee, 800) && wm->field->isInsideOppPenalty(alloP, 600))
		{
			return numeric_limits<double>::min();
		}

		ret += passee2p->x / 9.0;
		if (alloP->x < 0)
		{
			ret += alloP->x / 10.0;
		}

		if (alloP->y * alloPassee->y < 0)
		{
			ret += min(0.0, alloP->x);
		}
		if (alloP->angleToPoint(oppGoalMid) > M_PI / 3.0)
		{
			ret -= alloP->angleToPoint(oppGoalMid) * 250.0;
		}
		//distance to passeee:

		//Point2D p2GoalVec = goalPos - p;
		double dist2Passee = passee2p->length();

		//nice passing distances: 4000..9000:
		if (dist2Passee < 2000)
		{
			return numeric_limits<double>::min();
		}
		if (dist2Passee < 4000)
		{
			ret -= 4000 - dist2Passee;
		}
		else if (dist2Passee > 9000)
		{
			ret -= dist2Passee - 9000;
		}

		//else if (dist2Ball > 5000) ret -= (dist2Ball-5000)*(dist2Ball - 5000);

		double t, v;
		double ortX, ortY;
		double catchFactor = 0;

		//double goalFactor = 0;

		for (int i = 0; i < opponents->size(); i++)
		{
			//pass corridor
			t = ((opponents->at(i)->x - alloPassee->x) * passee2p->x
					+ (opponents->at(i)->y - alloPassee->y) * passee2p->y)
					/ (passee2p->x * passee2p->x + passee2p->y * passee2p->y);

			if (t > 0)
			{
				if (t < 1.0)
				{

					ortX = opponents->at(i)->x - (alloPassee->x + t * (passee2p->x));
					ortY = opponents->at(i)->y - (alloPassee->y + t * (passee2p->y));
					v = max(0.0, sqrt(ortX * ortX + ortY * ortY) - robotRadius) / dist2Passee;

					if (v / t * interceptQuotient < 1)
					{

						double cf = 5000 * (1 - ((v / t) * interceptQuotient));
						catchFactor = max(catchFactor, cf);
					}
				}
				ortX = opponents->at(i)->x - alloP->x;
				ortY = opponents->at(i)->y - alloP->y;
				v = sqrt(ortX * ortX + ortY * ortY);
				if (v < 2500)
					ret -= (2500 - v) * 10;
			}
		}
		ret -= catchFactor;

		//ret -= goalFactor;
		return ret;
	}
	*/
}
