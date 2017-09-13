/*
 * RobotMovement.cpp *
 *
 *  Created on: 17.12.2014
 *      Author: tobi
 */

#include "msl_robot/robotmovement/RobotMovement.h"
#include "Ball.h"
#include "GeometryCalculator.h"
#include "MSLFootballField.h"
#include "MSLWorldModel.h"
#include "RawSensorData.h"
#include "Robots.h"
#include "container/CNPoint2D.h"
#include "msl_robot/kicker/Kicker.h"
#include "msl_robot/robotmovement/AlloSearchArea.h"
#include "msl_robot/robotmovement/MovementQuery.h"
#include "msl_robot/robotmovement/SearchArea.h"
#include "obstaclehandler/Obstacles.h"
#include "pathplanner/PathProxy.h"
#include "pathplanner/evaluator/PathEvaluator.h"

#include <SystemConfig.h>

// remove commentary for debug output
//#define RM_DEBUG

namespace msl
{
	int RobotMovement::randomCounter = 0;
	int RobotMovement::beamSize = 3;
	shared_ptr<geometry::CNPoint2D> RobotMovement::randomTarget = nullptr;
	shared_ptr<vector<shared_ptr<SearchArea>>> RobotMovement::fringe = make_shared<vector<shared_ptr<SearchArea>>>();
	shared_ptr<vector<shared_ptr<SearchArea>>> RobotMovement::next = make_shared<vector<shared_ptr<SearchArea>>>();
	double RobotMovement::assume_enemy_velo = 4500;
	double RobotMovement::assume_ball_velo = 5000;
	double RobotMovement::interceptQuotient = RobotMovement::assume_ball_velo / RobotMovement::assume_enemy_velo;
	double RobotMovement::robotRadius = 300;

	RobotMovement::RobotMovement()
	{
		this->wm = MSLWorldModel::get();
		this->pp = PathProxy::getInstance();

		double defaultTranslation = 0;
		double defaultRotateP = 0;
		double fastTranslation = 0;
		double fastRotation = 0;

		double rotationP = 0;
		double rotationD = 0;
		double transP = 0;
		double transI = 0;

		readConfigParameters();
	}

	RobotMovement::~RobotMovement()
	{
	}

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
	 * @param velocityMode -> from type msl::MovementQuery::Velocity
	 *
	 */
	msl_actuator_msgs::MotionControl RobotMovement::moveToPoint(shared_ptr<MovementQuery> query)
	{
		msl_actuator_msgs::MotionControl mc;

		if (query == nullptr || query->egoDestinationPoint == nullptr)
		{
			cerr << "RobotMovement::moveToPoint() -> egoDestinationPoint == nullptr or query = nullptr" << endl;
			return setNAN();
		}
		shared_ptr<geometry::CNPoint2D> egoTarget = nullptr;
		if (query->pathEval == nullptr)
		{
			egoTarget = this->pp->getEgoDirection(query->egoDestinationPoint, make_shared<PathEvaluator>(),
													query->getPathPlannerQuery());
		}
		else
		{
			egoTarget = this->pp->getEgoDirection(query->egoDestinationPoint, query->pathEval,
													query->getPathPlannerQuery());
		}

		// ANGLE
		mc.motion.angle = egoTarget->angleTo();

		// TRANSLATION
		if (egoTarget->length() > query->snapDistance)
		{
			mc.motion.translation = egoTarget->length();

		}
		else
		{
			mc.motion.translation = 0;
		}

		// ROTATION
		if (query->egoAlignPoint != nullptr)
		{
			mc.motion.rotation = query->egoAlignPoint->rotate(M_PI)->angleTo();

		}

		// pt controller stuff
		query->initializePTControllerParameters();

		std::valarray<double> translation = query->ptController(mc.motion.rotation, mc.motion.translation);

		double maxTranslation = this->defaultTranslation;

		if (query->velocityMode == MovementQuery::Velocity::FAST)
		{
			maxTranslation = this->fastTranslation;

		} else if (query->velocityMode == MovementQuery::Velocity::CAREFULLY) {
			maxTranslation = this->carefullyTranslation;
		}

		mc.motion.translation = min(translation[0], maxTranslation);

		mc.motion.rotation = translation[1]; //for PT

		mc.motion.angle = egoTarget->angleTo();

		//angle correction to respect anlge change through rotation
		mc.motion.angle -= mc.motion.rotation / 30.0; //1/30 s= time step , time step * omega = phi

#ifdef RM_DEBUG
		cout << "RobotMovement::moveToPoint: Angle = " << mc.motion.angle << " Trans = " << mc.motion.translation << " Rot = " << mc.motion.rotation << endl;
#endif
    return mc;
}

msl_actuator_msgs::MotionControl RobotMovement::experimentallyAlignTo(shared_ptr<MovementQuery> m_Query)
{
	MotionControl mc;
//	auto egoBallPos = wm->ball->getEgoBallPosition();

	if (m_Query == nullptr)
	{
		cerr << "RobotMovement:experimentallyAlignTo: query is nullptr!" << endl;
		return setNAN();
	}
	if (m_Query->egoAlignPoint == nullptr)
	{
		cerr << "RobotMovement:experimentallyAlignTo: egoAlignPoint is nullptr!" << endl;
		return setNAN();
	}
//	if (egoBallPos == nullptr)
//	{
//		cerr << "RobotMovement:experimentallyAlignTo: egoBallPos is nullptr!" << endl;
//		return setNAN();
//	}

	if (m_Query->egoAlignPoint->y > 0)
	{
		// rigth = 1.57079632679
		mc.motion.angle = (0.5 * M_PI);
	}else {
		// left = 4.71238898038
		mc.motion.angle = (1.5 * M_PI);
	}
	mc.motion.angle = mc.motion.angle * M_PI;
	// right rotation is negative
	// left rotation is positive
	cout << "angle: " << mc.motion.angle << endl;
	double rotation = m_Query->egoAlignPoint->angleTo();
	mc.motion.rotation = rotation * -1;
	cout << "rotation: " << mc.motion.rotation << endl;
	cout << "egoAlignPoint: =" << m_Query->egoAlignPoint->x << " y=" << m_Query->egoAlignPoint->y << endl;
	// for testing ... maybe you can use the pt-controller
	// TODO need to stop if angle is good
	mc.motion.translation = 1000;

	return mc;
}

msl_actuator_msgs::MotionControl RobotMovement::alignTo(shared_ptr<MovementQuery> m_Query)
{
    cout << "RobotMovement::alignTo()" << endl;
    if (m_Query == nullptr)
    {
        cerr << "RobotMovement::alignTo() -> MovementQuery == nullptr" << endl;
        return setNAN();
    }

    if (m_Query->egoAlignPoint == nullptr)
    {
        cerr << "RobotMovement::alignTo() -> egoAlignPoint -> nullptr" << endl;
        return setNAN();
    }

    MotionControl mc;

    mc.motion.translation = 0;
    mc.motion.angle = 0;
    mc.motion.rotation = m_Query->rotationPDForDribble(m_Query->egoAlignPoint);

    if (m_Query->rotateAroundTheBall)
    {
        			if ((fabs(m_Query->egoAlignPoint->angleTo()) < (M_PI - m_Query->angleTolerance)))
//        if (wm->ball->haveBall() && (fabs(m_Query->egoAlignPoint->angleTo()) < (M_PI - m_Query->angleTolerance)))
        {
            //#ifdef RM_DEBUG
            cout << "RobotMovement::alignTo(): rotate around the ball" << endl;
            //#endif

            if (wm->ball->getEgoBallPosition() == nullptr)
            {
                cerr << "RobotMovement::alignTo(): egoBallPosition == nullptr" << endl;
                return setNAN();
            }

            // setting parameters for controller
            m_Query->setRotationPDParameters(rotationP, rotationD);
            m_Query->setTranslationPIParameters(transP, transI);

            m_Query->additionalPoints = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();
            m_Query->additionalPoints->push_back(wm->ball->getEgoBallPosition());

            shared_ptr<geometry::CNPoint2D> egoTarget = nullptr;
            if (m_Query->pathEval == nullptr)
            {
                egoTarget = this->pp->getEgoDirection(m_Query->egoAlignPoint, make_shared<PathEvaluator>(), m_Query->getPathPlannerQuery());
            }
            else
            {
                egoTarget = this->pp->getEgoDirection(m_Query->egoAlignPoint, m_Query->pathEval, m_Query->getPathPlannerQuery());
            }

            //				shared_ptr<PathEvaluator> eval = make_shared<PathEvaluator>();
            //				shared_ptr<geometry::CNPoint2D> egoTarget = this->pp->getEgoDirection(m_Query->egoAlignPoint, eval,
            //																						m_Query->additionalPoints);

            mc.motion.rotation = m_Query->rotationPDForDribble(egoTarget);
            double rotPointDist = 350.0;

            if (auto ballPos = wm->ball->getEgoBallPosition())
            {
                rotPointDist = min(rotPointDist, ballPos->length()); // the point around which we rotate
            }

            double transOrt = mc.motion.rotation * rotPointDist; // the translation corresponding to the curve we drive

            mc.motion.translation = m_Query->translationPIForDribble(transOrt);
            //				double toleranceDist = 500;
            mc.motion.angle = m_Query->egoAlignPoint->angleTo() < 0 ? M_PI / 2 : (M_PI + M_PI / 4);
            //			mc.motion.angle = m_Query->angleCalcForDribble(transOrt);
        }
    }
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
    shared_ptr<geometry::CNPosition> ownPos = wm->rawSensorData->getOwnPositionVision(); // OwnPositionCorrected;
    if (egoBallPos == nullptr || ownPos == nullptr)
    {
        return setNAN();
    }
    shared_ptr<geometry::CNPoint2D> alloBall = egoBallPos->egoToAllo(*ownPos);
    shared_ptr<geometry::CNPoint2D> dest = make_shared<geometry::CNPoint2D>();

    // ball is out, approach it carefully ================================================================
    if (!wm->field->isInsideField(alloBall, 500))
    {
        dest = ownPos - alloBall;
        dest = wm->field->mapInsideField(alloBall);
        dest = dest->alloToEgo(*ownPos);
        return placeRobot(dest, egoBallPos);
    }
    // handle ball in own penalty ========================================================================
    if (wm->field->isInsideOwnPenalty(alloBall, 0))
    {
        if (!wm->field->isInsideOwnGoalArea(alloBall, 200) && wm->field->isInsideOwnPenalty(ownPos->getPoint(), 0))
        {
            // if we are already in, and ball is in safe distance of keeper area, get it
            return setNAN();
        }
        if (wm->robots->teammates.teamMatesInOwnPenalty() > 1)
        {
            // do not enter penalty if someone besides keeper is already in there
            dest = wm->field->mapOutOfOwnPenalty(alloBall);
            dest = dest->alloToEgo(*ownPos);
            return placeRobot(dest, egoBallPos);
        }
        if (wm->field->isInsideOwnGoalArea(alloBall, 200))
        {
            // ball is dangerously close to keeper area, or even within
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
            dest = wm->field->mapOutOfOwnGoalArea(dest); // drive to the closest side of the ball and hope to get it somehow
            dest = dest->alloToEgo(*ownPos);
            return placeRobot(dest, egoBallPos);
        }
    }
    // ball is inside enemy penalty area ===============================================================
    if (wm->field->isInsideOppPenalty(alloBall, 0))
    {
        if (wm->robots->teammates.teamMatesInOppPenalty() > 0)
        {
            // if there is someone else, do not enter
            dest = wm->field->mapOutOfOppPenalty(alloBall);
            dest = dest->alloToEgo(*ownPos);
            return placeRobot(dest, egoBallPos);
        }
        if (wm->field->isInsideOppGoalArea(alloBall, 50))
        {
            // ball is inside keeper area
            dest = wm->field->mapOutOfOppGoalArea(alloBall); // just drive as close to the ball as you can
            dest = dest->alloToEgo(*ownPos);
            return placeRobot(dest, egoBallPos);
        }
    }
    //#ifdef RM_DEBUG
    //		cout << "RobotMovement::ruleActionForBallGetter: Angle = " << mc.motion.angle << " Trans = " << mc.motion.translation << " Rot = " <<
    //mc.motion.rotation << endl;
    //#endif
    return setNAN();
}

/*
 * Used in ruleActionForBallGetter()
 *
 * @return motion command by using moveToPoint() depending on
 * 		the distance to the destination and
 * 		the ball as additional point
 */
msl_actuator_msgs::MotionControl RobotMovement::placeRobot(shared_ptr<geometry::CNPoint2D> dest, shared_ptr<geometry::CNPoint2D> headingPoint)
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
            shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> additionalPoints = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();
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
		msl::PathProxy *pp = msl::PathProxy::getInstance();
		shared_ptr<msl::PathEvaluator> eval = make_shared<msl::PathEvaluator>();
		if (randomCounter == 0)
		{
			std::random_device rd;
			std::mt19937 gen(rd());
			std::uniform_real_distribution<> dis(0, 1);
			double ang = (dis(gen) - 0.5) * 2 * M_PI;
			randomTarget = make_shared<geometry::CNPoint2D>(cos(ang) * 5000, sin(ang) * 5000);
		}

		auto dest = pp->getEgoDirection(randomTarget, eval, make_shared<PathPlannerQuery>());

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
		cout << "RobotMovement::driveRandomly: Angle = " << bm.motion.angle << " Trans = " << bm.motion.translation << " Rot = " << bm.motion.rotation << endl;
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

		shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> ops = wm->obstacles->getEgoVisionObstaclePoints(); // WM.GetTrackedOpponents();
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
//		q->fast = true;
		mc = moveToPoint(q);
#ifdef RM_DEBUG
		cout << "RobotMovementmoveToFreeSpace: Angle = " << mc.motion.angle << " Trans = " << mc.motion.translation << " Rot = " << mc.motion.rotation << endl;
#endif
		return mc;
	}

	/*
	 * Used in moveToFreeSpace()
	 */
	double RobotMovement::evalPointDynamic(shared_ptr<geometry::CNPoint2D> alloP,
											shared_ptr<geometry::CNPoint2D> alloPassee,
											shared_ptr<geometry::CNPosition> ownPos,
											shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> opponents)
	{
		double ret = 0;

		// distance to point:
		ret -= ownPos->distanceTo(alloP) / 10.0;
		MSLWorldModel *wm = MSLWorldModel::get();
		shared_ptr<geometry::CNPoint2D> oppGoalMid = make_shared<geometry::CNPoint2D>(wm->field->getFieldLength() / 2, 0);

		shared_ptr<geometry::CNPoint2D> passee2p = alloP - alloPassee;
		// if (passee2p.X < 0 && Math.Abs(alloP.Y) < 1000) return Double.MinValue;

		shared_ptr<geometry::CNPoint2D> passee2LeftOwnGoal =
		make_shared<geometry::CNPoint2D>(-wm->field->getFieldLength() / 2.0 - alloPassee->x, wm->field->getGoalWidth() / 2.0 + 1000 - alloPassee->y);
		shared_ptr<geometry::CNPoint2D> passee2RightOwnGoal =
		make_shared<geometry::CNPoint2D>(-wm->field->getFieldLength() / 2.0 - alloPassee->x, -wm->field->getGoalWidth() / 2.0 - 1000 - alloPassee->y);

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
		// distance to passeee:

		// Point2D p2GoalVec = goalPos - p;
		double dist2Passee = passee2p->length();

		// nice passing distances: 4000..9000:
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

		// else if (dist2Ball > 5000) ret -= (dist2Ball-5000)*(dist2Ball - 5000);

		double t, v;
		double ortX, ortY;
		double catchFactor = 0;

		// double goalFactor = 0;

		for (int i = 0; i < opponents->size(); i++)
		{
			// pass corridor
			t = ((opponents->at(i)->x - alloPassee->x) * passee2p->x + (opponents->at(i)->y - alloPassee->y) * passee2p->y) /
			(passee2p->x * passee2p->x + passee2p->y * passee2p->y);

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

// ret -= goalFactor;
#ifdef RM_DEBUG
		cout << "RobotMovement::evalPointDynamic: ret = " << ret << endl;
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

	void RobotMovement::readConfigParameters()
	{
		supplementary::SystemConfig *sc = supplementary::SystemConfig::getInstance();
		carefullyTranslation = (*sc)["Drive"]->get<double>("Drive.Carefully.Velocity", NULL);
		defaultTranslation = (*sc)["Drive"]->get<double>("Drive.Default.Velocity", NULL);
		fastTranslation = (*sc)["Drive"]->get<double>("Drive.Fast.Velocity", NULL);
		carefullyRotation = (*sc)["Drive"]->get<double>("Drive.Carefully.RotateP", NULL);
		defaultRotation = (*sc)["Drive"]->get<double>("Drive.Default.RotateP", NULL);
		fastRotation = (*sc)["Drive"]->get<double>("Drive.Fast.RotateP", NULL);

		// for alignTo()
		rotationD = (*sc)["Drive"]->get<double>("Drive.RobotMovement.AlignTo.RotationD", NULL);
		rotationP = (*sc)["Drive"]->get<double>("Drive.RobotMovement.AlignTo.RotationP", NULL);
		transI = (*sc)["Drive"]->get<double>("Drive.RobotMovement.AlignTo.TranslationI", NULL);
		transP = (*sc)["Drive"]->get<double>("Drive.RobotMovement.AlignTo.TranslationP", NULL);
	}
}
