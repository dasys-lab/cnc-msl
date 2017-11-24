/*
 * RobotMovement.cpp *
 *
 *  Created on: 17.12.2014
 *      Author: tobi
 */

#include "msl_robot/robotmovement/RobotMovement.h"
#include "Ball.h"
#include "MSLFootballField.h"
#include "MSLWorldModel.h"
#include "RawSensorData.h"
#include "Robots.h"
#include "msl_robot/kicker/Kicker.h"
#include "msl_robot/robotmovement/AlloSearchArea.h"
#include "msl_robot/robotmovement/MovementQuery.h"
#include "msl_robot/robotmovement/SearchArea.h"
#include "obstaclehandler/Obstacles.h"
#include "pathplanner/PathProxy.h"
#include "pathplanner/evaluator/PathEvaluator.h"

#include <SystemConfig.h>
#include <nonstd/optional.hpp>

using nonstd::nullopt;
using nonstd::optional;
using nonstd::make_optional;

// remove commentary for debug output
//#define RM_DEBUG

namespace msl
{
    int RobotMovement::randomCounter = 0;
    int RobotMovement::beamSize = 3;
    nonstd::optional<geometry::CNPointEgo> RobotMovement::randomTarget = nullopt;
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
    msl_actuator_msgs::MotionControl RobotMovement::moveToPoint(MovementQuery& query)
    {
        msl_actuator_msgs::MotionControl mc;

        if (query.egoDestinationPoint == nullopt)
        {
            cerr << "RobotMovement::moveToPoint() -> egoDestinationPoint == nullptr or query = nullptr" << endl;
            return setNAN();
        }
        optional<geometry::CNPointEgo> egoTarget;
        if (query.pathEval == nullptr)
        {
            PathEvaluator pathEval;
            egoTarget = this->pp->getEgoDirection(*query.egoDestinationPoint, pathEval, *query.getPathPlannerQuery());
        }
        else
        {
            egoTarget = this->pp->getEgoDirection(*query.egoDestinationPoint, *query.pathEval,
                                                  *query.getPathPlannerQuery());
        }

        // ANGLE
        mc.motion.angle = egoTarget->angleZ();

        // TRANSLATION
        if (egoTarget->length() > query.snapDistance)
        {
            mc.motion.translation = egoTarget->length();

        }
        else
        {
            mc.motion.translation = 0;
        }

        // ROTATION
        if (query.egoAlignPoint != nullopt)
        {
            mc.motion.rotation = query.egoAlignPoint->rotateZ(M_PI).angleZ();

        }

        // pt controller stuff
        query.initializePTControllerParameters();

        std::valarray<double> translation = query.ptController(mc.motion.rotation, mc.motion.translation);

        double maxTranslation = this->defaultTranslation;

        if (query.velocityMode == MovementQuery::Velocity::FAST)
        {
            maxTranslation = this->fastTranslation;

        }
        else if (query.velocityMode == MovementQuery::Velocity::CAREFULLY)
        {
            maxTranslation = this->carefullyTranslation;
        }

        mc.motion.translation = min(translation[0], maxTranslation);

        mc.motion.rotation = translation[1]; //for PT

        mc.motion.angle = egoTarget->angleZ();

        //angle correction to respect anlge change through rotation
        mc.motion.angle -= mc.motion.rotation / 30.0; //1/30 s= time step , time step * omega = phi

#ifdef RM_DEBUG
        cout << "RobotMovement::moveToPoint: Angle = " << mc.motion.angle << " Trans = " << mc.motion.translation << " Rot = " << mc.motion.rotation << endl;
#endif
        return mc;
    }

    msl_actuator_msgs::MotionControl RobotMovement::alignTo(MovementQuery& m_Query)
    {
        msl_actuator_msgs::MotionControl mc;
//	auto egoBallPos = wm->ball->getEgoBallPosition();

        if (m_Query.egoAlignPoint == nullopt)
        {
            cerr << "RobotMovement:alignTo: egoAlignPoint is nullptr!" << endl;
            return setNAN();
        }
//	if (egoBallPos == nullptr)
//	{
//		cerr << "RobotMovement:experimentallyAlignTo: egoBallPos is nullptr!" << endl;
//		return setNAN();
//	}

        if (m_Query.rotateAroundTheBall)
        {
            if (m_Query.egoAlignPoint->y > 0)
            {
                // rigth = 1.57079632679
                mc.motion.angle = (0.5 * M_PI);
            }
            else
            {
                // left = 4.71238898038
                mc.motion.angle = (1.5 * M_PI);
            }

            mc.motion.angle = mc.motion.angle * M_PI;
            cout << "angle: " << mc.motion.angle << endl;
            // right rotation is negative
            // left rotation is positive
            double rotation = m_Query.egoAlignPoint->angleZ();
            mc.motion.rotation = rotation * -1;
            cout << "rotation: " << mc.motion.rotation << endl;
            cout << "egoAlignPoint: =" << m_Query.egoAlignPoint->x << " y=" << m_Query.egoAlignPoint->y << endl;
            // for testing ... maybe you can use the pt-controller
            // TODO need to stop if angle is good
            mc.motion.translation = 1000;
        }
        else
        {
            msl_actuator_msgs::MotionControl newMC = moveToPoint(m_Query);
            newMC.motion.translation = 0;
            mc = newMC;
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
        auto egoBallPos = wm->ball->getPositionEgo();
        auto ownPos = wm->rawSensorData->getOwnPositionVisionBuffer().getLastValidContent(); // OwnPositionCorrected;
        if (!egoBallPos || !ownPos)
        {
            return setNAN();
        }
        auto alloBall = egoBallPos->toAllo(*ownPos);
        auto dest = geometry::CNPointEgo();

        // ball is out, approach it carefully ================================================================
        if (!wm->field->isInsideField(alloBall, 500))
        {

            dest = wm->field->mapInsideField(alloBall).toEgo(*ownPos);
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
            if (wm->robots->teammates.teammatesInOwnPenalty() > 1)
            {
                // do not enter penalty if someone besides keeper is already in there
                dest = wm->field->mapOutOfOwnPenalty(alloBall).toEgo(*ownPos);
                return placeRobot(dest, egoBallPos);
            }
            if (wm->field->isInsideOwnGoalArea(alloBall, 200))
            {
                auto alloDest = geometry::CNPointAllo();
                // ball is dangerously close to keeper area, or even within
                if (!wm->field->isInsideOwnGoalArea(alloBall, 50))
                {
                    //TODO BUG? abs?
                    if ((ownPos->x - alloBall.x) < 150)
                    {
                        return setNAN();
                    }
                }
                alloDest.x = alloBall.x - 200;
                if (ownPos->y < alloBall.y)
                {
                    alloDest.y = alloBall.y - 500;
                }
                else
                {
                    alloDest.y = alloBall.y + 500;
                }
                dest = wm->field->mapOutOfOwnGoalArea(alloDest).toEgo(*ownPos); // drive to the closest side of the ball and hope to get it somehow
                return placeRobot(dest, egoBallPos);
            }
        }
        // ball is inside enemy penalty area ===============================================================
        if (wm->field->isInsideOppPenalty(alloBall, 0))
        {
            if (wm->robots->teammates.teammatesInOppPenalty() > 0)
            {
                // if there is someone else, do not enter
                dest = wm->field->mapOutOfOppPenalty(alloBall).toEgo(*ownPos);
                return placeRobot(dest, egoBallPos);
            }
            if (wm->field->isInsideOppGoalArea(alloBall, 50))
            {
                // ball is inside keeper area
                dest = wm->field->mapOutOfOppGoalArea(alloBall).toEgo(*ownPos); // just drive as close to the ball as you can
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
    msl_actuator_msgs::MotionControl RobotMovement::placeRobot(geometry::CNPointEgo dest,
                                                               nonstd::optional<geometry::CNPointEgo> headingPoint)
    {
        msl_actuator_msgs::MotionControl mc;
        double destTol = 100.0;
        auto ballPos = wm->ball->getPositionEgo();
        if (dest.length() < destTol)
        {
            // only align to point
            MovementQuery query;
            query.egoDestinationPoint = dest;
            query.egoAlignPoint = ballPos;
            query.additionalPoints = nullopt;

            mc = moveToPoint(query);
            mc.motion.translation = 0;

            return mc;
        }
        else
        {
            if (wm->ball->getPositionAllo())
            {
                auto additionalPoints = make_optional<vector<geometry::CNPointAllo>>();
                additionalPoints->push_back(*wm->ball->getPositionAllo());

                MovementQuery query;
                query.egoDestinationPoint = dest;
                query.egoAlignPoint = ballPos;
                query.additionalPoints = additionalPoints;

                mc = moveToPoint(query);
            }
            else
            {
                MovementQuery query;
                query.egoDestinationPoint = make_optional<geometry::CNPointEgo>(dest);
                query.additionalPoints = nullopt;
                query.egoAlignPoint = headingPoint;

                if (!headingPoint)
                {
                    query.egoAlignPoint = make_optional<geometry::CNPointEgo>(dest);
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
        if (randomCounter == 0)
        {
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_real_distribution<> dis(0, 1);
            double ang = (dis(gen) - 0.5) * 2 * M_PI;
            randomTarget = make_optional<geometry::CNPointEgo>(cos(ang) * 5000, sin(ang) * 5000);
        }

        auto dest = pp->getEgoDirection(*randomTarget, PathEvaluator(), PathPlannerQuery());

        if (!dest)
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
    msl_actuator_msgs::MotionControl RobotMovement::moveToFreeSpace(MovementQuery& query)
    {
        auto teamMatePosition = query.alloTeamMatePosition;
        msl_actuator_msgs::MotionControl mc;

        auto ownPos = wm->rawSensorData->getOwnPositionVisionBuffer().getLastValidContent();
        if (!ownPos)
        {
            return setNAN();
        }

        auto ops = wm->obstacles->getRawObstaclesAlloBuffer().getLastValidContent(); // WM.GetTrackedOpponents();
        fringe->clear();
        for (int i = 0; i < 16; i++)
        {
            for (int d = 0; d < 8000; d += 2000)
            {
                shared_ptr<AlloSearchArea> s = AlloSearchArea::getAnArea(i * M_PI / 8, (i + 1) * M_PI / 8, d, d + 2000,
                                                                         ownPos->getPoint(), *ownPos);
                if (s->isValid())
                {

                    s->val = evalPointDynamic(s->midP, *teamMatePosition, *ownPos, **ops);
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
                next->at(j)->val = evalPointDynamic(next->at(j)->midP, *teamMatePosition, *ownPos,
                                                    **wm->obstacles->getRawObstaclesAlloBuffer().getLastValidContent());
                fringe->push_back(next->at(j));
            }
            stable_sort(fringe->begin(), fringe->end(), SearchArea::compareTo);
        }
        auto dest = wm->field->mapOutOfOppGoalArea(wm->field->mapInsideField(best->midP)).toEgo(*ownPos);
        auto align = teamMatePosition->toEgo(*ownPos);

        //		mc = placeRobotAggressive(dest, align, maxTrans);

        MovementQuery q;
        q.egoDestinationPoint = make_optional<geometry::CNPointEgo>(dest);
        q.egoAlignPoint = make_optional<geometry::CNPointEgo>(align);
        mc = moveToPoint(q);
#ifdef RM_DEBUG
        cout << "RobotMovementmoveToFreeSpace: Angle = " << mc.motion.angle << " Trans = " << mc.motion.translation << " Rot = " << mc.motion.rotation << endl;
#endif
        return mc;
    }

    /*
     * Used in moveToFreeSpace()
     */
    double RobotMovement::evalPointDynamic(geometry::CNPointAllo alloP, geometry::CNPointAllo alloPassee,
                                           geometry::CNPositionAllo ownPos,
                                           const vector<geometry::CNPointAllo>& opponents)
    {
        double ret = 0;

        // distance to point:
        ret -= ownPos.distanceTo(alloP) / 10.0;
        MSLWorldModel *wm = MSLWorldModel::get();
        auto oppGoalMid = geometry::CNPointAllo(wm->field->getFieldLength() / 2, 0);

        auto passee2p = alloP - alloPassee;
        // if (passee2p.X < 0 && Math.Abs(alloP.Y) < 1000) return Double.MinValue;

        auto passee2LeftOwnGoal = geometry::CNVecAllo(
                -wm->field->getFieldLength() / 2.0 - alloPassee.x,
                wm->field->getGoalWidth() / 2.0 + 1000 - alloPassee.y);
        auto passee2RightOwnGoal = geometry::CNVecAllo(
                -wm->field->getFieldLength() / 2.0 - alloPassee.x,
                -wm->field->getGoalWidth() / 2.0 - 1000 - alloPassee.y);

        if (!geometry::leftOf(passee2LeftOwnGoal, passee2p) && geometry::leftOf(passee2RightOwnGoal, passee2p))
        {
            return numeric_limits<double>::min();
        }
        if (wm->field->isInsideOppPenalty(alloPassee, 800) && wm->field->isInsideOppPenalty(alloP, 600))
        {
            return numeric_limits<double>::min();
        }

        ret += passee2p.x / 9.0;
        if (alloP.x < 0)
        {
            ret += alloP.x / 10.0;
        }

        if (alloP.y * alloPassee.y < 0)
        {
            ret += min(0.0, alloP.x);
        }
        if (alloP.angleZToPoint(oppGoalMid) > M_PI / 3.0)
        {
            ret -= alloP.angleZToPoint(oppGoalMid) * 250.0;
        }
        // distance to passeee:

        // Point2D p2GoalVec = goalPos - p;
        double dist2Passee = passee2p.length();

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

        for (int i = 0; i < opponents.size(); i++)
        {
            // pass corridor
            t = ((opponents.at(i).x - alloPassee.x) * passee2p.x
                    + (opponents.at(i).y - alloPassee.y) * passee2p.y)
                    / (passee2p.x * passee2p.x + passee2p.y * passee2p.y);

            if (t > 0)
            {
                if (t < 1.0)
                {

                    ortX = opponents.at(i).x - (alloPassee.x + t * (passee2p.x));
                    ortY = opponents.at(i).y - (alloPassee.y + t * (passee2p.y));
                    v = max(0.0, sqrt(ortX * ortX + ortY * ortY) - robotRadius) / dist2Passee;

                    if (v / t * interceptQuotient < 1)
                    {

                        double cf = 5000 * (1 - ((v / t) * interceptQuotient));
                        catchFactor = max(catchFactor, cf);
                    }
                }
                ortX = opponents.at(i).x - alloP.x;
                ortY = opponents.at(i).y - alloP.y;
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
    }
}
