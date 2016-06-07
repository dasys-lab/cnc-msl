using namespace std;
#include "Plans/Behaviours/DefendGoal.h"

/*PROTECTED REGION ID(inccpp1459249294699) ENABLED START*/ //Add additional includes here
#include <Ball.h>
#include <container/CNPoint2D.h>
#include <container/CNPosition.h>
#include <msl_actuator_msgs/MotionControl.h>
#include <MSLFootballField.h>
#include <MSLWorldModel.h>
#include <robotmovement/RobotMovement.h>
#include <RawSensorData.h>
#include <exception>
#include <iostream>
#include <memory>
#include <string>
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1459249294699) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    DefendGoal::DefendGoal() :
            DomainBehaviour("DefendGoal")
    {
        /*PROTECTED REGION ID(con1459249294699) ENABLED START*/ //Add additional options here
        postOffset = 550.0;
        fieldOffset = 500.0;
        ownPosAngleMin = 2.2;
        query = make_shared<msl::MovementQuery>();
        /*PROTECTED REGION END*/
    }
    DefendGoal::~DefendGoal()
    {
        /*PROTECTED REGION ID(dcon1459249294699) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void DefendGoal::run(void* msg)
    {
        /*PROTECTED REGION ID(run1459249294699) ENABLED START*/ //Add additional options here
        msl::RobotMovement rm;

        auto ownPos = wm->rawSensorData->getOwnPositionVision();
        //List<TrackedOpponent> opponents = WM.GetTrackedOpponents();
        msl_actuator_msgs::MotionControl mc;
        shared_ptr < geometry::CNPoint2D > destAllo = make_shared<geometry::CNPoint2D>();
        shared_ptr < geometry::CNPoint2D > destEgo = make_shared<geometry::CNPoint2D>();

        destAllo->y = 0;
        destAllo->x = -wm->field->getFieldLength() / 2.0 + fieldOffset;

        if (ownPos == nullptr)
        {
            send(mc);
            return;
        }

        destEgo = destAllo->alloToEgo(*ownPos);
        if (ownPos->x > -wm->field->getFieldLength() / 2.0 + wm->field->getPenaltyAreaLength())
        {

            auto alignPoint = make_shared<geometry::CNPoint2D>()->alloToEgo(*ownPos);
            // attention: use care obstacles if the goalie sees the obstacles fine...

            // removed with new method using the Query-Object
//            mc = msl::RobotMovement::driveToPointAlignNoAvoidance(destEgo, alignPoint, 900, false);
            query->egoDestinationPoint = destEgo;
            query->egoAlignPoint = alignPoint;
            mc = rm.moveToPoint(query);
            mc.motion.translation = 900;

            //mc.Motion.Rotation = 0.0;
            //Console.WriteLine("wonPos.X : " + ownPos.X + " " + -field.FieldLength/2.0 + field.PenaltyAreaXSize);
            send(mc);
            return;
        }

        auto ballPos = wm->ball->getEgoBallPosition();

        if (ballPos == nullptr)
        {
            send(mc);
            return;
        }

        auto alloBall = ballPos->egoToAllo(*ownPos);

        // Predict Ball to Goal Line
        auto ballV3D = wm->ball->getBallVel3D();
        shared_ptr < geometry::CNPoint3D > ballV3DAllo = nullptr;
        bool hitPointFound = false;
        if (ballV3D != nullptr)
        {
            ballV3DAllo = ballV3D->egoToAllo(*ownPos);
            if (ballV3DAllo->x != 0.0)
            {
                double timeBallToGoal = (destAllo->x - alloBall->x) / ballV3DAllo->x;
                //Console.WriteLine(destAllo.X + " " + alloBall.X + " " + ballV3DAllo.Vx);
                if (timeBallToGoal > 0)
                {
                    //Console.WriteLine("here : " + alloBall.Y + " " + ballV3DAllo.Vy + " " + timeBallToGoal + " " + destAllo.Y);
                    hitPointFound = true;
                    destAllo->y = alloBall->y + ballV3DAllo->y * timeBallToGoal;
                }
            }
        }

        if (!hitPointFound)
        {
            destAllo->y = alloBall->y;
        }

        //Console.WriteLine (destAllo);
        destAllo = applyBoundaries4Pos(destAllo, postOffset);

        destEgo = destAllo->alloToEgo(*ownPos);
        if (destEgo->length() < wm->field->getPenaltyAreaLength())
        {
//			mc = DriveHelper.DriveToPointAndAlignCareObstacles(destEgo,ballPos, KeeperHelper.GetSpeed(destEgo),WM);
            mc = msl::RobotMovement::placeRobotCareBall(destEgo, ballPos, getSpeed(ballPos));
            //Console.WriteLine("Angle: " + mc.Motion.Angle + " Trans: " + mc.Motion.Translation + " Rotation: " + mc.Motion.Rotation);
        }
        else
        {
            // removed with new method using the Query-Object
//            mc = msl::RobotMovement::driveToPointAlignNoAvoidance(destEgo, ballPos, getSpeed(destEgo), false);
            query->egoDestinationPoint = destEgo;
            query->egoAlignPoint = ballPos;
            mc = rm.moveToPoint(query);
            mc.motion.translation = getSpeed(destEgo);

            cout << "DefendGoal: 2 - " << mc.motion.angle << " " << mc.motion.rotation << " " << mc.motion.translation
                    << endl;
            mc = applyBoundaries4Heading(mc, ownPos, ballPos, ownPosAngleMin);
            cout << "DefendGoal: 3 - " << mc.motion.angle << " " << mc.motion.rotation << " " << mc.motion.translation
                    << endl;
            //Console.WriteLine("DestAllo: " + destAllo.X + " " + destAllo.Y);
        }

        send(mc);
        /*PROTECTED REGION END*/
    }
    void DefendGoal::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1459249294699) ENABLED START*/ //Add additional options here
        bool success = true;
        string tmp;
        try
        {
            success &= getParameter("postOffset", tmp);
            postOffset = stod(tmp);
            success &= getParameter("positionOffset", tmp);
            fieldOffset = stod(tmp);
            success &= getParameter("ownPosAngle", tmp);
            ownPosAngleMin = stod(tmp);
        }
        catch (exception& e)
        {
            cerr << "Could not cast the parameter properly" << endl;
        }
        if (!success)
        {
            cerr << "DefendGoal: Parameter does not exist" << endl;
        }
        /*PROTECTED REGION END*/
    }
    /*PROTECTED REGION ID(methods1459249294699) ENABLED START*/ //Add additional methods here
    shared_ptr<geometry::CNPoint2D> DefendGoal::applyBoundaries4Pos(shared_ptr<geometry::CNPoint2D> dest,
                                                                    double postOffset)
    {
        shared_ptr < geometry::CNPoint2D > newP = make_shared < geometry::CNPoint2D > (*dest);
        //boundaries for position
        if (dest->y > (wm->field->posLeftOwnGoalPost()->y - postOffset))
            newP->y = (wm->field->posLeftOwnGoalPost()->y - postOffset);
        if (dest->y < (wm->field->posRightOwnGoalPost()->y + postOffset))
            newP->y = (wm->field->posRightOwnGoalPost()->y + postOffset);
        return newP;
    }

    double DefendGoal::getSpeed(shared_ptr<geometry::CNPoint2D> dest)
    {
        double speed = 0.0;
        double dist = dest->length();
        if (dist < 100)
        {
            speed = 0.0;
        }
        else if (dist < 500)
        {
            dist = dist / 500;
            speed = 900.0 * dist;
        }
        else
        {
            speed = 1400.0;
        }
        return speed;
    }
    msl_actuator_msgs::MotionControl DefendGoal::applyBoundaries4Heading(msl_actuator_msgs::MotionControl mc,
                                                                         shared_ptr<geometry::CNPosition> ownPos,
                                                                         shared_ptr<geometry::CNPoint2D> ballPos,
                                                                         double ownPosAngleMin)
    {
        //boundaries for angles
        mc.motion.rotation = msl::RobotMovement::alignToPointNoBall(ballPos, ballPos, 0.085).motion.rotation;
        mc.motion.angle = msl::RobotMovement::alignToPointNoBall(ballPos, ballPos, 0.085).motion.angle;

//		mc.motion.rotation = DriveHelper.GetRotation(ballPos,0.085,true);

        if (ownPos->theta > -ownPosAngleMin && mc.motion.rotation > 0 && ownPos->theta < 0)
        {
            mc.motion.rotation = 0;
        }
        else if (ownPos->theta < ownPosAngleMin && mc.motion.rotation < 0 && ownPos->theta > 0)
        {
            mc.motion.rotation = 0;
        }

        return mc;
    }
/*PROTECTED REGION END*/
} /* namespace alica */
