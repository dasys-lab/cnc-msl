using namespace std;
#include "Plans/Behaviours/KickOffPassDefault.h"

/*PROTECTED REGION ID(inccpp1438778042140) ENABLED START*/ //Add additional includes here
#include "msl_robot/robotmovement/RobotMovement.h"
#include "engine/model/EntryPoint.h"
#include "engine/RunningPlan.h"
#include "engine/Assignment.h"
#include "engine/model/Plan.h"
#include <RawSensorData.h>
#include <Ball.h>
#include <Robots.h>
#include <Game.h>
#include <msl_robot/kicker/Kicker.h>
#include <msl_helper_msgs/PassMsg.h>
#include <MSLWorldModel.h>
#include <msl_robot/MSLRobot.h>
#include <msl_robot/kicker/Kicker.h>
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1438778042140) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    KickOffPassDefault::KickOffPassDefault() :
            DomainBehaviour("KickOffPassDefault")
    {
        /*PROTECTED REGION ID(con1438778042140) ENABLED START*/ //Add additional options here
        // for alignToPointWithBall
        lastRotError = 0;
        lastRotErrorWithBall = 0;
        readConfigParameters();
        /*PROTECTED REGION END*/
    }
    KickOffPassDefault::~KickOffPassDefault()
    {
        /*PROTECTED REGION ID(dcon1438778042140) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void KickOffPassDefault::run(void* msg)
    {
        /*PROTECTED REGION ID(run1438778042140) ENABLED START*/ //Add additional options here
        shared_ptr < geometry::CNPosition > ownPos = wm->rawSensorData->getOwnPositionVision();
        shared_ptr < geometry::CNPoint2D > egoBallPos = wm->ball->getEgoBallPosition();

        if (ownPos == nullptr || egoBallPos == nullptr)
        {
            return;
        }

        shared_ptr < geometry::CNPoint2D > egoAlignPoint = make_shared < geometry::CNPoint2D > (0, 0);
        EntryPoint* ep = getParentEntryPoint(taskName);
        shared_ptr < geometry::CNPosition > pos = nullptr;
        int id = -1;
        if (ep != nullptr)
        {
            auto parent = this->runningPlan->getParent().lock();
            if (parent == nullptr)
            {
                cout << "parent null" << endl;
                return;
            }
            shared_ptr<vector<int>> ids = parent->getAssignment()->getRobotsWorking(ep);
            id = ids->at(0);
            if (id != -1)
            {
                pos = wm->robots->teammates.getTeamMatePosition(id);

                //TODO soll etwas in gegnerhälfte schießen
                if (pos != nullptr)
                {
                    egoAlignPoint = pos->getPoint()->alloToEgo(*ownPos);
                }
                else
                {
                    shared_ptr < geometry::CNPoint2D > alloAlignPoint = make_shared < geometry::CNPoint2D > (0, 0);
                    egoAlignPoint = alloAlignPoint->alloToEgo(*ownPos);
                }
            }
        }
        else
        {
            shared_ptr < geometry::CNPoint2D > alloAlignPoint = make_shared < geometry::CNPoint2D > (0, 0);
            egoAlignPoint = alloAlignPoint->alloToEgo(*ownPos);
        }

        msl_actuator_msgs::MotionControl mc = alignToPointWithBall(egoAlignPoint, egoBallPos, 0.005, 0.075);

        double angleTolerance = 0;

        // function to increase angle tolerance (quadratically) according to time since start (min 5deg, max 25)
        angleTolerance = 0.2 * pow(wm->game->getTimeSinceStart(), 2) + 5;
        if (fabs(egoBallPos->rotate(M_PI)->angleTo()) > (M_PI / 180) * angleTolerance
                && wm->game->getTimeSinceStart() < waitBeforeBlindKick)
        {
            send(mc);
        }
        else
        {
            msl_actuator_msgs::KickControl kc;
            kc.enabled = true;
            kc.kicker = 1;
            if (wm->game->getTimeSinceStart() < waitBeforeBlindKick)
            {
                kc.power = this->robot->kicker->getPassKickpower(egoAlignPoint->length(), 1);
            }
            else
            {
                kc.power = this->robot->kicker->getKickPowerSlowPass(egoAlignPoint->length());
            }

            send(kc);

            msl_helper_msgs::PassMsg pm;
            pm.validFor = 2000000000ul;

            auto dest = make_shared < geometry::CNPoint2D > (-1, 0);
            dest = dest * egoAlignPoint->length();
            dest = dest->egoToAllo(*ownPos);

            pm.destination.x = dest->x;
            pm.destination.y = dest->y;

            pm.origin.x = ownPos->x;
            pm.origin.y = ownPos->y;
            pm.receiverID = id;
            send(pm);

            this->setSuccess(true);
        }
        /*PROTECTED REGION END*/
    }
    void KickOffPassDefault::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1438778042140) ENABLED START*/ //Add additional options here
        supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();
        timeForPass = (*sc)["Rules"]->get<double>("Rules.Standards.PenaltyTimeForShot", NULL) * 1000000;
        waitBeforeBlindKick = timeForPass - 1000000000;
        string tmp;
        bool success = true;
        try
        {
            success &= getParameter("TaskName", tmp);
            if (success)
            {
                taskName = tmp;
            }
        }
        catch (exception& e)
        {
            cerr << "Could not cast the parameter properly" << endl;
        }
        if (!success)
        {
            cerr << "StandardPass: Parameter does not exist" << endl;
        }
        /*PROTECTED REGION END*/
    }
    /*PROTECTED REGION ID(methods1438778042140) ENABLED START*/ //Add additional methods here
    msl_actuator_msgs::MotionControl KickOffPassDefault::alignToPointWithBall(
            shared_ptr<geometry::CNPoint2D> egoAlignPoint, shared_ptr<geometry::CNPoint2D> egoBallPos,
            double angleTolerance, double ballAngleTolerance)
    {
        msl_actuator_msgs::MotionControl mc;
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
            shared_ptr < geometry::CNPoint2D > driveTo = egoBallPos->rotate(-M_PI / 2.0);
            driveTo = driveTo * mc.motion.rotation;

            // add the motion towards the ball
            driveTo = driveTo + egoBallPos->normalize() * 10;

            mc.motion.angle = driveTo->angleTo();
            mc.motion.translation = min(alignMaxVel, driveTo->length());
        }
        return mc;
    }

    void KickOffPassDefault::readConfigParameters()
    {
        defaultRotateP = (*sc)["Drive"]->get<double>("Drive.Default.RotateP", NULL);
        alignToPointpRot = (*sc)["Drive"]->get<double>("Drive", "AlignToPointpRot", NULL);
        alignToPointMaxRotation = (*sc)["Drive"]->get<double>("Drive", "AlignToPointMaxRotation", NULL);
        alignToPointMinRotation = (*sc)["Drive"]->get<double>("Drive", "AlignToPointMinRotation", NULL);
        alignMaxVel = (*sc)["Drive"]->get<double>("Drive", "MaxSpeed", NULL);
    }
/*PROTECTED REGION END*/
} /* namespace alica */
