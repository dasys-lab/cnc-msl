using namespace std;
#include "Plans/Standards/Own/SingleRobotKickIntoOppHalf.h"

/*PROTECTED REGION ID(inccpp1467436234548) ENABLED START*/ //Add additional includes here
#include "msl_robot/robotmovement/RobotMovement.h"
#include <RawSensorData.h>
#include <Ball.h>
#include <Robots.h>
#include <pathplanner/PathPlanner.h>
#include <msl_robot/kicker/Kicker.h>
#include <msl_robot/MSLRobot.h>
#include <MSLWorldModel.h>
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1467436234548) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    SingleRobotKickIntoOppHalf::SingleRobotKickIntoOppHalf() :
            DomainBehaviour("SingleRobotKickIntoOppHalf")
    {
        /*PROTECTED REGION ID(con1467436234548) ENABLED START*/ //Add additional options here
        this->pRot = 2.1;
        this->dRot = 0.0;
        this->lastRotError = 0;
        this->minRot = 0.1;
        this->maxRot = M_PI * 4;
        this->maxVel = 2000;
        /*PROTECTED REGION END*/
    }
    SingleRobotKickIntoOppHalf::~SingleRobotKickIntoOppHalf()
    {
        /*PROTECTED REGION ID(dcon1467436234548) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void SingleRobotKickIntoOppHalf::run(void* msg)
    {
        /*PROTECTED REGION ID(run1467436234548) ENABLED START*/ //Add additional options here
        shared_ptr < geometry::CNPosition > ownPos = wm->rawSensorData->getOwnPositionVision(); // actually ownPosition corrected
        shared_ptr < geometry::CNPoint2D > egoBallPos = wm->ball->getEgoBallPosition();
        // return if necessary information is missing
        if (ownPos == nullptr || egoBallPos == nullptr)
        {
            return;
        }
        shared_ptr < geometry::CNPoint2D > alloTarget = nullptr;
        shared_ptr < geometry::CNPoint2D > alloBall = egoBallPos->egoToAllo(*ownPos);
        shared_ptr < geometry::CNPoint2D > passPoint = nullptr;
        shared_ptr < geometry::CNPoint2D > aimPoint = passPoint->alloToEgo(*ownPos);

        //ball left, aim left
        if (alloBall->y > 0)
        {
            passPoint = make_shared < geometry::CNPoint2D
                    > (wm->field->posLeftOppRestartMarker()->x, -wm->field->posLeftOppRestartMarker()->y + 1000.0);
        }
        else // ball right, aim right
        {
            passPoint = make_shared < geometry::CNPoint2D
                    > (wm->field->posRightOppRestartMarker()->x, -wm->field->getFieldWidth() / 2 + 1000.0);
        }

        double aimAngle = aimPoint->angleTo();
        double ballAngle = egoBallPos->angleTo();
        double deltaAngle = geometry::deltaAngle(ballAngle, aimAngle);

        if (abs(deltaAngle) < M_PI / 36)
        { // +/-5 degree
          //Kick && PassMsg
          // Distance to aim point * direction of our kicker = actual pass point destination
            double dist = aimPoint->length();
            shared_ptr < geometry::CNPoint2D > dest = make_shared < geometry::CNPoint2D > (-dist, 0);
            dest = dest->egoToAllo(*ownPos);

            msl_actuator_msgs::KickControl km;
            km.enabled = true;
            km.kicker = 1; //(ushort)KickHelper.KickerToUseIndex(egoBallPos->angleTo());

            shared_ptr < geometry::CNPoint2D > goalReceiverVec = dest - make_shared < geometry::CNPoint2D
                    > (alloTarget->x, alloTarget->y);
            double v0 = 0;
            //considering network delay and reaction time 1s?:
            km.power = (ushort)this->robot->kicker->getKickPowerPass(aimPoint->length());

            send(km);

        }

        shared_ptr < geometry::CNVelocity2D > ballVel = this->wm->ball->getVisionBallVelocity();
        shared_ptr < geometry::CNPoint2D > ballVel2;
        if (ballVel == nullptr)
        {
            ballVel2 = make_shared < geometry::CNPoint2D > (0, 0);
        }
        else if (ballVel->length() > 5000)
        {
            shared_ptr < geometry::CNVelocity2D > v = ballVel->normalize() * 5000;
            ballVel2 = make_shared < geometry::CNPoint2D > (v->x, v->y);
        }
        else
        {
            ballVel2 = make_shared < geometry::CNPoint2D > (ballVel->x, ballVel->y);
        }

        msl_actuator_msgs::MotionControl mc;
        mc = msl_actuator_msgs::MotionControl();
        mc.motion.rotation = deltaAngle * pRot + (deltaAngle - lastRotError) * dRot;
        double sign = geometry::sgn(mc.motion.rotation);
        mc.motion.rotation = sign * min(this->maxRot, max(abs(mc.motion.rotation), this->minRot));
        lastRotError = deltaAngle;
        double transBallOrth = egoBallPos->length() * mc.motion.rotation; //may be negative!
        double transBallTo = min(1000.0, ballVel2->length()); //Math.Max(ballPos.Distance(),ballVel2.Distance());
        shared_ptr < geometry::CNPoint2D > driveTo = egoBallPos->rotate(-M_PI / 2.0);
        driveTo = driveTo->normalize() * transBallOrth;
        driveTo = driveTo + egoBallPos->normalize() * transBallTo;
        if (driveTo->length() > maxVel)
        {
            driveTo = driveTo->normalize() * maxVel;
        }
        mc.motion.angle = driveTo->angleTo();
        mc.motion.translation = driveTo->length();

        send(mc);
        /*PROTECTED REGION END*/
    }
    void SingleRobotKickIntoOppHalf::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1467436234548) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1467436234548) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
