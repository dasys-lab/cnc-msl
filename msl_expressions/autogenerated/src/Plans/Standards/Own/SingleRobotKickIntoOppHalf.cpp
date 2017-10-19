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
using geometry::CNVecEgo;
using geometry::CNPointAllo;
using geometry::CNPointEgo;
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

        /**
         * wtf is going on with this behaviour? it was totally broken before NiceGeom, doesn't do what the name suggests and
         * makes little sense in general. looks like a bunch of random stuff copy-pasted from other behaviours
         * don't drink and code
         */

        auto ownPos = wm->rawSensorData->getOwnPositionVisionBuffer().getLastValidContent(); // actually ownPosition corrected
        auto egoBallPos = wm->ball->getPositionEgo();
        // return if necessary information is missing
        if (!ownPos || !egoBallPos)
        {
            return;
        }
        nonstd::optional<geometry::CNPointAllo> alloTarget = nonstd::nullopt;
        auto alloBall = egoBallPos->toAllo(*ownPos);

        CNPointAllo passPoint;


        //ball left, aim left
        if (alloBall.y > 0)
        {
            passPoint = CNPointAllo(wm->field->posLeftOppRestartMarker().x,
                                    -wm->field->posLeftOppRestartMarker().y + 1000.0);
        }
        else // ball right, aim right
        {
            passPoint = CNPointAllo(wm->field->posRightOppRestartMarker().x,
                                                         -wm->field->getFieldWidth() / 2 + 1000.0);
        }

        auto aimPoint = passPoint.toEgo(*ownPos);

        double aimAngle = aimPoint.angleZ();
        double ballAngle = egoBallPos->angleZ();
        double deltaAngle = geometry::deltaAngle(ballAngle, aimAngle);

        if (abs(deltaAngle) < M_PI / 36)
        { // +/-5 degree
          //Kick && PassMsg
          // Distance to aim point * direction of our kicker = actual pass point destination
            double dist = aimPoint.length();
            auto dest = CNPointEgo(-dist, 0);
            auto alloDest = dest.toAllo(*ownPos);

            msl_actuator_msgs::KickControl km;
            km.enabled = true;
            km.kicker = 1; //(ushort)KickHelper.KickerToUseIndex(egoBallPos->angleTo());

            double v0 = 0;
            //considering network delay and reaction time 1s?:
            km.power = (ushort)this->robot->kicker->getKickPowerPass(aimPoint.length());

            send(km);

        }

        auto ballVel = this->wm->ball->getVisionBallVelocityBuffer().getLastValidContent();
        CNVecEgo ballVel2;
        if (!ballVel)
        {
            ballVel2 = geometry::CNVecEgo(0, 0);
        }
        else if (ballVel->length() > 5000)
        {
            auto v = ballVel->normalize() * 5000;
            ballVel2 = geometry::CNVecEgo(v.x, v.y);
        }
        else
        {
            ballVel2 = geometry::CNVecEgo(ballVel->x,ballVel->y);
        }

        msl_actuator_msgs::MotionControl mc;
        mc = msl_actuator_msgs::MotionControl();
        mc.motion.rotation = deltaAngle * pRot + (deltaAngle - lastRotError) * dRot;
        double sign = geometry::sgn(mc.motion.rotation);
        mc.motion.rotation = sign * min(this->maxRot, max(abs(mc.motion.rotation), this->minRot));
        lastRotError = deltaAngle;
        double transBallOrth = egoBallPos->length() * mc.motion.rotation; //may be negative!
        double transBallTo = min(1000.0, ballVel2.length()); //Math.Max(ballPos.Distance(),ballVel2.Distance());
        auto driveTo = egoBallPos->rotateZ(-M_PI / 2.0);
        driveTo = driveTo.normalize() * transBallOrth;
        driveTo = driveTo + egoBallPos->normalize() * transBallTo;
        if (driveTo.length() > maxVel)
        {
            driveTo = driveTo.normalize() * maxVel;
        }
        mc.motion.angle = driveTo.angleZ();
        mc.motion.translation = driveTo.length();

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
