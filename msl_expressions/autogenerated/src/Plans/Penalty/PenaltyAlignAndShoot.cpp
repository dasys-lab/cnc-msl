using namespace std;
#include "Plans/Penalty/PenaltyAlignAndShoot.h"

/*PROTECTED REGION ID(inccpp1431531496053) ENABLED START*/ // Add additional includes here
#include "msl_robot/robotmovement/RobotMovement.h"

#include <Ball.h>
#include <Game.h>
#include <MSLWorldModel.h>
#include <RawSensorData.h>
#include <msl_robot/MSLRobot.h>
#include <msl_robot/kicker/Kicker.h>
#include <obstaclehandler/Obstacles.h>
/*PROTECTED REGION END*/
namespace alica
{
/*PROTECTED REGION ID(staticVars1431531496053) ENABLED START*/ // initialise static variables here
/*PROTECTED REGION END*/
PenaltyAlignAndShoot::PenaltyAlignAndShoot()
    : DomainBehaviour("PenaltyAlignAndShoot")
{
    /*PROTECTED REGION ID(con1431531496053) ENABLED START*/ // Add additional options here
    lastAlignment = 0;
    waitBeforeBlindKick = timeForPenaltyShot - 1000000000;

    // for alignToPointWithBall
    lastRotError = 0;
    //        lastRotErrorWithBall = 0;
    readConfigParameters();

    /*PROTECTED REGION END*/
}
PenaltyAlignAndShoot::~PenaltyAlignAndShoot()
{
    /*PROTECTED REGION ID(dcon1431531496053) ENABLED START*/ // Add additional options here
    /*PROTECTED REGION END*/
}
void PenaltyAlignAndShoot::run(void *msg)
{
    /*PROTECTED REGION ID(run1431531496053) ENABLED START*/ // Add additional options here
    auto ownPos =
        wm->rawSensorData->getOwnPositionVisionBuffer().getLastValidContent(); // actually ownPosition corrected
    auto egoBallPos = wm->ball->getPositionEgo();

    // return if necessary information is missing
    if (!ownPos || !egoBallPos)
    {
        return;
    }

    // Constant ball handle wheel speed
    BallHandleCmd bhc;
    bhc.leftMotor = (int8_t) this->wheelSpeed;
    bhc.rightMotor = (int8_t) this->wheelSpeed;
    send(bhc);

    // Create target point next to left/right opp goal post
    auto alloLeftAimPoint = geometry::CNPointAllo(wm->field->getFieldLength() / 2 + ballDiameter,
                                                  goalLineLength / 2 - aimOffset * ballDiameter);
    auto alloRightAimPoint = geometry::CNPointAllo(wm->field->getFieldLength() / 2 + ballDiameter,
                                                   -goalLineLength / 2 + aimOffset * ballDiameter);

    // Create points for rectangle check
    auto frontLeft = geometry::CNPointAllo((wm->field->getFieldLength() - (4 * robotRadius)) / 2, goalLineLength / 2);
    auto frontRight = geometry::CNPointAllo((wm->field->getFieldLength() - (4 * robotRadius)) / 2, -goalLineLength / 2);

    // Create back point according to last alignment
    geometry::CNPointAllo back;

    // Hysteresis
    if (lastAlignment == 0) // not aligned before (default)
    {
        back = geometry::CNPointAllo(wm->field->getFieldLength() / 2 + robotRadius, 0);
    }
    else if (lastAlignment == 1) // last alignment left
    {
        back = geometry::CNPointAllo(wm->field->getFieldLength() / 2 + robotRadius, robotRadius);
    }
    else // last alignment right
    {
        back = geometry::CNPointAllo(wm->field->getFieldLength() / 2 + robotRadius, -robotRadius);
    }

    int counter = 0;

    for (int i = 0; i < wm->getRingBufferLength(); i++)
    {
        auto alloOpps = wm->obstacles->getRawObstaclesAlloBuffer().getLast(i);
        if (alloOpps != nullptr)
        {
            // weighted analysis of past and current obstacles
            for (auto alloObs : *alloOpps->getInformation())
            {
                // if obstacle is inside rectangle, increase counter by inverse age (10 = newest, 1 = oldest)
                if (geometry::isInsideRectangle(frontLeft, back, alloObs))
                {
                    counter += wm->getRingBufferLength() - i;
                }
                // if obstacle is inside rectangle, decrease counter by inverse age (10 = newest, 1 = oldest)
                if (geometry::isInsideRectangle(frontRight, back, alloObs))
                {
                    counter -= wm->getRingBufferLength() - i;
                }
            }
        }
        else
        {
            cout << "PenaltyBeh: no obstacles!" << endl;
        }
    }

    geometry::CNPointEgo egoTarget;
    // if counter <= 0, there are obstacles on the right side, so we aim left
    if (counter <= 0)
    {
        lastAlignment = 1;
        cout << "PenaltyBeh: left!" << endl;
        egoTarget = alloLeftAimPoint.toEgo(*ownPos);
    }
    // if counter > 0, there are obstacles on the left side, so we aim right
    else
    {
        lastAlignment = 2;
        cout << "PenaltyBeh: right!" << endl;
        egoTarget = alloRightAimPoint.toEgo(*ownPos);
    }
    // calculate angle difference between robot and target and ball and target
    double egoTargetAngle = egoTarget.angleZ();
    double deltaHoleAngle = geometry::deltaAngle(this->robot->kicker->kickerAngle, egoTargetAngle);
    // calculate passed time
    unsigned long timePassed = wm->getTime() - wm->game->getTimeSinceStart();
    // if too much time has passed or the robot is aligned, we shoot
    if (timePassed >= waitBeforeBlindKick || fabs(deltaHoleAngle) < this->angleTolerance)
    {
        cout << "PenaltyBeh: Success!" << endl;
        KickControl kc;
        kc.enabled = true;
        kc.kicker = egoBallPos->angleZ();
        kc.power = kickPower;
        send(kc);
        this->setSuccess(true);
    }
    // Create Motion Command for aiming
    MotionControl mc = alignToPointWithBall(egoTarget, *egoBallPos, this->angleTolerance, this->angleTolerance);
    send(mc);
    /*PROTECTED REGION END*/
}
void PenaltyAlignAndShoot::initialiseParameters()
{
    /*PROTECTED REGION ID(initialiseParameters1431531496053) ENABLED START*/ // Add additional options here
    lastAlignment = 0;
    /*PROTECTED REGION END*/
}
/*PROTECTED REGION ID(methods1431531496053) ENABLED START*/ // Add additional methods here
msl_actuator_msgs::MotionControl PenaltyAlignAndShoot::alignToPointWithBall(geometry::CNPointEgo egoAlignPoint,
                                                                            geometry::CNPointEgo egoBallPos,
                                                                            double angleTolerance,
                                                                            double ballAngleTolerance)
{
    msl_actuator_msgs::MotionControl mc;
    double egoTargetAngle = egoAlignPoint.angleZ();
    double egoBallAngle = egoBallPos.angleZ();
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
        mc.motion.rotation =
            -(deltaTargetAngle * defaultRotateP + (deltaTargetAngle - lastRotError) * alignToPointPRot);
        mc.motion.rotation = (mc.motion.rotation < 0 ? -1 : 1) *
                             min(alignToPointMaxRotation, max(fabs(mc.motion.rotation), alignToPointMinRotation));

        lastRotError = deltaTargetAngle;

        // crate the motion orthogonal to the ball
        geometry::CNPointEgo driveTo = egoBallPos.rotateZ(-M_PI / 2.0);
        driveTo = driveTo * mc.motion.rotation;

        // add the motion towards the ball
        driveTo = driveTo + egoBallPos.normalize() * 10;

        mc.motion.angle = driveTo.angleZ();
        mc.motion.translation = min(alignMaxVel, driveTo.length());
    }
    return mc;
}

void PenaltyAlignAndShoot::readConfigParameters()
{
    defaultRotateP = (*sc)["Drive"]->get<double>("Drive.Default.RotateP", NULL);
    alignToPointPRot = (*sc)["Drive"]->get<double>("Drive", "AlignToPointpRot", NULL);
    alignToPointMaxRotation = (*sc)["Drive"]->get<double>("Drive", "AlignToPointMaxRotation", NULL);
    alignToPointMinRotation = (*sc)["Drive"]->get<double>("Drive", "AlignToPointMinRotation", NULL);
    alignMaxVel = (*sc)["Drive"]->get<double>("Drive", "MaxSpeed", NULL);
    maxVel = (*this->sc)["Penalty"]->get<double>("Penalty.MaxSpeed", NULL);
    // Aiming/Rotation Stuff
    angleTolerance = (*this->sc)["Penalty"]->get<double>("Penalty.AngleTolerance", NULL);
    ballDiameter = (*this->sc)["Rules"]->get<double>("Rules.BallRadius", NULL) * 2;
    goalLineLength = wm->field->getGoalWidth();
    robotRadius = (*this->sc)["Rules"]->get<double>("Rules.RobotRadius", NULL);
    wheelSpeed = (*this->sc)["Penalty"]->get<double>("Penalty.WheelSpeed", NULL);
    aimOffset = (*this->sc)["Penalty"]->get<double>("Penalty.AimOffset", NULL);
    kickPower = (*this->sc)["Penalty"]->get<double>("Penalty.KickPower", NULL);
    timeForPenaltyShot = (*this->sc)["Rules"]->get<double>("Rules.Standards.PenaltyTimeForShot", NULL) * 1000000;
}

/*PROTECTED REGION END*/
} /* namespace alica */
