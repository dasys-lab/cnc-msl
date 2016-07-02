using namespace std;
#include "Plans/Standards/Own/FreeKick/AlignFreeGoalSpace.h"

/*PROTECTED REGION ID(inccpp1467039782450) ENABLED START*/ //Add additional includes here
//#include <msl_robot/robotmovement/MovementQuery.h>
#include "msl_robot/robotmovement/RobotMovement.h"
#include "SystemConfig.h"
#include <RawSensorData.h>
#include <Robots.h>
#include <Ball.h>
#include <MSLWorldModel.h>
#include <MSLFootballField.h>
#include <obstaclehandler/Obstacles.h>
#include <msl_robot/MSLRobot.h>
#include <msl_robot/kicker/Kicker.h>
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1467039782450) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    AlignFreeGoalSpace::AlignFreeGoalSpace() :
            DomainBehaviour("AlignFreeGoalSpace")
    {
        /*PROTECTED REGION ID(con1467039782450) ENABLED START*/ //Add additional options here
        lastAlignment = 0;
        // for alignToPointWithBall
        lastRotError = 0;
        /*PROTECTED REGION END*/
    }
    AlignFreeGoalSpace::~AlignFreeGoalSpace()
    {
        /*PROTECTED REGION ID(dcon1467039782450) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void AlignFreeGoalSpace::run(void* msg)
    {
        /*PROTECTED REGION ID(run1467039782450) ENABLED START*/ //Add additional options here
        shared_ptr < geometry::CNPosition > ownPos = wm->rawSensorData->getOwnPositionVision(); // actually ownPosition corrected
        shared_ptr < geometry::CNPoint2D > egoBallPos = wm->ball->getEgoBallPosition();

        // return if necessary information is missing
        if (ownPos == nullptr || egoBallPos == nullptr)
        {
            return;
        }

        //Constant ball handle wheel speed
//		BallHandleCmd bhc;
//		bhc.leftMotor = (int8_t)this->wheelSpeed;
//		bhc.rightMotor = (int8_t)this->wheelSpeed;
//		send(bhc);
        // Create ego-centric 2D target...
        shared_ptr < geometry::CNPoint2D > egoTarget = nullptr;
        // Create target point next to left/right opp goal post
        shared_ptr < geometry::CNPoint2D > alloLeftAimPoint = make_shared < geometry::CNPoint2D
                > (wm->field->getFieldLength() / 2 + ballDiameter, goalLineLength / 2 - ballDiameter);
        shared_ptr < geometry::CNPoint2D > alloRightAimPoint = make_shared < geometry::CNPoint2D
                > (wm->field->getFieldLength() / 2 + ballDiameter, -goalLineLength / 2 + ballDiameter);

        // Create points for rectangle check
        shared_ptr < geometry::CNPoint2D > frontLeft = make_shared < geometry::CNPoint2D
                > ((wm->field->getFieldLength() - (4 * robotRadius)) / 2, goalLineLength / 2);
        shared_ptr < geometry::CNPoint2D > frontRight = make_shared < geometry::CNPoint2D
                > ((wm->field->getFieldLength() - (4 * robotRadius)) / 2, -goalLineLength / 2);

        // Create back point according to last alignment
        shared_ptr < geometry::CNPoint2D > back = nullptr;

        // Hysteresis
        if (lastAlignment == 0) // not aligned before (default)
        {
            back = make_shared < geometry::CNPoint2D > (wm->field->getFieldLength() / 2 + robotRadius, 0);
        }
        else if (lastAlignment == 1) // last alignment left
        {
            back = make_shared < geometry::CNPoint2D > (wm->field->getFieldLength() / 2 + robotRadius, robotRadius);
        }
        else // last alignment right
        {
            back = make_shared < geometry::CNPoint2D > (wm->field->getFieldLength() / 2 + robotRadius, -robotRadius);
        }

        int counter = 0;

        shared_ptr < vector<shared_ptr<geometry::CNPoint2D> > > alloOpps = nullptr;

        for (int i = 0; i < wm->getRingBufferLength(); i++)
        {
            alloOpps = wm->robots->opponents.getOpponentsAlloClustered();
            if (alloOpps != nullptr)
            {
                // weighted analysis of past and current obstacles
                for (auto alloObs : *alloOpps)
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

        // if counter <= 0, there are obstacles on the right side, so we aim left
        if (counter <= 0)
        {
            lastAlignment = 1;
            cout << "AFGS: left!" << endl;
            egoTarget = alloLeftAimPoint->alloToEgo(*ownPos);
        }
        // if counter > 0, there are obstacles on the left side, so we aim right
        else
        {
            lastAlignment = 2;
            cout << "AFGS: right!" << endl;
            egoTarget = alloRightAimPoint->alloToEgo(*ownPos);
        }
        // calculate angle difference between robot and target and ball and target
        double egoTargetAngle = egoTarget->angleTo();
        double deltaHoleAngle = geometry::deltaAngle(this->robot->kicker->kickerAngle, egoTargetAngle);
        // Create Motion Command for aiming
        MotionControl mc = alignToPointWithBall(egoTarget, egoBallPos, this->angleTolerance, this->angleTolerance);
        send(mc);
        /*PROTECTED REGION END*/
    }
    void AlignFreeGoalSpace::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1467039782450) ENABLED START*/ //Add additional options here
        lastAlignment = 0;
        /*PROTECTED REGION END*/
    }
    /*PROTECTED REGION ID(methods1467039782450) ENABLED START*/ //Add additional methods here
    msl_actuator_msgs::MotionControl AlignFreeGoalSpace::alignToPointWithBall(
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

            lastRotError = deltaTargetAngle;

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
    void AlignFreeGoalSpace::readConfigParameters()
    {
        defaultRotateP = (*sc)["Drive"]->get<double>("Drive.Default.RotateP", NULL);
        alignToPointpRot = (*sc)["Drive"]->get<double>("Drive", "AlignToPointpRot", NULL);
        alignToPointMaxRotation = (*sc)["Drive"]->get<double>("Drive", "AlignToPointMaxRotation", NULL);
        alignToPointMinRotation = (*sc)["Drive"]->get<double>("Drive", "AlignToPointMinRotation", NULL);
        alignMaxVel = (*sc)["Drive"]->get<double>("Drive", "MaxSpeed", NULL);
        maxVel = (*this->sc)["Penalty"]->get<double>("Penalty.MaxSpeed", NULL);
        // Aiming/Rotation Stuff
        angleTolerance = (*this->sc)["Penalty"]->get<double>("Penalty.AngleTolerance", NULL);
        ballDiameter = (*this->sc)["Rules"]->get<double>("Rules.BallRadius", NULL) * 2;
        goalLineLength = wm->field->getGoalWidth();
        robotRadius = (*this->sc)["Rules"]->get<double>("Rules.RobotRadius", NULL);
//		wheelSpeed = (*this->sc)["Penalty"]->get<double>("Penalty.WheelSpeed", NULL);
        kickPower = (*this->sc)["Penalty"]->get<double>("Penalty.KickPower", NULL);
    }
/*PROTECTED REGION END*/
} /* namespace alica */
