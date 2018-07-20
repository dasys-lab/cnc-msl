using namespace std;
#include "Plans/Standards/Own/Penalty/PenaltyShoot.h"

/*PROTECTED REGION ID(inccpp1466940246275) ENABLED START*/ //Add additional includes here
#include "msl_robot/robotmovement/RobotMovement.h"
#include <RawSensorData.h>
#include <Ball.h>
#include <obstaclehandler/Obstacles.h>
#include <msl_robot/MSLRobot.h>
#include <msl_robot/kicker/Kicker.h>
#include <Game.h>
#include <MSLWorldModel.h>
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1466940246275) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    PenaltyShoot::PenaltyShoot() :
            DomainBehaviour("PenaltyShoot")
    {
        /*PROTECTED REGION ID(con1466940246275) ENABLED START*/ //Add additional options here
        lastAlignment = 0;
        // for alignToPointWithBall
        lastRotError = 0;
//		lastRotErrorWithBall = 0;
        readConfigParameters();

        /*PROTECTED REGION END*/
    }
    PenaltyShoot::~PenaltyShoot()
    {
        /*PROTECTED REGION ID(dcon1466940246275) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void PenaltyShoot::run(void* msg)
    {
        /*PROTECTED REGION ID(run1466940246275) ENABLED START*/ //Add additional options here
        shared_ptr < geometry::CNPosition > ownPos = wm->rawSensorData->getOwnPositionVision(); // actually ownPosition corrected
        shared_ptr < geometry::CNPoint2D > egoBallPos = wm->ball->getEgoBallPosition();

        // return if necessary information is missing
        if (ownPos == nullptr || egoBallPos == nullptr)
        {
            return;
        }

        //Constant ball handle wheel speed
        BallHandleCmd bhc;
        bhc.leftMotor = (int8_t)this->wheelSpeed;
        bhc.rightMotor = (int8_t)this->wheelSpeed;
        send(bhc);
        // Create ego-centric 2D target...
        shared_ptr < geometry::CNPoint2D > egoTarget = nullptr;
        // Create target point next to left/right opp goal post
        shared_ptr < geometry::CNPoint2D > alloLeftAimPoint = make_shared < geometry::CNPoint2D
                > ((wm->field->getFieldLength() / 2) + ballDiameter, (goalWidth / 2) - (aimOffset * ballDiameter));
        shared_ptr < geometry::CNPoint2D > alloRightAimPoint = make_shared < geometry::CNPoint2D
                > ((wm->field->getFieldLength() / 2) + ballDiameter, -(goalWidth / 2) + (aimOffset * ballDiameter));

        // Create points for rectangle check
        shared_ptr < geometry::CNPoint2D > frontLeft = make_shared < geometry::CNPoint2D
                > ((wm->field->getFieldLength() - (4 * robotRadius)) / 2, goalWidth / 2);
        shared_ptr < geometry::CNPoint2D > frontRight = make_shared < geometry::CNPoint2D
                > ((wm->field->getFieldLength() - (4 * robotRadius)) / 2, -goalWidth / 2);

        // Create back point according to last alignment
        shared_ptr < geometry::CNPoint2D > back = nullptr;

        // Hysteresis
        if (lastAlignment == 0) // not aligned before (default)
        {
            back = make_shared < geometry::CNPoint2D > ((wm->field->getFieldLength() / 2) + robotRadius, 0);
        }
        else if (lastAlignment == 1) // last alignment left
        {
            back = make_shared < geometry::CNPoint2D > ((wm->field->getFieldLength() / 2) + robotRadius, robotRadius);
        }
        else // last alignment right
        {
            back = make_shared < geometry::CNPoint2D > ((wm->field->getFieldLength() / 2) + robotRadius, -robotRadius);
        }

        int counter = 0;

        shared_ptr < vector<shared_ptr<geometry::CNPoint2D> > > alloOpps = nullptr;

        for (int i = 0; i < wm->getRingBufferLength(); i++)
        {
            alloOpps = wm->obstacles->getAlloObstaclePoints(i);
            if (alloOpps != nullptr)
            {
                // weighted analysis of past and current obstacles
                for (auto alloObs : *alloOpps)
                {
                    // if obstacle is inside rectangle, increase counter by inverse age (10 = newest, 1 = oldest)
                    if (geometry::isInsideRectangle(frontLeft, back, alloObs))
                    {
                        cout << "Found obstacle left" << endl;
                        counter += wm->getRingBufferLength() - i;
                    }
                    // if obstacle is inside rectangle, decrease counter by inverse age (10 = newest, 1 = oldest)
                    if (geometry::isInsideRectangle(frontRight, back, alloObs))
                    {
                        cout << "Found obstacle right" << endl;
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
            cout << "PenaltyBeh: left!" << endl;
            egoTarget = alloLeftAimPoint->alloToEgo(*ownPos);
        }
        // if counter > 0, there are obstacles on the left side, so we aim right
        else
        {
            lastAlignment = 2;
            cout << "PenaltyBeh: right!" << endl;
            egoTarget = alloRightAimPoint->alloToEgo(*ownPos);
        }
        // calculate angle difference between robot and target and ball and target
        double egoTargetAngle = egoTarget->angleTo();
        double deltaHoleAngle = geometry::deltaAngle(this->robot->kicker->kickerAngle, egoTargetAngle);
        // calculate passed time
        unsigned long timePassed = wm->getTime() - wm->game->getTimeSinceStart();
        // if too much time has passed or the robot is aligned, we shoot
        if (timePassed >= waitBeforeBlindKick || fabs(deltaHoleAngle) < this->angleTolerance)
        {
            cout << "PenaltyShoot: Success!" << endl;
            KickControl kc;
            kc.enabled = true;
            kc.kicker = egoBallPos->angleTo();
            kc.power = kickPower;
            send(kc);
            this->setSuccess(true);
        }
        // Create Motion Command for aiming
        MotionControl mc = alignToPointWithBall(egoTarget, egoBallPos, this->angleTolerance, this->angleTolerance);
        send(mc);
        /*PROTECTED REGION END*/
    }
    void PenaltyShoot::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1466940246275) ENABLED START*/ //Add additional options here
        lastAlignment = 0;
        /*PROTECTED REGION END*/
    }
    /*PROTECTED REGION ID(methods1466940246275) ENABLED START*/ //Add additional methods here
    msl_actuator_msgs::MotionControl PenaltyShoot::alignToPointWithBall(shared_ptr<geometry::CNPoint2D> egoAlignPoint,
                                                                        shared_ptr<geometry::CNPoint2D> egoBallPos,
                                                                        double angleTolerance,
                                                                        double ballAngleTolerance)
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

            //lastRotErrorWithBall = deltaTargetAngle;
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

    void PenaltyShoot::readConfigParameters()
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
        goalWidth = wm->field->getGoalWidth();
        robotRadius = (*this->sc)["Rules"]->get<double>("Rules.RobotRadius", NULL);
        wheelSpeed = (*this->sc)["Penalty"]->get<double>("Penalty.WheelSpeed", NULL);
        aimOffset = (*this->sc)["Penalty"]->get<double>("Penalty.AimOffset", NULL);
        kickPower = (*this->sc)["Penalty"]->get<double>("Penalty.KickPower", NULL);
        timeForPenaltyShot = (*this->sc)["Rules"]->get<double>("Rules.Standards.InGamePenaltyRimeForShoot", NULL);
        waitBeforeBlindKick = (*this->sc)["Rules"]->get<double>("Rules.Standards.InGamePenaltyWaitBeforeBlindKick",
                                                                NULL);
    }
/*PROTECTED REGION END*/
} /* namespace alica */
