using namespace std;
#include "Plans/Penalty/PenaltyAlignAndShoot.h"

/*PROTECTED REGION ID(inccpp1431531496053) ENABLED START*/ //Add additional includes here
#include "robotmovement/RobotMovement.h"
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1431531496053) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    PenaltyAlignAndShoot::PenaltyAlignAndShoot() :
            DomainBehaviour("PenaltyAlignAndShoot")
    {
        /*PROTECTED REGION ID(con1431531496053) ENABLED START*/ //Add additional options here
        maxVel = (*this->sc)["Penalty"]->get<double>("Penalty.MaxSpeed", NULL);
        // Aiming/Rotation Stuff
        angleTolerance = (*this->sc)["Penalty"]->get<double>("Penalty.AngleTolerance", NULL);
        ballDiameter = (*this->sc)["Rules"]->get<double>("Rules.BallRadius", NULL) * 2;
        goalLineLength = (*this->sc)["Globals"]->get<double>("Globals.FootballField.GoalWidth", NULL);
        robotRadius = (*this->sc)["Rules"]->get<double>("Rules.RobotRadius", NULL);
        wheelSpeed = (*this->sc)["Penalty"]->get<double>("Penalty.WheelSpeed", NULL);
        aimOffset = (*this->sc)["Penalty"]->get<double>("Penalty.AimOffset", NULL);
        kickPower = (*this->sc)["Penalty"]->get<double>("Penalty.KickPower", NULL);
        timeForPenaltyShot = (*this->sc)["Rules"]->get<double>("Rules.Standards.PenaltyTimeForShot", NULL) * 1000000;
        lastAlignment = 0;
        waitBeforeBlindKick = timeForPenaltyShot - 1000000000;

        /*PROTECTED REGION END*/
    }
    PenaltyAlignAndShoot::~PenaltyAlignAndShoot()
    {
        /*PROTECTED REGION ID(dcon1431531496053) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void PenaltyAlignAndShoot::run(void* msg)
    {
        /*PROTECTED REGION ID(run1431531496053) ENABLED START*/ //Add additional options here
        shared_ptr < geometry::CNPosition > ownPos = wm->rawSensorData.getOwnPositionVision(); // actually ownPosition corrected
        shared_ptr < geometry::CNPoint2D > egoBallPos = wm->ball.getEgoBallPosition();

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
                > (wm->field.getFieldLength() / 2 + ballDiameter, goalLineLength / 2 - aimOffset * ballDiameter);
        shared_ptr < geometry::CNPoint2D > alloRightAimPoint = make_shared < geometry::CNPoint2D
                > (wm->field.getFieldLength() / 2 + ballDiameter, -goalLineLength / 2 + aimOffset * ballDiameter);

        // Create points for rectangle check
        shared_ptr < geometry::CNPoint2D > frontLeft = make_shared < geometry::CNPoint2D
                > (wm->field.getFieldLength() / 2 - robotRadius, goalLineLength / 2);
        shared_ptr < geometry::CNPoint2D > frontRight = make_shared < geometry::CNPoint2D
                > (wm->field.getFieldLength() / 2 - robotRadius, -goalLineLength / 2);

        // Create back point according to last alignment
        shared_ptr < geometry::CNPoint2D > back = nullptr;

        // Hysteresis
        if (lastAlignment == 0) // not aligned before (default)
        {
            back = make_shared < geometry::CNPoint2D > (wm->field.getFieldLength() / 2 + robotRadius, 0);
        }
        else if (lastAlignment == 1) // last alignment left
        {
            back = make_shared < geometry::CNPoint2D > (wm->field.getFieldLength() / 2 + robotRadius, robotRadius);
        }
        else // last alignment right
        {
            back = make_shared < geometry::CNPoint2D > (wm->field.getFieldLength() / 2 + robotRadius, -robotRadius);
        }

        int counter = 0;

        shared_ptr < vector<shared_ptr<geometry::CNPoint2D> > > alloOpps = nullptr;

        for (int i = 0; i < wm->getRingBufferLength(); i++)
        {
            alloOpps = wm->obstacles.getAlloObstaclePoints(i);
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
        double deltaHoleAngle = geometry::deltaAngle(wm->kicker.kickerAngle, egoTargetAngle);
        // calculate passed time
        unsigned long timePassed = wm->getTime() - wm->game.getTimeSinceStart();
        // if too much time has passed or the robot is aligned, we shoot
        if (timePassed >= waitBeforeBlindKick || fabs(deltaHoleAngle) < this->angleTolerance)
        {
            cout << "PenaltyBeh: Success!" << endl;
            KickControl kc;
            kc.enabled = true;
            kc.kicker = egoBallPos->angleTo();
            kc.power = kickPower;
            send(kc);
            this->success = true;

        }
        // Create Motion Command for aiming
        MotionControl mc = msl::RobotMovement::alignToPointWithBall(egoTarget, egoBallPos, this->angleTolerance,
                                                                    this->angleTolerance);
        send(mc);
        /*PROTECTED REGION END*/
    }
    void PenaltyAlignAndShoot::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1431531496053) ENABLED START*/ //Add additional options here
        lastAlignment = 0;
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1431531496053) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
