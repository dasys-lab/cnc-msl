using namespace std;
#include "Plans/Penalty/PenaltyAlignAndShoot.h"

/*PROTECTED REGION ID(inccpp1431531496053) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1431531496053) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    PenaltyAlignAndShoot::PenaltyAlignAndShoot() :
            DomainBehaviour("PenaltyAlignAndShoot")
    {
        /*PROTECTED REGION ID(con1431531496053) ENABLED START*/ //Add additional options here
        field = msl::MSLFootballField::getInstance();
        maxVel = (*this->sc)["Penalty"]->get<double>("Penalty.MaxSpeed", NULL);

        // Aiming/Rotation Stuff
        lastRotError = 0;
        //timesOnTargetThreshold = (*this->sc)["Show"]->get<int>("TwoHoledWall.TimesOnTarget", NULL);
        pRot = (*this->sc)["Penalty"]->get<double>("Penalty.RotationP", NULL);
        dRot = (*this->sc)["Penalty"]->get<double>("Penalty.RotationD", NULL);
        minRot = (*this->sc)["Penalty"]->get<double>("Penalty.MinRotation", NULL);
        maxRot = (*this->sc)["Penalty"]->get<double>("Penalty.MaxRotation", NULL);
        angleTolerance = (*this->sc)["Penalty"]->get<double>("Penalty.AngleTolerance", NULL);
        ballAngleTolerance = (*this->sc)["Penalty"]->get<double>("Penalty.BallAngleTolerance", NULL);
        ballDiameter = (*this->sc)["Rules"]->get<double>("Rules.BallRadius", NULL) * 2;
        goalLineLength = (*this->sc)["Globals"]->get<double>("Globals.FootballField.GoalWidth", NULL);
        robotDiameter = (*this->sc)["Rules"]->get<double>("Rules.RobotRadius", NULL) * 2;
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
                > (field->FieldLength / 2 + ballDiameter, goalLineLength / 2 - aimOffset * ballDiameter);
        shared_ptr < geometry::CNPoint2D > alloRightAimPoint = make_shared < geometry::CNPoint2D
                > (field->FieldLength / 2 + ballDiameter, -goalLineLength / 2 + aimOffset * ballDiameter);

        // Create points for rectangle check
        shared_ptr < geometry::CNPoint2D > frontLeft = make_shared < geometry::CNPoint2D
                > (field->FieldLength / 2 - robotDiameter / 2, goalLineLength / 2);
        shared_ptr < geometry::CNPoint2D > frontRight = make_shared < geometry::CNPoint2D
                > (field->FieldLength / 2 - robotDiameter / 2, -goalLineLength / 2);

        // Create back point according to last alignment
        shared_ptr < geometry::CNPoint2D > back = nullptr;

        // Hysteresis
        if (lastAlignment == 0) // not aligned before (default)
        {
            back = make_shared < geometry::CNPoint2D > (field->FieldLength / 2 + robotDiameter / 2, 0);
        }
        else if (lastAlignment == 1) // last alignment left
        {
            back = make_shared < geometry::CNPoint2D > (field->FieldLength / 2 + robotDiameter / 2, robotDiameter / 2);
        }
        else // last alignment right
        {
            back = make_shared < geometry::CNPoint2D > (field->FieldLength / 2 + robotDiameter / 2, -robotDiameter / 2);
        }

        int counter = 0;

        for (int i = 0; i < wm->getRingBufferLength(); i++)
        {
            if (wm->obstacles.getEgoVisionObstacles(i) != nullptr)
            {
                // weighted analysis of past and current obstacles
                for (auto it = wm->obstacles.getEgoVisionObstacles(i)->begin(); it != wm->obstacles.getEgoVisionObstacles(i)->end(); it++)
                {
                    geometry::CNPoint2D obs(it->x, it->y);
                    shared_ptr < geometry::CNPoint2D > alloObs = obs.egoToAllo(*ownPos);

                    // if obstacle is inside rectangle, increase counter by inverse age (10 = newest, 1 = oldest)
                    if (geometry::GeometryCalculator::isInsideRectangle(frontLeft, back, alloObs))
                    {
                        counter += wm->getRingBufferLength() - i;
                    }
                    // if obstacle is inside rectangle, decrease counter by inverse age (10 = newest, 1 = oldest)
                    if (geometry::GeometryCalculator::isInsideRectangle(frontRight, back, alloObs))
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
//			cout << "PenaltyBeh: left!" << endl;
            egoTarget = alloLeftAimPoint->alloToEgo(*ownPos);
        }
        // if counter > 0, there are obstacles on the left side, so we aim right
        else
        {
            lastAlignment = 2;
//			cout << "PenaltyBeh: right!" << endl;
            egoTarget = alloRightAimPoint->alloToEgo(*ownPos);
        }
        // calculate angle difference between robot and target and ball and target
        double egoTargetAngle = egoTarget->angleTo();
        double egoBallAngle = egoBallPos->angleTo();
        double deltaHoleAngle = geometry::GeometryCalculator::deltaAngle(egoTargetAngle, M_PI);
        double deltaBallAngle = geometry::GeometryCalculator::deltaAngle(egoBallAngle, M_PI);
        // calculate passed time
        unsigned long timePassed = wm->getTime() - wm->game.getTimeSinceStart();
        // if too much time has passed or the robot is aligned, we shoot
        if (timePassed >= waitBeforeBlindKick || fabs(deltaHoleAngle) < this->angleTolerance)
        {
            KickControl kc;
            kc.enabled = true;
            kc.kicker = egoBallPos->angleTo();
            kc.power = kickPower;
            send(kc);
            this->success = true;

        }
        // Create Motion Command for aiming
        MotionControl mc;
        // PD Rotation Controller
        mc.motion.rotation = -(deltaHoleAngle * pRot + (deltaHoleAngle - lastRotError) * dRot);
        // check rotation direction and force rotation between maxRot and minRot
        mc.motion.rotation = (mc.motion.rotation < 0 ? -1 : 1)
                * min(this->maxRot, max(fabs(mc.motion.rotation), this->minRot));
        lastRotError = deltaHoleAngle;
        // crate the motion orthogonal to the ball
        shared_ptr < geometry::CNPoint2D > driveTo = egoBallPos->rotate(-M_PI / 2.0);
        driveTo = driveTo * mc.motion.rotation;

        // add the motion towards the ball
        driveTo = driveTo + egoBallPos->normalize() * 10;
        mc.motion.angle = driveTo->angleTo();
        mc.motion.translation = min(this->maxVel, driveTo->length());
        send(mc);
        /*PROTECTED REGION END*/
    }
    void PenaltyAlignAndShoot::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1431531496053) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1431531496053) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
