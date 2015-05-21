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
        this->setTrigger(&wm->visionTrigger);
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
        ballDiameter = (*this->sc)["Globals"]->get<double>("Globals.Dimensions.DiameterBall", NULL);
        goalLineLength = (*this->sc)["Globals"]->get<double>("Globals.FootballField.GoalWidth", NULL);
        robotDiameter = (*this->sc)["Globals"]->get<double>("Globals.Dimensions.DiameterRobot", NULL);
        wheelSpeed = (*this->sc)["Penalty"]->get<double>("Penalty.WheelSpeed", NULL);
        aimOffset = (*this->sc)["Penalty"]->get<double>("Penalty.AimOffset", NULL);
        kickPower = (*this->sc)["Penalty"]->get<double>("Penalty.KickPower", NULL);
        lastAlignment = 0;
        startTime = ros::Time::now().sec;
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
        shared_ptr < CNPosition > ownPos = wm->rawSensorData.getOwnPositionVision(); // actually ownPosition corrected
        shared_ptr < CNPoint2D > egoBallPos = wm->ball.getEgoBallPosition();

        if (ownPos == nullptr || egoBallPos == nullptr)
        {
            return;
        }

        //TODO remove if actuall behavior is used
        //Constant ball handle wheel speed
        BallHandleCmd bhc;
        bhc.leftMotor = (int8_t)this->wheelSpeed;
        bhc.rightMotor = (int8_t)this->wheelSpeed;
        send(bhc);
        bool aimingLeft = true;
        // Create ego-centric 2D target...
        shared_ptr < CNPoint2D > egoTarget;
        CNPoint2D alloLeftAimPoint(field->FieldLength / 2 + ballDiameter,
                                   goalLineLength / 2 - aimOffset * ballDiameter);
        CNPoint2D alloRightAimPoint(field->FieldLength / 2 + ballDiameter,
                                    -goalLineLength / 2 + aimOffset * ballDiameter);
        CNPoint2D frontLeft(field->FieldLength / 2 - robotDiameter / 2, goalLineLength / 2);
        CNPoint2D frontRight(field->FieldLength / 2 - robotDiameter / 2, -goalLineLength / 2);
        CNPoint2D back;
        if (lastAlignment == 0)
        {
            back.x = field->FieldLength / 2 + robotDiameter / 2;
            back.y = 0;
        }
        else if (lastAlignment == 1)
        {
            back.x = field->FieldLength / 2 + robotDiameter / 2;
            back.y = robotDiameter / 2;
        }
        else
        {
            back.x = field->FieldLength / 2 + robotDiameter / 2;
            back.y = -robotDiameter / 2;
        }
        for (auto it = wm->robots.getObstacles()->begin(); it != wm->robots.getObstacles()->end(); it++)
        {
            if (GeometryCalculator::isInsideRectangle(frontLeft, back, CNPoint2D(it->x, it->y)))
            {
                aimingLeft = false;
                lastAlignment = 2;
                break;
            }
            if (GeometryCalculator::isInsideRectangle(frontRight, back, CNPoint2D(it->x, it->y)))
            {
                aimingLeft = true;
                lastAlignment = 1;
                break;
            }

        }

        if (aimingLeft)
        {
            egoTarget = alloLeftAimPoint.alloToEgo(*ownPos);
        }
        else
        {
            egoTarget = alloRightAimPoint.alloToEgo(*ownPos);
        }

        double egoTargetAngle = egoTarget->angleTo();
        double egoBallAngle = egoBallPos->angleTo();
        double deltaHoleAngle = GeometryCalculator::deltaAngle(egoTargetAngle, M_PI);
        double deltaBallAngle = GeometryCalculator::deltaAngle(egoBallAngle, M_PI);

        // Counter for correct aiming

        if (ros::Time::now().sec - startTime >= 9 || fabs(deltaHoleAngle) < this->angleTolerance)
        {

            KickControl kc;
            kc.enabled = true;
            kc.kicker = egoBallPos->angleTo();
            kc.power = kickPower;
            send(kc);

        }

        // Create Motion Command for aiming
        MotionControl mc;

        // PD Rotation Controller
        mc.motion.rotation = -(deltaHoleAngle * pRot + (deltaHoleAngle - lastRotError) * dRot);
        mc.motion.rotation = (mc.motion.rotation < 0 ? -1 : 1)
                * min(this->maxRot, max(fabs(mc.motion.rotation), this->minRot));

        lastRotError = deltaHoleAngle;

        // crate the motion orthogonal to the ball
        shared_ptr < CNPoint2D > driveTo = egoBallPos->rotate(-M_PI / 2.0);
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
