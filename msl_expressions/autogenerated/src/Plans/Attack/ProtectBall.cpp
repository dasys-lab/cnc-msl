using namespace std;
#include "Plans/Attack/ProtectBall.h"

/*PROTECTED REGION ID(inccpp1457706592232) ENABLED START*/ //Add additional includes here
#include <msl_robot/robotmovement/RobotMovement.h>
#include <msl_robot/MSLRobot.h>
#include <pathplanner/PathProxy.h>
#include <Ball.h>
#include <RawSensorData.h>
#include <MSLWorldModel.h>
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1457706592232) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    ProtectBall::ProtectBall() :
            DomainBehaviour("ProtectBall")
    {
        /*PROTECTED REGION ID(con1457706592232) ENABLED START*/ //Add additional options here
        sc = supplementary::SystemConfig::getInstance();
        pp = msl::PathProxy::getInstance();
        eval = make_shared<msl::PathEvaluator>();

        maxVel = 2000;
        pRot = 2.1;
        dRot = 0.0;
        lastRotError = 0;
        minRot = 0.1;
        maxRot = M_PI * 4;

        randomCounter = 0;

        pRot = (*sc)["Dribble"]->get<double>("OneEighty", "RotationP", NULL);
        dRot = (*sc)["Dribble"]->get<double>("OneEighty", "RotationD", NULL);
        minRot = (*sc)["Dribble"]->get<double>("OneEighty", "MinRotation", NULL);
        maxRot = (*sc)["Dribble"]->get<double>("OneEighty", "MaxRotation", NULL);
        maxVel = (*sc)["Behaviour"]->get<double>("Behaviour", "MaxSpeed", NULL);

        /*PROTECTED REGION END*/
    }
    ProtectBall::~ProtectBall()
    {
        /*PROTECTED REGION ID(dcon1457706592232) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void ProtectBall::run(void* msg)
    {
        /*PROTECTED REGION ID(run1457706592232) ENABLED START*/ //Add additional options here
        auto ballPos = wm->ball->getEgoBallPosition();
        auto ballVel = wm->ball->getEgoBallVelocity();
        shared_ptr < geometry::CNPoint2D > ballVel2;
        auto ownPos = wm->rawSensorData->getOwnPositionVision();
        shared_ptr<vector<double>> dstscan = wm->rawSensorData->getDistanceScan();

        msl_actuator_msgs::MotionControl mc;
        if (ownPos == nullptr)
        {
            mc = this->robot->robotMovement->driveRandomly(500);
            send(mc);
            return;
        }
        if (ballPos == nullptr || ownPos == nullptr)
        {
            return;
        }
        if (ballVel == nullptr)
        {
            ballVel2 = make_shared < geometry::CNPoint2D > (0, 0);
        }
        else if (ballVel->length() > 5000)
        {
            auto v = ballVel->normalize() * 5000;
            ballVel2 = make_shared < geometry::CNPoint2D > (v->x, v->y);
        }
        else
        {
            ballVel2 = make_shared < geometry::CNPoint2D > (ballVel->x, ballVel->y);
        }

        shared_ptr < geometry::CNPoint2D > aimPoint;
        if (alloAimPoint != nullptr)
        {
            aimPoint = alloAimPoint->alloToEgo(*ownPos);
        }
        else
        {
            this->initialiseParameters();
        }

        if (aimPoint == nullptr)
        {
            this->setSuccess(true);
            return;
        }

        double aimAngle = aimPoint->angleTo();
        double ballAngle = ballPos->angleTo();

        double deltaAngle = geometry::deltaAngle(ballAngle, aimAngle);
        cout << "ProtectBall: angle:\t\t\t\t\t " << deltaAngle << endl;
        cout << "ProtectBall: aimPoint X: " << aimPoint->x << " Y: " << aimPoint->y << endl;

        if (abs(deltaAngle) < 20 * M_PI / 180)
        {
            this->setSuccess(true);
        }

        if (dstscan != nullptr)
        {
            double distBeforeBall = minFree(ballPos->angleTo(), 200, dstscan);
            if (distBeforeBall < 600)
                this->setSuccess(true);
        }

        mc.motion.rotation = deltaAngle * pRot + (deltaAngle - lastRotError) * dRot;

        mc.motion.rotation = ((mc.motion.rotation) > 0 ? 1 : -1)
                * min(this->maxRot, max(abs(mc.motion.rotation), this->minRot));

        lastRotError = deltaAngle;

        double transBallOrth = ballPos->length() * mc.motion.rotation; //may be negative!
        double transBallTo = min(1000.0, ballVel2->length()); //Math.Max(ballPos.Distance(),ballVel2.Distance());

        auto driveTo = ballPos->rotate(-M_PI / 2.0);
        driveTo = driveTo->normalize() * transBallOrth;
        driveTo = driveTo + ballPos->normalize() * transBallTo;

        if (driveTo->length() > maxVel)
        {
            driveTo = driveTo->normalize() * maxVel;
        }

        mc.motion.angle = driveTo->angleTo();
        mc.motion.translation = driveTo->length();

        send(mc);
//        msl::RobotMovement::updateLastTurnTime();
        /*PROTECTED REGION END*/
    }
    void ProtectBall::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1457706592232) ENABLED START*/ //Add additional options here
        shared_ptr < geometry::CNPosition > ownPos = wm->rawSensorData->getOwnPositionVision();
        shared_ptr < geometry::CNPoint2D > ballPos = wm->ball->getEgoBallPosition();

        if (ownPos == nullptr || ballPos == nullptr)
        {
            alloAimPoint = nullptr;
        }
        else
        {

            shared_ptr < geometry::CNPoint2D > egoGoalPoint = make_shared < geometry::CNPoint2D > (1000.0, 0);

            shared_ptr < geometry::CNPoint2D > aimPoint = pp->getEgoDirection(egoGoalPoint, eval);

            if (abs(aimPoint->angleTo()) > M_PI / 2.0)
            {
                aimPoint->x = 0;
                aimPoint->y = 1000.0 * ((aimPoint->y) > 0 ? 1 : -1);
            }

            if (aimPoint != nullptr)
            {
                alloAimPoint = (aimPoint->normalize() * 10000)->egoToAllo(*ownPos);
            }
        }
        lastRotError = 0;
        /*PROTECTED REGION END*/
    }
    /*PROTECTED REGION ID(methods1457706592232) ENABLED START*/ //Add additional methods here
    double ProtectBall::minFree(double angle, double width, shared_ptr<vector<double>> dstscan)
    {

        double sectorWidth = 2 * M_PI / dstscan->size();
        int startSector = ((int)floor(angle / sectorWidth) % dstscan->size());
        double minfree = dstscan->at(startSector);
        double dist, dangle;
        for (int i = 1; i < dstscan->size() / 4; i++)
        {
            dist = dstscan->at(((startSector + i) % dstscan->size()));
            dangle = sectorWidth * i;
            if (abs(dist * sin(dangle)) < width)
            {
                minfree = min(minfree, abs(dist * cos(dangle)));
            }

            dist = dstscan->at(((startSector - i) % dstscan->size()));
            if (abs(dist * sin(dangle)) < width)
            {
                minfree = min(minfree, abs(dist * cos(dangle)));
            }

        }
        return minfree;
    }
/*PROTECTED REGION END*/
} /* namespace alica */
