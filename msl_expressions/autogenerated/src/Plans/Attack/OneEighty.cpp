using namespace std;
#include "Plans/Attack/OneEighty.h"

/*PROTECTED REGION ID(inccpp1434650892176) ENABLED START*/ //Add additional includes here
#include "SystemConfig.h"
#include "msl_actuator_msgs/MotionControl.h"
#include "GeometryCalculator.h"
#include "pathplanner/evaluator/PathEvaluator.h"
#include "pathplanner/PathProxy.h"
#include <RawSensorData.h>
#include <Ball.h>
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1434650892176) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    OneEighty::OneEighty() :
            DomainBehaviour("OneEighty")
    {
        /*PROTECTED REGION ID(con1434650892176) ENABLED START*/ //Add additional options here
        //TODO needs to be tested
        this->maxVel = 0;
        this->pRot = 0;
        this->dRot = 0;
        this->minRot = 0;
        this->maxRot = 0;
        this->lastRotError = 0;
        /*PROTECTED REGION END*/
    }
    OneEighty::~OneEighty()
    {
        /*PROTECTED REGION ID(dcon1434650892176) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void OneEighty::run(void* msg)
    {
        /*PROTECTED REGION ID(run1434650892176) ENABLED START*/ //Add additional options here
        shared_ptr < geometry::CNPoint2D > ballPos = wm->ball->getEgoBallPosition();
        shared_ptr < geometry::CNVelocity2D > ballVel = wm->ball->getEgoBallVelocity();
        shared_ptr < geometry::CNPoint2D > ballVel2;
        shared_ptr < geometry::CNPosition > ownPos = wm->rawSensorData->getOwnPositionVision();
        shared_ptr<vector<double>> dstscan = wm->rawSensorData->getDistanceScan();

        msl_actuator_msgs::MotionControl mc;
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
            shared_ptr < geometry::CNVelocity2D > v = ballVel->normalize() * 5000;
            ballVel2 = make_shared < geometry::CNPoint2D > (v->x, v->y);
        }
        else
        {
            ballVel2 = make_shared < geometry::CNPoint2D > (ballVel->x, ballVel->y);
        }

        shared_ptr < geometry::CNPoint2D > aimPoint = make_shared < geometry::CNPoint2D
                > (wm->field->getFieldLength() / 2.0 - 500, 0);
        aimPoint = msl::PathProxy::getInstance()->getEgoDirection(aimPoint, make_shared<msl::PathEvaluator>());
        shared_ptr < geometry::CNPoint2D > alloAimPoint = nullptr;
        if (aimPoint != nullptr)
        {
            aimPoint = aimPoint->normalize() * 10000;
            alloAimPoint = aimPoint->egoToAllo(*ownPos);
        }
        aimPoint = nullptr;
        if (alloAimPoint != nullptr)
        {
            aimPoint = alloAimPoint->alloToEgo(*ownPos);
        }

        if (aimPoint == nullptr)
        {
            this->setFailure(true);
            return;
        }
        double aimAngle = aimPoint->angleTo();

        double ballAngle = ballPos->angleTo();

        double deltaAngle = geometry::deltaAngle(ballAngle, aimAngle);
        if (abs(deltaAngle) < 20 * M_PI / 180)
        {
            this->setSuccess(true);
        }
        if (dstscan != nullptr)
        {
            double distBeforeBall = minFree(ballPos->angleTo(), 200, dstscan);
            if (distBeforeBall < 600)
                this->setFailure(true);
        }

        mc.motion.rotation = deltaAngle * pRot + (deltaAngle - lastRotError) * dRot;

        double sign = 0;
        if (mc.motion.rotation == 0)
        {
            sign = 0;
        }
        else if (mc.motion.rotation > 0)
        {
            sign = 1;
        }
        else
        {
            sign = -1;
        }
        mc.motion.rotation = sign * min(this->maxRot, max(abs(mc.motion.rotation), this->minRot));

        lastRotError = deltaAngle;

        double transBallOrth = ballPos->length() * mc.motion.rotation; //may be negative!
        double transBallTo = min(1000.0, ballVel2->length());

        shared_ptr < geometry::CNPoint2D > driveTo = ballPos->rotate(-M_PI / 2.0);
        driveTo = driveTo->normalize() * transBallOrth;
        driveTo->x += ballPos->normalize()->x * transBallTo;
        driveTo->y += ballPos->normalize()->y * transBallTo;

        if (driveTo->length() > maxVel)
        {
            driveTo = driveTo->normalize() * maxVel;
        }

        mc.motion.angle = driveTo->angleTo();
        mc.motion.translation = driveTo->length();

        send(mc);
        /*PROTECTED REGION END*/
    }
    void OneEighty::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1434650892176) ENABLED START*/ //Add additional options here
        supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();
        this->maxVel = (*this->sc)["Drive"]->get<double>("Drive", "MaxSpeed", NULL);
        this->pRot = (*this->sc)["Dribble"]->get<double>("OneEighty", "RotationP", NULL);
        this->dRot = (*this->sc)["Dribble"]->get<double>("OneEighty", "RotationD", NULL);
        this->minRot = (*this->sc)["Dribble"]->get<double>("OneEighty", "MinRotation", NULL);
        this->maxRot = (*this->sc)["Dribble"]->get<double>("OneEighty", "MaxRotation", NULL);
        this->lastRotError = 0;
        /*PROTECTED REGION END*/
    }
    /*PROTECTED REGION ID(methods1434650892176) ENABLED START*/ //Add additional methods here
    double OneEighty::minFree(double angle, double width, shared_ptr<vector<double> > dstscan)
    {
        double sectorWidth = 2.0 * M_PI / dstscan->size();
        int startSector = mod((int)floor(angle / sectorWidth), dstscan->size());
        double minfree = dstscan->at(startSector);
        double dist, dangle;
        for (int i = 1; i < dstscan->size() / 4; i++)
        {
            dist = dstscan->at(mod((startSector + i), dstscan->size()));
            dangle = sectorWidth * i;
            if (abs(dist * sin(dangle)) < width)
            {
                minfree = min(minfree, abs(dist * cos(dangle)));
            }

            dist = dstscan->at(mod((startSector - i), dstscan->size()));
            if (abs(dist * sin(dangle)) < width)
            {
                minfree = min(minfree, abs(dist * cos(dangle)));
            }

        }
        return minfree;
    }
    int OneEighty::mod(int x, int y)
    {
        int z = x % y;
        if (z < 0)
            return y + z;
        else
            return z;
    }
/*PROTECTED REGION END*/
} /* namespace alica */
