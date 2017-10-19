using namespace std;
#include "Plans/Attack/OneEighty.h"

/*PROTECTED REGION ID(inccpp1434650892176) ENABLED START*/ //Add additional includes here
#include "SystemConfig.h"
#include "msl_actuator_msgs/MotionControl.h"
#include "pathplanner/evaluator/PathEvaluator.h"
#include "pathplanner/PathProxy.h"
#include <RawSensorData.h>
#include <Ball.h>
using geometry::CNVecEgo;
using geometry::CNPointAllo;
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
        auto ballPos = wm->ball->getPositionEgo();
        auto ballVel = wm->ball->getVelocityEgo();
        geometry::CNVecEgo ballVel2;
        auto ownPos = wm->rawSensorData->getOwnPositionVisionBuffer().getLastValidContent();
        auto dstscan = wm->rawSensorData->getDistanceScanBuffer().getLastValidContent();

        msl_actuator_msgs::MotionControl mc;
        if (!ballPos || !ownPos)
        {
            return;
        }

        if (!ballVel)
        {
            ballVel2 = CNVecEgo(0, 0);
        }
        else if (ballVel->length() > 5000)
        {
            CNVecEgo v = ballVel->normalize() * 5000;
            ballVel2 = v;
        }
        else
        {
            ballVel2 = *ballVel;
        }

        //TODO corrected possible allo/ego hiccup during nice geom
        auto alloAim = CNPointAllo(wm->field->getFieldLength() / 2.0 - 500, 0);
        auto aimPoint = nonstd::make_optional<geometry::CNPointEgo>(alloAim.toEgo(*ownPos));
        aimPoint = msl::PathProxy::getInstance()->getEgoDirection(*aimPoint, msl::PathEvaluator());
        nonstd::optional<geometry::CNPointAllo> alloAimPoint = nonstd::nullopt;
        if (aimPoint)
        {
            aimPoint = aimPoint->normalize() * 10000;
            alloAimPoint = aimPoint->toAllo(*ownPos);
        }
        aimPoint = nonstd::nullopt;
        if (alloAimPoint)
        {
            aimPoint = alloAimPoint->toEgo(*ownPos);
        }

        if (!aimPoint)
        {
            this->setFailure(true);
            return;
        }
        double aimAngle = aimPoint->angleZ();

        double ballAngle = ballPos->angleZ();

        double deltaAngle = geometry::deltaAngle(ballAngle, aimAngle);
        if (abs(deltaAngle) < 20 * M_PI / 180)
        {
            this->setSuccess(true);
        }
        if (dstscan)
        {
            double distBeforeBall = minFree(ballPos->angleZ(), 200, *dstscan);
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
        double transBallTo = min(1000.0, ballVel2.length());

        auto rotatedVec = ballPos->rotateZ(-M_PI / 2.0);
        auto driveTo = CNVecEgo(rotatedVec.x, rotatedVec.y);
        driveTo = driveTo.normalize() * transBallOrth;
        driveTo.x += ballPos->normalize().x * transBallTo;
        driveTo.y += ballPos->normalize().y * transBallTo;

        if (driveTo.length() > maxVel)
        {
            driveTo = driveTo.normalize() * maxVel;
        }

        mc.motion.angle = driveTo.angleZ();
        mc.motion.translation = driveTo.length();

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
    double OneEighty::minFree(double angle, double width, shared_ptr<const vector<double> > dstscan)
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
