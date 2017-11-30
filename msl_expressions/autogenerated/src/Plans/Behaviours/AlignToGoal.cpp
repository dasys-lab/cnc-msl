using namespace std;
#include "Plans/Behaviours/AlignToGoal.h"

/*PROTECTED REGION ID(inccpp1415205272843) ENABLED START*/ //Add additional includes here
#include <Ball.h>
#include <RawSensorData.h>
#include <MSLWorldModel.h>
#include <msl_robot/MSLRobot.h>
#include <msl_robot/kicker/Kicker.h>
using nonstd::optional;
using nonstd::make_optional;
using nonstd::nullopt;
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1415205272843) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    AlignToGoal::AlignToGoal() :
            DomainBehaviour("AlignToGoal")
    {
        /*PROTECTED REGION ID(con1415205272843) ENABLED START*/ //Add additional options here
        maxVel = 2000;
        maxRot = M_PI * 4;
        minRot = 0.1;
        maxYTolerance = 15;
        pRot = 2.1;
        dRot = 0.0;
        iter = 0;
        kicked = false;
        lastRotError = 0;
        /*PROTECTED REGION END*/
    }
    AlignToGoal::~AlignToGoal()
    {
        /*PROTECTED REGION ID(dcon1415205272843) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void AlignToGoal::run(void* msg)
    {
        /*PROTECTED REGION ID(run1415205272843) ENABLED START*/ //Add additional options here
        auto ballPos = wm->ball->getPositionEgo();
        auto ballVel = wm->ball->getVisionBallVelocityBuffer().getLastValidContent();
        auto ownPos = wm->rawSensorData->getOwnPositionVisionBuffer().getLastValidContent();

        msl_actuator_msgs::MotionControl mc;
        if (ballPos || ownPos)
        {
            return;
        }
        if (!ballVel)
        {
            ballVel = geometry::CNVecEgo(0, 0);
        }
        else if (ballVel->length() > 5000)
        {
            ballVel = ballVel->normalize() * 5000;
        }

        if (!aimEgo)
        {
            aimEgo = getFreeGoalVector();
            if (!aimEgo)
            {
                this->setFailure(true);
                cout << "AlignToGoal: no aimPoint" << endl;
                return;
            }
        }

        double aimAngle = aimEgo->angleZ();
        double ballAngle = this->robot->kicker->kickerAngle;

        double deltaAngle = -geometry::deltaAngle(aimAngle, ballAngle);
        auto dstscan = this->wm->rawSensorData->getDistanceScanBuffer().getLastValidContent();
        if (dstscan)
        {
            double distBeforeBall = this->robot->kicker->minFree(ballAngle, 200, *(*dstscan));
            if (deltaAngle < 20 * M_PI / 180 && distBeforeBall < 1000)
            {
                cout << "AlignToGoal: failure!" << endl;
                this->setFailure(true);
            }
        }
        mc.motion.rotation = deltaAngle * pRot + (deltaAngle - lastRotError) * dRot;
        double sign = mc.motion.rotation < 0 ? -1 : 1;

        mc.motion.rotation = sign * min(this->maxRot, max(abs(mc.motion.rotation), this->minRot));
        lastRotError = deltaAngle;
        double transBallOrth = ballPos->length() * mc.motion.rotation; //may be negative!
        double transBallTo = max(ballPos->length(), ballVel->length()); //Math.Min(1000,ballVel2.Distance());//
        if (abs(deltaAngle) < 12.0 * M_PI / 180.0)
        {
            transBallTo = max(500.0, transBallTo);
        }

        auto driveTo = ballPos->rotateZ(-M_PI / 2.0);
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
    void AlignToGoal::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1415205272843) ENABLED START*/ //Add additional options here
        //TODO needs to be tested
        maxVel = 2000;
        maxRot = M_PI * 4;
        minRot = 0.1;
        maxYTolerance = 15;
        pRot = 2.1;
        dRot = 0.0;
        iter = 0;
        kicked = false;
        lastRotError = 0;
        auto ownPos = wm->rawSensorData->getOwnPositionVisionBuffer().getLastValidContent();
        if (ownPos)
        {
            this->aimEgo = getFreeGoalVector();
        }
        /*PROTECTED REGION END*/
    }
    /*PROTECTED REGION ID(methods1415205272843) ENABLED START*/ //Add additional methods here
    double AlignToGoal::goalLineHitPoint(geometry::CNPositionAllo ownPos, double egoAngle)
    {

        geometry::CNVecAllo hitVector;
        hitVector.x = cos(egoAngle + ownPos.theta);
        hitVector.y = sin(egoAngle + ownPos.theta);
        double t = (wm->field->getFieldLength() / 2 - ownPos.x) / hitVector.x;
        if (t < 0)
        {
            return numeric_limits<double>::max();
        }
        return ownPos.y + t * hitVector.y;
    }

    double AlignToGoal::minFree(double angle, double width, shared_ptr<vector<double> > dstscan)
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

    int AlignToGoal::mod(int x, int y)
    {
        int z = x % y;
        if (z < 0)
            return y + z;
        else
            return z;
    }

    nonstd::optional<geometry::CNPointEgo> AlignToGoal::getFreeGoalVector()
    {

        auto ownPos = wm->rawSensorData->getOwnPositionVisionBuffer().getLastValidContent();
        auto dstscan = wm->rawSensorData->getDistanceScanBuffer().getLastValidContent();
        if (!ownPos || !dstscan)
        {
            return nonstd::nullopt;
        }
        vector<nonstd::optional<geometry::CNPointEgo>> validGoalPoints;
        double x = wm->field->getFieldLength() / 2;
        //TODO add config param
        double y = -1000 + 150;
        geometry::CNPointAllo aim(x, y);
        double samplePoints = 4;

        for (double i = 0.0; i < samplePoints; i += 1.0)
        {
            auto egoAim = aim.toEgo(*ownPos);
            double dist = egoAim.length();
            double opDist = this->robot->kicker->minFree(egoAim.angleZ(), 200, *(*dstscan));
            if (opDist > 1000 && (opDist >= dist || abs(opDist - dist) > 1500))
            {
                validGoalPoints.push_back(nonstd::make_optional<geometry::CNPointEgo>(egoAim));
                //std::cout << " AlignPoint " << i << ":" << aim->x << ", " << aim->y << endl;
            }
            aim.y += 2 * abs(y) / samplePoints;
        }

        if (validGoalPoints.size() > 0)
        {
            nonstd::optional<geometry::CNPointEgo> ret = nonstd::nullopt;
            double max = numeric_limits<double>::min();
            for (int i = 0; i < validGoalPoints.size(); i++)
            {
                if (validGoalPoints[i]->length() > max)
                {
                    max = validGoalPoints[i]->length();
                    ret = validGoalPoints[i];
                }
            }
            return ret;
        }
        else
        {
            return nonstd::nullopt;
        }
    }
/*PROTECTED REGION END*/
} /* namespace alica */
