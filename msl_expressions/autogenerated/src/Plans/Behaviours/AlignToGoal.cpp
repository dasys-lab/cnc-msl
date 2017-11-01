using namespace std;
#include "Plans/Behaviours/AlignToGoal.h"

/*PROTECTED REGION ID(inccpp1415205272843) ENABLED START*/ //Add additional includes here
#include "GeometryCalculator.h"
#include <Ball.h>
#include <RawSensorData.h>
#include <MSLWorldModel.h>
#include <msl_robot/MSLRobot.h>
#include <msl_robot/kicker/Kicker.h>
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
        shared_ptr < geometry::CNPoint2D > ballPos = wm->ball->getEgoBallPosition();
        shared_ptr < geometry::CNVelocity2D > ballVel = wm->ball->getEgoBallVelocity();
        shared_ptr < geometry::CNPosition > ownPos = wm->rawSensorData->getOwnPositionVision();
        shared_ptr < vector<double> > dstscan = wm->rawSensorData->getDistanceScan();

        msl_actuator_msgs::MotionControl mc;
        if (ballPos == nullptr || ownPos == nullptr)
        {
            return;
        }
        if (ballVel == nullptr)
        {
            ballVel = make_shared < geometry::CNVelocity2D > (0, 0);
        }
        else if (ballVel->length() > 5000)
        {
            ballVel = ballVel->normalize() * 5000;
        }
        else
        {
            ballVel = ballVel->clone();
        }

        shared_ptr < geometry::CNPoint2D > aimPoint;
        if (alloAimPoint != nullptr)
        {
            aimPoint = alloAimPoint->alloToEgo(*ownPos);
        }
        else
        {
            aimPoint = getFreeGoalVector();
            if (aimPoint != nullptr)
            {
                alloAimPoint = aimPoint->egoToAllo(*ownPos);
            }
        }

        if (aimPoint == nullptr)
        {
            this->setFailure(true);
            cout << "AlignToGoal: no aimPoint" << endl;
            return;
        }

        double aimAngle = aimPoint->angleTo();
        double ballAngle = this->robot->kicker->kickerAngle;

        double deltaAngle = -geometry::deltaAngle(aimAngle, ballAngle);
        if (dstscan != nullptr)
        {
            double distBeforeBall = minFree(ballAngle, 200, dstscan);
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
        shared_ptr < geometry::CNPosition > ownPos = wm->rawSensorData->getOwnPositionVision();
        if (ownPos == nullptr)
        {
            alloAimPoint = nullptr;
        }
        else
        {
            shared_ptr < geometry::CNPoint2D > aimPoint = getFreeGoalVector();
            if (aimPoint != nullptr)
            {
                alloAimPoint = aimPoint->egoToAllo(*ownPos);
            }
        }
        /*PROTECTED REGION END*/
    }
    /*PROTECTED REGION ID(methods1415205272843) ENABLED START*/ //Add additional methods here
    double AlignToGoal::goalLineHitPoint(shared_ptr<geometry::CNPosition> ownPos, double egoAngle)
    {

        geometry::CNPoint2D hitVector = geometry::CNPoint2D();
        hitVector.x = cos(egoAngle + ownPos->theta);
        hitVector.y = sin(egoAngle + ownPos->theta);
        double t = (wm->field->getFieldLength() / 2 - ownPos->x) / hitVector.x;
        if (t < 0)
        {
            return numeric_limits<double>::max();
        }
        return ownPos->y + t * hitVector.y;
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

    shared_ptr<geometry::CNPoint2D> AlignToGoal::getFreeGoalVector()
    {

        shared_ptr < geometry::CNPosition > ownPos = wm->rawSensorData->getOwnPositionVision();
        shared_ptr<vector<double>> dstscan = wm->rawSensorData->getDistanceScan();
        if (ownPos == nullptr || dstscan == nullptr)
        {
            return nullptr;
        }
        vector < shared_ptr < geometry::CNPoint2D >> validGoalPoints;
        double x = wm->field->getFieldLength() / 2;
        //TODO add config param
        double y = -1000 + 150;
        shared_ptr < geometry::CNPoint2D > aim = make_shared < geometry::CNPoint2D > (x, y);
        double samplePoints = 4;

        for (double i = 0.0; i < samplePoints; i += 1.0)
        {
            shared_ptr < geometry::CNPoint2D > egoAim = aim->alloToEgo(*ownPos);
            double dist = egoAim->length();
            double opDist = minFree(egoAim->angleTo(), 200, dstscan);
            if (opDist > 1000 && (opDist >= dist || abs(opDist - dist) > 1500))
            {
                validGoalPoints.push_back(egoAim);
                //std::cout << " AlignPoint " << i << ":" << aim->x << ", " << aim->y << endl;
            }
            aim->y += 2 * abs(y) / samplePoints;
        }

        if (validGoalPoints.size() > 0)
        {
            shared_ptr < geometry::CNPoint2D > ret = nullptr;
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
            return nullptr;
        }
    }
/*PROTECTED REGION END*/
} /* namespace alica */
