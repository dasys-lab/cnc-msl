using namespace std;
#include "Plans/Attack/DribbleToAttackPoint.h"

/*PROTECTED REGION ID(inccpp1436855838589) ENABLED START*/ //Add additional includes here
#include "robotmovement/RobotMovement.h"
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1436855838589) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    DribbleToAttackPoint::DribbleToAttackPoint() :
            DomainBehaviour("DribbleToAttackPoint")
    {
        /*PROTECTED REGION ID(con1436855838589) ENABLED START*/ //Add additional options here
        this->wheelSpeed = -50;
        this->sc = nullptr;
        this->field = nullptr;
        /*PROTECTED REGION END*/
    }
    DribbleToAttackPoint::~DribbleToAttackPoint()
    {
        /*PROTECTED REGION ID(dcon1436855838589) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void DribbleToAttackPoint::run(void* msg)
    {
        /*PROTECTED REGION ID(run1436855838589) ENABLED START*/ //Add additional options here
        auto ownPos = wm->rawSensorData.getOwnPositionVision();
        auto egoBallPos = wm->ball.getEgoBallPosition();
        auto opponents = wm->robots.getObstacles();
        shared_ptr < geometry::CNPoint2D > egoAlignPoint = nullptr;
        cout << 5<< endl;
        if (ownPos == nullptr || egoBallPos == nullptr)
        {
            return;
        }

        // Constant ball handle wheel speed
        BallHandleCmd bhc;
        bhc.leftMotor = (int8_t)this->wheelSpeed;
        bhc.rightMotor = (int8_t)this->wheelSpeed;
        send(bhc);
        auto ownPoint = make_shared < geometry::CNPoint2D > (ownPos->x, ownPos->y);
        egoTargetPoint = alloTargetPoint->alloToEgo(*ownPos);
        shared_ptr < geometry::CNPoint2D > closestOpponent = nullptr;
        double lowestDist = numeric_limits<double>::max();
        double dist = 0;
        for (int i = 0; i < opponents->size(); i++)
        {
        	cout << 14<< endl;
            auto opp = make_shared < geometry::CNPoint2D > (opponents->at(i).x, opponents->at(i).y);
            dist = opp->distanceTo(ownPoint);
            if (dist < 3000)
            {
                if (dist < lowestDist)
                {
                	cout << 18<< endl;
                    lowestDist = dist;
                    closestOpponent = opp;
                }
            }
        }
        if (closestOpponent == nullptr)
        {
        	cout << 20<< endl;
            egoAlignPoint = egoTargetPoint;
        }
        else
        {
        	cout << 21<< endl;
            auto weightedOppVector = closestOpponent->normalize()->rotate(M_PI) * 1000
                    * (1.0 / closestOpponent->length());
            cout << 22<< endl;
            auto weightedTargetVector = egoTargetPoint->normalize() * 1000
                    * (1.0 / egoTargetPoint->length());
            cout << 23<< endl;
            egoAlignPoint = (weightedOppVector + weightedTargetVector)->normalize() * 1000;
        }
        //left = 1
        //right = -1
        cout << 24<< endl;
        int opponentSide = 0;
        if (closestOpponent->y > ownPos->y)
        {
            opponentSide = 1;
        }
        else
        {
            opponentSide = -1;
        }
        msl_actuator_msgs::MotionControl mc = msl::RobotMovement::moveToPointCarefully(egoTargetPoint, egoAlignPoint,
                                                                                       250);
        if (egoTargetPoint->length() < 250)
        {
            this->success = true;
        }
        send(mc);
        /*PROTECTED REGION END*/
    }
    void DribbleToAttackPoint::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1436855838589) ENABLED START*/ //Add additional options here
        field = msl::MSLFootballField::getInstance();
        sc = supplementary::SystemConfig::getInstance();
        alloTargetPoint = field->posOppPenaltyMarker();
        wheelSpeed = -75;
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1436855838589) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
