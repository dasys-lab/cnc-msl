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
            auto opp = make_shared < geometry::CNPoint2D > (opponents->at(i).x, opponents->at(i).y);
            dist = opp->distanceTo(ownPoint);
            if (dist < 3000)
            {
                if (dist < lowestDist)
                {
                    lowestDist = dist;
                    closestOpponent = opp;
                }
            }
        }
        if (closestOpponent == nullptr)
        {
            egoAlignPoint = egoTargetPoint;
        }
        else
        {
            auto weightedOppVector = closestOpponent->rotate(M_PI)
                    * (1.0 / closestOpponent->length());
            auto weightedTargetVector = egoTargetPoint
                    * (1.0 / egoTargetPoint->length());
            egoAlignPoint = (weightedOppVector + weightedTargetVector)->normalize() * 1000;
//            egoAlignPoint = weightedOppVector;
        }
        //left = 1
        //right = -1
        int sign = 0;
        if (closestOpponent != nullptr)
        {
        	double angle = atan2(closestOpponent->y, closestOpponent->x);
        	if(angle > 0 )
        	{
        		sign = -1;
        	}
        	else
        	{
				sign = 1;
        	}
        }
        msl_actuator_msgs::MotionControl mc = msl::RobotMovement::moveToPointCarefully(egoTargetPoint, egoAlignPoint,
                                                                                       250);
        mc.motion.rotation *= sign;
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
