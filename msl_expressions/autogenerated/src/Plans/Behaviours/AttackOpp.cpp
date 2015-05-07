using namespace std;
#include "Plans/Behaviours/AttackOpp.h"

/*PROTECTED REGION ID(inccpp1430324527403) ENABLED START*/ //Add additional includes here
#include "robotmovement/RobotMovement.h"
#include <cmath>
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1430324527403) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    AttackOpp::AttackOpp() :
            DomainBehaviour("AttackOpp")
    {
        /*PROTECTED REGION ID(con1430324527403) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    AttackOpp::~AttackOpp()
    {
        /*PROTECTED REGION ID(dcon1430324527403) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void AttackOpp::run(void* msg)
    {
        /*PROTECTED REGION ID(run1430324527403) ENABLED START*/

        auto me = wm->rawSensorData.getOwnPositionVision();

        auto ballPos = wm->rawSensorData.getBallPosition();

        auto obstacles = wm->robots.getObstacles();

        for(auto obstacle : *obstacles)
        {
        	// TODO: Get closest obstacle to ball
        }

        if (!me.operator bool())
        {
            return;
        }

        auto egoTarget = alloTarget.alloToEgo(*me);

        msl_actuator_msgs::MotionControl mc;

        if (ballPos != nullptr)
        {
            mc = RobotMovement::moveToPointCarefully(egoTarget, ballPos, 0);
        }
        else
        {
            mc = RobotMovement::moveToPointCarefully(egoTarget, make_shared < msl::CNPoint2D > (0.0, 0.0), 0);
        }

//        if (egoTarget->length() < 250)
//        {
//            this->success = true;
//        }

        // TODO: Prüfen ob Wert korrekt ist
        auto radius_own = sqrt(pow((me->x - ballPos->x),2) + pow(me->y - ballPos->y,2));
        std::cout << "Eigener Radius zum Ball: " << radius_own << std::endl;

        auto radius_ball = 60; // in cm???

        // TODO: Schnittpunkt berechnen

        send(mc);

    	//Add additional options here
        /*PROTECTED REGION END*/
    }
    void AttackOpp::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1430324527403) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1430324527403) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
