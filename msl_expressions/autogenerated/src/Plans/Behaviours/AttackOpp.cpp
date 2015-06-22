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
        old_x = 0;
        old_y = 0;
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

        shared_ptr < geometry::CNPosition > me = wm->rawSensorData.getOwnPositionVision();

        shared_ptr < geometry::CNPoint2D > egoBallPos = wm->ball.getEgoBallPosition();

        //auto obstacles = wm->robots.getObstacles();

        //for (auto obstacle : *obstacles)
        //{
        // TODO: Get closest obstacle to ball
        //}

        auto x = egoBallPos->x;
        auto y = egoBallPos->y;

        if (old_x == 0 && old_y == 0)
        {
            old_x = x;
            old_y = y;
            return;
        }

        msl_actuator_msgs::MotionControl mc;
        // TODO: remove later
        mc = RobotMovement::moveToPointCarefully(egoBallPos, egoBallPos, 300);
        cout << "x: " << x << endl;
        cout << "y: " << y << endl;
        if ((x > old_x && y > old_y) && ( x < 0 && y < 0) )
        {
            // x+ && y+ Ball kommt von vorne links
            cout << "von vorne links" << endl;
        }
        else if ((x > old_x && y < old_y) && (x > 0 && y < 0))
        {
            // x+ && y- Ball kommt von vorne rechts
            cout << "von vorne rechts" << endl;
        }
        else if ((x < old_x && y < old_y) && (x > 0 && y > 0))
        {
            // x- && y- Ball kommt von hinten rechts
            cout << "von hinten rechts" << endl;
        }
        else if ((x < old_x && y > old_y) && (x > 0 && y < 0))
        {
            // x- && y+ Ball kommt von hinten links
            cout << "von hinten links" << endl;
        }
        else
        {
            cout << "else" << endl;
        }

        old_x = x;
        old_y = y;

        //cout << "egoBallPos x: " << x << " y: " << y << endl;

        if (me == nullptr || egoBallPos == nullptr)
        {
            cerr << "insufficient information for AttackOpp" << endl;
            return;
        }

        if (!me.operator bool())
        {
            return;
        }

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
