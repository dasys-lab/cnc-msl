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

        auto me = wm->rawSensorData.getOwnPositionVision();

        auto egoBallPos = wm->ball.getEgoBallPosition();

        //auto obstacles = wm->robots.getObstacles();

        //for (auto obstacle : *obstacles)
        //{
        // TODO: Get closest obstacle to ball
        //}

        auto x = egoBallPos->x;
        auto y = egoBallPos->y;

        if (old_x == 0 && old_y == 0) {
        	old_x = x;
        	old_y = y;
        	return;
        }

        if (x > old_x && y > old_y) {
        	// x+ && y+ Ball kommt von vorne rechts
        	cout << "von vorne rechts" << endl;
        } else if (x > old_x && y < old_y) {
        	// x+ && y- Ball kommt von vorne links
        	cout << "von vorne links" << endl;
        } else if (x < old_x && y < old_y) {
        	// x- && y- Ball kommt von hinten links
        	cout << "von hinten links" << endl;
        } else if (x < old_x && y > old_y) {
        	// x- && y+ Ball kommt von hinten rechts
        	cout << "von hinten rechts" << endl;
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

        msl_actuator_msgs::MotionControl mc;
        msl_actuator_msgs::BallHandleCmd bhc;

        mc = RobotMovement::moveToPointCarefully(egoBallPos, egoBallPos, 300);

        const double rotate_P = 1.8;

        mc.motion.angle = egoBallPos->angleTo();
        mc.motion.rotation = egoBallPos->rotate(M_PI)->angleTo() * rotate_P;

        double summe = 0.0;
        static double olddistance = 0.0;

        const double Kp = 2.0;
        const double Ki = 0.0;
        const double Kd = 1.7;

        //distance ball to robot
        double distance = egoBallPos->length();

        summe = summe + distance;
        double movement = Kp * distance + Ki * summe + Kd * (distance - olddistance);
        olddistance = distance;

        auto egoBallVelocity = wm->ball.getEgoBallVelocity();

        double ball_speed = egoBallVelocity->length();

        movement += ball_speed;

        //cout << "movement: " << movement << endl;
        //cout << "ball speed: " << ball_speed << endl;
        //cout << "distance: " << distance << endl;

        // translation = 1000 => 1 m/s
        mc.motion.translation = movement;

        if (egoBallPos->length() < 300)
        {

            bhc.leftMotor = -30;
            bhc.rightMotor = -30;

            this->send(bhc);
            //this->success = true;
        }
        /*
         // TODO: Prüfen ob Wert korrekt ist
         auto radius_own = sqrt(pow((me->x - ballPos->x), 2) + pow(me->y - ballPos->y, 2));
         std::cout << "Eigener Radius zum Ball: " << radius_own << std::endl;

         auto radius_distance_ball = 600;

         // TODO: Schnittpunkt berechnen
         */
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
