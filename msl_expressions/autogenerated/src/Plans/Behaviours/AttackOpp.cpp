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
<<<<<<< HEAD
        old_x = 0;
        old_y = 0;
=======
>>>>>>> a15bdeedf6f216132146f89a91075152d997eee2
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

<<<<<<< HEAD
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
        // TODO : remove later
        mc = RobotMovement::moveToPointCarefully(egoBallPos, egoBallPos, 300);
        cout << "x: " << x << endl;
        cout << "y: " << y << endl;
        auto egoBallVelocity = wm->ball.getEgoBallVelocity();
        auto vector = egoBallVelocity + egoBallPos;
        double vectorLength = vector->length();

        if (vectorLength < egoBallPos->length())
        {
            cout << "get closer" << endl;

        }
        else
        {
            cout << "roll away" << endl;
        }

        //	if ((x > old_x && y > old_y) && (x < 0 && y < 0))
        //	{
        // x+ && y+ Ball kommt von vorne links
        //cout << "von vorne links" << endl;
        // TODO : dreh dich nach links und schau zum Ball
        /*
         mc.motion.translation = 0;

         /////////////// PID

         const double Ki = 0;
         const double Kd = 1;

         const double Sollwert = 0;
         const double Kp = 0.23;

         double Abweichung = 0.0;
         double Abweichung_Summe = 0.0;
         double Abweichung_Alt = 0.0;
         double Stellwert = 0.0;

         Abweichung_Summe += Abweichung;
         Abweichung = Sollwert - y;
         Stellwert = Kp * Abweichung;
         Stellwert += Ki * Abweichung_Summe;
         Stellwert += Kd * (Abweichung - Abweichung_Alt);

         Abweichung_Alt = Abweichung;

         mc.motion.angle = M_PI * 1 / 2;
         mc.motion.translation = Stellwert;
         */
        /*
         double summe = 0.0;
         static double oldY = 0.0;

         const double Kp = 2.0;

         const double Kd = 1.7;

         //distance ball to robot
         double distance = egoBallPos->length();


         double movement = Kp * y + Kd * (y - oldY);
         oldY =y;
         */
        //auto egoBallVelocity = wm->ball.getEgoBallVelocity();
        //double ball_speed = egoBallVelocity->length();
        //movement += ball_speed;
        //mc = RobotMovement::moveToPointCarefully(me, me, 300);
//////////////
        /*
         }
         else if ((x > old_x && y < old_y) && (x < 0 && y > 0))
         {
         // x+ && y- Ball kommt von vorne rechts
         cout << "von vorne rechts" << endl;
         // TODO : dreh dich nach rechts und schau zum Ball
         mc.motion.translation = 0;
         }
         else if ((x < old_x && y < old_y) && (x > 0 && y > 0))
         {
         // x- && y- Ball kommt von hinten rechts
         cout << "von hinten rechts" << endl;
         // TODO : umdrehen und auf den Ball Schauen
         mc.motion.translation = 0;
         }
         else if ((x < old_x && y > old_y) && (x > 0 && y < 0))
         {
         // x- && y+ Ball kommt von hinten links
         cout << "von hinten links" << endl;
         // TODO : umdrehen und auf den Ball schauen
         mc.motion.translation = 0;
         }
         else
         {
         cout << "else" << endl;
         } */

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

        mc.motion.translation = 0;
=======
        auto vNet = wm->pathPlanner.getCurrentVoronoiNet();

        if (me == nullptr || egoBallPos == nullptr || vNet == nullptr)
        {
            return;
        }

        auto obstacles = wm->robots.getObstacles();
        bool blocked = false;
        msl_actuator_msgs::MotionControl mc;
        for (int i = 0; i < obstacles->size(); i++)
        {
            if (wm->pathPlanner.corridorCheck(
                    vNet, make_shared < geometry::CNPoint2D > (me->x, me->y), egoBallPos->egoToAllo(*me),
                    make_shared < geometry::CNPoint2D > (obstacles->at(i).x, obstacles->at(i).y)))
            {
                blocked = true;
                break;
            }
        }
        if (!blocked)
        {
            auto egoBallVelocity = wm->ball.getEgoBallVelocity();
            cout << "ego ball vel: " << egoBallVelocity->x << "|" << egoBallVelocity->y << " "
                    << egoBallVelocity->length() << endl;
            auto vector = egoBallVelocity + egoBallPos;
            double vectorLength = vector->length();
            if (wm->ball.haveBall())
            {
                isMovingAwayIter = 0;
                isMovingCloserIter = 0;
            }
            else if (vectorLength < egoBallPos->length() && egoBallVelocity->length() > 250)
            {
                isMovingCloserIter++;
                isMovingAwayIter = 0;
            }
            else if (vectorLength > egoBallPos->length() && egoBallVelocity->length() > 250)
            {
                isMovingAwayIter++;
                isMovingCloserIter = 0;
            }
            if (isMovingAwayIter >= maxIter || egoBallVelocity->length() < 250)
            {
                cout << "roll away" << endl;
                mc = driveToMovingBall(egoBallPos, egoBallVelocity);
            }
            else if (isMovingCloserIter >= maxIter)
            {
                cout << "get closer" << endl;
                mc = ballGetsCloser(me, egoBallVelocity, egoBallPos);

            }
            else
            {
                mc.motion.angle = 0;
                mc.motion.translation = 0;
                mc.motion.rotation = 0;

            }
        }
        else
        {
            mc = msl::RobotMovement::moveToPointCarefully(egoBallPos, egoBallPos, 0);
        }
>>>>>>> a15bdeedf6f216132146f89a91075152d997eee2
        send(mc);

//Add additional options here
        /*PROTECTED REGION END*/
    }
    void AttackOpp::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1430324527403) ENABLED START*/ //Add additional options here
<<<<<<< HEAD
        /*PROTECTED REGION END*/
    }
    /*PROTECTED REGION ID(methods1430324527403) ENABLED START*/ //Add additional methods here
    msl_actuator_msgs::MotionControl AttackOpp::driveToMovingBall(shared_ptr<geometry::CNPoint2D> egoBallPos)
    {

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
    }
=======
        oldDistance = 0.0;
        kP = 2.0;
        kI = 0.0;
        kD = 1.7;
        rotate_P = 1.8;
        isMovingCloserIter = 0;
        isMovingAwayIter = 0;
        maxIter = 4;
        /*PROTECTED REGION END*/
    }
    /*PROTECTED REGION ID(methods1430324527403) ENABLED START*/ //Add additional methods here
    msl_actuator_msgs::MotionControl AttackOpp::driveToMovingBall(shared_ptr<geometry::CNPoint2D> egoBallPos,
                                                                  shared_ptr<geometry::CNVelocity2D> egoBallVel)
    {

        msl_actuator_msgs::MotionControl mc;
        msl_actuator_msgs::BallHandleCmd bhc;

        mc.motion.angle = egoBallPos->angleTo();
        mc.motion.rotation = egoBallPos->rotate(M_PI)->angleTo() * rotate_P;

        double distance = egoBallPos->length();
        double movement = kP * distance + kD * (distance - oldDistance);
        oldDistance = distance;

        double ball_speed = egoBallVel->length();

        movement += ball_speed;
        mc.motion.translation = movement;

        if (egoBallPos->length() < 300)
        {

            bhc.leftMotor = -30;
            bhc.rightMotor = -30;

            this->send(bhc);
            //this->success = true;
        }
        return mc;
    }

    msl_actuator_msgs::MotionControl AttackOpp::ballGetsCloser(shared_ptr<geometry::CNPosition> robotPosition,
                                                               shared_ptr<geometry::CNVelocity2D> ballVelocity,
                                                               shared_ptr<geometry::CNPoint2D> egoBallPos)
    {
        double yIntersection = egoBallPos->y + (-(egoBallPos->x / ballVelocity->x)) * ballVelocity->y;

        shared_ptr < geometry::CNPoint2D > interPoint = make_shared < geometry::CNPoint2D > (0, yIntersection);

        msl_actuator_msgs::MotionControl mc;
        // TODO : remove later
        mc = RobotMovement::moveToPointCarefully(interPoint, egoBallPos, 300);

        return mc;
    }

>>>>>>> a15bdeedf6f216132146f89a91075152d997eee2
/*PROTECTED REGION END*/
} /* namespace alica */
