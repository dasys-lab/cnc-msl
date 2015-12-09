using namespace std;
#include "Plans/Goalie/Test/GoalieBehaviours/WatchBall.h"

/*PROTECTED REGION ID(inccpp1447863466691) ENABLED START*/ //Add additional includes here
#include "robotmovement/RobotMovement.h"
#include <cmath>
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1447863466691) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    WatchBall::WatchBall() :
            DomainBehaviour("WatchBall")
    {
        /*PROTECTED REGION ID(con1447863466691) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    WatchBall::~WatchBall()
    {
        /*PROTECTED REGION ID(dcon1447863466691) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void WatchBall::run(void* msg)
    {
        /*PROTECTED REGION ID(run1447863466691) ENABLED START*/ //Add additional options here
        auto ballPos = wm->ball.getEgoBallPosition();
        auto me = wm->rawSensorData.getOwnPositionMotion();

    	msl_actuator_msgs::MotionControl mc;
    	auto ballX = ballPos->x;
    	auto ballY = ballPos->y;
    	auto goalMidX = MSLFootballField::posOwnGoalMid()->alloToEgo(*me)->x;
    	auto goalMidY = MSLFootballField::posOwnGoalMid()->alloToEgo(*me)->y;

    	auto targetX = goalMidX - 100;
    	auto targetY = ballY;
    	shared_ptr <geometry::CNPoint2D> fieldCenterTarget = MSLFootballField::posCenterMarker()->alloToEgo(*me);

    	//mc = RobotMovement::alignToPointNoBall(ballPos, ballPos, 3.0);
    	//mc = RobotMovement::alignToPointNoBall(make_shared < geometry::CNPoint2D > (egoX, egoY), ballPos, 3.0);
    	//cout << "WatchBall: Inside run" << endl;

    	//auto penaltyWidth = MSLFootballField::PenaltyAreaWidth;
    	//auto goalLength = MSLFootballField::GoalAreaLength;
    	auto penaltyLength = MSLFootballField::PenaltyAreaLength;
    	auto goalWidth = MSLFootballField::GoalAreaWidth;

    	//cout << "GoalieDefault: GoalAreaWidth=" << goalWidth<< endl;
    	//cout << "GoalieDefault: egoX=" << egoX<< endl;
    	cout << "GoalieDefault: ballY=" << ballY<< endl;

    	if(std::abs (targetY) + 100 >= goalWidth/2) {
    		targetY *= (goalWidth/2 - 150)/std::abs (targetY);
    	}

    	mc = RobotMovement::moveToPointCarefully(make_shared < geometry::CNPoint2D > (targetX, targetY), ballPos, 100);

    	send(mc);

        /*PROTECTED REGION END*/
    }
    void WatchBall::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1447863466691) ENABLED START*/ //Add additional options here

        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1447863466691) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
