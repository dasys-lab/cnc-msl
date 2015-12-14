using namespace std;
#include "Plans/GenericBehaviours/CheckGoalKick.h"

/*PROTECTED REGION ID(inccpp1449076008755) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1449076008755) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    CheckGoalKick::CheckGoalKick() :
            DomainBehaviour("CheckGoalKick")
    {
        /*PROTECTED REGION ID(con1449076008755) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    CheckGoalKick::~CheckGoalKick()
    {
        /*PROTECTED REGION ID(dcon1449076008755) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void CheckGoalKick::run(void* msg)
    {
        /*PROTECTED REGION ID(run1449076008755) ENABLED START*/ //Add additional options here
        //get own Pos
        auto ownPosition = wm->rawSensorData.getOwnPositionVision();
        ownPos->x = ownPosition->x;
        ownPos->y = ownPosition->y;
        //get ego ball pos
        egoBallPos = wm->ball.getEgoBallPosition();
        //get goal pos
        goalPosLeft = field->posLeftOppGoalPost();
        goalPosRight = field->posRightOppGoalPost();
        goalPosMiddle = field->posOppGoalMid();

        egoAlignPoint = goalPosMiddle;

        //check if the robot is looking to opp goal
        if (checkGoalLine())
        {
            //check if an enemy is between the robot and goal
            if (checkFreeCorridor())
            {
                cout << "ready to shoot!" << endl;
            }
            //check if it is possible to shoot to the goal
            else if (checkShootPossibility())
            {
                cout << "ready to shoot!" << endl;
            }
        }
        /*PROTECTED REGION END*/
    }
    void CheckGoalKick::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1449076008755) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    /*PROTECTED REGION ID(methods1449076008755) ENABLED START*/ //Add additional methods here
    bool CheckGoalKick::checkGoalLine()
    {
        double toleranceAngle = 1 / 18; // 10 degree

        // if angle is smaller then 10 degree return true
        if (egoAlignPoint->rotate(M_PI)->angleTo() < M_PI * toleranceAngle
                || egoAlignPoint->rotate(M_PI)->angleTo() > -M_PI * toleranceAngle)
        {
            return true;
        }
        return false;
    }

    bool CheckGoalKick::checkFreeCorridor()
    {
        auto obstacles = wm->robots.getObstacles();

//		if (!wm->pathPlanner.corridorCheck(wm->pathPlanner.getCurrentVoronoiNet(), ownPos, goalPosMiddle, obstacles))
//		{
//			return true;
//		}
        return false;
    }

    bool CheckGoalKick::checkShootPossibility()
    {
        // doing some math
//		double x1 = ownPos->x;
//		double y1 = ownPos->y;
//		double x2 = goalPosMiddle->x;
//		double y2 = goalPosMiddle->y;
//		double evenWidth = wm->pathPlanner.getAdditionalCorridorWidth();
//		double evenWidth = 10;

        // check if obstacle lays in corridor
        auto obstacles = wm->robots.getObstacles();
        shared_ptr < geometry::CNPoint2D > obstaclePoint = nullptr;
        bool foundObstacle = false;
        int obstacleAt = -1;

        for (int i = 0; i < obstacles->size(); i++)
        {
            obstaclePoint->x = obstacles->at(i).x;
            obstaclePoint->y = obstacles->at(i).y;

            if (wm->pathPlanner.corridorCheck(wm->pathPlanner.getCurrentVoronoiNet(), ownPos, goalPosMiddle,
                                              obstaclePoint))
            {
                foundObstacle = true;
                obstacleAt = i;
                break;
            }
        }

        if (obstacleAt > -1)
        {
            // check if obstacle is blocking (distance to obstacle)
            auto obstacle = obstacles->at(obstacleAt);
        }
        else
        {
            return true;
        }

//		for (int j = 0; j < obstacles->size(); j++)
//		{
//			for (int i = 0; i < evenWidth; i++)
//			{
//				// calculate the even
//				// y = mx + b
//				// m = (y1 - y2) / (x1 - x2)
//				double m = (y1*(i - evenWidth/2) - y2*(i - evenWidth/2)) / (x1*(i - evenWidth/2) - x2*(i - evenWidth/2));
//				// b = -m * x1 + y1
//				double b = -m * x1*(i - evenWidth/2) + y1*(i - evenWidth/2);
//
//				auto obs = obstacles->at(j);
//				if (obs.y == m * obs.x + b)
//				{
//					return true;
//				}
//			}
//		}
        return false;
    }

/*PROTECTED REGION END*/
} /* namespace alica */
