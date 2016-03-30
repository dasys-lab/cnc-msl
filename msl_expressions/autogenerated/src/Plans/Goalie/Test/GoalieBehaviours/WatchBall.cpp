using namespace std;
#include "Plans/Goalie/Test/GoalieBehaviours/WatchBall.h"

/*PROTECTED REGION ID(inccpp1447863466691) ENABLED START*/ //Add additional includes here
#include "robotmovement/RobotMovement.h"
#include <cmath>
#include <vector>
#include <string>
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1447863466691) ENABLED START*/ //initialise static variables here
    const string WatchBall::LEFT = "LEFT";
    const string WatchBall::MID = "MID";
    const string WatchBall::RIGHT = "RIGHT";
    /*
     *
     *			 _______________________________
     *			|								|
     *			| <---------- 2000mm ---------> |
     *			|   ._.		   ._.		  ._.	|
     *			| \/   \/	 \/   \/    \/   \/ |
     *	 _______| |__0__| _ _|__1__|_ _ |__2__| |__________
     *
     *			  \__ __/
     *				 V
     *			140+630+140
     *			   910mm
     */
    /*PROTECTED REGION END*/
    WatchBall::WatchBall() :
            DomainBehaviour("WatchBall")
    {
        /*PROTECTED REGION ID(con1447863466691) ENABLED START*/ //Add additional options here
        simulating = (*this->sc)["Behaviour"]->get<int>("Goalie.Simulating", NULL);
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
        cout << "### WatchBall ###" << endl;
        me = wm->rawSensorData.getOwnPositionVision();
        shared_ptr < geometry::CNPoint2D > alloBall = wm->ball.getAlloBallPosition();

        if (me == nullptr)
        {
            cout << "me is null" << endl;
            return;
        }

        /*if (simulating > 0)
         alloGoalMid = MSLFootballField::getInstance()->posOppGoalMid();
         else
         alloGoalMid = MSLFootballField::getInstance()->posOwnGoalMid();*/

        alloGoalMid = MSLFootballField::getInstance()->posOwnGoalMid();

        alloGoalLeft = make_shared < geometry::CNPoint2D
                > (alloGoalMid->x, MSLFootballField::getInstance()->posLeftOwnGoalPost()->y - GOALIE_SIZE / 2);
        alloGoalRight = make_shared < geometry::CNPoint2D
                > (alloGoalMid->x, MSLFootballField::getInstance()->posRightOwnGoalPost()->y + GOALIE_SIZE / 2);

        if (alloBall == nullptr || abs(alloBall->x) > abs(alloGoalMid->x) + 50)
        {
            cout << "Goalie can't see ball! Moving to GoalMid" << endl;
            mc = RobotMovement::moveGoalie(prevTarget, alloFieldCntr, SNAP_DIST);
            send (mc);
            return;
        }
        else
        {
            watchBall();
            //targetIndex = modRingBuffer(targetIndex + 1, TARGET_BUFFER_SIZE);
        }
        //cout << "### WatchBall ###\n" << endl;
        /*PROTECTED REGION END*/
    }
    void WatchBall::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1447863466691) ENABLED START*/ //Add additional options here
        prevTarget = MSLFootballField::getInstance()->posOwnGoalMid();
        /*PROTECTED REGION END*/
    }
    /*PROTECTED REGION ID(methods1447863466691) ENABLED START*/ //Add additional methods here
    void WatchBall::watchBall()
    {
        std::vector < shared_ptr < geometry::CNPoint2D >> ballPositions;
        for (int i = 0; i < BALL_BUFFER_SIZE; i++)
        {
            auto currentBall = wm->ball.getVisionBallPosition(i);
            if (currentBall)
            {
                ballPositions.push_back(currentBall->egoToAllo(*me));
            }
        }

        /*vector<shared_ptr<geometry::CNPoint2D>> temp = ballPositions;
         std::sort(std::begin(temp), std::end(temp),
         [](shared_ptr<geometry::CNPoint2D> a, shared_ptr<geometry::CNPoint2D> b)
         {
         return b->y >= a->y;
         });

         cout << "sorted: " << endl;
         for (shared_ptr<geometry::CNPoint2D> pos : temp)
         {
         cout << pos->y << endl;
         }

         cout << "unsorted: " << endl;
         for (shared_ptr<geometry::CNPoint2D> pos : ballPositions)
         {
         cout << pos->y << endl;
         }
         if (ballPositions.size() > 2)
         {

         auto begin = temp.at(0);
         auto afterBegin = temp.at(1);
         auto end = temp.at(BALL_BUFFER_SIZE - 1);
         auto preEnd = temp.at(BALL_BUFFER_SIZE - 2);

         double diffBot = end->length() - preEnd->length();
         double diffTop = begin->length() - afterBegin->length();

         int delX, delY;
         if (diffBot > diffTop)
         {
         // not using first position
         //cout << "remove Bot" << endl;
         //auto delBallPos = temp.at(0);
         auto delBallPos = *(temp.begin());
         delX = delBallPos->x;
         delY = delBallPos->y;
         //cout << "top" << delBallPos->toString() << endl;
         //targetY = (preEnd->y + (end)->y) / (BALL_BUFFER_SIZE - 1);
         }
         else
         {
         // not using last position
         //cout << "remove top" << endl;
         //auto delBallPos = temp.at(ballPositions.size()-1);
         auto delBallPos = *(temp.end() - 1);
         delX = delBallPos->x;
         delY = delBallPos->y;
         //cout << "bot" << delBallPos->toString() << endl;
         //targetY = (begin->y + afterBegin->y) / (BALL_BUFFER_SIZE - 1);
         }

         int delIndex = -1;
         for (int i = 0; i < ballPositions.size(); i++)
         {
         if (ballPositions.at(i)->y == delY && ballPositions.at(i)->x == delX)
         {
         delIndex = i;
         break;
         }
         }

         if (delIndex > 0)
         {
         ballPositions.erase(ballPositions.begin() + delIndex);
         }
         else
         {
         cout << "No error ballPos found! BallPositions.size(): " << ballPositions.size() << endl;
         cout << "delX: " << delX << endl;
         cout << "delY: " << delY << endl;
         }
         }*/

        shared_ptr < geometry::CNPoint2D > alloTarget;
        if (ballPositions.size() > 0)
        {
            alloTarget = calcGoalImpactY(ballPositions);
            prevTarget = alloTarget;
        }
        else
        {
            alloTarget = prevTarget;
        }

        //targetY = fitTargetY(targetY);

        /*if (targetY > alloGoalLeft->y + GOALIE_SIZE / 2 * SIMULATING)
         {
         targetY = alloGoalLeft->y + GOALIE_SIZE / 2;
         }
         else if (targetY < alloGoalRight->y - GOALIE_SIZE / 2 * SIMULATING)
         {
         targetY = alloGoalRight->y - GOALIE_SIZE / 2;
         }*/

        //auto alloTarget = make_shared<geometry::CNPoint2D>(alloGoalMid->x, targetY);
        //targetPosBuffer[targetIndex] = alloTarget;
        //shared_ptr<geometry::CNPoint2D> egoALignPoint = alloAlignPt->alloToEgo(*me);
        mc = RobotMovement::moveGoalie(alloTarget, alloFieldCntr, SNAP_DIST);
        send (mc);
    }

    void WatchBall::moveInsideGoal(shared_ptr<geometry::CNPoint2D> alloBall, shared_ptr<geometry::CNPosition> me)
    {
        /*ballPosBuffer[ballIndex] = alloBall;
         auto alloTrgt = calcGoalImpactY(TARGET_BUFFER_SIZE);
         auto prevTarget = targetPosBuffer[modRingBuffer(targetIndex - 1, TARGET_BUFFER_SIZE)];

         if (alloTrgt == nullptr)
         {
         cout << "alloTarget NULL" << endl;
         if (prevTarget != nullptr)
         alloTrgt = prevTarget;
         else
         cout << "prevTarget NULL" << endl;
         alloTrgt = alloGoalMid;
         }
         string targetPos = fitTargetY(alloTrgt->y);

         //cout << "currentBall: " << ballPosBuffer[ballIndex]->toString() << endl;
         //cout << "actualTarget: " << alloTrgt->toString();
         sendMC(targetPos);*/
        /*mc = RobotMovement::moveToPointFast(alloTrgt->alloToEgo(*me), alloAlignPt->alloToEgo(*me), 100, 0);
         send(mc);*/
    }

    shared_ptr<geometry::CNPoint2D> WatchBall::calcGoalImpactY(
            std::vector<shared_ptr<geometry::CNPoint2D>>& ballPositions)
    {
        //cout << "#####" << endl;
        double _slope, _yInt;
        double sumXY = 0, sumX2 = 0, sumX2Y2 = 0;
        shared_ptr < geometry::CNPoint2D > avgBall = make_shared < geometry::CNPoint2D > (0.0, 0.0);
        int nPoints = 0;

        /*for (int i = modRingBuffer(ballIndex - nPoints, BALL_BUFFER_SIZE);
         i < modRingBuffer(ballIndex, BALL_BUFFER_SIZE); i = modRingBuffer(i + 1, BALL_BUFFER_SIZE))
         {*/

        for (int i = 0; i < ballPositions.size(); i++)
        {
            auto currentBall = ballPositions.at(i);

            shared_ptr < geometry::CNPoint2D > ppprevBall;
            shared_ptr < geometry::CNPoint2D > pprevBall;
            shared_ptr < geometry::CNPoint2D > prevBall;

            if (i > 2)
            {
                ppprevBall = ballPositions.at(i - 3);
                pprevBall = ballPositions.at(i - 2);
                prevBall = ballPositions.at(i - 1);
            }

            if (prevBall != nullptr && pprevBall != nullptr && ppprevBall != nullptr)
            {
                // check if function is continual
                double diffppp = ppprevBall->y - pprevBall->y;
                double diffpp = pprevBall->y - prevBall->y;
                double diffp = prevBall->y - currentBall->y;
                int buffer = 100;

                if (diffppp >= diffpp)
                {
                    if (!(diffpp >= diffp || diffpp + buffer >= diffp || diffpp - buffer >= diffp))
                    {
                        cout << "[WatchBall] corner detected! cond1" << endl;
                        break;
                    }
                }
                else
                {
                    if (!(diffpp <= diffp || diffpp + buffer <= diffp || diffpp - buffer <= diffp))
                    {
                        cout << "[WatchBall] corner detected! cond2" << endl;
                        break;
                    }
                }
            }
            cout << "[WatchBall] currentBall: " << currentBall->toString() << endl;

            avgBall->x += currentBall->x;
            avgBall->y += currentBall->y;

            sumXY = sumXY + (currentBall->y * currentBall->x);
            sumX2 = sumX2 + (currentBall->x * currentBall->x);
            sumX2Y2 = sumX2Y2 + (currentBall->x * currentBall->x + currentBall->y * currentBall->y);

            nPoints = nPoints + 1;
        }

        double sumX = avgBall->x;
        double sumY = avgBall->y;
        avgBall = avgBall / nPoints;
        double denom = 0;
        double nomi = 0;

        double calcTargetY;
        double variance = (sumX2Y2 + nPoints * ((avgBall->x * avgBall->x) + (avgBall->y * avgBall->y))
                - 2 * ((avgBall->x * sumX) + (avgBall->y * sumY))) / nPoints;
        cout << "[WatchBall] Variance: " << variance << endl;
        if (nPoints > 1 && variance > 2000)
        {
            for (int i = 0; i < nPoints; i++)
            {
                auto curBall = ballPositions.at(i);
                nomi = nomi + ((curBall->x - avgBall->x) * (curBall->y - avgBall->y));
                denom = denom + ((curBall->x - avgBall->x) * (curBall->x - avgBall->x));
            }

            if (denom < 1e-3)
            {
                cout << "[WatchBall] prevTarget, cause no hitPoint " << endl;
                return prevTarget;
            }

            _slope = nomi / denom;
            _yInt = avgBall->y - _slope * avgBall->x;

            calcTargetY = _slope * alloGoalMid->x + _yInt;
            cout << "[WatchBall] calcTargetY   : " << calcTargetY << endl;
        }
        else
        {
            cout << "[WatchBall] noRegression ball y is " << ballPositions.at(0)->y << endl;
            calcTargetY = ballPositions.at(0)->y;
        }

        //targetPosBuffer[targetIndex] = currentTarget;
        //cout << "#####" << endl;
        //cout << "currentTargetY: " << currentTarget->y << endl;
        calcTargetY = fitTargetY(calcTargetY);

//		cout << "[WatchBall] ballPosX      : " << ballPositions.at(0)->x << endl;
//		cout << "[WatchBall] ballPosY      : " << ballPositions.at(0)->y << endl;
        //cout << "#####" << endl;
        //cout << "#####" << endl;
        return make_shared < geometry::CNPoint2D > (alloGoalMid->x, calcTargetY);
    }

    double WatchBall::fitTargetY(double targetY)
    {
        //cout << "leftCond : " << targetY * SIMULATING << ">" << alloGoalLeft->y + GOALIE_SIZE / 1.8 << endl;
        //cout << "rightCond: " << targetY * SIMULATING << "<" << alloGoalRight->y - GOALIE_SIZE / 1.8 << endl;
        //cout << "before fitTargetY: " << targetY << endl;

        if (targetY > alloGoalLeft->y)
        {
            //targetY = alloGoalLeft->y;
            //cout << "left" << endl;
            return alloGoalLeft->y;
        }
        else if (targetY < alloGoalRight->y)
        {
            //targetY = alloGoalRight->y;
            //cout << "right" << endl;
            return alloGoalRight->y;
        }
        else
        {
            //targetY = alloGoalMid->y;
            //cout << "mid" << endl;
            return targetY;
        }
        //cout << "after fitTargetY: " << targetY << "\n" << endl;
    }

    int WatchBall::modRingBuffer(int k, int bufferSize)
    {
        return ((k %= bufferSize) < 0) ? k + bufferSize : k;
    }
/*PROTECTED REGION END*/
} /* namespace alica */
