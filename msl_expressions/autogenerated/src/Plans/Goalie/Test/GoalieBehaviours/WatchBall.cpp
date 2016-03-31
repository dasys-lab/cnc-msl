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

		if (simulating > 0)
			alloGoalMid = MSLFootballField::posOppGoalMid();
		else
			alloGoalMid = MSLFootballField::posOwnGoalMid();

		alloGoalLeft = make_shared<geometry::CNPoint2D>(alloGoalMid->x, alloGoalMid->y + GOALIE_SIZE / 2 * simulating);
		alloGoalRight = make_shared<geometry::CNPoint2D>(alloGoalMid->x, alloGoalMid->y - GOALIE_SIZE / 2 * simulating);

		if (alloBall == nullptr)
		{
			cout << "Goalie can't see ball! Moving to GoalMid" << endl;
			mc = RobotMovement::moveGoalie(alloGoalMid, alloFieldCntr, SNAP_DIST, alloGoalMid);
			send(mc);
			return;
		}
		else
		{

			if (abs(alloBall->x) > abs(alloGoalMid->x) + 50)
			{
				cout << "Ball is behind goal line" << endl;
				mc = RobotMovement::moveGoalie(alloGoalMid, alloFieldCntr, SNAP_DIST, alloGoalMid);
				send(mc);
				return;
			}
			else
			{
				watchBall();

				//targetIndex = modRingBuffer(targetIndex + 1, TARGET_BUFFER_SIZE);
			}
		}
		//cout << "### WatchBall ###\n" << endl;
		/*PROTECTED REGION END*/
	}
	void WatchBall::initialiseParameters()
	{
		/*PROTECTED REGION ID(initialiseParameters1447863466691) ENABLED START*/ //Add additional options here
		/*PROTECTED REGION END*/
	}
	/*PROTECTED REGION ID(methods1447863466691) ENABLED START*/ //Add additional methods here
	void WatchBall::watchBall()
	{
		bool isNull = false;
		std::vector<shared_ptr<geometry::CNPoint2D>> ballPositions;
		for (int i = 0; i < BALL_BUFFER_SIZE; i++)
		{
			auto currentBall = wm->ball.getVisionBallPosition(i);
			if (currentBall == nullptr)
			{
				cout << "currentBall null" << endl;
				isNull = true;
				//return;
			}
			else
			{

				ballPositions.push_back(currentBall);
				//cout << "index: " << i << " pushBackY: " << currentBall->y << endl;
			}
		}

		for (shared_ptr<geometry::CNPoint2D> pos : ballPositions)
		{
			if (pos == nullptr)
			{
				isNull = true;
				cout << "break" << endl;
				break;
			}
		}
		if (isNull == false)
		{
			std::sort(std::begin(ballPositions), std::end(ballPositions),
						[](shared_ptr<geometry::CNPoint2D> a, shared_ptr<geometry::CNPoint2D> b)
						{
							return b->y >= a->y;
						});

			/*cout << "sorted: " << endl;
			 for(shared_ptr<geometry::CNPoint2D> pos:ballPositions) {
			 cout << pos->y << endl;
			 }*/
			auto begin = ballPositions.at(0);
			auto afterBegin = ballPositions.at(1);
			auto end = ballPositions.at(BALL_BUFFER_SIZE - 1);
			auto preEnd = ballPositions.at(BALL_BUFFER_SIZE - 2);

			double diffBot = end->length() - preEnd->length();
			double diffTop = begin->length() - afterBegin->length();

			if (diffBot > diffTop)
			{
				// not using first position
				//cout << "remove Bot" << endl;
				ballPositions.erase(ballPositions.begin());
				//targetY = (preEnd->y + (end)->y) / (BALL_BUFFER_SIZE - 1);
			}
			else
			{
				// not using last position
				//cout << "remove top" << endl;
				ballPositions.erase(ballPositions.end());
				//targetY = (begin->y + afterBegin->y) / (BALL_BUFFER_SIZE - 1);
			}

			auto alloTarget = calcGoalImpactY(ballPositions);
			prevTarget = alloTarget;

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
			mc = RobotMovement::moveGoalie(alloTarget, alloFieldCntr, SNAP_DIST, alloGoalMid);
			send(mc);
		}
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
			std::vector<shared_ptr<geometry::CNPoint2D>> ballPositions)
	{
		//cout << "#####" << endl;
		double _slope, _yInt;
		double sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
		int nPoints = 0;
		if (prevTarget == nullptr)
		{
			cout << "prevTarget == null!" << endl;
			prevTarget = alloGoalMid;
		}

		/*for (int i = modRingBuffer(ballIndex - nPoints, BALL_BUFFER_SIZE);
		 i < modRingBuffer(ballIndex, BALL_BUFFER_SIZE); i = modRingBuffer(i + 1, BALL_BUFFER_SIZE))
		 {*/

		bool dirChange = false;
		for (std::vector<int>::size_type i = 0; i != ballPositions.size(); i++)
		{
			/*std::cout << ' ' << *it;
			 }
			 for (int i = 0; i < BALL_BUFFER_SIZE; i++)
			 {
			 /*if (ballPosBuffer[i] == nullptr)
			 {
			 // need more points to
			 cout << "calcGoal failed! [ballPosBuffer]" << endl;
			 //return prevTarget;
			 }*/

			auto currentBall = ballPositions.at(i);

			if (currentBall != nullptr)
			{
				shared_ptr<geometry::CNPoint2D> ppprevBall;
				shared_ptr<geometry::CNPoint2D> pprevBall;
				shared_ptr<geometry::CNPoint2D> prevBall;

				if (i > 2)
				{
					ppprevBall = ballPositions.at(i - 3);
					pprevBall = ballPositions.at(i - 2);
					prevBall = ballPositions.at(i - 1);
				}

				if (prevBall != nullptr && pprevBall != nullptr && ppprevBall != nullptr)
				{
					// check if function is continual
					// TODO: correct?
					double diffppp = ppprevBall->y - pprevBall->y;
					double diffpp = pprevBall->y - prevBall->y;
					double diffp = prevBall->y - currentBall->y;
					int buffer = 100;

					// TODO: insert puffer!
					if (diffppp >= diffpp)
					{
						if (diffpp >= diffp)
						{

						}
						else if (diffpp + buffer >= diffp)
						{

						}
						else if (diffpp - buffer >= diffp)
						{

						}
						else
						{
							cout << "corner detected!" << endl;
							dirChange = true;
						}

					}
					else if (diffppp <= diffpp)
					{
						if (diffpp <= diffp)
						{

						}
						else if (diffpp + buffer <= diffp)
						{

						}
						else if (diffpp - buffer <= diffp)
						{

						}
						else
						{
							cout << "corner detected!" << endl;
							dirChange = true;
						}
					}

					/*if (diffppp <= diffpp && diffpp <= diffp)
					 {

					 }
					 else if (diffppp >= diffpp && diffpp >= diffp)
					 {

					 }
					 else
					 {
					 // direction change discovered!
					 dirChange = true;
					//cout << "###### direction change discovered! ######" << endl;
					cout << "diffppp: " << diffppp << endl;
					cout << "diffpp: " << diffpp << endl;
					cout << "diffp: " << diffp << endl;
					 }*/
				}

				sumX = sumX + (currentBall->x);
				sumY = sumY + (currentBall->y);
				sumXY = sumXY + (currentBall->y * currentBall->x);
				sumX2 = sumX2 + (currentBall->x * currentBall->x);
				nPoints = nPoints + 1;
				//cout << "BallX: " << currentBall->x << " BallY: " << currentBall->y << endl;
			}
			else
			{
				cout << "currentBall == nullptr" << endl;
			}
		}
		shared_ptr<geometry::CNPoint2D> currentTarget;

		if (dirChange == true)
		{

			/*for (std::vector<int>::size_type i = 0; i != ballPositions.size(); i++)
			 {
			 cout << "BallX: " << ballPositions.at(i)->x << " BallY: " << ballPositions.at(i)->y << endl;
			 }*/
			currentTarget = prevTarget;
		}
		else
		{

			double xMean = sumX / nPoints;
			double yMean = sumY / nPoints;
			double denominator = sumX2 - sumX * xMean;

			// You can tune the eps (1e-7)
			if (std::fabs(denominator) < 1e-3)
			{
				// Fail: it seems a vertical line
				// todo: do what? this causes some jumping around!
				cout << "calcGoal failed! [vertical Line ]" << endl;
				return prevTarget;
			}

			_slope = (sumXY - sumX * yMean) / denominator;
			_yInt = yMean - _slope * xMean;
			double calcTargetY = _slope * alloGoalMid->x + _yInt;

			//cout << "slope: " << _slope << endl;
			//cout << "yIntercept: " << _yInt << endl;
			//cout << "alloGoalMidX: " << alloGoalMid->x << endl;
			//cout << "calculated yImpact: " << calcTargetY << endl;

			/*if (prevTarget != nullptr && prePrevTarget != nullptr)
			 {
			 int factor = 1;
			 double meanY = (prePrevTarget->y + prevTarget->y * factor + calcTargetY) / 3;
			 string targetPos = fitTargetY(meanY);
			 if (targetPos == LEFT)
			 currentTarget->y = alloGoalLeft->y;
			 else if (targetPos == RIGHT)
			 currentTarget->y = alloGoalRight->y;
			 else
			 currentTarget->y = alloGoalMid->y;
			 }*/

			currentTarget = make_shared<geometry::CNPoint2D>(alloGoalMid->x, fitTargetY(calcTargetY));
			//targetPosBuffer[targetIndex] = currentTarget;
			//cout << "#####" << endl;
			//cout << "currentTargetY: " << currentTarget->y << endl;
			//cout << "calcTargetY   : " << calcTargetY << endl;
			//cout << "ballPosX      : " << wm->ball.getAlloSharedBallPosition()->x << endl;
			//cout << "ballPosY      : " << wm->ball.getAlloSharedBallPosition()->y << endl;
			//cout << "#####" << endl;
			//cout << "#####" << endl;
		}
		return currentTarget;
	}

	double WatchBall::fitTargetY(double targetY)
	{
		//cout << "leftCond : " << targetY * SIMULATING << ">" << alloGoalLeft->y + GOALIE_SIZE / 1.8 << endl;
		//cout << "rightCond: " << targetY * SIMULATING << "<" << alloGoalRight->y - GOALIE_SIZE / 1.8 << endl;
		//cout << "before fitTargetY: " << targetY << endl;

		if (targetY * simulating > alloGoalLeft->y + (GOALIE_SIZE / 1.8))
		{
			//targetY = alloGoalLeft->y;
			//cout << "left" << endl;
			return alloGoalLeft->y;
		}
		else if (targetY * simulating < alloGoalRight->y - (GOALIE_SIZE / 1.8))
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
