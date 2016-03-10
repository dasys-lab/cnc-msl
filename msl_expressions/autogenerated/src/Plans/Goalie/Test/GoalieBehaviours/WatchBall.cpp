using namespace std;
#include "Plans/Goalie/Test/GoalieBehaviours/WatchBall.h"

/*PROTECTED REGION ID(inccpp1447863466691) ENABLED START*/ //Add additional includes here
#include "robotmovement/RobotMovement.h"
#include <cmath>
#include <vector>
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
		//cout << "### WatchBall ###" << endl;
		me = wm->rawSensorData.getOwnPositionVision();
		alloBall = wm->ball.getAlloBallPosition();
		// alloGoalMid = MSLFootballField::posOwnGoalMid();
		alloGoalMid = MSLFootballField::posOppGoalMid();

		alloFieldCenter = MSLFootballField::posCenterMarker();

		if (me == nullptr || alloGoalMid == nullptr)
		{
			cout << "me: " << me->toString() << ", goalMid: " << alloGoalMid->toString() << endl;
			return;
		}
		else if (alloBall == nullptr)
		{
			cout << "Goalie can't see ball!" << endl;
			mc = RobotMovement::moveToPointFast(alloGoalMid->alloToEgo(*me), alloFieldCenter->alloToEgo(*me), 100, 0);
			send(mc);
		}
		else
		{

			//double targetX = alloGoalMid->x + 100;
			//double targetY = wm->ball.getAlloBallPosition()->y;
			shared_ptr<geometry::CNPoint2D> alloTarget;
			shared_ptr<geometry::CNPoint2D> egoAlignPoint = alloBall->alloToEgo(*me);

			double leftGoalPostY = MSLFootballField::posLeftOwnGoalPost()->y;
			 double rightGoalPostY = MSLFootballField::posRightOwnGoalPost()->y;

			 int goalieHalfSize = 315; // 630mm/2 + 140mm = 445mm
			 //int extendedArmWidth = 140;
			 int ballRadius = (int)wm->ball.getBallDiameter() / 2; // Umfang 68cm => Radius 10.8225cm

			 //int puffer = ballRadius + goalieHalfSize + extendedArmWidth;
			 int puffer = ballRadius + goalieHalfSize;

			 /*if (targetY <= rightGoalPostY || (targetY > rightGoalPostY && targetY <= rightGoalPostY + puffer))
			 {
			 targetY = rightGoalPostY + puffer;
			 }
			 else if (targetY >= leftGoalPostY || (targetY < leftGoalPostY && targetY >= leftGoalPostY - puffer))
			 {
			 targetY = leftGoalPostY - puffer;
			 }
			 // else, ballY is between goalposts and not closer than ballRadius + robot's CenterToArmDistance away from goal posts
			 */


			// fill buffer
			ballPosBuffer[currentIndex] = alloBall;

			alloTarget = calcGoalImpact(ballPosBuffer, 20, puffer);
			currentIndex = (currentIndex + 1) % RING_BUFFER_SIZE;
			//cout << "ballVel X: " << wm->ball.getEgoBallVelocity()->x << " Y: " << wm->ball.getEgoBallVelocity()->y << endl;
			auto egoBallVel = wm->ball.getEgoBallVelocity();

			if (alloTarget != nullptr && abs(egoBallVel->x) > 100 && abs(egoBallVel->y) > 100)
			{
				if(egoAlignPoint->distanceTo(me) < 1000) {
					egoAlignPoint = oldEgoAlignPoint;
				} else {
					oldEgoAlignPoint = egoAlignPoint;
				}
				cout << "alloBallX: " << alloTarget->x << " alloBallY: " << alloTarget->y << endl;
				mc = RobotMovement::moveToPointFast(alloTarget->alloToEgo(*me), egoAlignPoint, 100, 0);
				send(mc);
			}
			else
			{
				//cout << "alloTarget null" << endl;
			}
			//cout << "### WatchBall ###\n" << endl;
		}
		/*PROTECTED REGION END*/
	}
	void WatchBall::initialiseParameters()
	{
		/*PROTECTED REGION ID(initialiseParameters1447863466691) ENABLED START*/ //Add additional options here
		/*PROTECTED REGION END*/
	}
	/*PROTECTED REGION ID(methods1447863466691) ENABLED START*/ //Add additional methods here
	shared_ptr<geometry::CNPoint2D> WatchBall::calcGoalImpact(shared_ptr<geometry::CNPoint2D> ballPosBuffer[],
																int nPoints, int puffer)
	{

		double _slope, _yInt;

		double sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
		for (int i = modRingBuffer(currentIndex - nPoints); i < modRingBuffer(currentIndex); i = modRingBuffer(i + 1))
		{
			if (ballPosBuffer[i] == nullptr)
			{
				return nullptr;
			}
			sumX += ballPosBuffer[i]->x;
			sumY += ballPosBuffer[i]->y;
			sumXY += ballPosBuffer[i]->y * ballPosBuffer[i]->x;
			sumX2 += ballPosBuffer[i]->x * ballPosBuffer[i]->x;
		}
		double xMean = sumX / nPoints;
		double yMean = sumY / nPoints;
		double denominator = sumX2 - sumX * xMean;
		// You can tune the eps (1e-7) below for your specific task
		if (std::fabs(denominator) < 1e-7)
		{
			// Fail: it seems a vertical line
			return nullptr;
		}
		_slope = (sumXY - sumX * yMean) / denominator;
		_yInt = yMean - _slope * xMean;
		// todo: remove minus

		double goalY = _slope * alloGoalMid->x + _yInt;
		double goalX = alloGoalMid->x - 100;
		if(goalY < -1000) {
			goalY = -1000 + puffer;
		} else if(goalY > 1000) {
			goalY = 1000 - puffer;
		}

		return make_shared<geometry::CNPoint2D>(goalX, goalY);
	}

	int WatchBall::ballMovesTowardsGoal(int nrOfPos)
	{
		int isMovingCloserIter = 0;

		int threshhold = 7; // ball has to move at least 7 iterations closer to goal
		// cout << "ego ball vel: " << egoBallVelocity->x << "|" << egoBallVelocity->y << " " << egoBallVelocity->length() << endl;

		int firstIndex = modRingBuffer(currentIndex - nrOfPos);
		if (ballPosBuffer[firstIndex] == nullptr)
		{
			cout << isMovingCloserIter << " last ball positions were moving closer to goal!" << endl;
			return 0;
		}
		int firstBallX = round(ballPosBuffer[firstIndex]->x / 10) * 10;

		for (int i = modRingBuffer(firstIndex + 1); i < modRingBuffer(currentIndex); i = modRingBuffer(i + 1))
		{
			// TODO: figure out if ball is coming closer to goal
			int nextBallX = round(ballPosBuffer[i]->x / 10) * 10;

			int prevIndex = modRingBuffer((i - 1));
			int prevBallX = round(ballPosBuffer[prevIndex]->x / 10) * 10;

			if (nextBallX - prevBallX)
			{
				isMovingCloserIter++;
			}
			/*else if (prevIndex != firstIndex)
			 {
			 prevIndex = modRingBuffer(prevIndex);
			 prevBallX = ballPosBuffer[prevIndex]->x;

			 if (nextBallX - prevBallX)
			 {
			 isMovingCloserIter++;
			 }
			 else
			 {
			 isMovingCloserIter--;
			 }

			 }
			 else
			 {
			 isMovingCloserIter--;
			 }*/
		}
		//return false;
		cout << isMovingCloserIter << endl;
		return isMovingCloserIter - threshhold;
	}

	void WatchBall::createCSV(bool movingTowardsGoal)
	{
		auto fileName = "ballCoords.csv";
		int i = 0;
		ofstream outFile(fileName, std::ios::out);
		outFile << "X, Y, Z" << endl;
		for (shared_ptr<geometry::CNPoint2D> pos : ballPosBuffer)
		{
			if (pos != nullptr)
			{
				outFile << round(pos->x / 10) * 10 << ',' << round(pos->y / 10) * 10 << ',' << round(pos->z / 10) * 10;
				//cout << "X: " << pos->x << " Y: " << pos->y;
				if (i == currentIndex)
				{
					outFile << ',' << "CURRENT";
					//cout << " ###" << endl;
				}
				else
					outFile << ',' << "";
				if (movingTowardsGoal)
				{
					outFile << ',' << "MOVING TOWARDS GOAL";
				}
				else
					outFile << ',' << "";
				outFile << endl;
				//cout << " " << i << endl;
			}
			else
			{
				//cout << "null" << endl;
			}
			i++;
		}
		outFile.close();
	}

	int WatchBall::modRingBuffer(int k)
	{
		return ((k %= RING_BUFFER_SIZE) < 0) ? k + RING_BUFFER_SIZE : k;
	}
/*PROTECTED REGION END*/
} /* namespace alica */
