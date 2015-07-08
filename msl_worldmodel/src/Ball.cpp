/*
 * Ball.cpp
 *
 *  Created on: Feb 24, 2015
 *      Author: Stefan Jakob
 */

#include <Ball.h>
#include "MSLWorldModel.h"

namespace msl
{
	Ball::Ball(MSLWorldModel* wm)
	{
		haveBallDistanceDynamic = 0;
		hadBefore = false;
		hasBallIteration = 0;
		this->wm = wm;
		hasBall = false;
		sc = SystemConfig::getInstance();
		KICKER_DISTANCE = (*this->sc)["Dribble"]->get<double>("Dribble", "KickerDistance", NULL);
		KICKER_ANGLE = M_PI;
		HAVE_BALL_TOLERANCE_DRIBBLE = (*this->sc)["Dribble"]->get<double>("Dribble", "HaveBallToleranceDribble", NULL);
		HAVE_BALL_MAX_ANGLE_DELTA = (*this->sc)["Dribble"]->get<double>("Dribble", "HaveBallMaxAngleDelta", NULL);
	}

	Ball::~Ball()
	{
	}

	shared_ptr<geometry::CNPoint2D> Ball::getAlloBallPosition()
	{
		shared_ptr<geometry::CNPoint2D> p;
		auto ownPos = this->wm->rawSensorData.getOwnPositionVision();
		if (ownPos.operator bool())
		{
			p = this->getEgoBallPosition();
			p = p->egoToAllo(*ownPos);
		}
		return p;
	}

	shared_ptr<geometry::CNPoint2D> Ball::getEgoBallPosition()
	{
		if (this->wm->rawSensorData.getBallPosition() != nullptr)
			return this->wm->rawSensorData.getBallPosition();

		// In order to be consistent with the haveBall() return value, return the last known ball...
		if (hasBallIteration > 0)
		{
			return lastKnownBallPos;
		}
		else
		{
			return nullptr;
		}
	}

	shared_ptr<geometry::CNVelocity2D> Ball::getEgoBallVelocity()
	{
		if (this->wm->rawSensorData.getBallVelocity() != nullptr)
			return this->wm->rawSensorData.getBallVelocity();
		return nullptr;

		// TODO: create buffer for ball velocity, like in getEgoBallPosition(), with this hasBallIteration stuff...
	}

	bool Ball::haveBall()
	{
		return hasBallIteration > 0;
	}

	void Ball::updateOnWorldModelData()
	{
		if (hasBallIteration > 0)
		{
			haveBallDistanceDynamic = min(haveBallDistanceDynamic + 2, HAVE_BALL_TOLERANCE_DRIBBLE);
		}
		else
		{
			haveBallDistanceDynamic = 0;
		}

		shared_ptr<geometry::CNPoint2D> ballPos = this->wm->rawSensorData.getBallPosition();
		if (ballPos == nullptr)
		{
			// TODO predict ball, Endy do it!
			hasBallIteration = max(min(--hasBallIteration, 2), 0);
			//cout << "Ball: NullPointer check failed!" << endl;
			return;
		}

		lastKnownBallPos = ballPos;

		// check distance to ball
		if (KICKER_DISTANCE + haveBallDistanceDynamic < ballPos->length())
		{
			hasBallIteration = max(min(--hasBallIteration, 2), 0);
			//cout << "Ball: Distance Tolerance check failed!" << endl;
			return;
		}

		// turn ball angle by 180°, in order to get a working reference value for the HAVE_BALL_MAX_ANGLE_DELTA parameter
		double ballAngle = ballPos->angleTo();
		if (ballAngle < 0)
		{
			ballAngle += M_PI;
		}
		else
		{
			ballAngle -= M_PI;
		}

		// check angle to ball
		if (abs(ballAngle) > HAVE_BALL_MAX_ANGLE_DELTA)
		{
			hasBallIteration = max(min(--hasBallIteration, 2), 0);
			//cout << "Ball: Angle Tolerance check failed!" << endl;
			return;
		}

		hasBallIteration = max(min(++hasBallIteration, 2), 0);
	}

	shared_ptr<geometry::CNPoint2D> Ball::getEgoRawBallPosition()
	{
		return this->wm->rawSensorData.getBallPosition();
	}

	bool Ball::robotHasBall(int robotId)
	{
		bool ret = false;
		shared_ptr<geometry::CNPoint2D> ballPos = getEgoBallPosition();
		if (ballPos == nullptr)
		{
			ret = false;
		}
		shared_ptr<bool> hadBall = getTeamMateBallPossession(robotId, 1);
		if (hadBall != nullptr && *hadBall == true)
		{
			ret = ballPos->length() < 600;
		}
		else
		{
			ret = ballPos->length() < 400;
		}
		return ret;
	}

	void Ball::processSharedWorldModelData(msl_sensor_msgs::SharedWorldInfo data)
	{
		if (ballPositionsByRobot.find(data.senderID) == ballPositionsByRobot.end())
		{
			shared_ptr<RingBuffer<InformationElement<geometry::CNPoint2D>>> buffer= make_shared<RingBuffer<InformationElement<geometry::CNPoint2D>>>(wm->getRingBufferLength());
			pair<int, shared_ptr<RingBuffer<InformationElement<geometry::CNPoint2D>>>> pair(data.senderID, buffer);
			ballPositionsByRobot.insert(pair);
		}
		shared_ptr<InformationElement<geometry::CNPoint2D>> info = make_shared<InformationElement<geometry::CNPoint2D>>(
				make_shared<geometry::CNPoint2D>(data.ball.point.x, data.ball.point.y),
				wm->getTime());
		ballPositionsByRobot.at(data.senderID)->add(info);
		if (ballVelocitiesByRobot.find(data.senderID) == ballVelocitiesByRobot.end())
		{
			shared_ptr<RingBuffer<InformationElement<geometry::CNVelocity2D>>> buffer= make_shared<RingBuffer<InformationElement<geometry::CNVelocity2D>>>(wm->getRingBufferLength());
			pair<int, shared_ptr<RingBuffer<InformationElement<geometry::CNVelocity2D>>>> pair(data.senderID, buffer);
			ballVelocitiesByRobot.insert(pair);
		}
		shared_ptr<InformationElement<geometry::CNVelocity2D>> i = make_shared<InformationElement<geometry::CNVelocity2D>>(
				make_shared<geometry::CNVelocity2D>(data.ball.velocity.vx, data.ball.velocity.vy),
				wm->getTime());
		ballVelocitiesByRobot.at(data.senderID)->add(i);
		if (ballPossession.find(data.senderID) == ballPossession.end())
		{
			shared_ptr<RingBuffer<InformationElement<bool>>> buffer= make_shared<RingBuffer<InformationElement<bool>>>(wm->getRingBufferLength());
			pair<int, shared_ptr<RingBuffer<InformationElement<bool>>>> pair(data.senderID, buffer);
			ballPossession.insert(pair);
		}
		bool ret = robotHasBall(data.senderID);
		shared_ptr<InformationElement<bool>> in = make_shared<InformationElement<bool>>(
				make_shared<bool>(ret),
				wm->getTime());
		ballPossession.at(data.senderID)->add(in);
		bool r = oppHasBall(data);
		shared_ptr<InformationElement<bool>> inf = make_shared<InformationElement<bool>>(
				make_shared<bool>(r),
				wm->getTime());
		oppBallPossession->add(inf);
	}

	shared_ptr<bool> Ball::getTeamMateBallPossession(int teamMateId, int index)
	{
		if (ballPossession.find(teamMateId) == ballPossession.end())
		{
			return nullptr;
		}
		auto x = ballPossession.at(teamMateId)->getLast(index);
		if (x == nullptr || wm->getTime() - x->timeStamp > maxInformationAge)
		{
			return nullptr;
		}
		return x->getInformation();
	}

	shared_ptr<bool> msl::Ball::getOppBallPossession(int index)
	{
		auto x = oppBallPossession->getLast(index);
		if (x == nullptr || wm->getTime() - x->timeStamp > maxInformationAge)
		{
			return nullptr;
		}
		return x->getInformation();
	}

	shared_ptr<geometry::CNPoint2D> msl::Ball::getSharedBallPosition()
	{
		//TODO implement shared ball
		// replaced by ego ball
		//return sharedBallPosition;
		return this->wm->ball.getEgoBallPosition();
	}

	bool Ball::oppHasBall(msl_sensor_msgs::SharedWorldInfo data)
	{
		bool ret = false;
		double oppDist = 0;
		shared_ptr<geometry::CNPoint2D> ballPos = wm->ball.getEgoBallPosition();
		if (ballPos == nullptr)
		{
			oppDist = 0;
		}
		//TODO GetOpponentListEgoClustered()
		auto ops = data.obstacles;
		double minDist = 100000;
		if (ops.size() == 0)
		{
			return minDist;
		}
		for (int i = 0; i < ops.size(); ++i)
		{
			geometry::CNPoint2D obstacle(ops.at(i).x, ops.at(i).y);
			minDist = min(obstacle.distanceTo(ballPos), minDist);
		}
		return minDist;

		if (getOppBallPossession(1))
		{
			ret = (oppDist <= 900);
		}
		else
		{
			ret = (oppDist <= 700);
		}
		return ret;
	}

}
/* namespace alica */

