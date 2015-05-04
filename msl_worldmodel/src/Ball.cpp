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
		sc = SystemConfig::getInstance();
		KICKER_DISTANCE = (*this->sc)["Dribble"]->get<double>("Dribble", "KickerDistance", NULL);
		KICKER_ANGLE = M_PI;
		HAVE_BALL_TOLERANCE_DRIBBLE = (*this->sc)["Dribble"]->get<double>("Dribble", "HaveBallToleranceDribble", NULL);
		HAVE_BALL_MAX_ANGLE_DELTA = (*this->sc)["Dribble"]->get<double>("Dribble", "HaveBallMaxAngleDelta", NULL);
	}

	Ball::~Ball()
	{
	}

	shared_ptr<CNPoint2D> Ball::getAlloBallPosition()
	{
		shared_ptr<CNPoint2D> p;
		auto ownPos = this->wm->rawSensorData.getOwnPositionVision();
		if (ownPos.operator bool())
		{
			p = this->getEgoBallPosition();
			p = p->egoToAllo(*ownPos);
		}
		return p;
	}

	shared_ptr<CNPoint2D> Ball::getEgoBallPosition()
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

	bool Ball::haveBall()
	{
		return hasBallIteration > 0;
	}

	void Ball::updateOnWorldModelData()
	{
		if (hasBallIteration > 0)
		{
			haveBallDistanceDynamic = min (haveBallDistanceDynamic + 2, HAVE_BALL_TOLERANCE_DRIBBLE);
		}
		else
		{
			haveBallDistanceDynamic = 0;
		}

		shared_ptr<CNPoint2D> ballPos = this->wm->rawSensorData.getBallPosition();
		if (ballPos == nullptr)
		{
			// TODO predict ball, Endy do it!
			hasBallIteration = max(min(--hasBallIteration, 2), 0);
			return;
		}

		lastKnownBallPos = ballPos;


		// check distance to ball
		if (KICKER_DISTANCE + haveBallDistanceDynamic < ballPos->length())
		{
			hasBallIteration = max(min(--hasBallIteration, 2), 0);
			return;
		}

		// check angle to ball
		if (abs(ballPos->angleTo()) > HAVE_BALL_MAX_ANGLE_DELTA)
		{
			hasBallIteration = max(min(--hasBallIteration, 2), 0);
			return;
		}

		hasBallIteration = max(min(++hasBallIteration, 2), 0);
	}

	shared_ptr<CNPoint2D> Ball::getEgoRawBallPosition()
	{
		return this->wm->rawSensorData.getBallPosition();
	}

} /* namespace alica */
