/*
 * Ball.cpp
 *
 *  Created on: Feb 24, 2015
 *      Author: Stefan Jakob
 */

#include <Ball.h>
#include "MSLWorldModel.h"

#include "container/CNPoint3D.h"
#include "container/CNPoint2D.h"
#include "container/CNVelocity2D.h"
#include "ballTracking/BallIntegrator.h"
#include "ballTracking/ObjectTracker.h"
#include "ballTracking/BallZTracker.h"
#include "ballTracking/TimeHelper.h"
#include "ballTracking/EgoMotionEstimator.h"
#include "ballTracking/SharedMemoryHelper.h"
#include "MSLFootballField.h"

namespace msl
{
	Ball::Ball(MSLWorldModel* wm, int ringbufferLength) :
			ballPosition(ringbufferLength), ballVelocity(ringbufferLength), ballBuf(30)
	{
		haveBallDistanceDynamic = 0;
		hadBefore = false;
		hasBallIteration = 0;
		haveDistance = 0;
		this->wm = wm;
		hasBall = false;
		sc = SystemConfig::getInstance();
		KICKER_DISTANCE = (*this->sc)["Dribble"]->get<double>("Dribble", "KickerDistance", NULL);
		KICKER_ANGLE = M_PI;
		HAVE_BALL_TOLERANCE_DRIBBLE = (*this->sc)["Dribble"]->get<double>("Dribble", "HaveBallToleranceDribble", NULL);
		HAVE_BALL_MAX_ANGLE_DELTA = (*this->sc)["Dribble"]->get<double>("Dribble", "HaveBallMaxAngleDelta", NULL);
		BALL_DIAMETER = (*this->sc)["Rules"]->get<double>("Rules.BallRadius", NULL) * 2;
		lastUpdateReceived = 0;
	}

	Ball::~Ball()
	{
	}

	shared_ptr<geometry::CNPoint2D> Ball::getVisionBallPosition(int index)
	{
		auto x = ballPosition.getLast(index);
		if (x == nullptr || wm->getTime() - x->timeStamp > maxInformationAge)
		{
			return nullptr;
		}
		return x->getInformation();
	}

	shared_ptr<geometry::CNVelocity2D> Ball::getVisionBallVelocity(int index)
	{
		auto x = ballVelocity.getLast(index);
		if (x == nullptr || wm->getTime() - x->timeStamp > maxInformationAge)
		{
			return nullptr;
		}
		return x->getInformation();
	}

	shared_ptr<pair<shared_ptr<geometry::CNPoint2D>, double>> Ball::getVisionBallPositionAndCertaincy(int index)
	{
		shared_ptr<pair<shared_ptr<geometry::CNPoint2D>, double>> ret = make_shared<
				pair<shared_ptr<geometry::CNPoint2D>, double>>();
		auto x = ballPosition.getLast(index);
		if (x == nullptr || wm->getTime() - x->timeStamp > maxInformationAge)
		{
			return nullptr;
		}
		ret->first = x->getInformation();
		ret->second = x->certainty;
		return ret;
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
		if (getVisionBallPosition() != nullptr)
			return getVisionBallPosition();

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
		if (getVisionBallVelocity() != nullptr)
			return getVisionBallVelocity();
		return nullptr;

		// TODO: create buffer for ball velocity, like in getEgoBallPosition(), with this hasBallIteration stuff...
	}

	/**
	 * @return True, if I had the ball for at least one iteration. False otherwise.
	 */
	bool Ball::haveBall()
	{
		return hasBallIteration > 0;
	}

	void Ball::updateBallPos(shared_ptr<geometry::CNPoint2D> ballPos, shared_ptr<geometry::CNVelocity2D> ballVel,
								double certainty)
	{
		InfoTime time = wm->getTime();

		shared_ptr<InformationElement<geometry::CNPoint2D>> ball = make_shared<InformationElement<geometry::CNPoint2D>>(
				ballPos, time);
		ball->certainty = certainty;
		ballPosition.add(ball);

		shared_ptr<InformationElement<geometry::CNVelocity2D>> ballV = make_shared<
				InformationElement<geometry::CNVelocity2D>>(ballVel, time);
		ballV->certainty = certainty;
		ballVelocity.add(ballV);
		updateHaveBall();
	}

	void Ball::updateOnLocalizationData(unsigned long long imageTime)
	{
		if (lastUpdateReceived > 0 && lastUpdateReceived == imageTime)
		{
			processHypothesis();
		}
	}

	void Ball::updateOnBallHypothesisList(unsigned long long imageTime)
	{
		if (lastUpdateReceived > 0 && lastUpdateReceived == imageTime)
		{
			processHypothesis();
		}
	}

	void Ball::processHypothesis()
	{
		static int noBallCycles = 0;
		bool ballIntegrated = false;

		ObservedPoint ballPos;
		ROIData ballROI;

		// Integrate balls from directed camera
		ObservedPoint * opDirected = SharedMemoryHelper::getInstance()->readDirectedBallPosition();

		for (int i = 0; i < 10; i++)
		{
			if (opDirected[i].valid)
			{
				BallIntegrator::getInstance()->integratePoint(opDirected[i], 1000.0);
			}
		}

		// Integrate balls from Kinect
		opDirected = SharedMemoryHelper::getInstance()->readKinectBallPosition();

		for (int i = 0; i < 10; i++)
		{
			if (opDirected[i].valid)
			{
				BallIntegrator::getInstance()->integratePoint(opDirected[i], 1000.0);
			}
		}

		ballPos.x = 0.0;
		ballPos.y = 0.0;
		ballPos.z = 0.0;
		ballPos.confidence = 0.0;
		ballPos.valid = false;
		Point3D p;
		auto ballList = wm->rawSensorData.getBallHypothesisList();

		//Its clearly wring that both have the same time but we don't know anything better here! :(
		TimeHelper::getInstance()->setVisionTimeDirected(ballList->imageTime);
		TimeHelper::getInstance()->setVisionTimeOmniCam(ballList->imageTime);
		if (ballList->hypothesis.size() == 0)
			BallIntegrator::getInstance()->integratePoint(ballPos, 1000.0);
		auto ownPosition = wm->rawSensorData.getOwnPositionVision();
		bool inField;

		for (int i = 0; i < ballList->hypothesis.size(); i++)
		{
			double relFactor = 200;
			ballPos.x = ballList->hypothesis[i].egoPosition.x;
			ballPos.y = ballList->hypothesis[i].egoPosition.y;
			ballPos.z = ballList->hypothesis[i].egoPosition.z;

			if (ownPosition == nullptr)
			{
				inField = true;
			}
			else
			{
				double alloBallPosX = ownPosition->x;
				double alloBallPosY = ownPosition->y;

				alloBallPosX += cos(ownPosition->theta) * p.x - sin(ownPosition->theta) * p.y;
				alloBallPosY += sin(ownPosition->theta) * p.x + cos(ownPosition->theta) * p.y;

				if (fabs(alloBallPosX) < MSLFootballField::FieldLength / 2.0 + relFactor
						&& fabs(alloBallPosY) < MSLFootballField::FieldWidth / 2.0 + relFactor)
				{
					inField = true;
				}
			}

			if (!inField)
				continue;
			if (p.z < -300)
			{
				continue;
			}

			if (p.z > 350)
				continue;
			if (p.x * p.x + p.y * p.y > 8000 * 8000)
				continue;
			//if(isGoalie && p.x*p.x+p.y*p.y>5500*5500) continue;

			ballPos.confidence = 0.3 + (ballList->hypothesis[i].errors * 0.1);
			if (ballList->hypothesis[i].radius > 5)
				ballPos.confidence += 0.2;
			if (p.x * p.x + p.y * p.y < 1000 * 1000)
				ballPos.confidence = 0.8;
			if (p.z > 350)
			{
				ballPos.confidence *= 0.9;

			}
			if (ballPos.confidence > 0.9)
				ballPos.confidence = 0.9;

			ballPos.timestamp = TimeHelper::getInstance()->getVisionTimeOmniCam();

			BallIntegrator::getInstance()->integratePoint(ballPos, 1000.0);
			ballIntegrated = true;
			noBallCycles = 0;
		}
		if (!ballIntegrated)
			noBallCycles++;

		BallIntegrator::getInstance()->decreaseDirtyPointCertainty();
		ObservedPoint op = BallIntegrator::getInstance()->getPoint();
		op.timestamp = TimeHelper::getInstance()->getVisionTimeOmniCam();

		ObjectContainer * currBallBuf = BallIntegrator::getInstance()->getContainer();

		if (currBallBuf == NULL)
		{
			currBallBuf = &ballBuf;
		}
		currBallBuf->invalidate(400);

		double dist = sqrt((op.x - mv.point.x) * (op.x - mv.point.x) + (op.y - mv.point.y) * (op.y - mv.point.y));
		mv = ObjectTracker::getInstance()->trackObject(currBallBuf->getPoints(), currBallBuf->getSize(),
														currBallBuf->getStartIndex(), currBallBuf->getLastIndex(),
														0.3E07);
		bool noValidPoint = false;

		if (fabs(mv.point.x) > 50000.0 && fabs(mv.point.y) > 50000.0)
			noValidPoint = true;

		ZEstimate ze = BallZTracker::getInstance()->trackObject(currBallBuf->getPoints(), currBallBuf->getSize(),
																currBallBuf->getStartIndex(),
																currBallBuf->getLastIndex());
		geometry::CNPoint3D ballPoint;
		geometry::CNPoint3D ballVelocity;
		MovingObject mv2 = mv;
		mv2.point = allo2Ego(mv.point, BallIntegrator::getInstance()->getRefPosition());
		mv2.velocity = allo2Ego(mv.velocity, BallIntegrator::getInstance()->getRefPosition());

		if (op.valid && !noValidPoint)
		{
			ballPoint.x = (mv2.point.x);
			ballPoint.y = (mv2.point.y);
			ballPoint.z = (ze.z);
			ballVelocity.x = (mv2.velocity.vx);
			ballVelocity.y = (mv2.velocity.vy);
			ballVelocity.z = (ze.vz);

			auto pos = make_shared<geometry::CNPoint2D>(ballPoint.x, ballPoint.y);
			auto vel = make_shared<geometry::CNVelocity2D>(ballVelocity.x, ballVelocity.y);
			this->updateBallPos(pos, vel, op.confidence);
			//Here you can do something for the z-coordinate, e.g. create a point3d
		}
	}

	void Ball::updateHaveBall()
	{
		if (hasBallIteration > 0)
		{
			// increase the haveBallDistanceDynamic by 2 millimeter to at most HAVE_BALL_TOLERANCE_DRIBBLE
			haveBallDistanceDynamic = min(haveBallDistanceDynamic + 2, HAVE_BALL_TOLERANCE_DRIBBLE);
		}
		else
		{
			// reset the haveBalldistanceDynamic to 0
			haveBallDistanceDynamic = 0;
		}

		shared_ptr<geometry::CNPoint2D> ballPos = getVisionBallPosition();
		if (ballPos == nullptr)
		{
			// TODO predict ball, Endy do it!

			// if you don't see the ball, further pretend that you have it for at most 2 iterations
			hasBallIteration = max(min(--hasBallIteration, 2), 0);

			//cout << "Ball: NullPointer check failed!" << endl;
			return;
		}

		lastKnownBallPos = ballPos;

		// check distance to ball
		if (KICKER_DISTANCE + haveBallDistanceDynamic < ballPos->length())
		{
			// if you lost the ball, further pretend that you have it for at most 2 iterations
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
			// if you lost the ball, further pretend that you have it for at most 2 iterations
			hasBallIteration = max(min(--hasBallIteration, 2), 0);
			//cout << "Ball: Angle Tolerance check failed!" << endl;
			return;
		}

		hasBallIteration = max(min(++hasBallIteration, 2), 0);
	}

	// TODO: this is broken, as nobody enters true into the ring buffer
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

	double Ball::getBallDiameter()
	{
		return BALL_DIAMETER;
	}

	Point Ball::allo2Ego(Point p, Position pos)
	{
		Point ego;
		double x = p.x - pos.x;
		double y = p.y - pos.y;
		double angle = atan2(y, x) - pos.heading;
		double dist = sqrt(x * x + y * y);

		ego.x = cos(angle) * dist;
		ego.y = sin(angle) * dist;

		return ego;
	}

	//TODO impelement light barrier and use sim
	bool Ball::simpleHaveBallDribble(bool hadBefore)
	{
//		bool lightBarrierRunning = wm.SimpleMonitor.ActuatorRunning;
//		if (lightBarrierWasRunning && !lightBarrierRunning)
//		{
//			SwitchLightBarrier(false);
//		}
//		else if (!lightBarrierWasRunning && lightBarrierRunning)
//		{
//			//SwitchLightBarrier(true);
//		}
//		if (wm.UseSimulator)
//		{
//			if (wm.EgoBallPosition != null)
//			{
//				return wm.EgoBallPosition.Distance() < 130;
//			}
//			return false;
//		}
//		if (HAVE_LIGHT_BARRIER == 1)
//		{
//			return HaveBall(wm, true);
//		}
//		else
//		{
		if (!hadBefore)
		{
			haveDistance = 0;
		}
		else
		{
			haveDistance += 2; //TODO: this won't work, this method is called arbitrarily often in each iteration
			haveDistance = min(haveDistance, HAVE_BALL_TOLERANCE_DRIBBLE);
		}

		bool ret = true;
		shared_ptr<geometry::CNPoint2D> ballPos = wm->ball.getEgoBallPosition();
		if (ballPos == nullptr)
		{
			return false;
		}

		double angle = ballPos->angleTo();
		double ballDist = ballPos->length();

		double usedKicker;
//			if ((0.0 <= angle) && (angle < 2 * Math.PI / 3))
//			{ //Kicker 3
//				if (hadBefore)
//				{
//					if (KICKER3_DISTANCE + haveDistance < ballDist)
//						ret = false;
//				}
//				else if (KICKER3_DISTANCE < ballDist)
//					ret = false;
//				usedKicker = KICKER3_ANGLE;
//			}
//			else if ((0.0 > angle) && (angle > -2 * Math.PI / 3))
//			{ //Kicker 2
//				if (hadBefore)
//				{
//					if (KICKER2_DISTANCE + haveDistance < ballDist)
//						ret = false;
//				}
//				else if (KICKER2_DISTANCE < ballDist)
//					ret = false;
//				usedKicker = KICKER2_ANGLE;
//
//			}
//			else
//			{ //Kicker 1
		if (hadBefore)
		{
			if (KICKER_DISTANCE + haveDistance < ballDist)
			{
				ret = false;
			}
		}
		else if (KICKER_DISTANCE < ballDist)
		{
			ret = false;
		}
		usedKicker = KICKER_ANGLE;

//			}
		//calc angle tolerance to the ball
		double tmpTol = angle - usedKicker;
		//Normalize rotation
		if (tmpTol < -M_PI)
		{
			tmpTol += 2.0 * M_PI;
		}
		else if (tmpTol > M_PI)
		{
			tmpTol -= 2.0 * M_PI;
		}
		tmpTol = abs(tmpTol);

		if (tmpTol > HAVE_BALL_MAX_ANGLE_DELTA)
			ret = false;

		return ret;
//		}
	}

	Velocity Ball::allo2Ego(Velocity vel, Position pos)
	{
		Velocity ego;
		double angle = atan2(vel.vy, vel.vx) - pos.heading;
		double length = sqrt(vel.vx * vel.vx + vel.vy * vel.vy);
		ego.vx = cos(angle) * length;
		ego.vy = sin(angle) * length;
		return ego;
	}

	bool Ball::haveBallDribble(bool hadBefore)
	{
		bool ret = (simpleHaveBallDribble(hadBefore) && (hadBefore || this->hasBallIteration > 4));
		this->hadBefore = ret;
		return ret;
	}
}

/* namespace alica */

