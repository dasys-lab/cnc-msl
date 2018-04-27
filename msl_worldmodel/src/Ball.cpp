/*
 * Ball.cpp
 *  Test
 *  Created on: Feb 24, 2015
 *      Author: Stefan Jakob
 */

#include "Ball.h"
#include "MSLWorldModel.h"

#include "LightBarrier.h"
#include "MSLFootballField.h"
#include "RawSensorData.h"
#include "Robots.h"
#include "ballTracking/BallIntegrator.h"
#include "ballTracking/BallZTracker.h"
#include "ballTracking/EgoMotionEstimator.h"
#include "ballTracking/ObjectTracker.h"
#include "ballTracking/SharedMemoryHelper.h"
#include "ballTracking/TimeHelper.h"
#include "container/CNPoint2D.h"
#include "container/CNPoint3D.h"
#include "container/CNVelocity2D.h"

namespace msl
{
Ball::Ball(MSLWorldModel *wm, int ringbufferLength)
    : ballPosition(ringbufferLength)
    , ballVelocity(ringbufferLength)
    , ballBuf(30)
    , oppBallPossession(ringbufferLength)
    , sharedBallPosition(ringbufferLength)
    , ballGuessPosition(ringbufferLength)
    , ballVel3D(ringbufferLength)
    , ballPoint3D(ringbufferLength)
{
    this->haveBallDistanceDynamic = 0;
    this->hadBefore = false;
    this->ballInKicker = false;
    this->hasBallIteration = 0;
    this->haveDistance = 0;
    this->selfInBallPossesion = false;
    this->wm = wm;
    this->hasBall = false;
    this->sc = SystemConfig::getInstance();
    this->KICKER_DISTANCE = (*this->sc)["Dribble"]->get<double>("Dribble", "KickerDistance", NULL);
    this->KICKER_ANGLE = M_PI;
    this->HAVE_BALL_TOLERANCE_DRIBBLE = (*this->sc)["Dribble"]->get<double>("Dribble", "HaveBallToleranceDribble", NULL);
    this->HAVE_BALL_MAX_ANGLE_DELTA = (*this->sc)["Dribble"]->get<double>("Dribble", "HaveBallMaxAngleDelta", NULL);
    this->BALL_DIAMETER = (*this->sc)["Rules"]->get<double>("Rules.BallRadius", NULL) * 2;
    this->LOCALIZATION_SUCCESS_CONFIDENCE = (*sc)["Localization"]->get<double>("Localization", "LocalizationSuccess", NULL);
    this->KICKER_DISTANCE_SIMULATOR = 500;
    this->lastUpdateReceived = 0;
    this->sharedBallSupporters = 0;
    this->MIN_HAVE_BALL_CYCLE = (*this->sc)["Dribble"]->get<int>("Dribble", "minHaveBallCycles", NULL);
    this->AMOUNT_OF_HISTORIZED_CYCLE = (*this->sc)["Dribble"]->get<int>("Dribble", "amountOfHistorizedCycles", NULL);
    this->ballPossessionStatus = BallPossessionStatus::NoBallSeen;
    this->oppDistWithoutBallBefore = (*this->sc)["GameState"]->get<double>("GameState.oppDistWithoutBallBefore", NULL);
    this->oppDistWithBallBefore = (*this->sc)["GameState"]->get<double>("GameState.oppDistWithBallBefore", NULL);
    this->oppBallPossessionHystersis = (*this->sc)["GameState"]->get<double>("GameState.oppBallPossessionHystersis", NULL);
    this->VISION_HAVE_BALL_ITERATIONS_AFTER_LOSS = (*this->sc)["Dribble"]->get<int>("Dribble", "visionHaveBallIterationsAfterLoss", NULL);
    this->LIGHTBARRIER_HAVE_BALL_ITERATIONS_AFTER_LOSS = (*this->sc)["Dribble"]->get<int>("Dribble", "lightbarrierHaveBallIterations", NULL);
    this->oppHasBallCounter = 0;
    this->visionHaveBallCounter = 0;
    this->lightbarrierTriggeredCounter = 0;
    if (this->AMOUNT_OF_HISTORIZED_CYCLE < this->MIN_HAVE_BALL_CYCLE)
    {
        throw;
    }
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

shared_ptr<geometry::CNPoint2D> Ball::getBallPickupPosition()
{
    return this->ballPickupPosition;
}

shared_ptr<pair<shared_ptr<geometry::CNPoint2D>, double>> Ball::getVisionBallPositionAndCertaincy(int index)
{
    shared_ptr<pair<shared_ptr<geometry::CNPoint2D>, double>> ret = make_shared<pair<shared_ptr<geometry::CNPoint2D>, double>>();
    auto x = ballPosition.getLast(index);
    if (x == nullptr || wm->getTime() - x->timeStamp > maxInformationAge)
    {
        return nullptr;
    }
    ret->first = x->getInformation();

    if (ret->first == nullptr)
        return nullptr;

    ret->second = x->certainty;
    return ret;
}

shared_ptr<geometry::CNPoint2D> Ball::getAlloBallPosition()
{
    shared_ptr<geometry::CNPoint2D> p;
    auto ownPos = this->wm->rawSensorData->getOwnPositionVision();
    if (ownPos != nullptr)
    {
        p = this->getEgoBallPosition();
        if (p != nullptr)
        {
            p = p->egoToAllo(*ownPos);
        }
    }
    return p;
}

shared_ptr<geometry::CNPoint2D> Ball::getEgoBallPosition()
{
    auto rawBall = getVisionBallPositionAndCertaincy();
    auto lsb = getAlloSharedBallPosition();
    if (rawBall == nullptr)
    {
        // In order to be consistent with the haveBall() return value, return the last known ball...
        if (hasBallIteration > 0 && lastKnownBallPos != nullptr)
        {
            return lastKnownBallPos->clone();
        }

        auto pos = wm->rawSensorData->getOwnPositionVision();
        if (pos == nullptr)
        {
            return nullptr;
        }

        if (lsb != nullptr && pos != nullptr)
        {
            return lsb->alloToEgo(*pos);
        }

        // Here we could use ball guess
        auto guess = getAlloBallGuessPosition();
        if (guess != nullptr)
        {
            return guess->alloToEgo(*pos);
        }
        return nullptr;
    }
    else
    {
        // If we have sharedball AND rawball:
        if (lsb != nullptr)
        {
            // Closer than 2m or closer than 4m with good confidence or we cannot transform to sb to egocoordinates -> Always use rawball!
            auto pos = wm->rawSensorData->getOwnPositionVision();
            if (rawBall->first->length() < 2000 || (rawBall->first->length() < 4000 && rawBall->second > 0.55) || pos == nullptr)
            {

                return rawBall->first->clone();
            }

            // if rawball and sharedball are close to each other: Use rawball
            if (rawBall->first->egoToAllo(*pos)->distanceTo(lsb) < 1500)
            {
                return rawBall->first->clone();
            }

            // otherwise use shared ball
            return lsb->alloToEgo(*pos);
        }

        // if only rawball: use rawball
        return rawBall->first->clone();
    }
}

shared_ptr<geometry::CNVelocity2D> Ball::getEgoBallVelocity()
{
    if (getVisionBallVelocity() != nullptr)
        return getVisionBallVelocity();
    return nullptr;
}

/**
 * @return True, if I had the ball for at least one iteration. False otherwise.
 */
bool Ball::haveBall()
{
    return ballInKicker;
}

bool Ball::ballMovedSiginficantly()
{
    static const int BALLBUFSIZE = 10;
    vector<double> ballVelAngles;
    vector<double> ballVelValues;
    bool ballMoved = true;

    for (int i = 0; i < BALLBUFSIZE; i++)
    {
        auto vel = this->getVisionBallVelocity(i);
        if (vel == nullptr)
        {
            ballMoved = false;
            break;
        }

        ballVelAngles.push_back(atan2(vel->y, vel->x));
        ballVelValues.push_back(sqrt(vel->x * vel->x + vel->y * vel->y));
        if (ballVelValues.at(ballVelValues.size() - 1) < 600.0)
        {
            ballMoved = false;
        }
    }

    if (ballMoved)
    {
        for (int i = 0; i < ballVelAngles.size() - 1; i++)
        {
            for (int j = i + 1; j < ballVelAngles.size(); j++)
            {
                double diffAngle = ballVelAngles[i] - ballVelAngles[j];
                if (diffAngle < -M_PI)
                    diffAngle += 2.0 * M_PI;

                if (diffAngle > M_PI)
                    diffAngle -= 2.0 * M_PI;

                if (abs(diffAngle) > M_PI / 4.0)
                    ballMoved = false;
            }
        }
    }

    if (ballMoved)
    {
        return true;
    }

    return false;
}

void Ball::updateBallPos(shared_ptr<geometry::CNPoint3D> ballPos, shared_ptr<geometry::CNPoint3D> ballVel, double certainty)
{
    InfoTime time = wm->getTime();
    shared_ptr<geometry::CNPoint2D> ball2d;
    if (ballPos != nullptr)
    {
        ball2d = make_shared<geometry::CNPoint2D>(ballPos->x, ballPos->y);
    }

    shared_ptr<InformationElement<geometry::CNPoint2D>> ball = make_shared<InformationElement<geometry::CNPoint2D>>(ball2d, time);
    ball->certainty = certainty;
    ballPosition.add(ball);

    shared_ptr<InformationElement<geometry::CNPoint3D>> ball3D = make_shared<InformationElement<geometry::CNPoint3D>>(ballPos, time);
    ball->certainty = certainty;
    ballPoint3D.add(ball3D);

    shared_ptr<geometry::CNVelocity2D> vel2d;
    if (ballVel != nullptr)
    {
        vel2d = make_shared<geometry::CNVelocity2D>(ballVel->x, ballVel->y);
    }
    shared_ptr<InformationElement<geometry::CNVelocity2D>> ballV = make_shared<InformationElement<geometry::CNVelocity2D>>(vel2d, time);

    shared_ptr<InformationElement<geometry::CNPoint3D>> ballVelInfo3D = make_shared<InformationElement<geometry::CNPoint3D>>(ballVel, time);
    ball->certainty = certainty;
    ballVel3D.add(ballVelInfo3D);

    ballV->certainty = certainty;
    ballVelocity.add(ballV);
    updateSharedBall();
    updateHaveBall();
    updateBallPossession();
    updateBallGuess();
}
shared_ptr<geometry::CNPoint3D> Ball::getBallPoint3D(int index)
{
    auto x = ballPoint3D.getLast(index);
    if (x == nullptr || wm->getTime() - x->timeStamp > maxInformationAge)
    {
        return nullptr;
    }
    return x->getInformation();
}
shared_ptr<geometry::CNPoint3D> Ball::getBallVel3D(int index)
{
    auto x = ballVel3D.getLast(index);
    if (x == nullptr || wm->getTime() - x->timeStamp > maxInformationAge)
    {
        return nullptr;
    }
    return x->getInformation();
}
void Ball::updateBallGuess()
{
    shared_ptr<geometry::CNPoint2D> nguess;
    double guessConf = 0;
    auto sb = getAlloSharedBallPositionAndCertaincy();
    InfoTime time = wm->getTime();

    // If we have a shared ball -> use sb
    if (sb != nullptr && sb->first != nullptr && sb->second > 0.0)
    {
        guessConf = 1;
        nguess = sb->first->clone();
    }
    else
    {
        // Generate guess -> confidence decaying linearly with time!
        auto x = ballGuessPosition.getLast();
        if (x != nullptr)
        {
            // 1s
            if (x->getInformation() != nullptr && wm->getTime() < x->timeStamp + 1000000000)
            {
                guessConf = (double)((1000000000 + x->timeStamp) - wm->getTime()) / 1000000000.0;
                time = x->timeStamp;
                nguess = x->getInformation()->clone();
            }
            else
            {
                guessConf = 0;
            }
        }
    }

    shared_ptr<InformationElement<geometry::CNPoint2D>> bgi = make_shared<InformationElement<geometry::CNPoint2D>>(nguess, time);
    bgi->certainty = guessConf;
    ballGuessPosition.add(bgi);
}

void Ball::updateBallPossession()
{
    if (this->haveBall())
    {
        if (ballPickupPosition == nullptr && getAlloBallPosition() != nullptr)
        {
            ballPickupPosition = getAlloBallPosition();
        }
    }
    else
    {
        ballPickupPosition.reset();
    }

    if (!this->selfInBallPossesion)
    {
        if (!this->haveBall())
            return;
    }

    auto myEgoBall = this->getEgoBallPosition();
    if (myEgoBall == nullptr)
    {
        this->selfInBallPossesion = false;
    }
    else if (myEgoBall->length() < 450 || (this->selfInBallPossesion && myEgoBall->length() < 600))
    {
        if (wm->lightBarrier->mayUseLightBarrier() && !wm->isUsingSimulator())
        {
            if (wm->rawSensorData->getLightBarrier())
            {

                this->selfInBallPossesion = true;
            }
            else
            {
                this->selfInBallPossesion = false;
            }
        }
        else
        {
            this->selfInBallPossesion = true;
        }
    }
    else
    {
        this->selfInBallPossesion = false;
    }
}

void Ball::updateSharedBall()
{
    std::lock_guard<std::mutex> lock(sbMutex);
    sbvotingList.clear();

    double unknown = 0.4;
    double sure = 0.95;

    double m = 0.5 / (sure - unknown);
    double t = 1.0 - sure * m;

    for (auto &pair : wm->robots->sharedWolrdModelData)
    {
        if (pair.first == 1) // No Shared ball for goalie!
        {
            continue;
        }

        // no shared world model from this robot
        if (pair.second->getLast() == nullptr)
        {
            continue;
        }

        // Use nothing older than 300ms
        if (wm->getTime() - pair.second->getLast()->timeStamp > 300000000)
        {
            continue;
        }

        auto shwmdata = pair.second->getLast()->getInformation();
        if (shwmdata == nullptr || shwmdata->ball.confidence < 0.1 || shwmdata->odom.certainty < LOCALIZATION_SUCCESS_CONFIDENCE)
        {
            continue;
        }
        shared_ptr<geometry::CNPoint2D> point = make_shared<geometry::CNPoint2D>(shwmdata-> ball.point.x, shwmdata->ball.point.y);
        double thresholdSqr = 1000000.0;

        bool found = false;
        double distSqr;
        for (auto &bv : sbvotingList)
        {
            distSqr = (bv.ballPos->x - point->x) * (bv.ballPos->x - point->x) + (bv.ballPos->y - point->y) * (bv.ballPos->y - point->y);
            if (distSqr < thresholdSqr)
            {
                found = true;
                bv.ballPos->x = (bv.ballPos->x * bv.supporters.size() + point->x) / (bv.supporters.size() + 1);
                bv.ballPos->y = (bv.ballPos->y * bv.supporters.size() + point->y) / (bv.supporters.size() + 1);

                double conf2 = m * bv.teamConfidence + t;
                double inp2 = m * shwmdata->ball.confidence + t;

                conf2 = (conf2 * inp2) / (conf2 * inp2 + (1 - conf2) * (1 - inp2));

                bv.teamConfidence = (conf2 - t) / m;

                bv.supporters.push_back(shwmdata);
            }
        }

        if (!found)
        {
            BallVoting bv;
            bv.ballPos = point;
            bv.teamConfidence = shwmdata->ball.confidence;
            bv.supporters.push_back(shwmdata);

            sbvotingList.push_back(bv);
        }
    }

    BallVoting *bestVoting = nullptr;
    double maxConfidence = 0.0;

    for (BallVoting &bv : sbvotingList)
    {
        if (bv.teamConfidence > maxConfidence)
        {
            maxConfidence = bv.teamConfidence;
            bestVoting = &bv;
        }
    }

    auto sb = make_shared<geometry::CNPoint2D>(0.0, 0.0);
    if (bestVoting != nullptr)
    {
        sb = bestVoting->ballPos;
        sharedBallSupporters = bestVoting->supporters.size();
    }
    else
    {
        sb.reset();
        sharedBallSupporters = 0;
    }
    InfoTime time = wm->getTime();

    shared_ptr<InformationElement<geometry::CNPoint2D>> sbi = make_shared<InformationElement<geometry::CNPoint2D>>(sb, time);
    sbi->certainty = bestVoting == nullptr ? 0.0 : bestVoting->teamConfidence;
    sharedBallPosition.add(sbi);
}

int Ball::getSharedBallSupporter()
{
    return sharedBallSupporters;
}

BallPossessionStatus Ball::getBallPossessionStatus()
{
    return this->ballPossessionStatus;
}

void Ball::updateHaveBall()
{
	if ( (wm->lightBarrier->mayUseLightBarrier() && !wm->isUsingSimulator()) && (wm->rawSensorData->getLightBarrier()) )
	{
		if(lightbarrierTriggeredCounter < LIGHTBARRIER_HAVE_BALL_ITERATIONS_AFTER_LOSS) {
			if (!this->ballInKicker) {
				this->lightbarrierTriggeredCounter = this->LIGHTBARRIER_HAVE_BALL_ITERATIONS_AFTER_LOSS + 10;
			} else {
				this->lightbarrierTriggeredCounter = this->LIGHTBARRIER_HAVE_BALL_ITERATIONS_AFTER_LOSS;
			}
		}
	}

	if ( getVisionBallPosition() != nullptr // kicerdistance + dynamic_distance < ballpos.length
			&& ((!wm->isUsingSimulator() &&  (getVisionBallPosition()->length() < (KICKER_DISTANCE + HAVE_BALL_TOLERANCE_DRIBBLE)))
					|| (wm->isUsingSimulator() &&  getVisionBallPosition()->length() < KICKER_DISTANCE_SIMULATOR)) )
	{
	    this->visionHaveBallCounter = this->VISION_HAVE_BALL_ITERATIONS_AFTER_LOSS;
	}

	this->lightbarrierTriggeredCounter--;
	//this->visionHaveBallCounter--;

	if ((wm->lightBarrier->mayUseLightBarrier() && lightbarrierTriggeredCounter <= 0)// || visionHaveBallCounter <= 0
			|| this->lightbarrierTriggeredCounter > this->LIGHTBARRIER_HAVE_BALL_ITERATIONS_AFTER_LOSS)
	{
		this->ballInKicker = false;
		//if (visionHaveBallCounter <= 0) this->ballPossessionStatus = BallPossessionStatus::NoBallSeen;
		if (lightbarrierTriggeredCounter <= 0) this->ballPossessionStatus = BallPossessionStatus::LightBarrierUnblocked;
		return;
	}
	this->ballInKicker = true;
	this->ballPossessionStatus = BallPossessionStatus::HaveBall;
}

bool Ball::robotHasBall(int robotId)
{

    auto shwmData = wm->robots->getSHWMData(robotId);
    if (shwmData == nullptr)
    {
        return false;
    }
    return shwmData->ballInPossession;
}

void Ball::processSharedWorldModelData(msl_sensor_msgs::SharedWorldInfo &data)
{
    if (ballPositionsByRobot.find(data.senderID) == ballPositionsByRobot.end())
    {
        shared_ptr<RingBuffer<InformationElement<geometry::CNPoint2D>>> buffer =
            make_shared<RingBuffer<InformationElement<geometry::CNPoint2D>>>(wm->getRingBufferLength());
        pair<int, shared_ptr<RingBuffer<InformationElement<geometry::CNPoint2D>>>> pair(data.senderID, buffer);
        ballPositionsByRobot.insert(pair);
    }
    shared_ptr<InformationElement<geometry::CNPoint2D>> info =
        make_shared<InformationElement<geometry::CNPoint2D>>(make_shared<geometry::CNPoint2D>(data.ball.point.x, data.ball.point.y), wm->getTime());
    ballPositionsByRobot.at(data.senderID)->add(info);
    if (ballVelocitiesByRobot.find(data.senderID) == ballVelocitiesByRobot.end())
    {
        shared_ptr<RingBuffer<InformationElement<geometry::CNVelocity2D>>> buffer =
            make_shared<RingBuffer<InformationElement<geometry::CNVelocity2D>>>(wm->getRingBufferLength());
        pair<int, shared_ptr<RingBuffer<InformationElement<geometry::CNVelocity2D>>>> pair(data.senderID, buffer);
        ballVelocitiesByRobot.insert(pair);
    }
    shared_ptr<InformationElement<geometry::CNVelocity2D>> i = make_shared<InformationElement<geometry::CNVelocity2D>>(
        make_shared<geometry::CNVelocity2D>(data.ball.velocity.vx, data.ball.velocity.vy), wm->getTime());
    ballVelocitiesByRobot.at(data.senderID)->add(i);
    if (ballPossession.find(data.senderID) == ballPossession.end())
    {
        shared_ptr<RingBuffer<InformationElement<bool>>> buffer = make_shared<RingBuffer<InformationElement<bool>>>(wm->getRingBufferLength());
        pair<int, shared_ptr<RingBuffer<InformationElement<bool>>>> pair(data.senderID, buffer);
        ballPossession.insert(pair);
    }
    shared_ptr<InformationElement<bool>> in = make_shared<InformationElement<bool>>(make_shared<bool>(data.ballInPossession), wm->getTime());
    ballPossession.at(data.senderID)->add(in);
    bool r = oppHasBall();
    shared_ptr<InformationElement<bool>> inf = make_shared<InformationElement<bool>>(make_shared<bool>(r), wm->getTime());
    oppBallPossession.add(inf);
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
    auto x = oppBallPossession.getLast(index);
    if (x == nullptr || wm->getTime() - x->timeStamp > maxInformationAge)
    {
        return nullptr;
    }
    return x->getInformation();
}

shared_ptr<geometry::CNPoint2D> msl::Ball::getAlloSharedBallPosition(int index)
{
    auto x = sharedBallPosition.getLast(index);
    if (x == nullptr || wm->getTime() - x->timeStamp > maxInformationAge)
    {
        return nullptr;
    }
    return x->getInformation();
}

shared_ptr<pair<shared_ptr<geometry::CNPoint2D>, double>> Ball::getAlloSharedBallPositionAndCertaincy(int index)
{
    shared_ptr<pair<shared_ptr<geometry::CNPoint2D>, double>> ret = make_shared<pair<shared_ptr<geometry::CNPoint2D>, double>>();
    auto x = sharedBallPosition.getLast(index);
    if (x == nullptr || wm->getTime() - x->timeStamp > maxInformationAge)
    {
        return nullptr;
    }
    ret->first = x->getInformation();
    ret->second = x->certainty;
    return ret;
}

shared_ptr<geometry::CNPoint2D> msl::Ball::getAlloBallGuessPosition(int index)
{
    auto x = ballGuessPosition.getLast(index);
    if (x == nullptr || wm->getTime() - x->timeStamp > maxInformationAge)
    {
        return nullptr;
    }
    return x->getInformation();
}

bool Ball::oppHasBall()
{
    shared_ptr<geometry::CNPoint2D> ballPos = wm->ball->getEgoBallPosition();
    if (ballPos == nullptr)
    {
        return false;
    }
    auto ops = wm->robots->opponents.getOpponentsEgoClustered();
    if (ops == nullptr)
    {
        return false;
    }
    double minDist = 100000;
    if (ops->size() > 0)
    {
        for (int i = 0; i < ops->size(); ++i)
        {
            geometry::CNPoint2D obstacle(ops->at(i)->x, ops->at(i)->y);
            minDist = min(obstacle.distanceTo(ballPos), minDist);
        }
    }

    auto before = getOppBallPossession(0);
    if (before != nullptr && *before)
    {
        if(minDist <= this->oppDistWithoutBallBefore)
        {
        	this->oppHasBallCounter = min(++this->oppHasBallCounter , this->oppBallPossessionHystersis);
        }
        else
        {
        	this->oppHasBallCounter = max(--this->oppHasBallCounter , 0);
        }
    }
    else
    {
       if(minDist <= this->oppDistWithBallBefore)
       {
    	   this->oppHasBallCounter = min(++this->oppHasBallCounter , this->oppBallPossessionHystersis);
       }
       else
       {
       	this->oppHasBallCounter = max(--this->oppHasBallCounter , 0);
       }
    }
    return this->oppHasBallCounter == this->oppBallPossessionHystersis;
}

double Ball::getBallDiameter()
{
    return BALL_DIAMETER;
}

// TODO impelement light barrier and use sim
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
        haveDistance += 2; // TODO: this won't work, this method is called arbitrarily often in each iteration
        haveDistance = min(haveDistance, HAVE_BALL_TOLERANCE_DRIBBLE);
    }

    bool ret = true;
    shared_ptr<geometry::CNPoint2D> ballPos = wm->ball->getEgoBallPosition();
    if (ballPos == nullptr)
    {
        return false;
    }

    double angle = ballPos->angleTo();
    double ballDist = ballPos->length();

    double usedKicker;

    if (hadBefore)
    {
        if ((KICKER_DISTANCE + haveDistance < ballDist && !wm->isUsingSimulator()) ||
            (KICKER_DISTANCE_SIMULATOR + haveDistance < ballPos->length() && wm->isUsingSimulator()))
        {
            ret = false;
        }
    }
    else if ((KICKER_DISTANCE < ballDist && !wm->isUsingSimulator()) || (KICKER_DISTANCE_SIMULATOR < ballPos->length() && wm->isUsingSimulator()))
    {
        ret = false;
    }
    usedKicker = KICKER_ANGLE;

    // calc angle tolerance to the ball
    double tmpTol = angle - usedKicker;
    // Normalize rotation
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
}

double Ball::getBallConfidenceVision(int index)
{
    auto x = ballPosition.getLast(index);
    if (x == nullptr || wm->getTime() - x->timeStamp > maxInformationAge)
    {
        return 0;
    }
    return x->certainty;
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

// to be removed because it is only used in Intercept for setting success and in own corner behaviour
bool Ball::haveBallDribble(bool hadBefore)
{
    bool ret = (simpleHaveBallDribble(hadBefore) && (hadBefore || this->hasBallIteration > 4));
    this->hadBefore = ret;
    return ret;
}

void Ball::updateOnLocalizationData(unsigned long long imageTime)
{
    if (lastUpdateReceived > 0 && lastUpdateReceived == imageTime)
    {
        processHypothesis();
    }
    lastUpdateReceived = imageTime;
}

void Ball::updateOnBallHypothesisList(unsigned long long imageTime)
{
    if (lastUpdateReceived > 0 && lastUpdateReceived == imageTime)
    {
        processHypothesis();
    }
    lastUpdateReceived = imageTime;
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

void Ball::processHypothesis()
{
    static int noBallCycles = 0;
    bool ballIntegrated = false;

    ObservedPoint ballPos;
    ROIData ballROI;

    // Integrate balls from directed camera
    ObservedPoint *opDirected = SharedMemoryHelper::getInstance()->readDirectedBallPosition();

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
    auto ballList = wm->rawSensorData->getBallHypothesisList();

    // Its clearly wring that both have the same time but we don't know anything better here! :(
    TimeHelper::getInstance()->setVisionTimeDirected(ballList->imageTime);
    TimeHelper::getInstance()->setVisionTimeOmniCam(ballList->imageTime);
    if (ballList->hypothesis.size() == 0)
        BallIntegrator::getInstance()->integratePoint(ballPos, 1000.0);
    auto ownPosition = wm->rawSensorData->getOwnPositionVision();
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

            if (fabs(alloBallPosX) < wm->field->getFieldLength() / 2.0 + relFactor && fabs(alloBallPosY) < wm->field->getFieldWidth() / 2.0 + relFactor)
            {
                inField = true;
            }
        }

        if (!inField)
        {
            continue;
        }
        if (p.z < -300)
        {
            continue;
        }

        if (p.z > 350)
        {
            continue;
        }
        if (p.x * p.x + p.y * p.y > 8000 * 8000)
        {
            continue;
        }
        // if(isGoalie && p.x*p.x+p.y*p.y>5500*5500) continue;

        ballPos.confidence = 0.3 + (ballList->hypothesis[i].errors * 0.1);
        if (ballList->hypothesis[i].radius > 5)
        {
            ballPos.confidence += 0.2;
        }
        if (p.x * p.x + p.y * p.y < 1000 * 1000)
        {
            ballPos.confidence = 0.8;
        }
        if (p.z > 350)
        {
            ballPos.confidence *= 0.9;
        }
        if (ballPos.confidence > 0.9)
        {
            ballPos.confidence = 0.9;
        }
        ballPos.valid = true;
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

    ObjectContainer *currBallBuf = BallIntegrator::getInstance()->getContainer();

    if (currBallBuf == NULL)
    {
        currBallBuf = &ballBuf;
    }
    currBallBuf->invalidate(400);

    double dist = sqrt((op.x - mv.point.x) * (op.x - mv.point.x) + (op.y - mv.point.y) * (op.y - mv.point.y));
    mv = ObjectTracker::getInstance()->trackObject(currBallBuf->getPoints(), currBallBuf->getSize(), currBallBuf->getStartIndex(), currBallBuf->getLastIndex(),
                                                   0.3E07);
    bool noValidPoint = false;

    if (fabs(mv.point.x) > 50000.0 && fabs(mv.point.y) > 50000.0)
    {
        noValidPoint = true;
    }
    ZEstimate ze =
        BallZTracker::getInstance()->trackObject(currBallBuf->getPoints(), currBallBuf->getSize(), currBallBuf->getStartIndex(), currBallBuf->getLastIndex());
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

        auto pos = make_shared<geometry::CNPoint3D>(ballPoint.x, ballPoint.y, ballPoint.z);
        auto vel = make_shared<geometry::CNPoint3D>(ballVelocity.x, ballVelocity.y, ballVelocity.z);
        this->updateBallPos(pos, vel, op.confidence);
        // Here you can do something for the z-coordinate, e.g. create a point3d
    }
}
}

/* namespace alica */
