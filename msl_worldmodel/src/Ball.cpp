/*
 * Ball.cpp
 *
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

using std::atan;
using std::cos;
using std::make_shared;
using nonstd::nullopt;
using nonstd::optional;
using supplementary::SystemConfig;

namespace msl
{

    Ball::Ball(MSLWorldModel *wm, int ringbufferLength) :
            visionBallPositionBuffer(ringbufferLength), visionBallVelocityBuffer(ringbufferLength), ballBuf(30), oppBallPossession(
                    ringbufferLength), sharedBallPosition(ringbufferLength), ballGuessPosition(ringbufferLength)
    {
        haveBallDistanceDynamic = 0;
        hadBefore = false;
        hasBallIteration = 0;
        haveDistance = 0;
        selfInBallPossesion = false;
        this->wm = wm;
        hasBall = false;
        sc = SystemConfig::getInstance();
        KICKER_DISTANCE = (*this->sc)["Dribble"]->get<double>("Dribble", "KickerDistance", NULL);
        KICKER_ANGLE = M_PI; //TODO: rotate ego positions: rotate KICKER_ANGLE
        HAVE_BALL_TOLERANCE_DRIBBLE = (*this->sc)["Dribble"]->get<double>("Dribble", "HaveBallToleranceDribble", NULL);
        HAVE_BALL_MAX_ANGLE_DELTA = (*this->sc)["Dribble"]->get<double>("Dribble", "HaveBallMaxAngleDelta", NULL);
        BALL_DIAMETER = (*this->sc)["Rules"]->get<double>("Rules.BallRadius", NULL) * 2;
        LOCALIZATION_SUCCESS_CONFIDENCE = (*sc)["Localization"]->get<double>("Localization", "LocalizationSuccess",
                                                                             NULL);
        KICKER_DISTANCE_SIMULATOR = 432;
        lastUpdateReceived = 0;
        sharedBallSupporters = 0;
    }

    Ball::~Ball()
    {
    }

    const InfoBuffer<geometry::CNPointEgo> &Ball::getVisionBallPositionBuffer() const
    {
        return this->visionBallPositionBuffer;
    }

    const InfoBuffer<geometry::CNVecEgo> &Ball::getVisionBallVelocityBuffer() const
    {
        return this->visionBallVelocityBuffer;
    }

    optional<geometry::CNPointAllo> Ball::getBallPickupPosition()
    {
        return this->ballPickupPosition;
    }

    optional<geometry::CNPointAllo> Ball::getPositionAllo() const
    {
        auto ownPos = this->wm->rawSensorData->getOwnPositionVisionBuffer().getLastValidContent();

        if (!ownPos)
        {
            return nullopt;
        }

        auto egoBallPos = this->getPositionEgo();
        if (!egoBallPos)
        {
            return nullopt;
        }

        return egoBallPos->toAllo(*ownPos);
    }

    optional<geometry::CNPointEgo> Ball::getPositionEgo() const
    {
        auto rawBallInfo = this->visionBallPositionBuffer.getLastValid();
        auto sharedBallInfo = this->sharedBallPosition.getLast();

        if (rawBallInfo == nullptr)
        {
            // In order to be consistent with the haveBall() return value, return the last known ball...
            if (hasBallIteration > 0)
            {
                // TODO: use buffer (validity time etc)
                return lastKnownBallPos;
            }

            auto ownPosInfo = wm->rawSensorData->getOwnPositionVisionBuffer().getLastValid();
            if (ownPosInfo == nullptr)
            {
                return nullopt;
            }
            auto ownPos = ownPosInfo->getInformation();

            if (sharedBallInfo)
            {
                return sharedBallInfo->getInformation().toEgo(ownPos);
            }

            // Here we could use ball guess
            auto guess = getAlloBallGuessPosition();
            if (guess)
            {
                return guess->toEgo(ownPos);
            }
            return nullopt;
        }
        else
        {
            auto rawBall = rawBallInfo->getInformation();

            // If we have sharedball AND rawball:
            if (sharedBallInfo)
            {
                // Closer than 2m or closer than 4m with good confidence or we cannot transform to sb to egocoordinates ->
                // Always use rawball!

                auto sharedBallPos = sharedBallInfo->getInformation();

                auto ownPosInfo = wm->rawSensorData->getOwnPositionVisionBuffer().getLastValid();
                if (ownPosInfo == nullptr)
                {
                    return nullopt;
                }
                auto ownPos = ownPosInfo->getInformation();

                if (rawBall.length() < 2000 || (rawBall.length() < 4000 && rawBallInfo->getCertainty() > 0.55)
                        || ownPosInfo == nullptr)
                {
                    return rawBall;
                }

                // if rawball and sharedball are close to each other: Use rawball
                if (rawBall.toAllo(ownPos).distanceTo(sharedBallPos) < 1500)
                {
                    return rawBall;
                }

                // otherwise use shared ball
                return sharedBallPos.toEgo(ownPos);
            }

            // if only rawball: use rawball
            return rawBall;
        }
    }

    /**
     * @return True, if I had the ball for at least one iteration. False otherwise.
     */
    bool Ball::haveBall()
    {
        return hasBallIteration > 0;
    }

    bool Ball::ballMovedSignificantly()
    {
        static const int BALLBUFSIZE = 10;
        vector<double> ballVelAngles;
        vector<double> ballVelValues;
        bool ballMoved = true;

        for (int i = 0; i < BALLBUFSIZE; i++)
        {
            auto velInfo = this->visionBallVelocityBuffer.getLast(i);
            if (velInfo == nullptr)
            {
                ballMoved = false;
                break;
            }

            auto vel = velInfo->getInformation();
            ballVelAngles.push_back(atan2(vel.y, vel.x));
            ballVelValues.push_back(sqrt(vel.x * vel.x + vel.y * vel.y));
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

        return ballMoved;
    }

    optional<geometry::CNPointAllo> Ball::getAlloBallGuessPosition() const
    {
        return ballGuessPosition.getLastValidContent();
    }

    void Ball::updateBallPos(geometry::CNPointEgo ballPos, geometry::CNVecEgo ballVel, double certainty)
    {
        InfoTime time = this->wm->getTime();
        auto ball = make_shared<InformationElement<geometry::CNPointEgo>>(ballPos, time, this->maxValidity, certainty);
        this->visionBallPositionBuffer.add(ball);

        auto ballV = make_shared<InformationElement<geometry::CNVecEgo>>(ballVel, time, this->maxValidity, certainty);
        this->visionBallVelocityBuffer.add(ballV);

        updateSharedBall();
        updateHaveBall();
        updateBallPossession();
        updateBallGuess();
    }

    void Ball::updateBallGuess()
    {
        geometry::CNPointAllo guessedPoint;
        double confidence = 0;
        auto sharedBallInfo = this->sharedBallPosition.getLastValid();
        InfoTime time = wm->getTime();

        // If we have a shared ball -> use sb
        if (sharedBallInfo != nullptr && sharedBallInfo->getCertainty() > 0.0)
        {
            confidence = 1;
            guessedPoint = sharedBallInfo->getInformation();
        }
        else
        {
            // Generate guess -> confidence decaying linearly with time!
            auto x = ballGuessPosition.getLast();
            if (x != nullptr)
            {
                // 1s
                if (wm->getTime() < x->getCreationTime() + 1000000000)
                {
                    confidence = (double)((1000000000 + x->getCreationTime()) - wm->getTime()) / 1000000000.0;
                    time = x->getCreationTime();
                    guessedPoint = x->getInformation();
                }
                else
                {
                    confidence = 0;
                }
            }
        }

        // TODO: change validity duration
        auto ballGuessInfo = make_shared<InformationElement<geometry::CNPointAllo>>(guessedPoint, time,
                                                                                    this->maxValidity, confidence);
        ballGuessPosition.add(ballGuessInfo);
    }

    void Ball::updateBallPossession()
    {
        if (this->haveBall())
        {
            if (!this->ballPickupPosition && this->getPositionAllo())
            {
                ballPickupPosition = this->getPositionAllo();
            }
        }
        else
        {
            this->ballPickupPosition = nullopt;
        }

        if (!this->selfInBallPossesion)
        {
            if (!this->haveBall())
                return;
        }

        auto myEgoBall = this->getPositionEgo();
        if (!myEgoBall)
        {
            this->selfInBallPossesion = false;
        }
        else if (myEgoBall->length() < 450 /*in c# before: 400 changed for simulator*/
        || (this->selfInBallPossesion && myEgoBall->length() < 600))
        {
            if (wm->lightBarrier->mayUseLightBarrier() && !wm->isUsingSimulator())
            {
                auto lightBarrierInfo = wm->rawSensorData->getLightBarrierBuffer().getLastValid();
                this->selfInBallPossesion = lightBarrierInfo && lightBarrierInfo->getInformation();
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

        auto sbVotingList = vector<BallVoting>();

        double unknown = 0.4;
        double sure = 0.95;

        double m = 0.5 / (sure - unknown);
        double t = 1.0 - sure * m;

        for (auto &pair : wm->robots->getSharedWmDataBuffersMap())
        {
            if (pair.first == 1) // No Shared ball for goalie!
            {
                continue;
            }

            // TODO: is lastValid() correct?
            // used to be: use nothing older than 300ms
            // now: get last valid shared worldmodel
            auto shwmdataInfo = pair.second->getLastValid();
            if (shwmdataInfo == nullptr)
            {
                continue;
            }

            auto shWmData = shwmdataInfo->getInformation();
            if (shWmData.ball.confidence < 0.1 || shWmData.odom.certainty < LOCALIZATION_SUCCESS_CONFIDENCE)
            {
                continue;
            }
            auto point = geometry::CNPointAllo(shWmData.ball.point.x, shWmData.ball.point.y);
            double thresholdSqr = 1000000.0;

            bool found = false;
            double distSqr;
            for (auto &bv : sbVotingList)
            {
                distSqr = (bv.ballPos.x - point.x) * (bv.ballPos.x - point.x)
                        + (bv.ballPos.y - point.y) * (bv.ballPos.y - point.y);
                if (distSqr < thresholdSqr)
                {
                    found = true;
                    bv.ballPos.x = (bv.ballPos.x * bv.supporters.size() + point.x) / (bv.supporters.size() + 1);
                    bv.ballPos.y = (bv.ballPos.y * bv.supporters.size() + point.y) / (bv.supporters.size() + 1);

                    double conf2 = m * bv.teamConfidence + t;
                    double inp2 = m * shWmData.ball.confidence + t;

                    conf2 = (conf2 * inp2) / (conf2 * inp2 + (1 - conf2) * (1 - inp2));

                    bv.teamConfidence = (conf2 - t) / m;

                    bv.supporters.push_back(shWmData);
                }
            }

            if (!found)
            {
                BallVoting bv;
                bv.ballPos = point;
                bv.teamConfidence = shWmData.ball.confidence;
                bv.supporters.push_back(shWmData);

                sbVotingList.push_back(bv);
            }
        }

        BallVoting *bestVoting = nullptr;
        double maxConfidence = 0.0;

        for (BallVoting &bv : sbVotingList)
        {
            if (bv.teamConfidence > maxConfidence)
            {
                maxConfidence = bv.teamConfidence;
                bestVoting = &bv;
            }
        }

        auto sb = geometry::CNPointAllo(0.0, 0.0);
        if (bestVoting != nullptr)
        {
            sb = bestVoting->ballPos;
            sharedBallSupporters = (int)bestVoting->supporters.size();
        }
        else
        {
            sharedBallSupporters = 0;
        }
        InfoTime time = wm->getTime();

        double certainty = bestVoting == nullptr ? 0.0 : bestVoting->teamConfidence;
        // TODO: replace maxValidity
        auto sbi = make_shared<InformationElement<geometry::CNPointAllo>>(sb, time, this->maxValidity, certainty);
        sharedBallPosition.add(sbi);
    }

    int Ball::getSharedBallSupporter()
    {
        return sharedBallSupporters;
    }

    void Ball::updateHaveBall()
    {
        // TODO: use time for haveBall
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

        auto ballPosInfo = this->visionBallPositionBuffer.getLastValid();
        if (ballPosInfo == nullptr)
        {
            // if you don't see the ball, further pretend that you have it for at most 2 iterations
            hasBallIteration = max(min(--hasBallIteration, 2), 0);

            // cout << "Ball: NullPointer check failed!" << endl;
            return;
        }

        auto ballPos = ballPosInfo->getInformation();

        lastKnownBallPos = ballPos;

        // check distance to ball
        if ((KICKER_DISTANCE + haveBallDistanceDynamic < ballPos.length() && wm->timeLastSimMsgReceived == 0)
                || (wm->timeLastSimMsgReceived > 0 && KICKER_DISTANCE_SIMULATOR < ballPos.length()))
        {
            // if you lost the ball, further pretend that you have it for at most 2 iterations
            hasBallIteration = max(min(--hasBallIteration, 2), 0);
            //			cout << "Ball: Distance Tolerance check failed! EgoBallDist: " << ballPos->length() <<
            // endl;
            return;
        }

        // turn ball angle by 180°, in order to get a working reference value for the HAVE_BALL_MAX_ANGLE_DELTA parameter
        if (fabs(ballPos.y) > 100)
        {
            // if you lost the ball, further pretend that you have it for at most 2 iterations
            hasBallIteration = max(min(--hasBallIteration, 2), 0);
            // cout << "Ball: Angle Tolerance check failed!" << endl;
            return;
        }

        // is lightbarrier triggered and its no simulation
        if (wm->lightBarrier->mayUseLightBarrier() && !wm->isUsingSimulator())
        {
            auto lightBarrierInfo = wm->rawSensorData->getLightBarrierBuffer().getLastValid();
            if (!(lightBarrierInfo && lightBarrierInfo->getInformation()))
            {
                // if you lost the ball, further pretend that you have it for at most 2 iterations
                hasBallIteration = max(min(--hasBallIteration, 2), 0);
                // cout << "Ball: Angle Tolerance check failed!" << endl;
                return;
            }
        }
        //		double ballAngle = ballPos->angleTo();
        //		if (ballAngle < 0)
        //		{
        //			ballAngle += M_PI;
        //		}
        //		else
        //		{
        //			ballAngle -= M_PI;
        //		}
        //
        //		// check angle to ball
        //		if (abs(ballAngle) > HAVE_BALL_MAX_ANGLE_DELTA)
        //		{
        //			// if you lost the ball, further pretend that you have it for at most 2 iterations
        //			hasBallIteration = max(min(--hasBallIteration, 2), 0);
        //			//cout << "Ball: Angle Tolerance check failed!" << endl;
        //			return;
        //		}

        hasBallIteration = max(min(++hasBallIteration, 2), 0);
    }

    bool Ball::robotHasBall(int robotId)
    {

        auto shwmData = wm->robots->getSHWMDataBuffer(robotId);
        if (shwmData == nullptr)
        {
            return false;
        }

        auto shwmInfo = shwmData->getLastValid();
        if (shwmInfo)
        {
            return shwmInfo->getInformation().ballInPossession;
        }

        return false;
    }

    void Ball::processSharedWorldModelData(msl_sensor_msgs::SharedWorldInfo &data)
    {
        // if buffer for robot does not exist, create one
        if (ballPositionsByRobot.find(data.senderID) == ballPositionsByRobot.end())
        {
            auto buffer = make_shared<InfoBuffer<geometry::CNPointAllo>>(wm->getRingBufferLength());
            auto pair = std::make_pair(data.senderID, buffer);
            ballPositionsByRobot.insert(pair);
        }
        auto ballPositionsInfo = make_shared<InformationElement<geometry::CNPointAllo>>(
                geometry::CNPointAllo(data.ball.point.x, data.ball.point.y), wm->getTime(), this->maxValidity, 1.0); // TODO: certainty?
        ballPositionsByRobot.at(data.senderID)->add(ballPositionsInfo);

        // if buffer for robot does not exist, create one
        if (ballVelocitiesByRobot.find(data.senderID) == ballVelocitiesByRobot.end())
        {
            auto buffer = make_shared<InfoBuffer<geometry::CNVecAllo>>(wm->getRingBufferLength());
            auto pair = make_pair(data.senderID, buffer);
            ballVelocitiesByRobot.insert(pair);
        }
        auto ballVelocitiesInfo = make_shared<InformationElement<geometry::CNVecAllo>>(
                geometry::CNVecAllo(data.ball.velocity.vx, data.ball.velocity.vy), wm->getTime(), this->maxValidity,
                1.0); // TODO: certainty?
        ballVelocitiesByRobot.at(data.senderID)->add(ballVelocitiesInfo);

        // if buffer for robot does not exist, create one
        if (ballPossession.find(data.senderID) == ballPossession.end())
        {
            auto buffer = make_shared<InfoBuffer<bool>>(wm->getRingBufferLength());
            auto pair = make_pair(data.senderID, buffer);
            ballPossession.insert(pair);
        }
        auto ballPossessionInfo = make_shared<InformationElement<bool>>(data.ballInPossession, wm->getTime(),
                                                                        this->maxValidity, 1.0); // TODO: certainty
        ballPossession.at(data.senderID)->add(ballPossessionInfo);

        auto oppBallPossessionInfo = make_shared<InformationElement<bool>>(oppHasBall(), wm->getTime(),
                                                                           this->maxValidity, 1.0); // TODO: certainty
        oppBallPossession.add(oppBallPossessionInfo);
    }

    optional<bool> Ball::getTeamMateBallPossession(int teamMateId)
    {
        if (ballPossession.find(teamMateId) == ballPossession.end())
        {
            return nullopt;
        }
        auto x = ballPossession.at(teamMateId)->getLastValid();
        if (x == nullptr)
        {
            return nullopt;
        }
        return x->getInformation();
    }

    bool Ball::oppHasBall()
    {
        bool ret = false;
        optional<geometry::CNPointEgo> ballPos = this->wm->ball->getPositionEgo();
        if (!ballPos)
        {
            return false;
        }

        auto opsInfo = this->wm->robots->opponents.getOpponentsEgoClusteredBuffer().getLastValid();
        if (opsInfo == nullptr)
        {
            return false;
        }

        auto ops = opsInfo->getInformation();

        double minDist = 100000;
        if (ops.size() > 0)
        {
            for (int i = 0; i < ops.size(); ++i)
            {
                geometry::CNPointEgo obstacle = ops.at(i);
                minDist = min(obstacle.distanceTo(*ballPos), minDist);
            }
        }

        auto before = this->oppBallPossession.getLastValid();

        if (before != nullptr && before->getInformation())
        {
            ret = (minDist <= 900);
        }
        else
        {
            ret = (minDist <= 700);
        }
        return ret;
    }

    double Ball::getBallDiameter()
    {
        return BALL_DIAMETER;
    }

// TODO implement light barrier and use sim
// TODO: rewrite
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
        optional<geometry::CNPointEgo> ballPos = wm->ball->getPositionEgo();
        if (!ballPos)
        {
            return false;
        }

        double angle = ballPos->angleZ();
        double ballDist = ballPos->length();

        if (hadBefore)
        {
            if ((KICKER_DISTANCE + haveDistance < ballDist && wm->timeLastSimMsgReceived == 0)
                    || (KICKER_DISTANCE_SIMULATOR + haveDistance < ballPos->length() && wm->timeLastSimMsgReceived > 0))
            {
                ret = false;
            }
        }
        else if ((KICKER_DISTANCE < ballDist && wm->timeLastSimMsgReceived == 0)
                || (KICKER_DISTANCE_SIMULATOR < ballPos->length() && wm->timeLastSimMsgReceived > 0))
        {
            ret = false;
        }

        // calc angle tolerance to the ball
        double tmpTol = angle - KICKER_ANGLE;
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

// TODO: clean
    void Ball::processHypothesis()
    {
        auto infoBallList = wm->rawSensorData->getBallHypothesisBuffer().getLastValid();
        if (infoBallList == nullptr)
        {
            return;
        }
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
        // Its clearly wrong that both have the same time but we don't know anything better here! :(
        auto ballList = infoBallList->getInformation();
        TimeHelper::getInstance()->setVisionTimeDirected(ballList.imageTime);
        TimeHelper::getInstance()->setVisionTimeOmniCam(ballList.imageTime);
        if (ballList.hypothesis.size() == 0)
        {
            BallIntegrator::getInstance()->integratePoint(ballPos, 1000.0);
        }

        auto ownPositionInfo = this->wm->rawSensorData->getOwnPositionVisionBuffer().getLastValid();
        // TODO was not in itialized before ?
        bool inField = false;
        // TODO why do we think the ball is in the field when we don't know its position
        if (ownPositionInfo == nullptr)
        {
            inField = true;
        }

        auto ownPosition = ownPositionInfo->getInformation();

        for (int i = 0; i < ballList.hypothesis.size(); i++)
        {
            double relFactor = 200;
            ballPos.x = ballList.hypothesis[i].egoPosition.x;
            ballPos.y = ballList.hypothesis[i].egoPosition.y;
            ballPos.z = ballList.hypothesis[i].egoPosition.z;

            double alloBallPosX = ownPosition.x;
            double alloBallPosY = ownPosition.y;

            alloBallPosX += cos(ownPosition.theta) * p.x - sin(ownPosition.theta) * p.y;
            alloBallPosY += sin(ownPosition.theta) * p.x + cos(ownPosition.theta) * p.y;

            if (fabs(alloBallPosX) < wm->field->getFieldLength() / 2.0 + relFactor
                    && fabs(alloBallPosY) < wm->field->getFieldWidth() / 2.0 + relFactor)
            {
                inField = true;
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

            ballPos.confidence = 0.3 + (ballList.hypothesis[i].errors * 0.1);
            if (ballList.hypothesis[i].radius > 5)
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
        mv = ObjectTracker::getInstance()->trackObject(currBallBuf->getPoints(), currBallBuf->getSize(),
                                                       currBallBuf->getStartIndex(), currBallBuf->getLastIndex(),
                                                       0.3E07);
        bool noValidPoint = false;

        if (fabs(mv.point.x) > 50000.0 && fabs(mv.point.y) > 50000.0)
        {
            noValidPoint = true;
        }
        ZEstimate ze = BallZTracker::getInstance()->trackObject(currBallBuf->getPoints(), currBallBuf->getSize(),
                                                                currBallBuf->getStartIndex(),
                                                                currBallBuf->getLastIndex());
        MovingObject mv2 = mv;
        mv2.point = allo2Ego(mv.point, BallIntegrator::getInstance()->getRefPosition());
        mv2.velocity = allo2Ego(mv.velocity, BallIntegrator::getInstance()->getRefPosition());

        if (op.valid && !noValidPoint)
        {
            this->updateBallPos(geometry::CNPointEgo(mv2.point.x, mv2.point.y, ze.z),
                                geometry::CNVecEgo(mv2.velocity.vx, mv2.velocity.vy, ze.vz), op.confidence);
            // Here you can do something for the z-coordinate, e.g. create a point3d
        }
    }

    std::shared_ptr<geometry::CNPointAllo> Ball::getAlloSharedBallPosition(int index)
    {
        //required for Game
        return nullptr;
    }

    std::shared_ptr<std::pair<geometry::CNPointAllo, double> > Ball::getAlloSharedBallPositionAndCertainty(int index)
    {

        //TODO required for MSLWorldModel
        return nullptr;
    }

    bool Ball::getOppBallPossession(int index)
    {
        //required for Game and OppFreeKickInOwnHalf
        return false;
    }

    nonstd::optional<geometry::CNVecAllo> Ball::getVelocityAllo() const
    {
        //required for WatchBall
        return nonstd::nullopt;
    }

    nonstd::optional<geometry::CNVecEgo> Ball::getVelocityEgo() const
    {
        //required for Goalie and some other Behaviours
        return nonstd::nullopt;
    }

} /* namespace alica */
