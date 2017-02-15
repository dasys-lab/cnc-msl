/*
 * Ball.h
 *
 *  Created on: Feb 24, 2015
 *      Author: Stefan Jakob
 */

#ifndef CNC_MSL_MSL_WORLDMODEL_INCLUDE_BALL_H_
#define CNC_MSL_MSL_WORLDMODEL_INCLUDE_BALL_H_

#include "InformationElement.h"
#include "RingBuffer.h"
#include "ballTracking/ObjectContainer.h"
#include "ballTracking/TrackingTypes.h"
#include <Geometry.h>
#include "msl_sensor_msgs/SharedWorldInfo.h"
#include <map>
#include <memory>
#include <nonstd/optional.hpp>

namespace supplementary
{
class SystemConfig;
}

namespace msl
{
class MSLWorldModel;

class BallVoting
{
  public:
    geometry::CNPointAllo ballPos;
    double teamConfidence = 0.0;
    vector<msl_sensor_msgs::SharedWorldInfo> supporters;
};

class Ball
{
  public:
    Ball(MSLWorldModel *wm, int ringbufferLength);
    virtual ~Ball();

    bool haveBall();
    bool haveBallDribble(bool hadBefore);

    nonstd::optional<geometry::CNPointEgo> getVisionBallPosition(int index = 0);
    nonstd::optional<std::pair<geometry::CNPointEgo, double>> getVisionBallPositionAndCertaincy(int index = 0);
    nonstd::optional<geometry::CNVecEgo> getVisionBallVelocity(int index = 0);
    double getBallConfidenceVision(int index = 0);

    nonstd::optional<geometry::CNPointAllo> getAlloBallPosition();
    nonstd::optional<geometry::CNPointEgo> getEgoBallPosition();
    nonstd::optional<geometry::CNVecEgo> getEgoBallVelocity();
    nonstd::optional<geometry::CNPointAllo> getAlloSharedBallPosition(int index = 0);
    nonstd::optional<std::pair<geometry::CNPointAllo, double>> getAlloSharedBallPositionAndCertaincy(int index = 0);
    nonstd::optional<geometry::CNPointAllo> getAlloBallGuessPosition(int index = 0);
    int getSharedBallSupporter();
    bool ballMovedSignificantly();

    void updateHaveBall();
    void updateOnBallHypothesisList(unsigned long long imageTime);
    void updateOnLocalizationData(unsigned long long imageTime);
    void processHypothesis();
    void updateBallPos(nonstd::optional<geometry::CNPointEgo> ballPos, nonstd::optional<geometry::CNVecEgo> ballVel, double certainty);
    void processSharedWorldModelData(msl_sensor_msgs::SharedWorldInfo &data);
    nonstd::optional<bool> getTeamMateBallPossession(int teamMateId, int index = 0);
    nonstd::optional<bool> getOppBallPossession(int index = 0);
    double getBallDiameter();

    nonstd::optional<geometry::CNPointAllo> getBallPickupPosition();

    void updateSharedBall();
    void updateBallGuess();
    double calculateSharedBallMassVector(bool withGoalie);
    bool simpleHaveBallDribble(bool hadBefore);
    bool hadBefore;
    bool closeToTheBall()
    {
        return selfInBallPossesion;
    };

  private:
    std::mutex sbMutex;
    vector<BallVoting> sbvotingList;
    int sharedBallSupporters;

    ObjectContainer ballBuf;
    MovingObject mv;
    unsigned long long lastUpdateReceived;
    nonstd::optional<geometry::CNPoint2D> lastKnownBallPos;
    double HAVE_BALL_TOLERANCE_DRIBBLE;
    double KICKER_DISTANCE;
    double KICKER_DISTANCE_SIMULATOR;
    double KICKER_ANGLE;
    double HAVE_BALL_MAX_ANGLE_DELTA;
    double BALL_DIAMETER;
    double LOCALIZATION_SUCCESS_CONFIDENCE;
    int hasBallIteration;
    bool hasBall; /**< True if the local robot has the ball */
    double haveBallDistanceDynamic;
    unsigned long maxInformationAge = 1000000000;
    MSLWorldModel *wm;
    supplementary::SystemConfig *sc;
    map<int, shared_ptr<RingBuffer<InformationElement<bool>>>> ballPossession;
    RingBuffer<InformationElement<bool>> oppBallPossession;
    map<int, shared_ptr<RingBuffer<InformationElement<geometry::CNPointAllo>>>> ballPositionsByRobot;
    map<int, shared_ptr<RingBuffer<InformationElement<geometry::CNVecAllo>>>> ballVelocitiesByRobot;
    RingBuffer<InformationElement<geometry::CNPointAllo>> sharedBallPosition;
    RingBuffer<InformationElement<geometry::CNPointAllo>> ballGuessPosition;
    RingBuffer<InformationElement<geometry::CNPointEgo>> ballPosition;
    RingBuffer<InformationElement<geometry::CNVecEgo>> ballVelocity;

    bool robotHasBall(int robotId);
    bool oppHasBall();
    Point allo2Ego(Point p, Position pos);
    Velocity allo2Ego(Velocity vel, Position pos);
    double haveDistance;

    bool selfInBallPossesion;
    nonstd::optional<geometry::CNPointAllo> ballPickupPosition;
    void updateBallPossession();
};

} /* namespace alica */

#endif /* CNC_MSL_MSL_WORLDMODEL_INCLUDE_BALL_H_ */
