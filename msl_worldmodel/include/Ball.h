#pragma once

#include "InfoBuffer.h"
#include "InformationElement.h"
#include "ballTracking/ObjectContainer.h"
#include "ballTracking/TrackingTypes.h"

#include <nonstd/optional.hpp>
#include <Geometry.h>
#include <msl_sensor_msgs/SharedWorldInfo.h>

#include <map>
#include <memory>

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
    std::vector<msl_sensor_msgs::SharedWorldInfo> supporters;
};

class Ball
{
  public:
    Ball(MSLWorldModel *wm, int ringbufferLength);
    virtual ~Ball();
    bool haveBall();
    bool haveBallDribble(bool hadBefore);

    std::shared_ptr<geometry::CNVecAllo> getBallVel3D(int index = 0);
    nonstd::optional<geometry::CNPointAllo> getAlloBallPosition();
    nonstd::optional<geometry::CNPointEgo> getEgoBallPosition();
    std::shared_ptr<geometry::CNVecEgo> getEgoBallVelocity();
    std::shared_ptr<geometry::CNPointAllo> getAlloSharedBallPosition(int index = 0);
    std::shared_ptr<std::pair<geometry::CNPointAllo, double>> getAlloSharedBallPositionAndCertaincy(int index = 0);
    std::shared_ptr<geometry::CNPointAllo> getAlloBallGuessPosition(int index = 0);
    int getSharedBallSupporter();
    bool ballMovedSiginficantly();

    void updateHaveBall();
    void updateOnBallHypothesisList(unsigned long long imageTime);
    void updateOnLocalizationData(unsigned long long imageTime);
    void processHypothesis();
    void updateBallPos(geometry::CNPointEgo ballPos, geometry::CNVecEgo ballVel,
                       double certainty);
    void processSharedWorldModelData(msl_sensor_msgs::SharedWorldInfo &data);
    bool getTeamMateBallPossession(int teamMateId, int index = 0);
    bool getOppBallPossession(int index = 0);
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

    const InfoBuffer<geometry::CNVecEgo> &getVisionBallVelocityBuffer() const;
    const InfoBuffer<geometry::CNPointEgo> &getVisionBallPositionBuffer() const;

  private:
    //TODO change ball tracking
    void updateBallPossession();
    bool robotHasBall(int robotId);
    bool oppHasBall();
    Point allo2Ego(Point p, Position pos);
    Velocity allo2Ego(Velocity vel, Position pos);

    std::mutex sbMutex;
    std::vector<BallVoting> sbvotingList;
    int sharedBallSupporters;

    ObjectContainer ballBuf;
    MovingObject mv;
    unsigned long long lastUpdateReceived;
    nonstd::optional<geometry::CNPointEgo> lastKnownBallPos;
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

    MSLWorldModel *wm;
    supplementary::SystemConfig *sc;

    InfoBuffer<bool> oppBallPossession;
    std::map<int, std::shared_ptr<InfoBuffer<bool>>> ballPossession;
    std::map<int, std::shared_ptr<InfoBuffer<geometry::CNPointAllo>>> ballPositionsByRobot;
    std::map<int, std::shared_ptr<InfoBuffer<geometry::CNVecAllo>>> ballVelocitiesByRobot;
    InfoBuffer<geometry::CNPointAllo> sharedBallPosition;
    InfoBuffer<geometry::CNPointAllo> ballGuessPosition;
    InfoBuffer<geometry::CNVecEgo> visionBallVelocityBuffer;
    InfoBuffer<geometry::CNPointEgo> visionBallPositionBuffer;

    // TODO: add validityDurations for each buffer
    const InfoTime maxValidity = 1000000000;

    double haveDistance;

    bool selfInBallPossesion;
    nonstd::optional<geometry::CNPointAllo> ballPickupPosition;

};
} /* namespace msl */
