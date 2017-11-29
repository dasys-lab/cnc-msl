#pragma once

#include "InfoBuffer.h"
#include "InformationElement.h"
#include "ballTracking/ObjectContainer.h"
#include "ballTracking/TrackingTypes.h"

#include <nonstd/optional.hpp>
#include <cnc_geometry/CNPointAllo.h>
#include <cnc_geometry/CNPointEgo.h>
#include <cnc_geometry/CNVecAllo.h>
#include <cnc_geometry/CNVecEgo.h>
#include <msl_sensor_msgs/SharedWorldInfo.h>
#include "MSLEnums.h"
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

    nonstd::optional<geometry::CNPointAllo> getAlloSharedBallPosition();
    std::shared_ptr<std::pair<geometry::CNPointAllo, double>> getAlloSharedBallPositionAndCertainty(int index = 0);
    nonstd::optional<geometry::CNPointAllo> getAlloBallGuessPosition() const;
    int getSharedBallSupporter();
    bool ballMovedSignificantly();

    void updateHaveBall();
    void updateOnBallHypothesisList(unsigned long long imageTime);
    void updateOnLocalizationData(unsigned long long imageTime);
    void processHypothesis();
    void updateBallPos(geometry::CNPointEgo ballPos, geometry::CNVecEgo ballVel,
                       double certainty);
    void processSharedWorldModelData(msl_sensor_msgs::SharedWorldInfo &data);
    nonstd::optional<bool> getTeamMateBallPossession(int teamMateId);
    bool getOppBallPossession(int index = 0);
    double getBallDiameter();

    nonstd::optional<geometry::CNPointAllo> getBallPickupPosition();

    void updateSharedBall();
    void updateBallGuess();
    double calculateSharedBallMassVector(bool withGoalie);
    bool simpleHaveBallDribble(bool hadBefore);
    bool hadBefore;
    BallPossessionStatus getBallPossessionStatus();
    bool closeToTheBall()
    {
        return selfInBallPossesion;
    };

    const InfoBuffer<geometry::CNVecEgo> &getVisionBallVelocityBuffer() const;
    const InfoBuffer<geometry::CNPointEgo> &getVisionBallPositionBuffer() const;

    const InfoBuffer<geometry::CNPointAllo> &getAlloSharedBallPositionBuffer() const;

    nonstd::optional<geometry::CNPointAllo> getPositionAllo() const;
    nonstd::optional<geometry::CNPointEgo> getPositionEgo() const;

    nonstd::optional<geometry::CNVecAllo> getVelocityAllo() const;
    nonstd::optional<geometry::CNVecEgo> getVelocityEgo() const;

  private:
    //TODO change ball tracking
    void updateBallPossession();
    bool robotHasBall(int robotId);
    bool oppHasBall();
    Point allo2Ego(Point p, Position pos);
    Velocity allo2Ego(Velocity vel, Position pos);

    std::mutex sbMutex;
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

    /**
     * The minimum amount of ALICA cycles (the 30 Hz frequency) the ball has to be seen.
     * After passing this value the Ball class will return true for haveBall().
     */
    int MIN_HAVE_BALL_CYCLE;

    /**
     * The amount of ALICA cycles (the 30 Hz frequency) that is historized for the haveBall() detection.
     */
    int AMOUNT_OF_HISTORIZED_CYCLE;
    int hasBallIteration;
    bool hasBall; /**< True if the local robot has the ball */
    BallPossessionStatus ballPossessionStatus;
    double haveBallDistanceDynamic;

    MSLWorldModel *wm;
    supplementary::SystemConfig *sc;

    InfoBuffer<bool> oppBallPossession;
    std::map<int, std::shared_ptr<InfoBuffer<bool>>> ballPossession;
    std::map<int, std::shared_ptr<InfoBuffer<geometry::CNPointAllo>>> ballPositionsByRobot;
    std::map<int, std::shared_ptr<InfoBuffer<geometry::CNVecAllo>>> ballVelocitiesByRobot;
    InfoBuffer<geometry::CNPointAllo> alloSharedBallPositionBuffer;
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
