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
#include "container/CNPoint2D.h"
#include "container/CNPoint3D.h"
#include "container/CNVelocity2D.h"
#include "msl_sensor_msgs/SharedWorldInfo.h"
#include "MSLEnums.h"
#include <map>
#include <memory>

using namespace std;

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
    shared_ptr<geometry::CNPoint2D> ballPos;
    double teamConfidence = 0.0;
    vector<shared_ptr<msl_sensor_msgs::SharedWorldInfo>> supporters;
};

class Ball
{
  public:
    Ball(MSLWorldModel *wm, int ringbufferLength);
    virtual ~Ball();
    bool haveBall();
    bool haveBallDribble(bool hadBefore);

    shared_ptr<geometry::CNPoint2D> getVisionBallPosition(int index = 0);
    shared_ptr<pair<shared_ptr<geometry::CNPoint2D>, double>> getVisionBallPositionAndCertaincy(int index = 0);
    shared_ptr<geometry::CNVelocity2D> getVisionBallVelocity(int index = 0);
    double getBallConfidenceVision(int index = 0);

    shared_ptr<geometry::CNPoint3D> getBallPoint3D(int index = 0);
    shared_ptr<geometry::CNPoint3D> getBallVel3D(int index = 0);
    shared_ptr<geometry::CNPoint2D> getAlloBallPosition();
    shared_ptr<geometry::CNPoint2D> getEgoBallPosition();
    shared_ptr<geometry::CNVelocity2D> getEgoBallVelocity();
    shared_ptr<geometry::CNPoint2D> getAlloSharedBallPosition(int index = 0);
    shared_ptr<pair<shared_ptr<geometry::CNPoint2D>, double>> getAlloSharedBallPositionAndCertaincy(int index = 0);
    shared_ptr<geometry::CNPoint2D> getAlloBallGuessPosition(int index = 0);
    int getSharedBallSupporter();
    bool ballMovedSiginficantly();

    void updateHaveBall();
    void updateOnBallHypothesisList(unsigned long long imageTime);
    void updateOnLocalizationData(unsigned long long imageTime);
    void processHypothesis();
    void updateBallPos(shared_ptr<geometry::CNPoint3D> ballPos, shared_ptr<geometry::CNPoint3D> ballVel, double certainty);
    void processSharedWorldModelData(msl_sensor_msgs::SharedWorldInfo &data);
    shared_ptr<bool> getTeamMateBallPossession(int teamMateId, int index = 0);
    shared_ptr<bool> getOppBallPossession(int index = 0);
    double getBallDiameter();

    shared_ptr<geometry::CNPoint2D> getBallPickupPosition();

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

  private:
    std::mutex sbMutex;
    vector<BallVoting> sbvotingList;
    int sharedBallSupporters;

    ObjectContainer ballBuf;
    MovingObject mv;
    unsigned long long lastUpdateReceived;
    shared_ptr<geometry::CNPoint2D> lastKnownBallPos;
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
    unsigned long maxInformationAge = 1000000000;
    MSLWorldModel *wm;
    supplementary::SystemConfig *sc;
    map<int, shared_ptr<RingBuffer<InformationElement<bool>>>> ballPossession;
    RingBuffer<InformationElement<bool>> oppBallPossession;
    map<int, shared_ptr<RingBuffer<InformationElement<geometry::CNPoint2D>>>> ballPositionsByRobot;
    map<int, shared_ptr<RingBuffer<InformationElement<geometry::CNVelocity2D>>>> ballVelocitiesByRobot;
    RingBuffer<InformationElement<geometry::CNPoint2D>> sharedBallPosition;
    RingBuffer<InformationElement<geometry::CNPoint2D>> ballGuessPosition;
    RingBuffer<InformationElement<geometry::CNPoint2D>> ballPosition;
    RingBuffer<InformationElement<geometry::CNVelocity2D>> ballVelocity;

    RingBuffer<InformationElement<geometry::CNPoint3D>> ballPoint3D;
    RingBuffer<InformationElement<geometry::CNPoint3D>> ballVel3D;

    bool robotHasBall(int robotId);
    bool oppHasBall();
    Point allo2Ego(Point p, Position pos);
    Velocity allo2Ego(Velocity vel, Position pos);
    double haveDistance;

    bool selfInBallPossesion;
    shared_ptr<geometry::CNPoint2D> ballPickupPosition;
    void updateBallPossession();
};

} /* namespace alica */

#endif /* CNC_MSL_MSL_WORLDMODEL_INCLUDE_BALL_H_ */
