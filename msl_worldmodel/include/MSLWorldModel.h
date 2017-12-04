#pragma once

#include <iostream>
#include <list>
#include <mutex>
#include <tuple>

#include <ros/ros.h>

#include <EventTrigger.h>
#include <ITrigger.h>
#include <MSLEnums.h>
#include <supplementary/InfoBuffer.h>
#include <supplementary/WorldModel.h>

namespace std_msgs
{
ROS_DECLARE_MESSAGE(Bool)
}
namespace msl_msgs
{
ROS_DECLARE_MESSAGE(JoystickCommand)
}
namespace msl_actuator_msgs
{
ROS_DECLARE_MESSAGE(IMUData)
ROS_DECLARE_MESSAGE(RawOdometryInfo)
ROS_DECLARE_MESSAGE(MotionBurst)
}
namespace msl_sensor_msgs
{
ROS_DECLARE_MESSAGE(WorldModelData)
ROS_DECLARE_MESSAGE(SimulatorWorldModelData)
ROS_DECLARE_MESSAGE(CorrectedOdometryInfo)
ROS_DECLARE_MESSAGE(SharedWorldInfo)
ROS_DECLARE_MESSAGE(BallHypothesisList)
}
namespace gazebo_msgs
{
ROS_DECLARE_MESSAGE(ModelStates)
}
namespace msl_helper_msgs
{
ROS_DECLARE_MESSAGE(PassMsg)
ROS_DECLARE_MESSAGE(WatchBallMsg)
}
namespace alica
{
class AlicaEngine;
}
namespace supplementary
{
class SystemConfig;
}
namespace geometry
{
class CNPointAllo;
class CNPositionAllo;
}

namespace msl
{

class RawSensorData;
class Robots;
class Ball;
class Game;
class WhiteBoard;
class MSLFootballField;
class Prediction;
class Monitoring;
class LightBarrier;
class Obstacles;
class PathPlanner;
class MSLSharedWorldModel;
class Calibration;

class MSLWorldModel : public supplementary::WorldModel
{
  public:
    static MSLWorldModel *get();

    void onRawOdometryInfo(msl_actuator_msgs::RawOdometryInfoPtr msg);
    void onBallHypothesisList(msl_sensor_msgs::BallHypothesisListPtr msg);
    void onWorldModelData(msl_sensor_msgs::WorldModelDataPtr msg);
    void onJoystickCommand(msl_msgs::JoystickCommandPtr msg);
    void onMotionBurst(msl_actuator_msgs::MotionBurstPtr msg);
    void onSimWorldModel(msl_sensor_msgs::SimulatorWorldModelDataPtr msg);
    void onGazeboModelState(gazebo_msgs::ModelStatesPtr msg);
    void onSharedWorldInfo(msl_sensor_msgs::SharedWorldInfoPtr msg);
    void onPassMsg(msl_helper_msgs::PassMsgPtr msg);
    void onWatchBallMsg(msl_helper_msgs::WatchBallMsgPtr msg);
    void onCorrectedOdometryInfo(msl_sensor_msgs::CorrectedOdometryInfoPtr msg);
    void onLightBarrierInfo(std_msgs::BoolPtr msg);
    void onIMUData(msl_actuator_msgs::IMUDataPtr msg);

    MSLSharedWorldModel *getSharedWorldModel();
    void sendSharedWorldModelData();

    int getRingBufferLength();
    supplementary::ITrigger *getVisionDataEventTrigger();

    bool isUsingSimulator();

    // Raw Sensor Data
    RawSensorData *rawSensorData;

    // Processed Sensor Data
    Monitoring *monitoring;
    Robots *robots;
    Ball *ball;
    Game *game;
    Obstacles *obstacles;
    Prediction *prediction;
    LightBarrier *lightBarrier;

    PathPlanner *pathPlanner;
    MSLFootballField *field;
    WhiteBoard *whiteBoard;

    Calibration* calibration;

    supplementary::EventTrigger visionTrigger;
    supplementary::InfoTime timeLastSimMsgReceived;

  private:
    MSLWorldModel();
    virtual ~MSLWorldModel();

    supplementary::ITrigger *visionDataEventTrigger;

    int ringBufferLength;

    MSLSharedWorldModel *sharedWorldModel;

    ros::NodeHandle n;
    ros::Subscriber rawOdomSub;
    ros::Subscriber wmDataSub;
    ros::Subscriber wmBallListSub;
    ros::Subscriber joystickSub;
    ros::Subscriber motionBurstSub;
    ros::Subscriber simWorldModelSub;
    ros::Subscriber gazeboWorldModelSub;
    ros::Subscriber sharedWorldSub;
    ros::Subscriber passMsgSub;
    ros::Subscriber watchBallMsgSub;
    ros::Publisher sharedWorldPub;
    ros::Subscriber correctedOdometrySub;
    ros::Subscriber lightBarrierSub;
    ros::Subscriber imuDataSub;

    mutex wmMutex;
    mutex joystickMutex;
    mutex motionBurstMutex;
    mutex correctedOdemetryMutex;
    ros::AsyncSpinner *spinner;

	};

} /* namespace msl */
