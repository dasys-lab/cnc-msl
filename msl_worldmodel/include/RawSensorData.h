#pragma once

#include "InformationElement.h"
#include "InfoBuffer.h"

#include <msl_actuator_msgs/HaveBallInfo.h>
#include <msl_actuator_msgs/IMUData.h>
#include <msl_actuator_msgs/MotionBurst.h>
#include <msl_actuator_msgs/MotionControl.h>
#include <msl_actuator_msgs/RawOdometryInfo.h>
#include <msl_msgs/JoystickCommand.h>
#include <msl_msgs/MotionInfo.h>
#include <msl_sensor_msgs/BallHypothesisList.h>
#include <msl_sensor_msgs/BallInfo.h>
#include <msl_sensor_msgs/WorldModelData.h>
#include <std_msgs/Bool.h>

#include <cnc_geometry/CNPointAllo.h>
#include <cnc_geometry/CNPositionAllo.h>
#include <cnc_geometry/CNVecAllo.h>

#include <vector>
#include <memory>

namespace msl
{
using std::shared_ptr;
using std::vector;

class MSLWorldModel;
class RawSensorData
{

  public:
    RawSensorData(MSLWorldModel *wm, int ringBufferLength);
    virtual ~RawSensorData();
    shared_ptr<vector<double>> getDistanceScan(int index = 0);
    bool getLightBarrier(int index = 0);
    shared_ptr<geometry::CNPointAllo> getOpticalFlow(int index = 0);
    double getOpticalFlowQoS(int index = 0);
    shared_ptr<geometry::CNPositionAllo> getOwnPositionMotion(int index = 0);
    shared_ptr<geometry::CNPositionAllo> getOwnPositionVision(int index = 0);
    shared_ptr<std::pair<shared_ptr<geometry::CNPositionAllo>, double>> getOwnPositionVisionAndCertaincy(int index = 0);
    shared_ptr<msl_msgs::MotionInfo> getOwnVelocityMotion(int index = 0);
    shared_ptr<msl_msgs::MotionInfo> getOwnVelocityVision(int index = 0);
    shared_ptr<msl_actuator_msgs::MotionControl> getLastMotionCommand(int index = 0);
    shared_ptr<int> getCompassOrientation(int index = 0);
    shared_ptr<msl_msgs::JoystickCommand> getJoystickCommand(int index = 0);
    shared_ptr<msl_sensor_msgs::CorrectedOdometryInfo> getCorrectedOdometryInfo(int index = 0);
    shared_ptr<msl_sensor_msgs::BallHypothesisList> getBallHypothesisList(int index = 0);
    void processWorldModelData(msl_sensor_msgs::WorldModelDataPtr data);
    void processJoystickCommand(msl_msgs::JoystickCommandPtr msg);
    void processMotionBurst(msl_actuator_msgs::MotionBurstPtr msg);
    void processLightBarrier(std_msgs::BoolPtr msg);
    void processRawOdometryInfo(msl_actuator_msgs::RawOdometryInfoPtr msg);
    void processMotionControlMessage(msl_actuator_msgs::MotionControl &mc);
    void processCorrectedOdometryInfo(msl_sensor_msgs::CorrectedOdometryInfoPtr &coi);
    void processBallHypothesisList(msl_sensor_msgs::BallHypothesisListPtr &list);
    void processIMUData(msl_actuator_msgs::IMUDataPtr msg);

  private:
    InfoBuffer<InformationElement<vector<double>>> distanceScan;
    InfoBuffer<InformationElement<bool>> lightBarrier;
    InfoBuffer<InformationElement<geometry::CNPointAllo>> opticalFlow;
    InfoBuffer<InformationElement<geometry::CNPositionAllo>> ownPositionMotion;
    InfoBuffer<InformationElement<geometry::CNPositionAllo>> ownPositionVision;
    InfoBuffer<InformationElement<msl_msgs::MotionInfo>> ownVelocityMotion;
    InfoBuffer<InformationElement<msl_msgs::MotionInfo>> ownVelocityVision;
    InfoBuffer<InformationElement<msl_actuator_msgs::MotionControl>> lastMotionCommand;
    InfoBuffer<InformationElement<int>> compass;
    InfoBuffer<InformationElement<msl_msgs::JoystickCommand>> joystickCommands;
    InfoBuffer<InformationElement<msl_sensor_msgs::CorrectedOdometryInfo>> ownOdometry;
    InfoBuffer<InformationElement<msl_sensor_msgs::BallHypothesisList>> ballHypothesis;
    InfoBuffer<InformationElement<msl_actuator_msgs::IMUData>> imuData;
    MSLWorldModel *wm;

    unsigned long maxInformationAge;
    int ownID;
    bool loggingEnabled;
};

} /* namespace alica */
