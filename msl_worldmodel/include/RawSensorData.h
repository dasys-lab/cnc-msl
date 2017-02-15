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

class MSLWorldModel;
class RawSensorData
{

  public:
    RawSensorData(MSLWorldModel *wm, int ringBufferLength);
    virtual ~RawSensorData();

    // access methods
    std::shared_ptr<std::vector<double>> getDistanceScan(int index = 0);
    bool getLightBarrier(int index = 0);
    std::shared_ptr<geometry::CNPointAllo> getOpticalFlow(int index = 0);
    double getOpticalFlowQoS(int index = 0);
    std::shared_ptr<geometry::CNPositionAllo> getOwnPositionMotion(int index = 0);
    std::shared_ptr<geometry::CNPositionAllo> getOwnPositionVision(int index = 0);
    std::shared_ptr<std::pair<geometry::CNPositionAllo, double>> getOwnPositionVisionAndCertaincy(int index = 0);
    std::shared_ptr<msl_msgs::MotionInfo> getOwnVelocityMotion(int index = 0);
    std::shared_ptr<msl_msgs::MotionInfo> getOwnVelocityVision(int index = 0);
    std::shared_ptr<msl_actuator_msgs::MotionControl> getLastMotionCommand(int index = 0);
    std::shared_ptr<int> getCompassOrientation(int index = 0);
    std::shared_ptr<msl_msgs::JoystickCommand> getJoystickCommand(int index = 0);
    std::shared_ptr<msl_sensor_msgs::CorrectedOdometryInfo> getCorrectedOdometryInfo(int index = 0);
    std::shared_ptr<msl_sensor_msgs::BallHypothesisList> getBallHypothesisList(int index = 0);

    // add methods
    void processWorldModelData(msl_sensor_msgs::WorldModelDataPtr data);
    void processJoystickCommand(msl_msgs::JoystickCommandPtr msg);
    void processMotionBurst(msl_actuator_msgs::MotionBurstPtr msg);
    void processLightBarrier(std_msgs::BoolPtr msg);
    void processRawOdometryInfo(msl_actuator_msgs::RawOdometryInfoPtr msg);
    void processMotionControlMessage(msl_actuator_msgs::MotionControl &mc);
    void processCorrectedOdometryInfo(msl_sensor_msgs::CorrectedOdometryInfoPtr &coi);
    void processBallHypothesisList(msl_sensor_msgs::BallHypothesisListPtr &list);
    void processIMUData(msl_actuator_msgs::IMUDataPtr msg);

    const InfoBuffer<std::vector<double>> &getDistanceScanBuffer();
    const InfoBuffer<bool> &getLightBarrierBuffer();
    const InfoBuffer<geometry::CNVecEgo> &getOpticalFlow();
    const InfoBuffer<geometry::CNPositionAllo> getOwnPositionMotion();
    const InfoBuffer<geometry::CNPositionAllo> getOwnPositionVision();


  private:
    InfoBuffer<std::vector<double>> distanceScan;
    InfoBuffer<bool> lightBarrier;
    InfoBuffer<geometry::CNPointEgo> opticalFlow;
    InfoBuffer<geometry::CNPositionAllo> ownPositionMotion;
    InfoBuffer<geometry::CNPositionAllo> ownPositionVision;
    InfoBuffer<msl_msgs::MotionInfo> ownVelocityMotion;
    InfoBuffer<msl_msgs::MotionInfo> ownVelocityVision;
    InfoBuffer<msl_actuator_msgs::MotionControl> lastMotionCommand;
    InfoBuffer<int> compass;
    InfoBuffer<msl_msgs::JoystickCommand> joystickCommands;
    InfoBuffer<msl_sensor_msgs::CorrectedOdometryInfo> ownOdometry;
    InfoBuffer<std::shared_ptr<msl_sensor_msgs::BallHypothesisList>> ballHypothesis;
    InfoBuffer<msl_actuator_msgs::IMUData> imuData;
    MSLWorldModel *wm;

    // 1000000000[nsec] -> 1 [sec]
    // TODO: replace with ?DEFINES? or whatever for each info type
    const InfoTime maxValidity = 1000000000;
    int ownID;
};

} /* namespace msl */
