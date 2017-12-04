#pragma once

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
#include <supplementary/InfoBuffer.h>
#include <supplementary/InformationElement.h>

#include <memory>
#include <vector>

namespace msl
{

class MSLWorldModel;

class RawSensorData
{

  public:
    RawSensorData(MSLWorldModel *wm, int ringBufferLength);
    virtual ~RawSensorData();

    // Data Integration Methods
    void processWorldModelData(msl_sensor_msgs::WorldModelDataPtr data);
    void processJoystickCommand(msl_msgs::JoystickCommandPtr msg);
    void processMotionBurst(msl_actuator_msgs::MotionBurstPtr msg);
    void processLightBarrier(std_msgs::BoolPtr msg);
    void processRawOdometryInfo(msl_actuator_msgs::RawOdometryInfoPtr msg);
    void processMotionControlMessage(msl_actuator_msgs::MotionControl &mc);
    void processCorrectedOdometryInfo(msl_sensor_msgs::CorrectedOdometryInfoPtr &coi);
    void processBallHypothesisList(msl_sensor_msgs::BallHypothesisListPtr &list);
    void processIMUData(msl_actuator_msgs::IMUDataPtr msg);

    // Data Access Methods
    const supplementary::InfoBuffer<std::shared_ptr<const std::vector<double>>> &getDistanceScanBuffer();
    const supplementary::InfoBuffer<bool> &getLightBarrierBuffer();
    const supplementary::InfoBuffer<geometry::CNVecEgo> &getOpticalFlowBuffer();
    const supplementary::InfoBuffer<geometry::CNPositionAllo> &getOwnPositionMotionBuffer();
    const supplementary::InfoBuffer<geometry::CNPositionAllo> &getOwnPositionVisionBuffer();
    const supplementary::InfoBuffer<msl_msgs::MotionInfo> &getOwnVelocityMotionBuffer();
    const supplementary::InfoBuffer<msl_msgs::MotionInfo> &getOwnVelocityVisionBuffer();
    const supplementary::InfoBuffer<msl_actuator_msgs::MotionControl> &getLastMotionCommandBuffer();
    const supplementary::InfoBuffer<int> &getCompassBuffer();
    const supplementary::InfoBuffer<msl_sensor_msgs::CorrectedOdometryInfo> &getCorrectedOdometryBuffer();
    const supplementary::InfoBuffer<msl_actuator_msgs::IMUData> &getImuDataBuffer();
    const supplementary::InfoBuffer<msl_sensor_msgs::BallHypothesisList> &getBallHypothesisBuffer();
    const supplementary::InfoBuffer<msl_msgs::JoystickCommand> &getJoystickCommandsBuffer();

  private:
    // buffers
    supplementary::InfoBuffer<std::shared_ptr<const std::vector<double>>> distanceScan;
    supplementary::InfoBuffer<bool> lightBarrier;
    supplementary::InfoBuffer<geometry::CNVecEgo> opticalFlow;
    supplementary::InfoBuffer<geometry::CNPositionAllo> ownPositionMotion;
    supplementary::InfoBuffer<geometry::CNPositionAllo> ownPositionVision;
    supplementary::InfoBuffer<msl_msgs::MotionInfo> ownVelocityMotion;
    supplementary::InfoBuffer<msl_msgs::MotionInfo> ownVelocityVision;
    supplementary::InfoBuffer<msl_actuator_msgs::MotionControl> lastMotionCommand;
    supplementary::InfoBuffer<int> compass;
    supplementary::InfoBuffer<msl_actuator_msgs::IMUData> imuData;
    supplementary::InfoBuffer<msl_sensor_msgs::CorrectedOdometryInfo> correctedOdometry;
    supplementary::InfoBuffer<msl_msgs::JoystickCommand> joystickCommands;
    supplementary::InfoBuffer<msl_sensor_msgs::BallHypothesisList> ballHypothesis;

    MSLWorldModel *wm;

    // 1000000000[nsec] -> 1 [sec]
    // TODO: replace with ?DEFINES? or whatever for each info type
    const supplementary::InfoTime maxValidity = 1000000000;
    int ownID;
};

} /* namespace msl */
