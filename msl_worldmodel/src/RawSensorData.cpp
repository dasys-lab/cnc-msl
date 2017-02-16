#define IMULOG false

#include "RawSensorData.h"

#include "Ball.h"
#include "MSLWorldModel.h"

#include <SystemConfig.h>

namespace msl
{

using std::make_shared;
using std::vector;

// FIXME replace that absolute path with some config parameter
std::string logFile = "/home/cn/cnws/IMU.log";
FILE *lp = fopen(logFile.c_str(), "a");

void log(int index, float value)
{
    fprintf(lp, "%d:%f\n", index, value);
    fflush(lp);
}

RawSensorData::RawSensorData(MSLWorldModel *wm, int ringbufferLength)
    : distanceScan(ringbufferLength)
    , lightBarrier(ringbufferLength)
    , opticalFlow(ringbufferLength)
    , ownPositionMotion(ringbufferLength)
    , ownPositionVision(ringbufferLength)
    , ownVelocityMotion(ringbufferLength)
    , ownVelocityVision(ringbufferLength)
    , compass(ringbufferLength)
    , joystickCommands(ringbufferLength)
    , ownOdometry(ringbufferLength)
    , lastMotionCommand(ringbufferLength)
    , ballHypothesis(ringbufferLength)
    , imuData(ringbufferLength)
{
    this->wm = wm;
    ownID = supplementary::SystemConfig::getOwnRobotID();
}

RawSensorData::~RawSensorData()
{
}

const InfoBuffer<shared_ptr<vector<double>>> &RawSensorData::getDistanceScanBuffer()
{
    return this->distanceScan;
}

const InfoBuffer<bool> &RawSensorData::getLightBarrierBuffer()
{
    return this->lightBarrier;
}

const InfoBuffer<geometry::CNVecEgo> &RawSensorData::getOpticalFlow()
{
    return this->opticalFlow;
}

const InfoBuffer<geometry::CNPositionAllo> &RawSensorData::getOwnPositionMotion()
{
    return this->ownPositionMotion;
}

const InfoBuffer<geometry::CNPositionAllo> &RawSensorData::getOwnPositionVision()
{
    return this->ownPositionVision;
}

const InfoBuffer<msl_msgs::MotionInfo> &RawSensorData::getOwnVelocityMotion()
{
    return this->ownVelocityMotion;
}

const InfoBuffer<msl_msgs::MotionInfo> &RawSensorData::getOwnVelocityVision()
{
    return this->ownVelocityVision;
}

const InfoBuffer<msl_actuator_msgs::MotionControl> &RawSensorData::getLastMotionCommand()
{
	return this->lastMotionCommand;
}

const InfoBuffer<int> &RawSensorData::getCompass()
{
	return this->compass;
}

const InfoBuffer<msl_sensor_msgs::CorrectedOdometryInfo> &RawSensorData::getOwnOdometry()
{
	return this->ownOdometry;
}

const InfoBuffer<msl_actuator_msgs::IMUData> &RawSensorData::getImuData()
{
	return this->imuData;
}

const InfoBuffer<msl_sensor_msgs::BallHypothesisList> &RawSensorData::getBallHypothesis()
{
	return this->ballHypothesis;
}

const InfoBuffer<msl_msgs::JoystickCommand> &RawSensorData::getJoystickCommands()
{
    return this->joystickCommands;
}

void RawSensorData::processRawOdometryInfo(msl_actuator_msgs::RawOdometryInfoPtr msg)
{
    auto motion = make_shared<InformationElement<geometry::CNPositionAllo>>(
        make_shared<geometry::CNPositionAllo>(msg->position.x, msg->position.y, msg->position.angle), wm->getTime());
    ownPositionMotion.add(motion);

    auto vel = make_shared<InformationElement<msl_msgs::MotionInfo>>(msl_msgs::MotionInfo(msg->motion), wm->getTime(),
                                                                     this->maxValidity, msg->position.certainty);
    ownVelocityMotion.add(vel);
}

void RawSensorData::processJoystickCommand(msl_msgs::JoystickCommandPtr msg)
{

    if (msg->robotId == this->ownID)
    {
    	// TODO: set maxValididititityDuration to 250ms -> constant?
        auto joyInfo =
            make_shared<InformationElement<msl_msgs::JoystickCommand>>(*msg, wm->getTime(), this->maxValidity, 1.0);
        joystickCommands.add(joyInfo);
    }
}

void RawSensorData::processMotionBurst(msl_actuator_msgs::MotionBurstPtr msg)
{
    // CNPointEgo, but origin is not the center of the robot
    auto ofData = geometry::CNVecEgo(0, msg->x, msg->y);
    auto ofInfo =
        make_shared<InformationElement<geometry::CNVecEgo>>(ofData, wm->getTime(), this->maxValidity, msg->qos);

    opticalFlow.add(ofInfo);
}

void RawSensorData::processLightBarrier(std_msgs::BoolPtr msg)
{
    auto lbInfo = make_shared<InformationElement<bool>>(msg->data, wm->getTime(), this->maxValidity, 1.0);
    lightBarrier.add(lbInfo);
}

void RawSensorData::processMotionControlMessage(msl_actuator_msgs::MotionControl &cmd)
{
    msl_actuator_msgs::MotionControl mcData = cmd;
    auto mcInfo = make_shared<InformationElement<msl_actuator_msgs::MotionControl>>(mcData, wm->getTime(),
                                                                                    this->maxValidity, 1.0);
    lastMotionCommand.add(mcInfo);
}

void RawSensorData::processWorldModelData(msl_sensor_msgs::WorldModelDataPtr data)
{
    InfoTime time = wm->getTime();

    if (data->odometry.certainty > 0)
    {
        // full odometry
        auto odo = make_shared<InformationElement<msl_sensor_msgs::CorrectedOdometryInfo>>(
            data->odometry, time, this->maxValidity, data->odometry.certainty);
        ownOdometry.add(odo);

        // Vision
        auto posData = geometry::CNPositionAllo(data->odometry.position.x, data->odometry.position.y,
                                                data->odometry.position.angle);
        auto posInfo = make_shared<InformationElement<geometry::CNPositionAllo>>(posData, time, this->maxValidity,
                                                                                 data->odometry.certainty);
        ownPositionVision.add(posInfo);

        auto velInfo = make_shared<InformationElement<msl_msgs::MotionInfo>>(
            data->odometry.motion, time, this->maxValidity, data->odometry.certainty);
        ownVelocityVision.add(velInfo);
    }

    auto ballPos = make_shared<geometry::CNPointEgo>(data->ball.point.x, data->ball.point.y, data->ball.point.z);
    auto ballVel =
        make_shared<geometry::CNVecEgo>(data->ball.velocity.vx, data->ball.velocity.vy, data->ball.velocity.vz);

    if (data->ball.confidence > 0.00000001)
        this->wm->ball->updateBallPos(ballPos, ballVel, data->ball.confidence);

    auto distanceScanData = make_shared<vector<double>>(data->distanceScan.sectors);
    auto distanceScanInfo = make_shared<InformationElement<shared_ptr<vector<double>>>>(
        distanceScanData, time, this->maxValidity, data->odometry.certainty);
    distanceScan.add(distanceScanInfo);

    //    shared_ptr<vector<double>> dist =
    //    make_shared<vector<double>>(data->distanceScan.sectors);
    //    TODO This is a Taker workaround, should be removed when real error was
    //    found int count = 0;
    //    while (dist.use_count() == 0)
    //    {
    //        if (count > 5)
    //            return;
    //        dist = make_shared<vector<double>>(data->distanceScan.sectors);
    //        ++count;
    //    }
    //
    //    shared_ptr<InformationElement<vector<double>>> distance =
    //        make_shared<InformationElement<vector<double>>>(dist, time);
    //
    //    TODO This is a Taker workaround, should be removed when real error was
    //    found count = 0;
    //    while (dist.use_count() == 1)
    //    {
    //        if (count > 5)
    //            return;
    //        dist = make_shared<vector<double>>(data->distanceScan.sectors);
    //        ++count;
    //    }
    //    distance->certainty = data->ball.confidence;

    wm->getVisionDataEventTrigger()->run();
}

void RawSensorData::processCorrectedOdometryInfo(msl_sensor_msgs::CorrectedOdometryInfoPtr &coi)
{
    auto coData = make_shared<geometry::CNPositionAllo>(coi->position.x, coi->position.y, coi->position.angle);
    auto coInfo = make_shared<InformationElement<geometry::CNPositionAllo>>(coData, wm->getTime(), this->maxValidity,
                                                                            coi->certainty);
    ownPositionVision.add(coInfo);
    this->wm->ball->updateOnLocalizationData(coi->imageTime);
}

void RawSensorData::processBallHypothesisList(msl_sensor_msgs::BallHypothesisListPtr &list)
{
    auto bhlData = *list;
    auto bhlInfo = make_shared<InformationElement<msl_sensor_msgs::BallHypothesisList>>(bhlData, wm->getTime(),
                                                                                        this->maxValidity, 1.0);
    ballHypothesis.add(bhlInfo);
    this->wm->ball->updateOnBallHypothesisList(list->imageTime);
}

void RawSensorData::processIMUData(msl_actuator_msgs::IMUDataPtr msg)
{
    auto imuData = *msg;
    auto imuInfo =
        make_shared<InformationElement<msl_actuator_msgs::IMUData>>(imuData, wm->getTime(), this->maxValidity, 1.0);

    this->imuData.add(imuInfo);

    // TODO IMU-Baustelle Kai/Marci
    double bearing = atan2(msg->magnet.y, msg->magnet.x);
    if (IMULOG)
    {
        log(0, atan2(msg->magnet.y, msg->magnet.x));
        // log(1, imuData.getAverageMod());
    }
}

} /* namespace alica */
