#define IMULOG false

#include "RawSensorData.h"

#include "Ball.h"
#include "MSLWorldModel.h"

#include <SystemConfig.h>
#include <cnc_geometry/CNPointAllo.h>

#include <math.h>

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

const InfoBuffer<vector<double>>& RawSensorData::getDistanceScanBuffer()
{
	return this->distanceScan;
}



void RawSensorData::processRawOdometryInfo(msl_actuator_msgs::RawOdometryInfoPtr msg)
{
    shared_ptr<InformationElement<geometry::CNPositionAllo>> motion =
        make_shared<InformationElement<geometry::CNPositionAllo>>(
            make_shared<geometry::CNPositionAllo>(msg->position.x, msg->position.y, msg->position.angle),
            wm->getTime());
    ownPositionMotion.add(motion);
    shared_ptr<InformationElement<msl_msgs::MotionInfo>> vel = make_shared<InformationElement<msl_msgs::MotionInfo>>(
        make_shared<msl_msgs::MotionInfo>(msg->motion), wm->getTime());
    ownVelocityMotion.add(vel);
}

void RawSensorData::processJoystickCommand(msl_msgs::JoystickCommandPtr msg)
{
    // TODO: set maxValididititityDuration to 250ms
    if (msg->robotId == this->ownID)
    {
        /*
         * In order to convert the boost::shared_ptr to a std::shared_ptr
         * we use the conversion suggested in this post:
         * http://stackoverflow.com/questions/12314967/cohabitation-of-boostshared-ptr-and-stdshared-ptr
         */
        shared_ptr<msl_msgs::JoystickCommand> cmd = shared_ptr<msl_msgs::JoystickCommand>(
            msg.get(), [msg](msl_msgs::JoystickCommand *) mutable { msg.reset(); });
        shared_ptr<InformationElement<msl_msgs::JoystickCommand>> jcmd =
            make_shared<InformationElement<msl_msgs::JoystickCommand>>(cmd, wm->getTime());
        jcmd->certainty = 1.0;
        joystickCommands.add(jcmd);
    }
}

void RawSensorData::processMotionBurst(msl_actuator_msgs::MotionBurstPtr msg)
{
    shared_ptr<geometry::CNPointEgo> opt = make_shared<geometry::CNPointEgo>(0, msg->x, msg->y);
    shared_ptr<InformationElement<geometry::CNPointEgo>> o =
        make_shared<InformationElement<geometry::CNPointEgo>>(opt, wm->getTime(), this->maxValidity, msg->qos);
    o->certainty = msg->qos;

    opticalFlow.add(o);
}

void RawSensorData::processLightBarrier(std_msgs::BoolPtr msg)
{
    shared_ptr<bool> lb = make_shared<bool>(msg->data);
    shared_ptr<InformationElement<bool>> l = make_shared<InformationElement<bool>>(lb, wm->getTime());
    l->certainty = 1.0;
    lightBarrier.add(l);
}

void RawSensorData::processMotionControlMessage(msl_actuator_msgs::MotionControl &cmd)
{
    shared_ptr<msl_actuator_msgs::MotionControl> mc = make_shared<msl_actuator_msgs::MotionControl>();
    mc->motion.angle = cmd.motion.angle;
    mc->motion.translation = cmd.motion.translation;
    mc->motion.rotation = cmd.motion.rotation;
    mc->timestamp = cmd.timestamp;
    shared_ptr<InformationElement<msl_actuator_msgs::MotionControl>> smc =
        make_shared<InformationElement<msl_actuator_msgs::MotionControl>>(mc, wm->getTime());
    smc->certainty = 1;
    lastMotionCommand.add(smc);
}

void RawSensorData::processWorldModelData(msl_sensor_msgs::WorldModelDataPtr data)
{
    InfoTime time = wm->getTime();

    if (data->odometry.certainty > 0)
    {
        // full odometry
        auto odo = make_shared<InformationElement<msl_sensor_msgs::CorrectedOdometryInfo>>(data->odometry,
                                                                                           time, this->maxValidity,
                                                                                           data->odometry.certainty);
        ownOdometry.add(odo);


        // Vision
        shared_ptr<geometry::CNPositionAllo> pos = make_shared<geometry::CNPositionAllo>(
            data->odometry.position.x, data->odometry.position.y, data->odometry.position.angle);
        shared_ptr<InformationElement<geometry::CNPositionAllo>> odometry =
            make_shared<InformationElement<geometry::CNPositionAllo>>(pos, time);
        odometry->certainty = data->odometry.certainty;
        ownPositionVision.add(odometry);

        shared_ptr<msl_msgs::MotionInfo> vel = make_shared<msl_msgs::MotionInfo>(data->odometry.motion);
        shared_ptr<InformationElement<msl_msgs::MotionInfo>> v =
            make_shared<InformationElement<msl_msgs::MotionInfo>>(vel, time);
        v->certainty = data->odometry.certainty;
        ownVelocityVision.add(v);

        // Motion
        /*shared_ptr<geometry::CNPosition> posMotion = make_shared<geometry::CNPosition>(
         data->odometry.position.x, data->odometry.position.y, data->odometry.position.angle);
         shared_ptr<InformationElement<geometry::CNPosition>> odometryMotion = make_shared<
         InformationElement<geometry::CNPosition>>(posMotion, time);
         odometryMotion->certainty = data->odometry.certainty;
         ownPositionMotion.add(odometryMotion);

         // TODO: this is the same motion as for vision motion !?
         shared_ptr<msl_msgs::MotionInfo> velMotion = make_shared<msl_msgs::MotionInfo>(data->odometry.motion);
         shared_ptr<InformationElement<msl_msgs::MotionInfo>> vMotion = make_shared<
         InformationElement<msl_msgs::MotionInfo>>(velMotion, time);
         vMotion->certainty = data->odometry.certainty;
         ownVelocityMotion.add(vMotion);*/
    }

    shared_ptr<geometry::CNPointEgo> ballPos =
        make_shared<geometry::CNPointEgo>(data->ball.point.x, data->ball.point.y, data->ball.point.z);
    shared_ptr<geometry::CNVecEgo> ballVel =
        make_shared<geometry::CNVecEgo>(data->ball.velocity.vx, data->ball.velocity.vy, data->ball.velocity.vz);

    // cout << "RawSensorData: Ball X:" << ballVel->x << ", Y:" << ballVel->y << endl;
    if (data->ball.confidence > 0.00000001)
        this->wm->ball->updateBallPos(ballPos, ballVel, data->ball.confidence);

    auto information = make_shared<vector<double>>(data->distanceScan.sectors);
    auto element = make_shared<InformationElement<shared_ptr<vector<double>>>>(information, time, this->maxValidity,
                                                                               data->odometry.certainty);
    distanceScan.add(element);

    //    shared_ptr<vector<double>> dist = make_shared<vector<double>>(data->distanceScan.sectors);
    //    TODO This is a Taker workaround, should be removed when real error was found int count = 0;
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
    //    TODO This is a Taker workaround, should be removed when real error was found count = 0;
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
    shared_ptr<geometry::CNPositionAllo> opt =
        make_shared<geometry::CNPositionAllo>(coi->position.x, coi->position.y, coi->position.angle);
    shared_ptr<InformationElement<geometry::CNPositionAllo>> o =
        make_shared<InformationElement<geometry::CNPositionAllo>>(opt, wm->getTime());
    o->certainty = coi->position.certainty;
    ownPositionVision.add(o);
    this->wm->ball->updateOnLocalizationData(coi->imageTime);
}

void RawSensorData::processBallHypothesisList(msl_sensor_msgs::BallHypothesisListPtr &list)
{
    shared_ptr<msl_sensor_msgs::BallHypothesisList> nList = make_shared<msl_sensor_msgs::BallHypothesisList>(*list);
    shared_ptr<InformationElement<msl_sensor_msgs::BallHypothesisList>> o =
        make_shared<InformationElement<msl_sensor_msgs::BallHypothesisList>>(nList, wm->getTime());
    o->certainty = 1;
    ballHypothesis.add(o);
    this->wm->ball->updateOnBallHypothesisList(list->imageTime);
}

void RawSensorData::processIMUData(msl_actuator_msgs::IMUDataPtr msg)
{
    shared_ptr<msl_actuator_msgs::IMUData> cmd = make_shared<msl_actuator_msgs::IMUData>();
    cmd->accelSens = msg->accelSens;
    cmd->acceleration = msg->acceleration;
    cmd->gyro = msg->gyro;
    cmd->gyroSens = msg->gyroSens;
    cmd->magnet = msg->magnet;
    cmd->magnetSens = msg->magnetSens;
    cmd->temperature = msg->temperature;
    cmd->time = msg->time;
    shared_ptr<InformationElement<msl_actuator_msgs::IMUData>> o =
        make_shared<InformationElement<msl_actuator_msgs::IMUData>>(cmd, wm->getTime());
    o->certainty = 1;
    imuData.add(o);

    // TODO IMU-Baustelle Kai/Marci
    double bearing = atan2(cmd->magnet.y, cmd->magnet.x);
    if (IMULOG)
    {
        log(0, atan2(cmd->magnet.y, cmd->magnet.x));
        // log(1, imuData.getAverageMod());
    }
}
} /* namespace alica */
