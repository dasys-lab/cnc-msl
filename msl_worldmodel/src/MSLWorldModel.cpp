#include "MSLWorldModel.h"
#include "Ball.h"
#include "EventTrigger.h"
#include "Game.h"
#include "InformationElement.h"
#include "LightBarrier.h"
#include "MSLFootballField.h"
#include "Monitoring.h"
#include "Prediction.h"
#include "RawSensorData.h"
#include "Calibration.h"
#include "Robots.h"
#include "WhiteBoard.h"
#include "LaserScanner.h"
#include "engine/AlicaEngine.h"
#include "engine/IAlicaClock.h"
#include "msl_sensor_msgs/SharedWorldInfo.h"
#include "obstaclehandler/Obstacles.h"
#include "pathplanner/PathPlanner.h"
#include "sharedworldmodel/MSLSharedWorldModel.h"
#include "tf/tf.h"
#include "process_manager/ProcessCommand.h"
#include <GeometryCalculator.h>
#include <container/CNPoint2D.h>
#include <container/CNPosition.h>
#include <gazebo_msgs/ModelStates.h>
#include <msl_actuator_msgs/IMUData.h>
#include <msl_actuator_msgs/RawOdometryInfo.h>
#include <msl_actuator_msgs/MotionBurst.h>
#include <msl_helper_msgs/PassMsg.h>
#include <msl_helper_msgs/WatchBallMsg.h>
#include <msl_msgs/JoystickCommand.h>
#include <msl_sensor_msgs/BallHypothesisList.h>
#include <msl_sensor_msgs/CorrectedOdometryInfo.h>
#include <msl_sensor_msgs/SharedWorldInfo.h>
#include <msl_sensor_msgs/SimulatorWorldModelData.h>
#include <msl_sensor_msgs/WorldModelData.h>
#include <std_msgs/Bool.h>
#include <msl_sensor_msgs/LaserLocalization.h>

namespace msl
{

    MSLWorldModel *MSLWorldModel::get()
    {
        static MSLWorldModel instance;
        return &instance;
    }

    bool MSLWorldModel::setEngine(alica::AlicaEngine *ae)
    {
        if (this->alicaEngine == nullptr)
        {
            this->alicaEngine = ae;
            return true;
        }
        else
        {
            return false;
        }
    }

    alica::AlicaEngine *MSLWorldModel::getEngine()
    {
        return this->alicaEngine;
    }

    MSLWorldModel::MSLWorldModel()
    {
        ownID = supplementary::SystemConfig::getOwnRobotID();
        spinner = new ros::AsyncSpinner(4);
        spinner->start();
        sc = supplementary::SystemConfig::getInstance();

        visionDataEventTrigger = new supplementary::EventTrigger();
        rawOdomSub = n.subscribe("/RawOdometry", 10, &MSLWorldModel::onRawOdometryInfo, (MSLWorldModel *)this);

        joystickSub = n.subscribe("/Joystick", 10, &MSLWorldModel::onJoystickCommand, (MSLWorldModel *)this);

        wmDataSub = n.subscribe("/WorldModel/WorldModelData", 10, &MSLWorldModel::onWorldModelData,
                                (MSLWorldModel *)this);

        wmBallListSub = n.subscribe("/CNVision/BallHypothesisList", 10, &MSLWorldModel::onBallHypothesisList,
                                    (MSLWorldModel *)this);

        motionBurstSub = n.subscribe("/CNActuator/MotionBurst", 10, &MSLWorldModel::onMotionBurst,
                                     (MSLWorldModel *)this);

        simWorldModelSub = n.subscribe("/WorldModel/SimulatorWorldModelData", 10, &MSLWorldModel::onSimWorldModel,
                                       (MSLWorldModel *)this);

        gazeboWorldModelSub = n.subscribe("/gazebo/model_states", 10, &MSLWorldModel::onGazeboModelState,
                                          (MSLWorldModel *)this);

        sharedWorldPub = n.advertise<msl_sensor_msgs::SharedWorldInfo>("/WorldModel/SharedWorldInfo", 10);

        sharedWorldSub = n.subscribe("/WorldModel/SharedWorldInfo", 10, &MSLWorldModel::onSharedWorldInfo,
                                     (MSLWorldModel *)this);

        passMsgSub = n.subscribe("/WorldModel/PassMsg", 10, &MSLWorldModel::onPassMsg, (MSLWorldModel *)this);
        watchBallMsgSub = n.subscribe("/WorldModel/WatchBallMsg", 10, &MSLWorldModel::onWatchBallMsg,
                                      (MSLWorldModel *)this);

        correctedOdometrySub = n.subscribe("/CorrectedOdometryInfo", 10, &MSLWorldModel::onCorrectedOdometryInfo,
                                           (MSLWorldModel *)this);
        lightBarrierSub = n.subscribe("/LightBarrierInfo", 10, &MSLWorldModel::onLightBarrierInfo,
                                      (MSLWorldModel *)this);
        imuDataSub = n.subscribe("/IMUData", 10, &MSLWorldModel::onIMUData, (MSLWorldModel *)this);

        processCommandPub = n.advertise<process_manager::ProcessCommand>("/process_manager/ProcessCommand", 10);

        laserScanSub = n.subscribe("/laser_scan_localization", 10, &MSLWorldModel::onLaserScanData, (MSLWorldModel *)this);
        this->sharedWorldModel = new MSLSharedWorldModel(this);
        this->usingSimulator = false;
        this->ringBufferLength = (*this->sc)["WorldModel"]->get<int>("WorldModel", "RingBufferLength", NULL);
        this->maySendMessages = (*this->sc)["WorldModel"]->get<bool>("WorldModel", "MaySendMessages", NULL);
        // initialize ringbuffers
        this->field = new MSLFootballField(this);
        this->ball = new Ball(this, ringBufferLength);
        this->lightBarrier = new LightBarrier(this);
        this->rawSensorData = new RawSensorData(this, ringBufferLength);
        this->robots = new Robots(this, ringBufferLength);
        this->game = new Game(this, ringBufferLength);
        this->pathPlanner = new PathPlanner(this, ringBufferLength);
        // this->kicker = new Kicker(this); // TODO: delete this line
        this->alicaEngine = nullptr;
        this->whiteBoard = new WhiteBoard(this);
        this->obstacles = new Obstacles(this, ringBufferLength);
        this->prediction = new Prediction();
        this->monitoring = new Monitoring(this);
        this->calibration = new Calibration(this);
        this->laserScanner = new LaserScanner(this, ringBufferLength);
    }
    supplementary::ITrigger *MSLWorldModel::getVisionDataEventTrigger()
    {
        return this->visionDataEventTrigger;
    }

    void MSLWorldModel::onJoystickCommand(msl_msgs::JoystickCommandPtr msg)
    {
        this->rawSensorData->processJoystickCommand(msg);
    }

    void MSLWorldModel::onSimWorldModel(msl_sensor_msgs::SimulatorWorldModelDataPtr msg)
    {
        if (msg->receiverID == this->ownID)
        {
            msl_sensor_msgs::WorldModelDataPtr wmsim = boost::make_shared<msl_sensor_msgs::WorldModelData>(
                    msg->worldModel);
            onWorldModelData(wmsim);
        }
    }

    bool MSLWorldModel::isUsingSimulator()
    {
        return this->usingSimulator;
    }

    void MSLWorldModel::setUsingSimulator(bool usingSimulator)
    {
        this->usingSimulator = usingSimulator;
    }

    void MSLWorldModel::onGazeboModelState(gazebo_msgs::ModelStatesPtr msg)
    {
        if (!this->usingSimulator)
        {
            cout << "MSLWorldModel: Did you forget to start the base with '-sim'?" << endl;
            return;
        }

        alica::AlicaTime now = this->alicaEngine->getIAlicaClock()->now();

        msl_sensor_msgs::WorldModelDataPtr wmsim = boost::make_shared<msl_sensor_msgs::WorldModelData>();

        int modelCnt = msg->name.size();
        // cout << "WM: Gazebo Model Count: " << modelCnt <<  endl;
        for (int i = 0; i < modelCnt; i++)
        {
            if (msg->name[i].compare(supplementary::SystemConfig::getHostname()) == 0)
            {
                wmsim->timestamp = now;
                wmsim->odometry.certainty = 1.0;
                wmsim->odometry.locType.type = msl_sensor_msgs::LocalizationType::ErrorMin;
                wmsim->odometry.position.certainty = 1.0;

                tf::Quaternion q(msg->pose[i].orientation.x, msg->pose[i].orientation.y, msg->pose[i].orientation.z,
                                 msg->pose[i].orientation.w);
                tf::Matrix3x3 m(q);
                double roll, pitch, yaw;
                m.getRPY(roll, pitch, yaw);

                wmsim->odometry.position.angle = yaw + M_PI;
                wmsim->odometry.position.x = msg->pose[i].position.x * 1000.0;
                wmsim->odometry.position.y = msg->pose[i].position.y * 1000.0;
                wmsim->odometry.motion.angle = atan2(msg->twist[i].linear.y, msg->twist[i].linear.x);
                wmsim->odometry.motion.translation = sqrt(
                        msg->twist[i].linear.x * msg->twist[i].linear.x
                                + msg->twist[i].linear.y * msg->twist[i].linear.y) * 1000.0;
                wmsim->odometry.motion.rotation = msg->twist[i].angular.z;
            }
            else if (msg->name[i].compare("ground_plane") != 0 && msg->name[i].compare("field") != 0
                    && msg->name[i].compare("left_goal") != 0 && msg->name[i].compare("right_goal") != 0
                    && msg->name[i].compare("football") != 0)
            {
                msl_sensor_msgs::ObstacleInfo obsInfo;
                obsInfo.diameter = 500.0;
                obsInfo.x = msg->pose[i].position.x * 1000.0;
                obsInfo.y = msg->pose[i].position.y * 1000.0;

                wmsim->obstacles.push_back(obsInfo);
            }

            if (msg->name[i] == "football")
            {
                wmsim->ball.ballType = 1; // TODO: introduce constants for Type in BallInfo-Msg.
                wmsim->ball.confidence = 1.0;
                wmsim->ball.point.x = msg->pose[i].position.x * 1000.0;
                wmsim->ball.point.y = msg->pose[i].position.y * 1000.0;
                wmsim->ball.point.z = msg->pose[i].position.z * 1000.0;
                wmsim->ball.velocity.vx = msg->twist[i].linear.x * 1000.0;
                wmsim->ball.velocity.vy = msg->twist[i].linear.y * 1000.0;
                wmsim->ball.velocity.vz = msg->twist[i].linear.z * 1000.0;
            }
        }

        // allo to ego for ball and obstacles

        double x = wmsim->ball.point.x - wmsim->odometry.position.x;
        double y = wmsim->ball.point.y - wmsim->odometry.position.y;

        double angle = atan2(y, x) - wmsim->odometry.position.angle;
        double dist = sqrt(x * x + y * y);

        wmsim->ball.point.x = cos(angle) * dist;
        wmsim->ball.point.y = sin(angle) * dist;

        // Only if ball is closer than 7m
        if (dist > 7000)
        {
            wmsim->ball.point.x = 0;
            wmsim->ball.point.y = 0;
            wmsim->ball.confidence = 0;
        }

        for (int i = 0; i < 60; i++)
        {
            wmsim->distanceScan.sectors.push_back(20000);
        }

        for (int i = 0; i < wmsim->obstacles.size(); ++i)
        {
            auto &obs = wmsim->obstacles.at(i);

            double x = obs.x - wmsim->odometry.position.x;
            double y = obs.y - wmsim->odometry.position.y;
            double dist = sqrt(x * x + y * y);

            // only if obstacle is closer than 6m
            if (dist > 6000)
            {
                wmsim->obstacles.erase(wmsim->obstacles.begin() + i);
                i--;
                continue;
            }

            double angle = atan2(y, x) - wmsim->odometry.position.angle;

            int sector = (int)(angle / (2 * M_PI / 60.0)) % 60;
            if (sector < 0)
                sector += 60;
            wmsim->distanceScan.sectors[sector] = dist;

            obs.x = cos(angle) * dist;
            obs.y = sin(angle) * dist;
        }

        onWorldModelData(wmsim);
    }

    void MSLWorldModel::onRawOdometryInfo(msl_actuator_msgs::RawOdometryInfoPtr msg)
    {
        rawSensorData->processRawOdometryInfo(msg);
    }

    void MSLWorldModel::onWorldModelData(msl_sensor_msgs::WorldModelDataPtr msg)
    {
        if (game->ownGoalColor != Color::Yellow)
        {
            msg->odometry.position.x = -msg->odometry.position.x;
            msg->odometry.position.y = -msg->odometry.position.y;
            msg->odometry.position.angle += M_PI;
            while (msg->odometry.position.angle > M_PI)
            {
                msg->odometry.position.angle -= 2 * M_PI;
            }
            msg->odometry.motion.angle += M_PI;
            while (msg->odometry.motion.angle > M_PI)
            {
                msg->odometry.motion.angle -= 2 * M_PI;
            }
        }

        lock_guard<mutex> lock(wmMutex);
        rawSensorData->processWorldModelData(msg);
        obstacles->processWorldModelData(msg);
        pathPlanner->prepareVoronoiDiagram();
        visionTrigger.run();
    }

    void msl::MSLWorldModel::onMotionBurst(msl_actuator_msgs::MotionBurstPtr msg)
    {
        lock_guard<mutex> lock(motionBurstMutex);
        rawSensorData->processMotionBurst(msg);
    }

    void MSLWorldModel::onIMUData(msl_actuator_msgs::IMUDataPtr msg)
    {
        rawSensorData->processIMUData(msg);
    }

    MSLWorldModel::~MSLWorldModel()
    {
        spinner->stop();
        delete spinner;
        delete this->sharedWorldModel;
        delete this->ball;
        delete this->field;
        delete this->monitoring;
        delete this->lightBarrier;
        delete this->rawSensorData;
        delete this->robots;
        delete this->game;
        delete this->pathPlanner;
        delete this->whiteBoard;
        delete this->obstacles;
        delete this->prediction;
        delete this->calibration;
    }

    MSLSharedWorldModel *MSLWorldModel::getSharedWorldModel()
    {
        return this->sharedWorldModel;
    }

    InfoTime MSLWorldModel::getTime()
    {
        if (this->alicaEngine != nullptr)
        {
            return this->alicaEngine->getIAlicaClock()->now();
        }
        else
        {
            return 0;
        }
    }

    bool MSLWorldModel::isMaySendMessages() const
    {
        return this->maySendMessages;
    }

    void MSLWorldModel::setMaySendMessages(bool maySendMessages)
    {
        this->maySendMessages = maySendMessages;
    }

    void MSLWorldModel::sendSharedWorldModelData()
    {
        if (!this->maySendMessages)
        {
            return;
        }
        msl_sensor_msgs::SharedWorldInfo msg;
        msg.senderID = this->ownID;
        msg.ownGoalIsYellow = this->game->ownGoalColor == Color::Yellow;
        msg.ownTeamIsMagenta = this->game->ownTeamColor == Color::Magenta;
        msg.ballPossessionStatus = this->ball->getBallPossessionStatus();
        auto ball = this->ball->getVisionBallPositionAndCertaincy();
        auto pos = rawSensorData->getOwnPositionVision();
        if (pos == nullptr)
        {
            return;
        }
        if (ball != nullptr)
        {
            shared_ptr<geometry::CNPoint2D> point = make_shared<geometry::CNPoint2D>(ball->first->x, ball->first->y);
            auto p = point->egoToAllo(*pos);
            msg.ball.point.x = p->x;
            msg.ball.point.y = p->y;
            msg.ball.confidence = ball->second;
            msg.ballInPossession = this->ball->closeToTheBall();
        }
        else
        {
            msg.ballInPossession = false;
        }

        auto sb = this->ball->getAlloSharedBallPositionAndCertaincy();
        if (sb != nullptr && sb->first != nullptr && this->ball->getSharedBallSupporter() > 1)
        {
            msg.sharedBall.point.x = sb->first->x;
            msg.sharedBall.point.y = sb->first->y;
            msg.sharedBall.confidence = sb->second;
            msg.sharedBall.evidence = this->ball->getSharedBallSupporter();
        }
        else if (sb == nullptr || sb->first == nullptr)
        {
            // if sb == nullptr send ballguess
            auto guess = this->ball->getAlloBallGuessPosition();
            if (guess != nullptr)
            {
                msg.sharedBall.point.x = guess->x;
                msg.sharedBall.point.y = guess->y;
                msg.sharedBall.confidence = 0.1;
                msg.sharedBall.evidence = 1;
            }
        }

        auto ballVel = this->ball->getVisionBallVelocity();
        if (ballVel != nullptr)
        {

            msg.ball.velocity.vx = ballVel->x;
            msg.ball.velocity.vy = ballVel->y;
        }

        auto ownPos = rawSensorData->getOwnPositionVisionAndCertaincy();
        if (ownPos != nullptr)
        {
            msg.odom.position.x = ownPos->first->x;
            msg.odom.position.y = ownPos->first->y;
            msg.odom.position.angle = ownPos->first->theta;
            msg.odom.certainty = ownPos->second;
        }

        auto ownVel = rawSensorData->getOwnVelocityVision();
        if (ownVel != nullptr)
        {
            msg.odom.motion.angle = ownVel->angle;
            msg.odom.motion.rotation = ownVel->rotation;
            msg.odom.motion.translation = ownVel->translation;
        }

        auto obs = obstacles->getEgoVisionObstacles();
        {
            if (obs != nullptr)
            {
                msg.obstacles.reserve(obs->size());
                for (auto &x : *obs)
                {
                    shared_ptr<geometry::CNPoint2D> point = make_shared<geometry::CNPoint2D>(x.x, x.y);
                    auto p = point->egoToAllo(*pos);
                    msl_msgs::Point2dInfo info;
                    info.x = p->x;
                    info.y = p->y;
                    msg.obstacles.push_back(info);
                }
            }
        }

        if (ownPos != nullptr)
        {
            msg.participating = true;
            sharedWorldPub.publish(msg);
        }
    }

    void MSLWorldModel::onSharedWorldInfo(msl_sensor_msgs::SharedWorldInfoPtr msg)
    {
        robots->processSharedWorldModelData(msg);
        ball->processSharedWorldModelData(*msg);
        game->updateGameState();
    }

    int MSLWorldModel::getRingBufferLength()
    {
        return ringBufferLength;
    }

    void MSLWorldModel::onPassMsg(msl_helper_msgs::PassMsgPtr msg)
    {
        whiteBoard->processPassMsg(msg);
    }

    void MSLWorldModel::onWatchBallMsg(msl_helper_msgs::WatchBallMsgPtr msg)
    {
        whiteBoard->processWatchBallMsg(msg);
    }

    void MSLWorldModel::onCorrectedOdometryInfo(msl_sensor_msgs::CorrectedOdometryInfoPtr msg)
    {
        lock_guard<mutex> lock(correctedOdemetryMutex);
        rawSensorData->processCorrectedOdometryInfo(msg);
    }

    void MSLWorldModel::onBallHypothesisList(msl_sensor_msgs::BallHypothesisListPtr msg)
    {
        rawSensorData->processBallHypothesisList(msg);
    }

    int MSLWorldModel::getOwnId()
    {
        return ownID;
    }

    void msl::MSLWorldModel::onLightBarrierInfo(std_msgs::BoolPtr msg)
    {
        rawSensorData->processLightBarrier(msg);
    }
    void MSLWorldModel::onLaserScanData(msl_sensor_msgs::LaserLocalizationPtr msg)
    {
    	laserScanner->processLaserScannPoints(msg);
    }
} /* namespace msl */
