/*
 * MSLWorldModel.cpp
 *
 *  Created on: 27.10.2014
 *      Author: Andreas Witsch
 */

#include <GeometryCalculator.h>
#include "MSLWorldModel.h"
#include "sharedworldmodel/MSLSharedWorldModel.h"
#include "RawSensorData.h"
#include "msl_sensor_msgs/SharedWorldInfo.h"
#include "engine/AlicaEngine.h"
#include "engine/IAlicaClock.h"

namespace msl
{

	MSLWorldModel* MSLWorldModel::get()
	{
		static MSLWorldModel instance;
		return &instance;
	}

	bool MSLWorldModel::setEngine(alica::AlicaEngine* ae)
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

	alica::AlicaEngine* MSLWorldModel::getEngine()
	{
		return this->alicaEngine;
	}

	MSLWorldModel::MSLWorldModel() :
			ringBufferLength(10), rawSensorData(this, 10), robots(this, 10), ball(this, 10), game(this, 10), pathPlanner(
					this, 10), kicker(this), alicaEngine(nullptr), whiteBoard(this)
	{
		kickerVoltage = 0;
		ownID = supplementary::SystemConfig::getOwnRobotID();
		spinner = new ros::AsyncSpinner(4);
		spinner->start();
		rawOdomSub = n.subscribe("/RawOdometry", 10, &MSLWorldModel::onRawOdometryInfo, (MSLWorldModel*)this);

		joystickSub = n.subscribe("/Joystick", 10, &MSLWorldModel::onJoystickCommand, (MSLWorldModel*)this);

		wmDataSub = n.subscribe("/WorldModel/WorldModelData", 10, &MSLWorldModel::onWorldModelData,
								(MSLWorldModel*)this);

		wmBallListSub = n.subscribe("/CNVision/BallHypothesisList", 10, &MSLWorldModel::onBallHypothesisList,
									(MSLWorldModel*)this);

		motionBurstSub = n.subscribe("/CNActuator/MotionBurst", 10, &MSLWorldModel::onMotionBurst,
										(MSLWorldModel*)this);

		simWorldModelSub = n.subscribe("/WorldModel/SimulatorWorldModelData", 10, &MSLWorldModel::onSimWorldModel,
									(MSLWorldModel*)this);

		gazeboWorldModelSub = n.subscribe("/gazebo/model_states", 10, &MSLWorldModel::onGazeboModelState,
											(MSLWorldModel*)this);

		sharedWorldPub = n.advertise<msl_sensor_msgs::SharedWorldInfo>("/WorldModel/SharedWorldInfo", 10);

		sharedWorldSub = n.subscribe("/WorldModel/SharedWorldInfo", 10, &MSLWorldModel::onSharedWorldInfo,
										(MSLWorldModel*)this);

		passMsgSub = n.subscribe("/WorldModel/PassMsg", 10, &MSLWorldModel::onPassMsg, (MSLWorldModel*)this);

		correctedOdometrySub = n.subscribe("/CorrectedOdometryInfo", 10, &MSLWorldModel::onCorrectedOdometryInfo,
											(MSLWorldModel*)this);

		this->sharedWorldModel = new MSLSharedWorldModel(this);
	}

	void MSLWorldModel::onJoystickCommand(msl_msgs::JoystickCommandPtr msg)
	{
		this->rawSensorData.processJoystickCommand(msg);
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

	void MSLWorldModel::onGazeboModelState(gazebo_msgs::ModelStatesPtr msg)
	{
		alica::AlicaTime now = this->alicaEngine->getIAlicaClock()->now();

		msl_sensor_msgs::WorldModelDataPtr wmsim = boost::make_shared<msl_sensor_msgs::WorldModelData>();


		int modelCnt = msg->name.size();
		cout << "WM: Gazebo Model Count: " << modelCnt <<  endl;
		for (int i = 0; i < modelCnt; i++)
		{
			// TODO: check coordinate systems
			if (msg->name[i].find("bot"))
			{

				if (msg->name[i].compare("bot"+to_string(this->ownID)) != 0)
				{
					cout << "WM: Gazebo Pos" << endl;
					wmsim->timestamp = now;
					wmsim->odometry.certainty = 1.0;
					wmsim->odometry.locType.type = msl_sensor_msgs::LocalizationType::ErrorMin;
					wmsim->odometry.position.certainty = 1.0;
					wmsim->odometry.position.angle = atan2(msg->pose[i].orientation.y, msg->pose[i].orientation.x);
					wmsim->odometry.position.x = msg->pose[i].position.x * 1000.0;
					wmsim->odometry.position.y = msg->pose[i].position.y * 1000.0;
					wmsim->odometry.motion.angle = atan2(msg->twist[i].linear.y, msg->twist[i].linear.x);
					wmsim->odometry.motion.translation = sqrt(
							msg->twist[i].linear.x * msg->twist[i].linear.x
									+ msg->twist[i].linear.y * msg->twist[i].linear.y) * 1000.0;
					wmsim->odometry.motion.rotation = msg->twist[i].angular.z;
				}
				else
				{
					msl_sensor_msgs::ObstacleInfo obsInfo;
					obsInfo.diameter = 500.0;
					obsInfo.x = msg->pose[i].position.x * 1000.0;
					obsInfo.y = msg->pose[i].position.y * 1000.0;

					wmsim->obstacles.push_back(obsInfo);
				}
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

			// TODO: fill distance scan
		}

		onWorldModelData(wmsim);
	}

	void MSLWorldModel::onRawOdometryInfo(msl_actuator_msgs::RawOdometryInfoPtr msg)
	{
		rawSensorData.processRawOdometryInfo(msg);
	}

	void MSLWorldModel::onWorldModelData(msl_sensor_msgs::WorldModelDataPtr msg)
	{
		lock_guard<mutex> lock(wmMutex);
		rawSensorData.processWorldModelData(msg);
		robots.processWorldModelData(msg);
		pathPlanner.processWorldModelData(msg);
		visionTrigger.run();
	}

	void msl::MSLWorldModel::onMotionBurst(msl_actuator_msgs::MotionBurstPtr msg)
	{
		lock_guard<mutex> lock(motionBurstMutex);
		rawSensorData.processMotionBurst(msg);
	}

	MSLWorldModel::~MSLWorldModel()
	{
		spinner->stop();
		delete spinner;
		delete this->sharedWorldModel;
	}

	double MSLWorldModel::getKickerVoltage()
	{
		return this->kickerVoltage;
	}

	void MSLWorldModel::setKickerVoltage(double voltage)
	{
		this->kickerVoltage = voltage;
	}

	MSLSharedWorldModel* MSLWorldModel::getSharedWorldModel()
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

	void MSLWorldModel::sendSharedWorldModelData()
	{
		msl_sensor_msgs::SharedWorldInfo msg;
		msg.senderID = this->ownID;
		auto ball = this->ball.getVisionBallPositionAndCertaincy();
		auto pos = rawSensorData.getOwnPositionVision();
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
		}

		auto ballVel = this->ball.getVisionBallVelocity();
		if (ballVel != nullptr)
		{

			msg.ball.velocity.vx = ballVel->x;
			msg.ball.velocity.vy = ballVel->y;
		}

		auto ownPos = rawSensorData.getOwnPositionVisionAndCertaincy();
		if (ownPos != nullptr)
		{
			msg.odom.position.x = ownPos->first->x;
			msg.odom.position.y = ownPos->first->y;
			msg.odom.position.angle = ownPos->first->theta;
			msg.odom.certainty = ownPos->second;
		}

		auto ownVel = rawSensorData.getOwnVelocityVision();
		if (ownVel != nullptr)
		{
			msg.odom.motion.angle = ownVel->angle;
			msg.odom.motion.rotation = ownVel->rotation;
			msg.odom.motion.translation = ownVel->translation;
		}

		auto obstacles = robots.getObstacles();
		{
			if (obstacles != nullptr)
			{
				msg.obstacles.reserve(obstacles->size());
				for (auto& x : *obstacles)
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
		robots.processSharedWorldModelData(msg);
	}

	int MSLWorldModel::getRingBufferLength()
	{
		return ringBufferLength;
	}

	void MSLWorldModel::onPassMsg(msl_helper_msgs::PassMsgPtr msg)
	{
		whiteBoard.processPassMsg(msg);
	}

	void MSLWorldModel::onCorrectedOdometryInfo(msl_sensor_msgs::CorrectedOdometryInfoPtr msg)
	{
		lock_guard<mutex> lock(correctedOdemetryMutex);
		rawSensorData.processCorrectedOdometryInfo(msg);
	}

	void MSLWorldModel::onBallHypothesisList(msl_sensor_msgs::BallHypothesisListPtr msg)
	{
		rawSensorData.processBallHypothesisList(msg);
	}

	int MSLWorldModel::getOwnId()
	{
		return ownID;
	}
} /* namespace msl */

