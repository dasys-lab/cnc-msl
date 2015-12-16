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
		if (this->alicaEngine != nullptr)
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

		simWorldModel = n.subscribe("/WorldModel/SimulatorWorldModelData", 10, &MSLWorldModel::onSimWorldModel,
									(MSLWorldModel*)this);

		sharedWorldPub = n.advertise<msl_sensor_msgs::SharedWorldInfo>("/WorldModel/SharedWorldInfo", 10);

		sharedWorldSub = n.subscribe("/WorldModel/SharedWorldInfo", 10, &MSLWorldModel::onSharedWorldInfo,
										(MSLWorldModel*)this);

		passMsgSub = n.subscribe("/WorldModel/PassMsg", 10, &MSLWorldModel::onPassMsg, (MSLWorldModel*)this);

		correctedOdometrySub = n.subscribe("/CorrectedOdometryInfo", 10, &MSLWorldModel::onCorrectedOdometryInfo, (MSLWorldModel*)this);

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

	void MSLWorldModel::onRawOdometryInfo(msl_actuator_msgs::RawOdometryInfoPtr msg)
	{
		rawSensorData.processRawOdometryInfo(msg);
	}

	void MSLWorldModel::onWorldModelData(msl_sensor_msgs::WorldModelDataPtr msg)
	{
		lock_guard<mutex> lock(wmMutex);
		rawSensorData.processWorldModelData(msg);
		robots.processWorldModelData(msg);
		//pathPlanner.processWorldModelData(msg);
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

	void MSLWorldModel::onBallHypothesisList(msl_sensor_msgs::BallHypothesisListPtr msg) {
		rawSensorData.processBallHypothesisList(msg);
	}

	int MSLWorldModel::getOwnId()
	{
		return ownID;
	}
} /* namespace msl */

