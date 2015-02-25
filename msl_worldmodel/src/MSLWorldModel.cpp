/*
 * MSLWorldModel.cpp
 *
 *  Created on: 27.10.2014
 *      Author: endy
 */

#include <GeometryCalculator.h>
#include "MSLWorldModel.h"
#include "sharedworldmodel/MSLSharedWorldModel.h"
#include "RawSensorData.h"
#include "msl_sensor_msgs/SharedWorldInfo.h"
namespace msl
{

	MSLWorldModel* MSLWorldModel::get()
	{
		static MSLWorldModel instance;
		return &instance;
	}

	MSLWorldModel::MSLWorldModel() :
			ringBufferLength(10), rawSensorData(this, 10), robots(this, 10), ball(this), game(this)
	{
		kickerVoltage = 0;
		ownID = supplementary::SystemConfig::getOwnRobotID();
		spinner = new ros::AsyncSpinner(4);
		spinner->start();
		rawOdomSub = n.subscribe("/RawOdometry", 10, &MSLWorldModel::onRawOdometryInfo, (MSLWorldModel*)this);

		joystickSub = n.subscribe("/Joystick", 10, &MSLWorldModel::onJoystickCommand, (MSLWorldModel*)this);

		wmDataSub = n.subscribe("/WorldModel/WorldModelData", 10, &MSLWorldModel::onWorldModelData,
								(MSLWorldModel*)this);

		sharedWorldPub = n.advertise<msl_sensor_msgs::SharedWorldInfo>("/WorldModel/SharedWorldInfo", 10);

		this->sharedWolrdModel = new MSLSharedWorldModel(this);


	}
	void MSLWorldModel::onJoystickCommand(msl_msgs::JoystickCommandPtr msg)
	{
		if (msg->robotId == this->ownID)
		{
			lock_guard<mutex> lock(joystickMutex);
			if (joystickCommandData.size() > ringBufferLength)
			{
				joystickCommandData.pop_back();
			}
			joystickCommandData.push_front(msg);
		}
	}
	msl_msgs::JoystickCommandPtr MSLWorldModel::getJoystickCommandInfo()
	{
		lock_guard<mutex> lock(joystickMutex);
		if (joystickCommandData.size() == 0)
		{
			return nullptr;
		}
		return joystickCommandData.front();
	}
	void MSLWorldModel::onRawOdometryInfo(msl_actuator_msgs::RawOdometryInfoPtr msg)
	{
		lock_guard<mutex> lock(rawOdometryMutex);
		if (rawOdometryData.size() > ringBufferLength)
		{
			rawOdometryData.pop_back();
		}
		rawOdometryData.push_front(msg);
	}

	msl_actuator_msgs::RawOdometryInfoPtr MSLWorldModel::getRawOdometryInfo()
	{
		lock_guard<mutex> lock(rawOdometryMutex);
		if (rawOdometryData.size() == 0)
		{
			return nullptr;
		}
		return rawOdometryData.front();
	}

	void MSLWorldModel::onWorldModelData(msl_sensor_msgs::WorldModelDataPtr msg)
	{
		lock_guard<mutex> lock(wmMutex);
		rawSensorData.processWorldModelData(msg);
		robots.processWorldModelData(msg);
	}

	msl_sensor_msgs::WorldModelDataPtr MSLWorldModel::getWorldModelData()
	{
		lock_guard<mutex> lock(wmMutex);
		if (wmData.size() == 0)
		{
			return nullptr;
		}
		return wmData.front();
	}

	MSLWorldModel::~MSLWorldModel()
	{
		spinner->stop();
		delete spinner;
		delete this->sharedWolrdModel;
	}

	double MSLWorldModel::getKickerVoltage()
	{
		return this->kickerVoltage;
	}

	void MSLWorldModel::setKickerVoltage(double voltage)
	{
		this->kickerVoltage = voltage;
	}

	MSLSharedWorldModel* MSLWorldModel::getSharedWolrdModel()
	{
		return this->sharedWolrdModel;
	}

	unsigned long MSLWorldModel::getTime()
	{
		return (unsigned long)ros::Time::now().sec * (unsigned long)1000000000 + (unsigned long)ros::Time::now().nsec;
	}

	void MSLWorldModel::sendSharedWorldModelData()
	{
		msl_sensor_msgs::SharedWorldInfo msg;
		msg.senderID = 9;
		auto ball = rawSensorData.getBallPositionAndCertaincy();
		if (ball != nullptr)
		{
			auto pos = transformToWorldCoordinates(ball->first->x, ball->first->y);
			msg.ball.point.x = pos.first;
			msg.ball.point.y = pos.second;
			msg.ball.confidence = ball->second;
 		}

		auto ballVel = rawSensorData.getBallVelocity();
		if (ballVel != nullptr)
		{
			auto pos = transformToWorldCoordinates(ballVel->x, ballVel->y);
			msg.ball.velocity.vx = pos.first;
			msg.ball.velocity.vy = pos.second;
		}

		auto ownPos = rawSensorData.getOwnPositionVisionAndCertaincy();
		if (ownPos != nullptr)
		{
			auto pos = transformToWorldCoordinates(ownPos->first->x, ownPos->first->y);
			msg.odom.position.x = pos.first;
			msg.odom.position.y = pos.second;
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
			if(obstacles != nullptr)
			{
				msg.obstacles.reserve(obstacles->size());
				for(auto& x : *obstacles)
				{
					auto pos = transformToWorldCoordinates(x.x, x.y);
					msl_msgs::Point2dInfo info;
					info.x = pos.first;
					info.y = pos.second;
					msg.obstacles.push_back(info);
				}
			}
		}
		if(ownPos != nullptr)
		{
			sharedWorldPub.publish(msg);
		}
	}

	int MSLWorldModel::getRingBufferLength()
	{
		return ringBufferLength;
	}

	pair<double, double> MSLWorldModel::transformToWorldCoordinates(double x, double y)
	{
		pair<double, double> ret;
		ret.first = y;
		ret.second = -x;
		return ret;
	}

} /* namespace msl */

