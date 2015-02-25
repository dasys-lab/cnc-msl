/*
 * MSLWorldModel.h
 *
 *  Created on: 27.10.2014
 *      Author: endy
 */

#ifndef MSLWORLDMODEL_H_
#define MSLWORLDMODEL_H_

#include <ros/ros.h>
#include <msl_actuator_msgs/RawOdometryInfo.h>
#include <msl_sensor_msgs/WorldModelData.h>
#include <msl_msgs/JoystickCommand.h>
#include <list>
#include <iostream>
#include <tuple>
#include <mutex>

#include "SystemConfig.h"
#include "container/CNPoint2D.h"
#include "container/CNPosition.h"
#include "Situation.h"
#include "RawSensorData.h"
#include "Robots.h"
#include "Ball.h"
#include "Game.h"


using namespace std;

namespace msl
{

	class MSLSharedWorldModel;
	class MSLWorldModel
	{
	public:
		static MSLWorldModel* get();

		double getKickerVoltage();
		void setKickerVoltage(double voltage);

		void onRawOdometryInfo(msl_actuator_msgs::RawOdometryInfoPtr msg);
		void onWorldModelData(msl_sensor_msgs::WorldModelDataPtr msg);
		void onJoystickCommand(msl_msgs::JoystickCommandPtr msg);

		msl_actuator_msgs::RawOdometryInfoPtr getRawOdometryInfo();
		msl_sensor_msgs::WorldModelDataPtr getWorldModelData();
		MSLSharedWorldModel* getSharedWolrdModel();
		unsigned long getTime();
		void sendSharedWorldModelData();

		MSLWorldModel();
		virtual ~MSLWorldModel();
		int getRingBufferLength();

		RawSensorData rawSensorData;
		Robots robots;
		Ball ball;
		Game game;

	private:

		int ownID;
		int ringBufferLength;
		double kickerVoltage;
		MSLSharedWorldModel* sharedWolrdModel;

		ros::NodeHandle n;
		ros::Subscriber sub;
		ros::Subscriber rawOdomSub;
		ros::Subscriber wmDataSub;
		ros::Subscriber joystickSub;
		ros::Publisher sharedWorldPub;

		list<msl_actuator_msgs::RawOdometryInfoPtr> rawOdometryData;
		list<msl_msgs::JoystickCommandPtr> joystickCommandData;
		list<msl_sensor_msgs::WorldModelDataPtr> wmData;

		mutex rawOdometryMutex;
		mutex wmMutex;
		mutex joystickMutex;;
		ros::AsyncSpinner* spinner;

	protected:
		pair<double, double> transformToWorldCoordinates(double x, double y);
	};

} /* namespace msl */

#endif /* MSLWORLDMODEL_H_ */
