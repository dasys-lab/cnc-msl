/*
 * MSLWorldModel.h
 *
 *  Created on: 27.10.2014
 *      Author: Andreas Witsch
 */

#ifndef MSLWORLDMODEL_H_
#define MSLWORLDMODEL_H_

#include <ros/ros.h>
#include "msl_actuator_msgs/RawOdometryInfo.h"
#include <msl_sensor_msgs/WorldModelData.h>
#include <msl_msgs/JoystickCommand.h>
#include <msl_actuator_msgs/MotionBurst.h>
#include <msl_sensor_msgs/SimulatorWorldModelData.h>
#include <list>
#include <iostream>
#include <tuple>
#include <mutex>

#include <SystemConfig.h>
#include <container/CNPoint2D.h>
#include <container/CNPosition.h>
#include <MSLEnums.h>
#include "RawSensorData.h"
#include "Robots.h"
#include "Ball.h"
#include "Game.h"
#include "Kicker.h"
#include "pathplanner/PathPlanner.h"
#include "EventTrigger.h"
#include "InformationElement.h"

namespace alica {
	class AlicaEngine;
}

using namespace std;

namespace msl
{


	class MSLSharedWorldModel;
	class MSLWorldModel
	{
	public:
		static MSLWorldModel* get();
		bool setEngine(alica::AlicaEngine* ae);

		double getKickerVoltage();
		void setKickerVoltage(double voltage);


		void onRawOdometryInfo(msl_actuator_msgs::RawOdometryInfoPtr msg);
		void onWorldModelData(msl_sensor_msgs::WorldModelDataPtr msg);
		void onJoystickCommand(msl_msgs::JoystickCommandPtr msg);
		void onMotionBurst(msl_actuator_msgs::MotionBurstPtr msg);
		void onSimWorldModel(msl_sensor_msgs::SimulatorWorldModelDataPtr msg);
		void onSharedWorldInfo(msl_sensor_msgs::SharedWorldInfoPtr msg);

		MSLSharedWorldModel* getSharedWorldModel();
		InfoTime getTime();
		void sendSharedWorldModelData();

		MSLWorldModel();
		virtual ~MSLWorldModel();
		int getRingBufferLength();
		int getOwnId();

		RawSensorData rawSensorData;
		Robots robots;
		Ball ball;
		Game game;
		PathPlanner pathPlanner;
		Kicker kicker;
		supplementary::EventTrigger visionTrigger;

	private:

		int ownID;
		int ringBufferLength;
		double kickerVoltage;
		MSLSharedWorldModel* sharedWorldModel;
		alica::AlicaEngine* alicaEngine;

		ros::NodeHandle n;
		ros::Subscriber sub;
		ros::Subscriber rawOdomSub;
		ros::Subscriber wmDataSub;
		ros::Subscriber joystickSub;
		ros::Subscriber motionBurstSub;
		ros::Subscriber simWorldModel;
		ros::Subscriber sharedWorldSub;
		ros::Publisher sharedWorldPub;

		list<msl_msgs::JoystickCommandPtr> joystickCommandData;

		mutex wmMutex;
		mutex joystickMutex;
		mutex motionBurstMutex;
		ros::AsyncSpinner* spinner;

	protected:
	};

} /* namespace msl */



#endif /* MSLWORLDMODEL_H_ */
