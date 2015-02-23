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
#include <msl_msgs/RefereeBoxInfoBody.h>
#include <list>
#include <iostream>
#include <tuple>
#include <mutex>

#include "SystemConfig.h"
#include "container/CNPoint2D.h"
#include "container/CNPosition.h"
#include "HaveBall.h"
#include "Situation.h"
#include "RawSensorData.h"
#include "Robots.h"


using namespace std;

namespace msl
{

	class MSLSharedWorldModel;
	class MSLWorldModel
	{
	public:
		static MSLWorldModel* get();
		HaveBall haveBall;

		shared_ptr<CNPosition> getOwnPosition();
		shared_ptr<CNPoint2D> getAlloBallPosition();
		shared_ptr<CNPoint2D> getEgoBallPosition();
		double getKickerVoltage();
		void setKickerVoltage(double voltage);

		void onRawOdometryInfo(msl_actuator_msgs::RawOdometryInfoPtr msg);
		void onWorldModelData(msl_sensor_msgs::WorldModelDataPtr msg);
		void onJoystickCommand(msl_msgs::JoystickCommandPtr msg);
		void onRefereeBoxInfoBody(msl_msgs::RefereeBoxInfoBodyPtr msg);
		bool checkSituation(Situation situation);

		msl_actuator_msgs::RawOdometryInfoPtr getRawOdometryInfo();
		msl_sensor_msgs::WorldModelDataPtr getWorldModelData();
		msl_msgs::JoystickCommandPtr getJoystickCommandInfo();
		msl_msgs::RefereeBoxInfoBodyPtr getRefereeBoxInfoBody();
		MSLSharedWorldModel* getSharedWolrdModel();
		unsigned long getTime();
		void sendSharedWorldModelData();

		MSLWorldModel();
		virtual ~MSLWorldModel();

		RawSensorData rawSensorData;
		Robots robots;

	private:

		int ownID;
		int ringBufferLength;
		double kickerVoltage;
		Situation currentSituation;
		MSLSharedWorldModel* sharedWolrdModel;

		ros::NodeHandle n;
		ros::Subscriber sub;
		ros::Subscriber rawOdomSub;
		ros::Subscriber wmDataSub;
		ros::Subscriber joystickSub;
		ros::Subscriber refereeBoxInfoBodySub;
		ros::Publisher sharedWorldPub;

		list<msl_actuator_msgs::RawOdometryInfoPtr> rawOdometryData;
		list<msl_msgs::JoystickCommandPtr> joystickCommandData;
		list<msl_msgs::RefereeBoxInfoBodyPtr> refereeBoxInfoBodyCommandData;
		list<msl_sensor_msgs::WorldModelDataPtr> wmData;

		mutex rawOdometryMutex;
		mutex wmMutex;
		mutex joystickMutex;
		mutex refereeMutex;
		mutex situationChecker;
		ros::AsyncSpinner* spinner;

	protected:
		void transformToWorldCoordinates(msl_sensor_msgs::WorldModelDataPtr& msg);
	};

} /* namespace msl */

#endif /* MSLWORLDMODEL_H_ */
