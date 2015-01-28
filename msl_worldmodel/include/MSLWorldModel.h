/*
 * MSLWorldModel.h
 *
 *  Created on: 27.10.2014
 *      Author: endy
 */

#ifndef MSLWORLDMODEL_H_
#define MSLWORLDMODEL_H_

#include <ros/ros.h>
#include <msl_simulator/messages_robocup_ssl_wrapper.h>
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
#include "Situation.h"



using namespace std;

namespace msl
{

	class MSLWorldModel
	{
	public:
		static MSLWorldModel* get();

		shared_ptr<CNPosition> getOwnPosition();
		shared_ptr<CNPoint2D> getAlloBallPosition();
		shared_ptr<CNPoint2D> getEgoBallPosition();
		double getKickerVoltage();
		void setKickerVoltage(double voltage);

		void onSimulatorData(msl_simulator::messages_robocup_ssl_wrapperPtr msg);
		void onRawOdometryInfo(msl_actuator_msgs::RawOdometryInfoPtr msg);
		void onWorldModelData(msl_sensor_msgs::WorldModelDataPtr msg);
		void onJoystickCommand(msl_msgs::JoystickCommandPtr msg);
		void onRefereeBoxInfoBody(msl_msgs::RefereeBoxInfoBodyPtr msg);
		bool checkSituation(Situation situation);

		bool haveBall();
		msl_actuator_msgs::RawOdometryInfoPtr getRawOdometryInfo();
		msl_sensor_msgs::WorldModelDataPtr getWorldModelData();
		msl_msgs::JoystickCommandPtr getJoystickCommandInfo();
		msl_msgs::RefereeBoxInfoBodyPtr getRefereeBoxInfoBody();

		MSLWorldModel();
		virtual ~MSLWorldModel();

	private:
		int hasBallIteration;
		int ownID;
		int ringBufferLength;
		double kickerVoltage;

		ros::NodeHandle n;
		ros::Subscriber sub;
		ros::Subscriber rawOdomSub;
		ros::Subscriber wmDataSub;
		ros::Subscriber joystickSub;
		ros::Subscriber refereeBoxInfoBodySub;

		list<msl_simulator::messages_robocup_ssl_wrapperPtr> simData;
		list<msl_actuator_msgs::RawOdometryInfoPtr> rawOdometryData;
		list<msl_msgs::JoystickCommandPtr> joystickCommandData;
		list<msl_msgs::RefereeBoxInfoBodyPtr> refereeBoxInfoBodyCommandData;
		list<msl_sensor_msgs::WorldModelDataPtr> wmData;

		mutex rawOdometryMutex;
		mutex wmMutex;
		mutex joystickMutex;
		mutex refereeMutex;
		ros::AsyncSpinner* spinner;

	protected:
		void transformToWorldCoordinates(msl_sensor_msgs::WorldModelDataPtr& msg);
	};

} /* namespace msl */

#endif /* MSLWORLDMODEL_H_ */
