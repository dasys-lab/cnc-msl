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
#include <list>
#include <iostream>
#include <tuple>
#include <mutex>

#include "SystemConfig.h"
#include "container/CNPoint2D.h"
#include "container/CNPosition.h"



using namespace std;

namespace msl
{

	class MSLWorldModel
	{
	public:
		static MSLWorldModel* get();

		CNPosition getOwnPosition();
		CNPoint2D getBallPosition();


		void onSimulatorData(msl_simulator::messages_robocup_ssl_wrapperPtr msg);
		void onRawOdometryInfo(msl_actuator_msgs::RawOdometryInfoPtr msg);
		bool haveBall();
		bool nearPoint(CNPoint2D p);
		msl_actuator_msgs::RawOdometryInfoPtr getRawOdometryInfo();

		MSLWorldModel();
		virtual ~MSLWorldModel();

	private:
		int hasBallIteration;
		int ownID;
		int ringBufferLength;
		ros::NodeHandle n;
		ros::Subscriber sub;
		ros::Subscriber rawOdomSub;

		list<msl_simulator::messages_robocup_ssl_wrapperPtr> simData;
		list<msl_actuator_msgs::RawOdometryInfoPtr> rawOdometryData;
		mutex rawOdometryMutex;
		ros::AsyncSpinner* spinner;
	};

} /* namespace msl */

#endif /* MSLWORLDMODEL_H_ */
