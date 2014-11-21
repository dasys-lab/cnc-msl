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
#include <list>
#include <iostream>
#include <tuple>

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
		bool haveBall();
		bool nearPos(CNPoint2D p);

		MSLWorldModel();
		virtual ~MSLWorldModel();

	private:
		int hasBallIteration;
		int ownID;
		ros::NodeHandle n;
		ros::Subscriber sub;
		list<msl_simulator::messages_robocup_ssl_wrapperPtr> simData;
		ros::AsyncSpinner* spinner;
	};

} /* namespace msl */

#endif /* MSLWORLDMODEL_H_ */
