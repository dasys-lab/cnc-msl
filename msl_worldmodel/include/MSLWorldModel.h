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

using namespace std;

namespace msl
{

	class MSLWorldModel
	{
	public:
		static MSLWorldModel* get();

		tuple<double, double, double> getOwnPosition();
		pair<double, double> getBallPosition();
		pair<double, double> allo2Ego(pair<double, double>& p, tuple<double, double, double>& ownPos);

		void onSimulatorData(msl_simulator::messages_robocup_ssl_wrapperPtr msg);

		MSLWorldModel();
		virtual ~MSLWorldModel();

	private:
		int ownID;
		ros::NodeHandle n;
		ros::Subscriber sub;
		list<msl_simulator::messages_robocup_ssl_wrapperPtr> simData;
		ros::AsyncSpinner* spinner;
	};

} /* namespace msl */

#endif /* MSLWORLDMODEL_H_ */
