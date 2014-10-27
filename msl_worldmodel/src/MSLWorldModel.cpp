/*
 * MSLWorldModel.cpp
 *
 *  Created on: 27.10.2014
 *      Author: endy
 */

#include "MSLWorldModel.h"

namespace msl
{

	MSLWorldModel* MSLWorldModel::get()
	{
		static MSLWorldModel instance;
		return &instance;
	}

	void MSLWorldModel::onSimulatorData(msl_simulator::messages_robocup_ssl_wrapperPtr msg)
	{
		if (simData.size() > 10)
		{
			simData.pop_back();
		}
		simData.push_front(msg);
	}

	MSLWorldModel::MSLWorldModel()
	{
		ownID = supplementary::SystemConfig::getOwnRobotID();
		spinner = new ros::AsyncSpinner(4);
		sub = n.subscribe("/MSLSimulator/MessagesRoboCupSSLWrapper", 10,
											&MSLWorldModel::onSimulatorData, (MSLWorldModel*)this);
		spinner->start();
	}

	MSLWorldModel::~MSLWorldModel()
	{
		spinner->stop();
		delete spinner;
	}

	/**
	 * returns x,y orientation:
	 * (0,0) Is the field center
	 * positive y = right side of the field (playing direction from yellow to blue)
	 * negative x = blue goal
	 * orientation of -pi = towards blue side
	 *
	 */
	tuple<double, double, double> MSLWorldModel::getOwnPosition()
	{

		double x, y, oriantation;
		if (simData.size() > 0)
		{
			auto data = *simData.begin();
			for (auto& r : data->detection.robots_yellow)
			{
				if (r.robot_id == ownID)
				{
					x = r.x;
					y = r.y;
					oriantation = r.orientation;
				}
			}
		}
		return make_tuple(x, y, oriantation);
	}

	pair<double, double> MSLWorldModel::getBallPosition()
	{
		pair<double, double> ret;
		if (simData.size() > 0)
		{
			auto data = *simData.begin();
			if (data->detection.balls.size() > 0)
			{
				ret.first = data->detection.balls.begin()->x;
				ret.second = data->detection.balls.begin()->y;
			}
		}
		return ret;
	}

} /* namespace msl */
