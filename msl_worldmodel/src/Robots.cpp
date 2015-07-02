/*
 * Robots.cpp
 *
 *  Created on: Feb 23, 2015
 *      Author: Stefan Jakob
 */

#include <Robots.h>
#include "MSLWorldModel.h"

namespace msl
{

	Robots::Robots(MSLWorldModel* wm, int ringBufferLength) :
			obstacles(ringBufferLength)
	{
		this->wm = wm;
		maxInformationAge = 1000000000;
	}

	Robots::~Robots()
	{
		// TODO Auto-generated destructor stub
	}

	void Robots::processWorldModelData(msl_sensor_msgs::WorldModelDataPtr data)
	{
		unsigned long time = wm->getTime();
//		if ((time - data->odometry.timestamp) > 1000)
//		{
//			return;
//		}

		if (data->obstacles.size() > 0)
		{
			shared_ptr<vector<msl_sensor_msgs::ObstacleInfo>> obs = make_shared<vector<msl_sensor_msgs::ObstacleInfo>>(
					data->obstacles);
			shared_ptr<InformationElement<vector<msl_sensor_msgs::ObstacleInfo>>> o = make_shared<InformationElement<vector<msl_sensor_msgs::ObstacleInfo>>>(obs,
					time);
			obstacles.add(o);
		}
	}

	shared_ptr<vector<msl_sensor_msgs::ObstacleInfo> > Robots::getObstacles(int index)
	{
		auto x = obstacles.getLast(index);
		if (x == nullptr || wm->getTime() - x->timeStamp > maxInformationAge)
		{
			return nullptr;
		}
		return x->getInformation();
	}

	void Robots::processSharedWorldModelData(msl_sensor_msgs::SharedWorldInfoPtr data)
	{
		if (robotPositions.find(data->senderID) == robotPositions.end())
		{
			shared_ptr<RingBuffer<InformationElement<geometry::CNPosition>>> buffer= make_shared<RingBuffer<InformationElement<geometry::CNPosition>>>(wm->getRingBufferLength());
		pair<int, shared_ptr<RingBuffer<InformationElement<geometry::CNPosition>>>> pair(data->senderID, buffer);
		robotPositions.insert(pair);
	}
		shared_ptr<InformationElement<geometry::CNPosition>> info = make_shared<
				InformationElement<geometry::CNPosition>>(
						make_shared<geometry::CNPosition>(data->odom.position.x, data->odom.position.y, data->odom.position.angle),
						wm->getTime());
		robotPositions.at(data->senderID)->add(info);
	}

	shared_ptr<geometry::CNPosition> Robots::getTeamMatePosition(int teamMateId, int index)
	{
		if(robotPositions.find(teamMateId) == robotPositions.end())
		{
			return nullptr;
		}
		auto x = robotPositions.at(teamMateId)->getLast(index);
		if (x == nullptr || wm->getTime() - x->timeStamp > maxInformationAge)
		{
			return nullptr;
		}
		return x->getInformation();
	}

	shared_ptr<vector<shared_ptr<geometry::CNPosition> > > Robots::getPositionsOfTeamMates()
	{
		shared_ptr<vector<shared_ptr<geometry::CNPosition>>> ret = make_shared<vector<shared_ptr<geometry::CNPosition>>>();
		for(auto iter = robotPositions.begin(); iter != robotPositions.end(); iter++)
		{
			if(wm->getTime() - iter->second->getLast()->timeStamp < maxInformationAge)
			{
				ret->push_back(iter->second->getLast()->getInformation());
			}
		}
		return ret;
	}

}
/* namespace alica */

