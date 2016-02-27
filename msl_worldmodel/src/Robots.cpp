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
			obstacles(ringBufferLength), opponents(wm, ringBufferLength), teammates(wm, ringBufferLength)
	{
		this->wm = wm;
		this->sc = supplementary::SystemConfig::getInstance();
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
		if(sharedWolrdModelData.find(data->senderID) == sharedWolrdModelData.end()) {
			shared_ptr<RingBuffer<InformationElement<msl_sensor_msgs::SharedWorldInfo>>> buffer= make_shared<RingBuffer<InformationElement<msl_sensor_msgs::SharedWorldInfo>>>(wm->getRingBufferLength());
			pair<int, shared_ptr<RingBuffer<InformationElement<msl_sensor_msgs::SharedWorldInfo>>>> pair(data->senderID, buffer);
			sharedWolrdModelData.insert(pair);
		}
		if (teammates.robotPositions.find(data->senderID) == teammates.robotPositions.end())
		{
			shared_ptr<RingBuffer<InformationElement<geometry::CNPosition>>> buffer= make_shared<RingBuffer<InformationElement<geometry::CNPosition>>>(wm->getRingBufferLength());
			pair<int, shared_ptr<RingBuffer<InformationElement<geometry::CNPosition>>>> pair(data->senderID, buffer);
			teammates.robotPositions.insert(pair);
		}
		shared_ptr<InformationElement<geometry::CNPosition>> info = make_shared<
		InformationElement<geometry::CNPosition>>(
				make_shared<geometry::CNPosition>(data->odom.position.x, data->odom.position.y, data->odom.position.angle),
				wm->getTime());
		teammates.robotPositions.at(data->senderID)->add(info);

		shared_ptr<msl_sensor_msgs::SharedWorldInfo> shptr = make_shared<msl_sensor_msgs::SharedWorldInfo>();
		*shptr = *data;
		shared_ptr<InformationElement<msl_sensor_msgs::SharedWorldInfo>> infosh = make_shared<InformationElement<msl_sensor_msgs::SharedWorldInfo>>(shptr, wm->getTime());
		sharedWolrdModelData.at(data->senderID)->add(infosh);
	}

	shared_ptr<vector<shared_ptr<geometry::CNPoint2D> > > msl::Robots::getObstaclePoints(int index)
	{
		shared_ptr<vector<shared_ptr<geometry::CNPoint2D> > > ret = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();
		auto x = obstacles.getLast(index);
		if (x == nullptr || wm->getTime() - x->timeStamp > maxInformationAge)
		{
			return nullptr;
		}
		msl_sensor_msgs::ObstacleInfo current;
		for(int i = 0; i < x->getInformation()->size(); i++)
		{
			current = x->getInformation()->at(i);
			ret->push_back(make_shared<geometry::CNPoint2D>(current.x, current.y));
		}
		return ret;
	}

}

/* namespace alica */

