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

		if(data->obstacles.size() > 0)
		{
			shared_ptr<vector<msl_sensor_msgs::ObstacleInfo>> obs = make_shared<vector<msl_sensor_msgs::ObstacleInfo>>(data->obstacles);
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

} /* namespace alica */
