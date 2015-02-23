/*
 * Robots.h
 *
 *  Created on: Feb 23, 2015
 *      Author: Stefan Jakob
 */

#ifndef CNC_MSL_MSL_WORLDMODEL_INCLUDE_ROBOTS_H_
#define CNC_MSL_MSL_WORLDMODEL_INCLUDE_ROBOTS_H_

#include <vector>
#include "RingBuffer.h"
#include "InformationElement.h"
#include "msl_sensor_msgs/WorldModelData.h"
#include "msl_sensor_msgs/ObstacleInfo.h"

using namespace std;

namespace msl
{

	class MSLWorldModel;
	class Robots
	{
	public:
		Robots(MSLWorldModel* wm, int ringBufferLength);
		virtual ~Robots();
		void processWorldModelData(msl_sensor_msgs::WorldModelDataPtr data);
		shared_ptr<vector<msl_sensor_msgs::ObstacleInfo>> getObstacles(int index = 0);

	private:
		RingBuffer<InformationElement<vector<msl_sensor_msgs::ObstacleInfo>>> obstacles;
		MSLWorldModel* wm;
	};

} /* namespace alica */

#endif /* CNC_MSL_MSL_WORLDMODEL_INCLUDE_ROBOTS_H_ */
