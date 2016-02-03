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
#include "msl_sensor_msgs/SharedWorldInfo.h"
#include <container/CNPoint2D.h>
#include <container/CNPosition.h>
#include <SystemConfig.h>
#include <map>

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
		void processSharedWorldModelData(msl_sensor_msgs::SharedWorldInfoPtr data);
		shared_ptr<vector<msl_sensor_msgs::ObstacleInfo>> getObstacles(int index = 0);
		shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> getObstaclePoints(int index = 0);
		shared_ptr<geometry::CNPosition> getTeamMatePosition(int teamMateId, int index = 0);
		shared_ptr<vector<shared_ptr<pair<int, shared_ptr<geometry::CNPosition>>>>> getPositionsOfTeamMates();
		double getOpponentProtectDistance();
		double getOpponentProtectAngle();
		int teamMatesInOwnPenalty();
		int teamMatesInOppPenalty();

	private:
		RingBuffer<InformationElement<vector<msl_sensor_msgs::ObstacleInfo>>> obstacles;
		MSLWorldModel* wm;
		supplementary::SystemConfig* sc;
		unsigned long maxInformationAge = 1000000000;
		double opponentProtectDistance;
		double opponentProtectAngle;
		map<int, shared_ptr<RingBuffer<InformationElement<geometry::CNPosition>>>> robotPositions;
		map<int, shared_ptr<RingBuffer<InformationElement<msl_sensor_msgs::SharedWorldInfo>>>> sharedWolrdModelData;

	};

} /* namespace alica */

#endif /* CNC_MSL_MSL_WORLDMODEL_INCLUDE_ROBOTS_H_ */
