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
		this->sc = supplementary::SystemConfig::getInstance();
		maxInformationAge = 1000000000;
		this->opponentProtectAngle = (*sc)["WorldModel"]->get<double>("WorldModel.OpponentProtectAngle", NULL);
		this->opponentProtectDistance = (*sc)["WorldModel"]->get<double>("WorldModel.OpponentProtectDistance", NULL);
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

		shared_ptr<msl_sensor_msgs::SharedWorldInfo> shptr = make_shared<msl_sensor_msgs::SharedWorldInfo>();
		*shptr = *data;
		shared_ptr<InformationElement<msl_sensor_msgs::SharedWorldInfo>> infosh = make_shared<InformationElement<msl_sensor_msgs::SharedWorldInfo>>(shptr, wm->getTime());
		sharedWolrdModelData.at(data->senderID)->add(infosh);
	}

	shared_ptr<geometry::CNPosition> Robots::getTeamMatePosition(int teamMateId, int index)
	{
		if (robotPositions.find(teamMateId) == robotPositions.end())
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

	shared_ptr<vector<shared_ptr<pair<int, shared_ptr<geometry::CNPosition>>> >> Robots::getPositionsOfTeamMates()
	{
		shared_ptr<vector<shared_ptr<pair<int, shared_ptr<geometry::CNPosition>>>>> ret = make_shared<vector<shared_ptr<pair<int, shared_ptr<geometry::CNPosition>>>>>();
		for(auto iter = robotPositions.begin(); iter != robotPositions.end(); iter++)
		{
			if(wm->getTime() - iter->second->getLast()->timeStamp < maxInformationAge)
			{
				shared_ptr<pair<int, shared_ptr<geometry::CNPosition>>> element = make_shared<pair<int, shared_ptr<geometry::CNPosition>>>(iter->first, iter->second->getLast()->getInformation());
				ret->push_back(element);
			}
		}
		return ret;
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

	double Robots::getOpponentProtectDistance()
	{
		return this->opponentProtectDistance;
	}

	double Robots::getOpponentProtectAngle()
	{
		return this->opponentProtectAngle;
	}

	int Robots::teamMatesInOwnPenalty()
	{
		int count = 0;
		int myId = wm->getOwnId();
		MSLFootballField* field = MSLFootballField::getInstance();
		shared_ptr<vector<shared_ptr<pair<int, shared_ptr<geometry::CNPosition>>> >> teamMatePositions = getPositionsOfTeamMates();
		for (int i = 0; i < teamMatePositions->size(); i++)
		{
			if (teamMatePositions->at(i)->first != myId)
			{
				if (field->isInsideOwnPenalty(teamMatePositions->at(i)->second->getPoint(), 0.0))
				{
					count++;
				}
			}
		}
		return count;
	}

	int Robots::teamMatesInOppPenalty()
	{
		int count = 0;
		int myId = wm->getOwnId();
		MSLFootballField* field = MSLFootballField::getInstance();
		shared_ptr<vector<shared_ptr<pair<int, shared_ptr<geometry::CNPosition>>> >> teamMatePositions = getPositionsOfTeamMates();
		for (int i = 0; i < teamMatePositions->size(); i++)
		{
			if (teamMatePositions->at(i)->first != myId)
			{
				if (field->isInsideEnemyPenalty(teamMatePositions->at(i)->second->getPoint(), 100.0))
				{
					count++;
				}
			}
		}
		return count;
	}

}

/* namespace alica */

