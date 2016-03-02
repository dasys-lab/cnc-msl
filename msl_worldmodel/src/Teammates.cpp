/*
 * Teammates.cpp
 *
 *  Created on: Feb 26, 2016
 *      Author: Stefan Jakob
 */

#include "Teammates.h"
#include "MSLWorldModel.h"

namespace msl
{

	Teammates::Teammates(MSLWorldModel* wm, int ringBufferLength)
	{
		this->wm = wm;
		this->ringBufferLength = ringBufferLength;
	}

	Teammates::~Teammates()
	{
	}

	int Teammates::teamMatesInOwnPenalty()
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

	int Teammates::teamMatesInOppPenalty()
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

	shared_ptr<geometry::CNPosition> Teammates::getTeamMatePosition(int teamMateId, int index)
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

	shared_ptr<vector<shared_ptr<pair<int, shared_ptr<geometry::CNPosition>>> >> Teammates::getPositionsOfTeamMates()
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

	shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> Teammates::getTeammatesAlloClustered()
	{
		return teammatesAlloClustered;
	}

	void msl::Teammates::setTeammatesAlloClustered(shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> teammatesAlloClustered)
	{
		this->teammatesAlloClustered = teammatesAlloClustered;
	}

	shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> Teammates::getTeammatesEgoClustered()
	{
		return teammatesEgoClustered;
	}

	void Teammates::setTeammatesEgoClustered(shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> teammatesEgoClustered)
	{
		this->teammatesEgoClustered = teammatesEgoClustered;
	}

}
/* namespace msl */

