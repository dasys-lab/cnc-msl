/*
 * PathProxy.cpp
 *
 *  Created on: May 17, 2015
 *      Author: Stefan Jakob
 */

#include "pathplanner/PathProxy.h"
#include "pathplanner/VoronoiNet.h"
#include "container/CNPosition.h"

namespace msl
{

	PathProxy::PathProxy()
	{
		this->wm = MSLWorldModel::get();

	}

	PathProxy::~PathProxy()
	{
	}


	//TODO wie ersten knoten anfahren
	//TODO verschiedene mgl zum anfahren des ersten punktes
	shared_ptr<geometry::CNPoint2D> PathProxy::getEgoDirection(shared_ptr<geometry::CNPoint2D> egoTarget, shared_ptr<PathEvaluator> eval, shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> additionalPoints)
	{
		lastPathTarget = egoTarget;
		shared_ptr<VoronoiNet> net = this->wm->pathPlanner.getCurrentVoronoiNet();
		if(additionalPoints != nullptr)
		{
			net->insertAdditionalPoints(additionalPoints);
		}
		shared_ptr<geometry::CNPoint2D> retPoint = nullptr;
		shared_ptr<geometry::CNPosition> ownPos = this->wm->rawSensorData.getOwnPositionVision();
		if (ownPos != nullptr)
		{
			shared_ptr<geometry::CNPoint2D> alloTarget = egoTarget->egoToAllo(*ownPos);
			shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> path = this->wm->pathPlanner.plan(net,
					make_shared<geometry::CNPoint2D>(ownPos->x, ownPos->y), alloTarget, eval);
			if (path != nullptr)
			{
				retPoint = make_shared<geometry::CNPoint2D>(path->at(0)->x, path->at(0)->y);
			}
		}
		if(retPoint == nullptr)
		{
			return nullptr;
		}
		return retPoint->alloToEgo(*ownPos);

	}

	PathProxy* PathProxy::getInstance()
	{
		static PathProxy instance;
		return &instance;
	}

} /* namespace msl */
