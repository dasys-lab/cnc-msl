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

	PathProxy::PathProxy(MSLWorldModel* wm)
	{
		this->wm = wm;

	}

	PathProxy::~PathProxy()
	{
	}

	//TODO wie ersten knoten anfahren
	//TODO vielleicht sinnvoll aus pfadplaner mgl. pathproxi
	//TODO verschiedene mgl zum anfahren des ersten punktes
	shared_ptr<geometry::CNPoint2D> PathProxy::getEgoDirection(geometry::CNPoint2D egoTarget, bool stayInField, PathEvaluator* eval)
	{
		lastPathTarget = egoTarget;
		shared_ptr<VoronoiNet> net = this->wm->pathPlanner.getCurrentVoronoiNet();
		shared_ptr<geometry::CNPoint2D> retPoint = nullptr;
		shared_ptr<geometry::CNPosition> ownPos = this->wm->rawSensorData.getOwnPositionVision();
		if (ownPos != nullptr)
		{
			shared_ptr<geometry::CNPoint2D> alloTarget = egoTarget.egoToAllo(*ownPos);
			shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> path = this->wm->pathPlanner.aStarSearch(net,
					geometry::CNPoint2D(ownPos->x, ownPos->y)
					, geometry::CNPoint2D(alloTarget->x, alloTarget->y), eval);
			if (path != nullptr)
			{
				retPoint = make_shared<geometry::CNPoint2D>(path->at(0)->x, path->at(0)->y);
			}
		}

		return retPoint->alloToEgo(*ownPos);

	}

} /* namespace msl */
