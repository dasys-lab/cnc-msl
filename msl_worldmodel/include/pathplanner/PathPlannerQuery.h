/*
 * PathPlannerQuery.h
 *
 *  Created on: Jun 28, 2016
 *      Author: Stefan Jakob
 */

#ifndef CNC_MSL_MSL_WORLDMODEL_INCLUDE_PATHPLANNER_PATHPLANNERQUERY_H_
#define CNC_MSL_MSL_WORLDMODEL_INCLUDE_PATHPLANNER_PATHPLANNERQUERY_H_

#include <memory>
#include <vector>
#include <container/CNPoint2D.h>

using namespace std;

namespace msl
{

	class PathPlannerQuery
	{
	public:
		PathPlannerQuery();
		virtual ~PathPlannerQuery();
		bool blockOppPenaltyArea;
		bool blockOppGoalArea;
		bool blockOwnPenaltyArea;
		bool blockOwnGoalArea;
		bool block3MetersAroundBall;
		shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> additionalPoints;
	};

} /* namespace msl */

#endif /* CNC_MSL_MSL_WORLDMODEL_INCLUDE_PATHPLANNER_PATHPLANNERQUERY_H_ */
