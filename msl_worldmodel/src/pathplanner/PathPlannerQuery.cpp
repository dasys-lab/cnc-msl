/*
 * PathPlannerQuery.cpp
 *
 *  Created on: Jun 28, 2016
 *      Author: Stefan Jakob
 */

#include <pathplanner/PathPlannerQuery.h>

namespace msl
{

	PathPlannerQuery::PathPlannerQuery()
	{
		this->blockOppPenaltyArea = false;
		this->blockOppGoalArea = false;
		this->blockOwnPenaltyArea = false;
		this->blockOwnGoalArea = false;
		this->block3MetersAroundBall = false;
		this->additionalPoints = nullptr;

	}

	PathPlannerQuery::~PathPlannerQuery()
	{
	}

} /* namespace msl */
