/*
 * PathPlannerQuery.cpp
 *
 *  Created on: Jun 28, 2016
 *      Author: Stefan Jakob
 */

#include <pathplanner/PathPlannerQuery.h>

using nonstd::nullopt;

namespace msl
{

PathPlannerQuery::PathPlannerQuery()
{
    this->blockOppPenaltyArea = false;
    this->blockOppGoalArea = false;
    this->blockOwnPenaltyArea = false;
    this->blockOwnGoalArea = false;
    this->block3MetersAroundBall = false;
    this->additionalPoints = nullopt;
    this->circleCenterPoint = nullopt;
    this->circleRadius = -1;
    this->rectangleLowerRightCorner = nullopt;
    this->rectangleUpperLeftCorner = nullopt;
}

PathPlannerQuery::~PathPlannerQuery()
{
}

} /* namespace msl */
