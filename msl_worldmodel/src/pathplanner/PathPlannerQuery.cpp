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
    this->circleCenterPoint = nullptr;
    this->circleRadius = -1;
    this->rectangleLowerRightCorner = nullptr;
    this->rectangleUpperLeftCorner = nullptr;
}

PathPlannerQuery::~PathPlannerQuery()
{
}

} /* namespace msl */
