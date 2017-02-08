/*
 * PathPlannerQuery.h
 *
 *  Created on: Jun 28, 2016
 *      Author: Stefan Jakob
 */

#ifndef CNC_MSL_MSL_WORLDMODEL_INCLUDE_PATHPLANNER_PATHPLANNERQUERY_H_
#define CNC_MSL_MSL_WORLDMODEL_INCLUDE_PATHPLANNER_PATHPLANNERQUERY_H_

#include <container/CNPoint2D.h>
#include <memory>
#include <vector>

using namespace std;

namespace msl
{

/**
 * Encapsulates information passed to the PathPlanner to influence the Voronoi Diagram
 */
class PathPlannerQuery
{
  public:
    PathPlannerQuery();
    virtual ~PathPlannerQuery();
    /**
     * Add artificial obstacles around opp penalty area
     */
    bool blockOppPenaltyArea;
    /**
     * Add artificial obstacles around opp goal area
     */
    bool blockOppGoalArea;
    /**
     * Add artificial obstacles around own penalty area
     */
    bool blockOwnPenaltyArea;
    /**
     * Add artificial obstacles around own goal area
     */
    bool blockOwnGoalArea;
    /**
     * Add artificial obstacles 3 meters around the ball
     */
    bool block3MetersAroundBall;
    /**
     * Add  additional artificial obstacles
     */
    shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> additionalPoints;
    /**
     * Radius for arbitrary circle block
     */
    double circleRadius;
    /**
     * Center for arbitrary circle block
     */
    shared_ptr<geometry::CNPoint2D> circleCenterPoint;
    /**
     * Upper left corner for arbitrary rectangle block in allo coordinates
     */
    shared_ptr<geometry::CNPoint2D> rectangleUpperLeftCorner;
    /**
     * Lower right corner for arbitrary rectangle block in allo coordinates
     */
    shared_ptr<geometry::CNPoint2D> rectangleLowerRightCorner;
};

} /* namespace msl */

#endif /* CNC_MSL_MSL_WORLDMODEL_INCLUDE_PATHPLANNER_PATHPLANNERQUERY_H_ */
