#pragma once
#include "Ball.h"
#include "GeometryCalculator.h"
#include "RobotMovement.h"
#include "SystemConfig.h"
#include "msl_actuator_msgs/MotionControl.h"
#include "valarray"
#include "queue"
#include <MSLEnums.h>

using namespace std;
using namespace msl_actuator_msgs;
namespace msl
{
    class MSLWorldModel;
    class MSLRobot;
    class IPathEvaluator;
    class PathPlannerQuery;
    class MovementQuery
    {
        friend class msl::RobotMovement;

    public:
        MovementQuery();
        virtual ~MovementQuery();
        /**
         * Define the alignment at the egoDestinationPoint
         */
        shared_ptr<geometry::CNPoint2D> egoAlignPoint;
        /**
         * Point the robot is supposed to reach
         */
        shared_ptr<geometry::CNPoint2D> egoDestinationPoint;
        /**
         * Obstacles added to the PathPlanner’s Voronoi Diagram
         */
        shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> additionalPoints;

    /**
     * Velocity enum to decide how fast we want do go
     */
    VelocityMode velocityMode;
    /**
     * Distance when the goal is reached
     */
    double snapDistance;
    /**
     * Angle tolerance for Alignment
     */
    double angleTolerance;
    /**
     * Passing Teammate position for moving into free space
     */
    shared_ptr<geometry::CNPoint2D> alloTeamMatePosition;

    /**
     * PathEvaluator supposed to be used in the A*-Algorithm
     */
    shared_ptr<IPathEvaluator> pathEval;

    /**
     * Block opponent penalty area with artificial obstacles
     */
    bool blockOppPenaltyArea;
    /**
     * Block opponent Goal area with artificial obstacles
     */
    bool blockOppGoalArea;
    /**
     * Block own penalty area with artificial obstacles
     */
    bool blockOwnPenaltyArea;
    /**
     * Block own goal area with artificial obstacles
     */
    bool blockOwnGoalArea;
    /**
     * Block 3 meter circle around the ball during Standards
     */
    bool block3MetersAroundBall;
    /**
     * Block circle shaped area
     * @param centerPoint shared_ptr<geometry::CNPoint2D>
     * @param radius double
     */
    void blockCircle(shared_ptr<geometry::CNPoint2D> centerPoint, double radius);

    /**
     * Block rectangular penalty area
     * @param upLeftCorner shared_ptr<geometry::CNPoint2D>
     * @param lowRightCorner shared_ptr<geometry::CNPoint2D>
     */
    void blockRectangle(shared_ptr<geometry::CNPoint2D> upLeftCorner, shared_ptr<geometry::CNPoint2D> lowRightCorner);

    /**
     * Rotate orthogonally around the ball
     */
    bool rotateAroundTheBall;

    /**
     * Resets all parameters.
     */
    void reset();

    /**
     * Create PathPlannerQuery from fields of this class
     */
    shared_ptr<PathPlannerQuery> getPathPlannerQuery();

protected:
    double circleRadius;
    shared_ptr<geometry::CNPoint2D> circleCenterPoint;
    shared_ptr<geometry::CNPoint2D> rectangleUpperLeftCorner;
    shared_ptr<geometry::CNPoint2D> rectangleLowerRightCorner;


private:
    MSLWorldModel *wm;

};
}
