#pragma once

#include "RobotMovement.h"
#include <SystemConfig.h>
#include <MSLEnums.h>

#include <Ball.h>
#include <msl_actuator_msgs/MotionControl.h>
#include <msl_actuator_msgs/MotionControl.h>
#include <cnc_geometry/CNPointAllo.h>
#include <cnc_geometry/CNPointEgo.h>
#include <nonstd/optional.hpp>
#include <SystemConfig.h>
#include <valarray>
#include <queue>

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
        nonstd::optional<geometry::CNPointEgo> egoAlignPoint;
        /**
         * Point the robot is supposed to reach
         */
        nonstd::optional<geometry::CNPointEgo> egoDestinationPoint;
        /**
         * Obstacles added to the PathPlanner’s Voronoi Diagram
         */
        nonstd::optional<std::vector<geometry::CNPointAllo>> additionalPoints;

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
        nonstd::optional<geometry::CNPointAllo> alloTeamMatePosition;

        /**
         * PathEvaluator supposed to be used in the A*-Algorithm
         */
        std::shared_ptr<IPathEvaluator> pathEval;

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
        void blockCircle(geometry::CNPointAllo centerPoint, double radius);

        /**
         * Block rectangular penalty area
         * @param upLeftCorner shared_ptr<geometry::CNPoint2D>
         * @param lowRightCorner shared_ptr<geometry::CNPoint2D>
         */
        void blockRectangle(geometry::CNPointAllo upLeftCorner, geometry::CNPointAllo lowRightCorner);

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
        std::shared_ptr<PathPlannerQuery> getPathPlannerQuery() const;

    protected:
        double circleRadius;
        nonstd::optional<geometry::CNPointAllo> circleCenterPoint;
        nonstd::optional<geometry::CNPointAllo> rectangleUpperLeftCorner;
        nonstd::optional<geometry::CNPointAllo> rectangleLowerRightCorner;

    private:
        MSLWorldModel *wm;
    };
}
/* namespace msl */
