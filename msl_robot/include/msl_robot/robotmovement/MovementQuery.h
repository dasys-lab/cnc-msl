/*
 * MovementQuery.h
 *
 *  Created on: Apr 27, 2016
 *      Author: Michael Gottesleben
 */

#ifndef CNC_MSL_MSL_WORLDMODEL_SRC_ROBOTMOVEMENT_MOVEMENTQUERY_H_
#define CNC_MSL_MSL_WORLDMODEL_SRC_ROBOTMOVEMENT_MOVEMENTQUERY_H_

#include "Ball.h"
#include "GeometryCalculator.h"
#include "RobotMovement.h"
#include "SystemConfig.h"
#include "msl_actuator_msgs/MotionControl.h"
#include "valarray"
#include "queue"

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
	enum Velocity
	{
		CAREFULLY, DEFAULT, FAST
	};

	/**
	 * Velocity enum to decide how fast we want do go
	 */
	Velocity velocityMode;
	/**
	 * Carefully value for PT-Controller (Drive.conf)
	 */
	double carefullyControllerVelocity;
	/**
	 * Default value for PT-Controller (Drive.conf)
	 */
	double defaultControllerVelocity;
	/**
	 * fast value for PT-Controller (Drive.conf)
	 */
	double fastControllerVelocity;

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

	/**
	 * Initial, empty Array for PT-Controller
	 */
	double init[2] =
	{	0.0,0.0};

	/**
	 * PT-Controller for smooth translation acceleration
	 */
	std::valarray<double> ptController(double rotation, double translation);
	/**
	 * Initialize all needed parameters and queues for the PT-Controller
	 */
	void initializePTControllerParameters();

	void clearPTControllerQueues();

protected:
	double circleRadius;
	shared_ptr<geometry::CNPoint2D> circleCenterPoint;
	shared_ptr<geometry::CNPoint2D> rectangleUpperLeftCorner;
	shared_ptr<geometry::CNPoint2D> rectangleLowerRightCorner;

	double controllerVelocity;

	/**
	 * Past sent translation for PT-Controller
	 */
	std::queue<std::valarray<double>> pastTranslations;

	/**
	 * Past translation input for PT-Controller
	 */
	std::queue<std::valarray<double>> pastControlInput;

private:
	MSLWorldModel *wm;
	MSLRobot *robot;

	void readConfigParameters();
};
}
#endif /* CNC_MSL_MSL_WORLDMODEL_SRC_ROBOTMOVEMENT_MOVEMENTQUERY_H_ */
