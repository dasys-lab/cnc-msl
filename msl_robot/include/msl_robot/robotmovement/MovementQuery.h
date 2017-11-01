/*
 * MovementQuery.h
 *
 *  Created on: Apr 27, 2016
 *      Author: Michael Gottesleben
 */

#pragma once

#include "RobotMovement.h"
<<<<<<< HEAD
=======
#include "SystemConfig.h"
#include "msl_actuator_msgs/MotionControl.h"
#include "valarray"
#include "queue"
>>>>>>> 05da4723d163b63a32a72569805716ffcb1c36f7

#include <Ball.h>
#include <SystemConfig.h>
#include <msl_actuator_msgs/MotionControl.h>
#include <cnc_geometry/CNPointAllo.h>
#include <cnc_geometry/CNPointEgo.h>
#include <nonstd/optional.hpp>

namespace msl
{
<<<<<<< HEAD
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
     * FastTranslation or DefaultTranslation (Motion.conf)
     */
    bool fast;
    /**
     * Robot is dribbeling the ball
     */
    bool dribble;
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

    void resetAllPIDParameters();
    void resetRotationPDParameters();
    void resetTransaltionPIParameters();
    double rotationPDForDribble(geometry::CNPointEgo egoTarget);
    double translationPIForDribble(double transOrt);
    double angleCalcForDribble(double transOrt);

    void setRotationPDParameters(double pParam, double dParam);
    void setTranslationPIParameters(double pParam, double iParam);

    double curRotDribble;
    double curTransDribble;

  private:
    MSLWorldModel *wm;
    MSLRobot *robot;

    // PD variables for RobotMovement::moveToPoint() and RobotMovement::rotationDribblePD()
    double pRot;
    double dRot;
    double rotAccStep;
    double maxRot;
    double lastRotDribbleErr;

    // PD variables for RobotMovement::moveToPoint() and RobotMovement::translationDribblePD()
    double maxVel;
    double angleDeadBand;
    double iTrans;
    double transControlIntegralMax;
    double pTrans;
    double transAccStep;
    double transDecStep;
    double transControlIntegralDribble;

    void readConfigParameters();
=======
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
>>>>>>> 05da4723d163b63a32a72569805716ffcb1c36f7
};

} /* namespace msl */
