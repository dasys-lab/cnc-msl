/*
 * MovementQuery.h
 *
 *  Created on: Apr 27, 2016
 *      Author: Michael Gottesleben
 */

#ifndef CNC_MSL_MSL_WORLDMODEL_SRC_ROBOTMOVEMENT_MOVEMENTQUERY_H_
#define CNC_MSL_MSL_WORLDMODEL_SRC_ROBOTMOVEMENT_MOVEMENTQUERY_H_

#include "msl_actuator_msgs/MotionControl.h"
#include "GeometryCalculator.h"
#include "SystemConfig.h"
#include "Ball.h"
#include "RobotMovement.h"

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
		shared_ptr<geometry::CNPoint2D> egoAlignPoint;
		shared_ptr<geometry::CNPoint2D> egoDestinationPoint;
		shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> additionalPoints;
		bool fast;
		bool dribble;
		double snapDistance;
		double angleTolerance;
		shared_ptr<geometry::CNPoint2D> alloTeamMatePosition;

		//for Voronoi Stuff

		shared_ptr<IPathEvaluator> pathEval;

		bool blockOppPenaltyArea;
		bool blockOppGoalArea;
		bool blockOwnPenaltyArea;
		bool blockOwnGoalArea;
		bool block3MetersAroundBall;
		/**
		 * bolck circle shaped area
		 * @param centerPoint shared_ptr<geometry::CNPoint2D>
		 * @param radious double
		 * @return shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int>>>
		 */
		void blockCircle(shared_ptr<geometry::CNPoint2D> centerPoint, double radius);

		/**
		 * bolck opponent penalty area
		 * @param upLeftCorner shared_ptr<geometry::CNPoint2D>
		 * @param lowRightCorner shared_ptr<geometry::CNPoint2D>
		 * @return shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int>>>
		 */
		void blockRectangle(shared_ptr<geometry::CNPoint2D> upLeftCorner, shared_ptr<geometry::CNPoint2D> lowRightCorner);

		//for RobotMovement::alignTo() stuff
		bool rotateAroundTheBall;

		void reset();

		shared_ptr<PathPlannerQuery> getPathPlannerQuery();

	protected:

		double circleRadius;
		shared_ptr<geometry::CNPoint2D> circleCenterPoint;
		shared_ptr<geometry::CNPoint2D> rectangleUpperLeftCorner;
		shared_ptr<geometry::CNPoint2D> rectangleLowerRightCorner;
		void resetAllPIDParameters();
		void resetRotationPDParameters();
		void resetTransaltionPIParameters();
		double rotationPDForDribble(shared_ptr<geometry::CNPoint2D> egoTarget);
		double translationPIForDribble(double transOrt);
		double angleCalcForDribble(double transOrt);

		void setRotationPDParameters(double pParam, double dParam);
		void setTranslationPIParameters(double pParam, double iParam);

		double curRotDribble;
		double curTransDribble;

	private:
		MSLWorldModel* wm;
		MSLRobot* robot;

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
	};
}
#endif /* CNC_MSL_MSL_WORLDMODEL_SRC_ROBOTMOVEMENT_MOVEMENTQUERY_H_ */
