/*
 * RobotMovement.h
 *
 *  Created on: 17.12.2014
 *      Author: tobi
 */

#ifndef CNC_MSL_MSL_WORLDMODEL_SRC_ROBOTMOVEMENT_ROBOTMOVEMENT_H_
#define CNC_MSL_MSL_WORLDMODEL_SRC_ROBOTMOVEMENT_ROBOTMOVEMENT_H_

#include <memory>
#include "msl_actuator_msgs/MotionControl.h"
#include "DateTime.h"
#include "SystemConfig.h"

namespace geometry
{
	class CNPoint2D;
	class CNPosition;
}

using namespace std;

namespace msl
{

	class MovementQuery;
	class SearchArea;
	class MSLWorldModel;
	class PathProxy;
	class RobotMovement
	{
	public:
		RobotMovement();
		virtual ~RobotMovement();

		msl_actuator_msgs::MotionControl experimentallyMoveToPoint(shared_ptr<MovementQuery> const m_Query);
		msl_actuator_msgs::MotionControl experimentallyRuleActionForBallGetter();
		msl_actuator_msgs::MotionControl experimentallyDriveRandomly(double translation);
		msl_actuator_msgs::MotionControl experimantallyMoveToFreeSpace(shared_ptr<MovementQuery> m_Query);

		static msl_actuator_msgs::MotionControl moveToPointFast(shared_ptr<geometry::CNPoint2D> egoTarget,
																shared_ptr<geometry::CNPoint2D> egoAlignPoint,
																double snapDistance,
																shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> additionalPoints);
		static msl_actuator_msgs::MotionControl moveToPointCarefully(shared_ptr<geometry::CNPoint2D>egoTarget,
													shared_ptr<geometry::CNPoint2D> egoAlignPoint, double snapDistance, shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> additionalPoints = nullptr);
//		static MotionControl interceptCarefully(shared_ptr<geometry::CNPoint2D> egoTarget,
//												shared_ptr<geometry::CNPoint2D> egoAlignPoint, double snapDistance, shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> additionalPoints = nullptr);
		//TODO needs to be tested
		static msl_actuator_msgs::MotionControl alignToPointNoBall(shared_ptr<geometry::CNPoint2D> egoTarget,
												shared_ptr<geometry::CNPoint2D> egoAlignPoint, double angleTolerance);
		static msl_actuator_msgs::MotionControl alignToPointWithBall(shared_ptr<geometry::CNPoint2D> egoAlignPoint,
													shared_ptr<geometry::CNPoint2D> egoBallPos, double angleTolerance,
													double ballAngleTolerance);
		//TODO needs to be tested
		static msl_actuator_msgs::MotionControl rapidAlignToPointWithBall(shared_ptr<geometry::CNPoint2D> egoAlignPoint,
													shared_ptr<geometry::CNPoint2D> egoBallPos, double angleTolerance,
													double ballAngleTolerance);

		//TODO needs to be tested
		static msl_actuator_msgs::MotionControl ruleActionForBallGetter();

		//TODO needs to be implemented
		static msl_actuator_msgs::MotionControl driveRandomly(double translation);

		//TODO needs to be tested
		static msl_actuator_msgs::MotionControl placeRobotCareBall(shared_ptr<geometry::CNPoint2D> destinationPoint, shared_ptr<geometry::CNPoint2D> headingPoint, double translation);

		//TODO needs to be tested
		static msl_actuator_msgs::MotionControl placeRobot(shared_ptr<geometry::CNPoint2D> destinationPoint, shared_ptr<geometry::CNPoint2D> headingPoint, double translation);

		//TODO needs to be tested
		static msl_actuator_msgs::MotionControl placeRobotAggressive(shared_ptr<geometry::CNPoint2D> destinationPoint, shared_ptr<geometry::CNPoint2D> headingPoint, double translation);

		//TODO needs to be tested
		static msl_actuator_msgs::MotionControl moveToFreeSpace(shared_ptr<geometry::CNPoint2D> alloPassee, double maxTrans);


		static msl_actuator_msgs::MotionControl driveToPointAlignNoAvoidance(shared_ptr<geometry::CNPoint2D> destination, shared_ptr<geometry::CNPoint2D> alignPoint,
	                                                         double translation, bool alignSlow);
		static msl_actuator_msgs::MotionControl driveToPointNoAvoidance(shared_ptr<geometry::CNPoint2D> egoDest, double translation);
//		static msl_actuator_msgs::MotionControl align(MotionControl bm, shared_ptr<geometry::CNPoint2D> alignPoint, double rotTol, bool slow);

		static msl_actuator_msgs::MotionControl driveRandomly(int translation);
		static shared_ptr<msl_actuator_msgs::MotionControl> dribbleToPointConservative(shared_ptr<geometry::CNPoint2D> goalMid, shared_ptr<geometry::CNPoint2D>& ppp);
		static shared_ptr<geometry::CNPoint2D> dribbleNeedToTurn(shared_ptr<geometry::CNPosition> own, shared_ptr<geometry::CNPoint2D> ballPos, shared_ptr<geometry::CNPoint2D> pathPlanningPoint);
		static msl_actuator_msgs::MotionControl nearGoalArea(msl_actuator_msgs::MotionControl bm);
		static void reset();
		static void updateLastTurnTime();

		static shared_ptr<geometry::CNPoint2D> getRandomTarget();
		static void readConfigParameters();
		static double defaultTranslation;
		static double defaultRotateP;
		static double fastTranslation;
		static double fastRotation;
		static double interceptCarfullyRotateP;
		static double maxVel;

	private:
		static double lastRotError;
		static double lastRotErrorWithBall;
		static double alignToPointMinRotation;
		static double alignToPointMaxRotation;
		static double lastRotErrorWithBallRapid;
		static double alignToPointRapidMaxRotation;
		static double alignToPointpRot;
		static double alignMaxVel;
		static double maxVelo;
		static int randomCounter;
		static int beamSize;
		static shared_ptr<vector<shared_ptr<SearchArea>>> fringe;
		static shared_ptr<vector<shared_ptr<SearchArea>>> next;
		static shared_ptr<geometry::CNPoint2D> randomTarget;

		static double lastRotErr;
		static double curRot;
		static double curTrans;
		static double pRot;
		static double dRot;
		static double rotAccStep;
		static double maxRot;
		static double angleDeadBand;
		static double transControlIntegral;
		static double transControlIntegralMax;
		static double iTrans;
		static double pTrans;
		static double transAccStep;
		static double transDecStep;

		static double lastTurnTime;

		//shared_ptr<MovementQuery> query;
		MSLWorldModel* wm;
		PathProxy* pp;
		msl_actuator_msgs::MotionControl experimentallyPlaceRobot(shared_ptr<geometry::CNPoint2D> dest, shared_ptr<geometry::CNPoint2D> headingPoint);
		double experimentallyEvalPointDynamic(shared_ptr<geometry::CNPoint2D> alloP, shared_ptr<geometry::CNPoint2D> alloPassee,
											   shared_ptr<geometry::CNPosition> ownPos, shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> opponents);

		// PD regulator methods
		double rotationPDForDribble(shared_ptr<MovementQuery> query, shared_ptr<geometry::CNPoint2D> target);
		double translationPDForDribble(shared_ptr<MovementQuery> query, double transOrt);
		double anglePDForDribble(shared_ptr<MovementQuery> query, double transOrt);

	protected:
		static double evalPointDynamic(shared_ptr<geometry::CNPoint2D> alloP, shared_ptr<geometry::CNPoint2D> alloPassee,
									   shared_ptr<geometry::CNPosition> ownPos, shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> opponents);
		static double assume_enemy_velo;
		static double assume_ball_velo;
		static double interceptQuotient;
		static double robotRadius;
	};
}

#endif /* CNC_MSL_MSL_WORLDMODEL_SRC_ROBOTMOVEMENT_ROBOTMOVEMENT_H_ */
