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
#include "GeometryCalculator.h"
#include "DateTime.h"
#include "SystemConfig.h"

namespace geometry
{
	class CNPoint2D;
}

using namespace std;
using namespace msl_actuator_msgs;

namespace msl
{

	class SearchArea;
	class RobotMovement
	{
	public:
		virtual ~RobotMovement();

		static MotionControl moveGoalie(shared_ptr<geometry::CNPoint2D> alloTarget, shared_ptr<geometry::CNPoint2D> alloAlignPoint, double snapDistance);
		static MotionControl moveToPointFast(shared_ptr<geometry::CNPoint2D> egoTarget,
												shared_ptr<geometry::CNPoint2D> egoAlignPoint, double snapDistance,
												shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> additionalPoints);
		static MotionControl moveToPointCarefully(shared_ptr<geometry::CNPoint2D>egoTarget,
													shared_ptr<geometry::CNPoint2D> egoAlignPoint, double snapDistance, shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> additionalPoints = nullptr);
		static MotionControl interceptCarefully(shared_ptr<geometry::CNPoint2D> egoTarget,
												shared_ptr<geometry::CNPoint2D> egoAlignPoint, double snapDistance, shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> additionalPoints = nullptr);
		//TODO needs to be tested
		static MotionControl alignToPointNoBall(shared_ptr<geometry::CNPoint2D> egoTarget,
												shared_ptr<geometry::CNPoint2D> egoAlignPoint, double angleTolerance);
		static MotionControl alignToPointWithBall(shared_ptr<geometry::CNPoint2D> egoAlignPoint,
													shared_ptr<geometry::CNPoint2D> egoBallPos, double angleTolerance,
													double ballAngleTolerance);
		//TODO needs to be tested
		static MotionControl rapidAlignToPointWithBall(shared_ptr<geometry::CNPoint2D> egoAlignPoint,
													shared_ptr<geometry::CNPoint2D> egoBallPos, double angleTolerance,
													double ballAngleTolerance);

		//TODO needs to be tested
		static msl_actuator_msgs::MotionControl ruleActionForBallGetter();

		//TODO needs to be implemented
		static MotionControl driveRandomly(double translation);

		//TODO needs to be tested
		static MotionControl placeRobotCareBall(shared_ptr<geometry::CNPoint2D> destinationPoint, shared_ptr<geometry::CNPoint2D> headingPoint, double translation);

		//TODO needs to be tested
		static MotionControl placeRobot(shared_ptr<geometry::CNPoint2D> destinationPoint, shared_ptr<geometry::CNPoint2D> headingPoint, double translation);

		//TODO needs to be tested
		static MotionControl placeRobotAggressive(shared_ptr<geometry::CNPoint2D> destinationPoint, shared_ptr<geometry::CNPoint2D> headingPoint, double translation);

		//TODO needs to be tested
		static MotionControl moveToFreeSpace(shared_ptr<geometry::CNPoint2D> alloPassee, double maxTrans);


		static MotionControl driveToPointAlignNoAvoidance(shared_ptr<geometry::CNPoint2D> destination, shared_ptr<geometry::CNPoint2D> alignPoint,
	                                                         double translation, bool alignSlow);
		static MotionControl driveToPointNoAvoidance(shared_ptr<geometry::CNPoint2D> egoDest, double translation);
//		static MotionControl align(MotionControl bm, shared_ptr<geometry::CNPoint2D> alignPoint, double rotTol, bool slow);

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
