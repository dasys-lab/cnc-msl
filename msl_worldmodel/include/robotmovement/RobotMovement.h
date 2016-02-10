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


		static shared_ptr<geometry::CNPoint2D> getRandomTarget();
		static void readConfigParameters();
		static double defaultTranslation;
		static double defaultRotateP;
		static double fastTranslation;
		static double fastRotation;
		static double interceptCarfullyRotateP;

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
