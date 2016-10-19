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

		msl_actuator_msgs::MotionControl moveToPoint(shared_ptr<MovementQuery> const m_Query);
		msl_actuator_msgs::MotionControl alignTo(shared_ptr<MovementQuery> m_Query);
		msl_actuator_msgs::MotionControl ruleActionForBallGetter();
		msl_actuator_msgs::MotionControl driveRandomly(double translation);
		msl_actuator_msgs::MotionControl moveToFreeSpace(shared_ptr<MovementQuery> m_Query);

		void readConfigParameters();
		double defaultTranslation;
		double defaultRotateP;
		double fastTranslation;
		double fastRotation;

	private:
		static int randomCounter;
		static int beamSize;
		static shared_ptr<vector<shared_ptr<SearchArea>>> fringe;
	static shared_ptr<vector<shared_ptr<SearchArea>>> next;
	static shared_ptr<geometry::CNPoint2D> randomTarget;

	MSLWorldModel* wm;
	PathProxy* pp;
	msl_actuator_msgs::MotionControl placeRobot(shared_ptr<geometry::CNPoint2D> dest, shared_ptr<geometry::CNPoint2D> headingPoint);
	double evalPointDynamic(shared_ptr<geometry::CNPoint2D> alloP, shared_ptr<geometry::CNPoint2D> alloPassee,
			shared_ptr<geometry::CNPosition> ownPos, shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> opponents);
	msl_actuator_msgs::MotionControl setNAN();


	//for alignTO()
	double rotationP;
	double rotationD;
	double transP;
	double transI;

protected:
	static double assume_enemy_velo;
	static double assume_ball_velo;
	static double interceptQuotient;
	static double robotRadius;

};
}

#endif /* CNC_MSL_MSL_WORLDMODEL_SRC_ROBOTMOVEMENT_ROBOTMOVEMENT_H_ */
