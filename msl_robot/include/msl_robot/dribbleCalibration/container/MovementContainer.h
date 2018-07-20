#pragma once

#include "msl_actuator_msgs/MotionControl.h"
#include "msl_robot/robotmovement/MovementQuery.h"
//#include <Plans/DribbleCalibration/Container/DribbleCalibrationContainer.h>

//#define DEBUG_MOVE_CONT

namespace msl
{
	class MovementContainer
	{
	public:
		MovementContainer();
		virtual ~MovementContainer();

		enum Movement
		{
			Forward, Backward, Left, Right, ForwardRight, ForwardLeft, BackwardRight, BackwardLeft
		};
		const char* movementToString[8] = {"Forward", "Backward", "Left", "Right", "Forward right", "Forward left",
											"Backward right", "Backward left"};

		msl_actuator_msgs::MotionControl getBall();
		msl_actuator_msgs::MotionControl move(Movement movement, int translation);
		bool checkObstacles(Movement movement, double distance);

	private:
		// output variables
		bool changeDirFlag;

		msl::MSLWorldModel* wm;

		shared_ptr<msl::MovementQuery> query;

		double defaultDistance;
		double distToObs;
		bool changeDirections;
		bool rotateAroundTheBall;
		double angleTolerance;
		shared_ptr<geometry::CNPoint2D> alloAlignPoint;

		shared_ptr<geometry::CNPoint2D> getEgoDestinationPoint(Movement movement, double distance);
		shared_ptr<geometry::CNPoint2D> calcNewAlignPoint(Movement curMove);
		bool checkFieldLines(shared_ptr<geometry::CNPoint2D> egoDest);
		MotionControl setZero(MotionControl mc);
		MotionControl setNaN(MotionControl mc);
		Movement getNewDirection(int curDir, vector<Movement> movement, int next);
		void readOwnConfigParameter();
	};
}
