/*
 * DribbleCalibrationContainer.h
 *
 *  Created on: Jul 22, 2016
 *      Author: Carpe Noctem
 */

#ifndef CNC_MSL_MSL_EXPRESSIONS_AUTOGENERATED_INCLUDE_PLANS_DRIBBLECALIBRATION_CONTAINER_DRIBBLECALIBRATIONCONTAINER_H_
#define CNC_MSL_MSL_EXPRESSIONS_AUTOGENERATED_INCLUDE_PLANS_DRIBBLECALIBRATION_CONTAINER_DRIBBLECALIBRATIONCONTAINER_H_

#include "msl_actuator_msgs/MotionControl.h"
#include "msl_robot/robotmovement/MovementQuery.h"

#define DEBUG_DC

using namespace msl;
namespace alica
{

	struct subsection
	{
		string name;
		double robotSpeed;
		double actuatorSpeed;
	};

	class DribbleCalibrationContainer
	{
	public:
		DribbleCalibrationContainer();
		virtual ~DribbleCalibrationContainer();

		msl_actuator_msgs::MotionControl getBall();
		msl_actuator_msgs::MotionControl move(int movement, int translation);

		// opticalFlow stuff
		double getAverageOpticalFlowXValue(shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> queue);
		double getAverageOpticalFlowYValue(shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> queue);

		enum Dribble
		{
			DribbleForward,
			DribbleBackward,
			DribbleLeft,
			DribbleRight
		};
//		static const int DRIBBLE_FORWARD = 100;
//		static const int DRIBBLE_BACKWARD = 200;
//		static const int DRIBBLE_LEFT = 300;
//		static const int DRIBBLE_RIGHT = 400;


		bool queueFilled;
		bool fillOpticalFlowQueue(int queueSize, shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> opQueue);

		double readConfigParameter(const char *path);
		void writeConfigParameters(vector<subsection> sections, const char* path);
	private:
		msl::MSLWorldModel* wm;

		shared_ptr<msl::MovementQuery> query;

		enum OPValue
		{
			XValue,
			YValue,
			QOSValue
		};
//		static const int XVALUE = 34;
//		static const int YVALUE = 35;
//		static const int QOSVALUE = 36;
		double getAverageOpticalFlowValue(OPValue value, shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> queue);
	};

} /* namespace alica */

#endif /* CNC_MSL_MSL_EXPRESSIONS_AUTOGENERATED_INCLUDE_PLANS_DRIBBLECALIBRATION_CONTAINER_DRIBBLECALIBRATIONCONTAINER_H_ */