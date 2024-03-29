/*
 * DribbleCalibrationContainer.h
 *
 *  Created on: Jul 22, 2016
 *      Author: Carpe Noctem
 */

#ifndef CNC_MSL_MSL_EXPRESSIONS_AUTOGENERATED_INCLUDE_PLANS_DRIBBLECALIBRATION_CONTAINER_DRIBBLECALIBRATIONCONTAINER_H_
#define CNC_MSL_MSL_EXPRESSIONS_AUTOGENERATED_INCLUDE_PLANS_DRIBBLECALIBRATION_CONTAINER_DRIBBLECALIBRATIONCONTAINER_H_

#include <container/CNPoint2D.h>
#include <msl_actuator_msgs/MotionControl.h>
#include <msl_robot/dribbleCalibration/behaviours/DribbleBackward.h>
#include <msl_robot/dribbleCalibration/behaviours/DribbleForward.h>
#include <msl_robot/dribbleCalibration/behaviours/DribbleRotateRight.h>
#include <msl_robot/dribbleCalibration/behaviours/DribbleRotateLeft.h>
#include <msl_robot/dribbleCalibration/container/DribbleCalibrationQuery.h>
#include <memory>
#include <vector>

//#define DEBUG_DC

//using namespace msl;
namespace msl
{
	class DribbleCalibrationContainer
	{
	public:
		DribbleCalibrationContainer();
		virtual ~DribbleCalibrationContainer();

		enum Param
		{
			DribbleForwardParm, DribbleBackwardParm, DribbleLeftParm, DribbleRightParm, RotateLeftParm, RotateRightPram,
			ErrParm
		};

		enum MethodParam
		{
			Move, AdaptParams, WriteConfigParam, ResetParams, SaveParams
		};

		shared_ptr<DribbleCalibrationQuery> paramToMove(Param param, int trans);
		void adaptParam(Param param);
		void writeConfigParameres(Param parm);
		void resetParameters(Param parm);
		void saveParameters(Param parm);

		// opticalFlow stuff
		double getAverageOpticalFlowXValue(shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> queue);
		double getAverageOpticalFlowYValue(shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> queue);

		bool queueFilled;
		bool fillOpticalFlowQueue(int queueSize, shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> opQueue);

	private:
		msl::MSLWorldModel* wm;

		enum OPValue
		{
			XValue, YValue, QOSValue
		};

		shared_ptr<DribbleCalibrationQuery> callBehaviour(MethodParam mParm, Param parm, int trans = 0);
		shared_ptr<DribbleCalibrationQuery> callMethod(ICalibration *behavior, MethodParam parm, int trans = 0);

		// calibration behaviours
		DribbleForward df;
		DribbleBackward db;
		DribbleRotateLeft drl;
		DribbleRotateRight drr;


		double getAverageOpticalFlowValue(OPValue value, shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> queue);
		shared_ptr<msl_actuator_msgs::MotionControl> setNaN(shared_ptr<msl_actuator_msgs::MotionControl> mc);
	};

}
		/* namespace alica */

#endif /* CNC_MSL_MSL_EXPRESSIONS_AUTOGENERATED_INCLUDE_PLANS_DRIBBLECALIBRATION_CONTAINER_DRIBBLECALIBRATIONCONTAINER_H_ */
