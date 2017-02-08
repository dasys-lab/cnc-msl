/*
 * DribbleCalibrationContainer.cpp
 *
 *  Created on: Jul 22, 2016
 *      Author: Carpe Noctem
 */

#include <msl_robot/dribbleCalibration/container/DribbleCalibrationContainer.h>
#include <SystemConfig.h>
#include <Ball.h>
#include <MSLWorldModel.h>
#include <msl_robot/robotmovement/RobotMovement.h>
#include <RawSensorData.h>
#include <MSLFootballField.h>
#include <obstaclehandler/Obstacles.h>
#include <pathplanner/PathPlanner.h>

namespace msl
{

	DribbleCalibrationContainer::DribbleCalibrationContainer()
	{
		queueFilled = false;
		this->wm = msl::MSLWorldModel::get();
	}

	DribbleCalibrationContainer::~DribbleCalibrationContainer()
	{

	}

	/**
	 * @mParm Parameter to choose the method that should be used
	 * @parm Parameter to call the behavior (something like DribbleForwardParm)
	 * @trans translation
	 */
	shared_ptr<DribbleCalibrationQuery> DribbleCalibrationContainer::callBehaviour(MethodParam mParm, Param parm, int trans)
	{
		switch (parm)
		{
			case DribbleForwardParm:
			{
				cout << "Dribble forward..." << endl;
				return callMethod(&df, mParm, trans);
				break;
			}
			case DribbleBackwardParm:
			{
				return callMethod(&db, mParm, trans);
				break;
			}
			case RotateLeftParm:
			{
				return callMethod(&drl, mParm, trans);
				break;
			}
			case RotateRightPram:
			{
				return callMethod(&drr, mParm, trans);
				break;
			}
		}
		shared_ptr<DribbleCalibrationQuery> query = make_shared<DribbleCalibrationQuery>();
		shared_ptr<msl_actuator_msgs::MotionControl> mc = make_shared<msl_actuator_msgs::MotionControl>();
		mc = setNaN(mc);

		query->setMc(mc);
		return query;
	}

	shared_ptr<DribbleCalibrationQuery> DribbleCalibrationContainer::callMethod(ICalibration* behavior, MethodParam parm, int trans)
	{
		switch (parm)
		{
			case Move:
				return behavior->move(trans);
				break;
			case AdaptParams:
				behavior->adaptParams();
				break;
			case WriteConfigParam:
				behavior->writeConfigParameters();
				break;
			case ResetParams:
				behavior->resetParams();
				break;
			case SaveParams:
				behavior->saveParams();
				break;
			default:
				break;
		}

		shared_ptr<DribbleCalibrationQuery> query = make_shared<DribbleCalibrationQuery>();
		shared_ptr<msl_actuator_msgs::MotionControl> mc = make_shared<msl_actuator_msgs::MotionControl>();
		query->setMc(setNaN(mc));
		return query;
	}

	shared_ptr<DribbleCalibrationQuery> DribbleCalibrationContainer::paramToMove(Param param, int trans)
	{
		return callBehaviour(MethodParam::Move, param, trans);
	}

	void DribbleCalibrationContainer::adaptParam(Param param)
	{
		callBehaviour(MethodParam::AdaptParams, param);
	}

	void DribbleCalibrationContainer::writeConfigParameres(Param parm)
	{
		callBehaviour(MethodParam::WriteConfigParam, parm);
	}

	void DribbleCalibrationContainer::resetParameters(Param parm)
	{
		callBehaviour(MethodParam::ResetParams, parm);
	}
	void DribbleCalibrationContainer::saveParameters(Param parm)
	{
		callBehaviour(MethodParam::SaveParams, parm);
	}

	/**
	 * @return true if opQueue is filled
	 */
	bool DribbleCalibrationContainer::fillOpticalFlowQueue(int queueSize, shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> opQueue)
	{
		if (wm->rawSensorData->getOpticalFlow(0) == nullptr)
		{
			cerr << "no OpticalFLow signal!" << endl;
			return false;
		}
		if (opQueue->size() >= queueSize)
		{
			cout << "1.1" << endl;
			return true;
		}
		if (!queueFilled)
		{
			cout << "filling optical flow queue!" << endl;
			queueFilled = true;
		}
		opQueue->push_back(wm->rawSensorData->getOpticalFlow(0));
		return false;
	}

	double DribbleCalibrationContainer::getAverageOpticalFlowXValue(shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> queue)
	{
		return getAverageOpticalFlowValue(XValue, queue);
	}

	double DribbleCalibrationContainer::getAverageOpticalFlowYValue(shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> queue)
	{
		return getAverageOpticalFlowValue(YValue, queue);
	}

	double DribbleCalibrationContainer::getAverageOpticalFlowValue(OPValue value, shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> queue)
	{
		if (value != XValue && value != YValue /*&& value != QOSValue*/)
		{
			cerr << "DribbleCalibrationContainer::getAverageOpticalFlowValue -> wrong method input!" << endl;
			return -1;
		}

		int sum = 0;

#ifdef DEBUG_DC
		cout << "in getAverageOpticalFlowValue() " << endl;
#endif

		for (shared_ptr<geometry::CNPoint2D> val : *queue)
		{
			if (value == XValue)
			{
				sum += val->x;
			}
			else
			{
				sum += val->y;
			}
		}
		double ret = fabs(sum) / queue->size();
		return sum < 0 ? -ret : ret;

	}

	shared_ptr<msl_actuator_msgs::MotionControl> DribbleCalibrationContainer::setNaN(shared_ptr<msl_actuator_msgs::MotionControl> mc)
	{
		mc->senderID = -1;
		mc->motion.translation = NAN;
		mc->motion.rotation = NAN;
		mc->motion.angle = NAN;
		return mc;
	}
} /* namespace alica */
