/*
 * DribbleCalibrationContainer.cpp
 *
 *  Created on: Jul 22, 2016
 *      Author: Carpe Noctem
 */

#include <Plans/DribbleCalibration/Container/DribbleCalibrationContainer.h>

#include <SystemConfig.h>
#include <Ball.h>
#include <MSLWorldModel.h>
#include <msl_robot/robotmovement/RobotMovement.h>
#include <RawSensorData.h>
#include <MSLFootballField.h>
#include <obstaclehandler/Obstacles.h>

namespace alica
{

	DribbleCalibrationContainer::DribbleCalibrationContainer()
	{
		queueFilled = false;
		this->wm = msl::MSLWorldModel::get();
		this->query = make_shared<MovementQuery>();
	}

	DribbleCalibrationContainer::~DribbleCalibrationContainer()
	{

	}

	msl_actuator_msgs::MotionControl DribbleCalibrationContainer::getBall()
	{
		msl::RobotMovement rm;
		query->reset();
		msl_actuator_msgs::MotionControl mc;
		shared_ptr<geometry::CNPosition> me;
		if (wm->rawSensorData->getOwnPositionVision() != nullptr)
		{
			me = wm->rawSensorData->getOwnPositionVision();
		}
		else
		{
			cerr << "DribbleCalibrationContainer::getBall() -> couldn't get vision data" << endl;
		}
		shared_ptr<geometry::CNPoint2D> egoBallPos;
		if (wm->ball->getAlloBallPosition()->alloToEgo(*me) != nullptr)
		{
			egoBallPos = wm->ball->getAlloBallPosition()->alloToEgo(*me);
		}
		else
		{
			cerr << "DribbleCalibrationContainer::getBall() -> couldn't get own position" << endl;
		}

		query->egoDestinationPoint = egoBallPos;
		query->egoAlignPoint = egoBallPos;

		mc = rm.moveToPoint(query);
		mc.motion.translation += 400;
		return mc;
	}

	/**
	 * movement may be: FORWARD, BACKWARD, LEFT, RIGHT
	 *
	 * @return MotionControl to drive in a direction while using PathPlaner and avoid obstacles
	 */
	msl_actuator_msgs::MotionControl DribbleCalibrationContainer::move(Movement movement, int translation)
	{
		msl_actuator_msgs::MotionControl mc;
		msl::RobotMovement rm;

		if (movement != Forward && movement != Backward && movement != Left && movement != Right)
		{
			cerr << "DribbleCalibrationContainer::move() -> invalid input parameter" << endl;
			mc.senderID = -1;
			mc.motion.translation = NAN;
			mc.motion.rotation = NAN;
			mc.motion.angle = NAN;
			return mc;
		}

		query->reset();

		shared_ptr<geometry::CNPoint2D> egoDestination = getEgoDestinationPoint(movement);

		query->egoDestinationPoint = egoDestination;
		query->egoAlignPoint = egoDestination;

		mc = rm.moveToPoint(query);

//		mc = movement == AvoidObstacels ? calcNewPath(movement) : mc;

		mc.motion.translation = translation;
		return mc;
	}

	/**
	 * @return true if there is an obstacle in movement direction
	 */
	bool DribbleCalibrationContainer::checkObstacles(Movement movement)
	{
		// check field lines

		// egoDestinationPoint to allo
		shared_ptr<geometry::CNPoint2D> egoDestination = getEgoDestinationPoint(movement);
		shared_ptr<geometry::CNPoint2D> alloDestination = egoDestination->egoToAllo(
				*wm->rawSensorData->getOwnPositionVision());

		// check if destinationPoint is inside the field area
		double fieldLength = wm->field->getFieldLength();
		double fieldWidth = wm->field->getFieldWidth();

		if (fabs(alloDestination->x) > (fieldLength / 2) || fabs(alloDestination->y) > (fieldWidth / 2))
			return false;

		// check obstacles on the field
		shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> ops = wm->obstacles->getAlloObstaclePoints();
		for (shared_ptr<geometry::CNPoint2D> op : *ops)
		{
			shared_ptr<geometry::CNPoint2D> egoOp = op->alloToEgo(*wm->rawSensorData->getOwnPositionVision());
			// check if obstacle is in destination direction
			double distance;

			if (movement == Forward)
				if (egoOp->x > distance && (fabs(egoOp->y) < distance)) // check if correct (first is left coordinate system on whiteboard)
//				if (egoOp->x < -distance && (fabs(egoOp->y) < distance))
					return false;

			if (movement == Backward)
				if (egoOp->x < -distance && (fabs(egoOp->y) < distance))
//				if (egoOp->x > distance && (fabs(egoOp->y) < distance))
					return false;

			if (movement == Right)
				if (egoOp->y < -distance && (fabs(egoOp->x) < distance))
//				if (egoOp->y > distance && (fabs(egoOp->y) < distance))
					return false;

			if (movement == Left)
				if (egoOp->y > distance && (fabs(egoOp->y) < distance))
//				if (egoOp->y < -distance && (fabs(egoOp->x) < distance))
					return false;
		}
		return true;
	}

	shared_ptr<geometry::CNPoint2D> DribbleCalibrationContainer::getEgoDestinationPoint(Movement movement)
	{
		if (movement != Forward && movement != Backward && movement != Left && movement != Right)
		{
			return nullptr;
		}

		shared_ptr<geometry::CNPoint2D> egoDestination = make_shared<geometry::CNPoint2D>(0, 0);
		// 1000 = 1m
		double distance = 1000;
		// drive forward
		egoDestination = movement == Forward ? make_shared<geometry::CNPoint2D>(-distance, 0) : egoDestination;
		// drive backward
		egoDestination = movement == Backward ? make_shared<geometry::CNPoint2D>(distance, 0) : egoDestination;
		// drive left
		egoDestination = movement == Left ? make_shared<geometry::CNPoint2D>(0, distance) : egoDestination;
		// drive right
		egoDestination = movement == Right ? make_shared<geometry::CNPoint2D>(0, -distance) : egoDestination;

		return egoDestination;
	}

	shared_ptr<geometry::CNPoint2D> DribbleCalibrationContainer::calcNewAlignPoint()
	{
		// use checkObstacels to choose a new direction
		// move distance parameter to method parameters
		// return egoAlignPoint


		return nullptr;
	}

	/**
	 * @return true if opQueue is filled
	 */
	bool DribbleCalibrationContainer::fillOpticalFlowQueue(int queueSize,
															shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> opQueue)
	{
		// 10s of rotating the ball
//		int queueSize = 285;

		if (wm->rawSensorData->getOpticalFlow(0) == nullptr)
		{
			cerr << "no OpticalFLow signal!" << endl;
			return nullptr;
		}
//		cout << "queueSize = " << queueSize << endl;
//		cout << "opQueue.size() = " << opQueue->size() << endl;
		if (opQueue->size() >= queueSize)
		{
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

	double DribbleCalibrationContainer::getAverageOpticalFlowValue(OPValue value,
																	shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> queue)
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
//    			cout << val->y << endl;s
				sum += val->y;
			}
		}
		double ret = fabs(sum) / queue->size();
		return sum < 0 ? -ret : ret;

	}

	/**
	 * helps to read config parameters out of the Actuation.conf
	 *
	 * @path in Actuation config to the needed config parameter
	 */
	double DribbleCalibrationContainer::readConfigParameter(const char *path)
	{
		supplementary::SystemConfig* sys = supplementary::SystemConfig::getInstance();
		return (*sys)["Actuation"]->get<double>(path, NULL);
	}

	/**
	 * writes sections to config file
	 *
	 * @sections are the sections
	 * @path is the path to the section e.g. "ForwardDribbleSpeeds"
	 */
	void DribbleCalibrationContainer::writeConfigParameters(vector<subsection> sections, const char* path)
	{
#ifdef DEBUG_DC
		cout << "write sections to config!" << endl;
#endif
		supplementary::SystemConfig* sys = supplementary::SystemConfig::getInstance();
		for (subsection section : sections)
		{
			string s(path);
			s += "." + section.name;

			(*sys)["Actuation"]->set(boost::lexical_cast<std::string>(section.actuatorSpeed),
										(s + ".actuatorSpeed").c_str(), NULL);
#ifdef DEBUG_DC
			cout << "wrote: " << s.append(".actuatorSpeed").c_str() << " with value " << section.actuatorSpeed << endl;
#endif
			(*sys)["Actuation"]->set(boost::lexical_cast<std::string>(section.robotSpeed), (s + ".robotSpeed").c_str(),
			NULL);
		}
		(*sys)["Actuation"]->store();
	}
} /* namespace alica */
