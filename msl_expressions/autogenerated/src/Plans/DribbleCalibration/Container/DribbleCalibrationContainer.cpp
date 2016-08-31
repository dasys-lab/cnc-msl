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

namespace alica
{

	DribbleCalibrationContainer::DribbleCalibrationContainer()
	{
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
		return mc;
	}

	/**
	 * movement may be: FORWARD, BACKWARD, LEFT, RIGHT
	 *
	 * @return MotionControl to drive in a direction while using PathPlaner and avoid obstacles
	 */
	msl_actuator_msgs::MotionControl DribbleCalibrationContainer::move(int movement, int translation)
	{
		msl_actuator_msgs::MotionControl mc;
		msl::RobotMovement rm;

		if (movement != DRIBBLE_FORWARD && movement != DRIBBLE_BACKWARD && movement != DRIBBLE_LEFT
				&& movement != DRIBBLE_RIGHT)
		{
			cerr << "DribbleCalibrationContainer::move() -> invalid input parameter" << endl;
			mc.senderID = -1;
			mc.motion.translation = NAN;
			mc.motion.rotation = NAN;
			mc.motion.angle = NAN;
			return mc;
		}

		query->reset();

		shared_ptr<geometry::CNPoint2D> egoDestination = make_shared<geometry::CNPoint2D>(0, 0);
		double distance = 300;
		// drive forward
		egoDestination = movement == DRIBBLE_FORWARD ? make_shared<geometry::CNPoint2D>(-distance, 0) : egoDestination;
		// drive backward
		egoDestination = movement == DRIBBLE_BACKWARD ? make_shared<geometry::CNPoint2D>(distance, 0) : egoDestination;
		// drive left
		egoDestination = movement == DRIBBLE_LEFT ? make_shared<geometry::CNPoint2D>(0, distance) : egoDestination;
		// drive right
		egoDestination = movement == DRIBBLE_RIGHT ? make_shared<geometry::CNPoint2D>(0, -distance) : egoDestination;

		query->egoDestinationPoint = egoDestination;
		query->egoAlignPoint = egoDestination;

		mc = rm.moveToPoint(query);

		mc.motion.translation = translation;
		return mc;
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
			cout << "no OpticalFLow signal!" << endl;
			return nullptr;
		}

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
		return getAverageOpticalFlowValue(XVALUE, queue);
	}

	double DribbleCalibrationContainer::getAverageOpticalFlowYValue(shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> queue)
	{
		return getAverageOpticalFlowValue(YVALUE, queue);
	}

	double DribbleCalibrationContainer::getAverageOpticalFlowValue(int value,
																   shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> queue)
	{
		if (value != XVALUE && value != YVALUE && value != QOSVALUE)
		{
			cerr << "DribbleCalibrationContainer::getAverageOpticalFlowValue -> wrong method input!" << endl;
			return -1;
		}

		int sum = 0;
		cout << "in getAverageOpticalFlowValue() " << endl;
		for (shared_ptr<geometry::CNPoint2D> val : *queue)
		{
			if (value == XVALUE)
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
