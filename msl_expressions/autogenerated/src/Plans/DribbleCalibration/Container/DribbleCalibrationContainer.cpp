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
#include <pathplanner/PathPlanner.h>

#include <Plans/DribbleCalibration/Behaviours/Calibrations/DribbleForward.h>

namespace alica
{

	DribbleCalibrationContainer::DribbleCalibrationContainer()
	{
		queueFilled = false;
		this->wm = msl::MSLWorldModel::get();
		this->query = make_shared<MovementQuery>();
		robotRadius = 0;
		defaultDistance = 0;
		distToObs = 0;
		changeDirections = false;
		rotateAroundTheBall = false;
		angleTolerance = 0;
		alloAlignPoint = nullptr;

		// for output
		changeDirFlag = false;

		readOwnConfigParameter();
	}

	DribbleCalibrationContainer::~DribbleCalibrationContainer()
	{

	}

	void DribbleCalibrationContainer::checkParam(Parm parm)
	{
		switch (parm) {
			case DribbleForward:
//				DribbleForward df;
//				df.move();
				break;
			default:
				break;
		}
	}

	msl_actuator_msgs::MotionControl DribbleCalibrationContainer::getBall()
	{
		msl_actuator_msgs::MotionControl mc;
		return mc;
	}

	/**
	 * movement may be: FORWARD, BACKWARD, LEFT, RIGHT
	 *
	 * @return MotionControl to drive in a direction while using PathPlaner and avoid obstacles
	 */
	msl_actuator_msgs::MotionControl DribbleCalibrationContainer::move(Movement movement, int translation)
	{
		MotionControl mc;
		return mc;
	}

	/**
	 * @movement describes the direction in witch you will check for obstacles
	 *
	 * @return true if there is an obstacle in movement direction
	 */
	bool DribbleCalibrationContainer::checkObstacles(Movement movement, double distance)
	{
		return false;
	}

	/**
	 * @return true if the movement destination is outside the field
	 */
	bool DribbleCalibrationContainer::checkFieldLines(shared_ptr<geometry::CNPoint2D> egoDest)
	{
		return false;
	}

	/**
	 * @movement describes the movement direction
	 * @distance to the returned ego destination point
	 *
	 * @return an ego destination point depending on the movement direction
	 */
	shared_ptr<geometry::CNPoint2D> DribbleCalibrationContainer::getEgoDestinationPoint(Movement movement,
																						double distance)
	{
		return nullptr;
	}

	shared_ptr<geometry::CNPoint2D> DribbleCalibrationContainer::calcNewAlignPoint(Movement curMove)
	{
		return nullptr;
	}

	/**
	 * helper function for calcNewAlignPoint
	 * uses vector<Movement> movement like a ring list
	 *
	 * @return the new align point if the robot is driving in another direction than forward
	 */
	DribbleCalibrationContainer::Movement DribbleCalibrationContainer::getNewDirection(int curDir, vector<Movement> movement, int next)
	{
		return movement.at(0);
	}

	/**
	 * @return true if opQueue is filled
	 */
	bool DribbleCalibrationContainer::fillOpticalFlowQueue(int queueSize,
															shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> opQueue)
	{
		if (wm->rawSensorData->getOpticalFlow(0) == nullptr)
		{
			cerr << "no OpticalFLow signal!" << endl;
			return false;
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

	void DribbleCalibrationContainer::readOwnConfigParameter()
	{
		supplementary::SystemConfig* sys = supplementary::SystemConfig::getInstance();
		robotRadius = (*sys)["Rules"]->get<double>("Rules.RobotRadius", NULL);
		defaultDistance = (*sys)["DribbleCalibration"]->get<double>("DribbleCalibration.Default.DefaultDistance", NULL);
		rotateAroundTheBall = (*sys)["DribbleCalibration"]->get<bool>("DribbleCalibration.Default.RotateAroundTheBall",
		NULL);
		angleTolerance = (*sys)["DribbleCalibration"]->get<double>("DribbleCalibration.Default.AngleTolerance", NULL);
	}

	MotionControl DribbleCalibrationContainer::setZero(MotionControl mc)
	{
		mc.motion.translation = 0;
		mc.motion.angle = 0;
		mc.motion.rotation = 0;
		return mc;
	}

	MotionControl DribbleCalibrationContainer::setNaN(MotionControl mc)
	{
		mc.senderID = -1;
		mc.motion.translation = NAN;
		mc.motion.rotation = NAN;
		mc.motion.angle = NAN;
		return mc;
	}
} /* namespace alica */
